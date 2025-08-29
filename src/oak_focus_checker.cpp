// oak_focus_checker.cc
//
// Captures mono frames from an OAK FFC 4P, computes Brenner focus on a center
// ROI (1/3 image), and either:
//   - CHECK MODE (default): grab first frames, report out-of-focus cameras via
//     concise stderr ("OUT_OF_FOCUS: CAM_...[,CAM_...]"), exit(1) if any fail.
//   - STREAM MODE (--stream): live 2x2 view with per-camera ROI score/labels.
//
// CLI:
//   oak_focus_checker [--stream] [--threshold=N] [--timeout-ms=M]
//
// Notes (Google C++ Style):
//   - Functions: UpperCamelCase
//   - Variables: lower_snake_case; data members end with underscore
//   - Constants: kConstantCase
//   - No exceptions escape main(); internal calls catch and return status.

#include <algorithm>
#include <chrono>
#include <depthai/depthai.hpp>
#include <iomanip>
#include <iostream>
#include <map>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace oak_focus {

// Default constants.
constexpr double kDefaultThreshold = 500.0; // Brenner ROI threshold
constexpr int kDefaultTimeoutMs = 2000;     // Wait for first frames

// Window dimensions for stream mode display.
constexpr int kDisplayWidth = 1280;
constexpr int kDisplayHeight = 750;
constexpr int kCellW = 640;
constexpr int kCellH = 360;

struct Options {
  bool stream = false;
  double threshold = kDefaultThreshold;
  int timeout_ms = kDefaultTimeoutMs;
};

class FocusChecker {
public:
  explicit FocusChecker(const Options &opts) : opts_(opts) {}

  // Initializes device and pipeline. Returns false on failure.
  bool Initialize() {
    if (!InitializeDevice())
      return false;
    return InitializePipeline();
  }

  // Non-interactive: capture first frames, score ROI, report OOF.
  // Returns 0 if all in focus; 1 if any out of focus; 2 on error.
  int RunCheckOnce() {
    auto frames = CaptureFirstFrames();
    if (!frames.has_value())
      return 2;

    std::vector<std::string> out_of_focus;
    for (const auto &[name, mat] : frames.value()) {
      const double score = mat.empty() ? 0.0 : BrennerRoi(mat);
      if (score < opts_.threshold)
        out_of_focus.push_back(name);
    }

    if (!out_of_focus.empty()) {
      std::cerr << "OUT_OF_FOCUS: ";
      for (size_t i = 0; i < out_of_focus.size(); ++i) {
        std::cerr << out_of_focus[i];
        if (i + 1 < out_of_focus.size())
          std::cerr << ",";
      }
      std::cerr << std::endl;
      return 1;
    }
    return 0;
  }

  // Interactive streaming view.
  int RunStream() {
    std::cout << "\n=== OAK FFC 4P Focus Checker (stream) ===\n";
    std::cout << "Brenner on center ROI (1/3). IN FOCUS if ROI >= "
              << opts_.threshold << "\n";
    std::cout << "Keys: q/ESC quit, space pause, s save current frames\n\n";

    bool paused = false;
    bool save_next = false;

    while (true) {
      if (!paused) {
        StreamOnce(save_next);
        save_next = false;
      }

      const int key = cv::waitKey(1);
      if (key == 'q' || key == 27)
        break;
      if (key == ' ') {
        paused = !paused;
        std::cout << (paused ? "--- PAUSED ---\n" : "--- RESUMED ---\n");
      }
      if (key == 's') {
        save_next = true;
        std::cout << "Saving next frame set...\n";
      }
    }
    cv::destroyAllWindows();
    return 0;
  }

private:
  // ---- Device / pipeline ----
  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<dai::Pipeline> pipeline_;
  Options opts_;

  const std::map<std::string, dai::CameraBoardSocket> cams_ = {
      {"CAM_A", dai::CameraBoardSocket::CAM_A},
      {"CAM_B", dai::CameraBoardSocket::CAM_B},
      {"CAM_C", dai::CameraBoardSocket::CAM_C},
      {"CAM_D", dai::CameraBoardSocket::CAM_D},
  };

  // Latest frames/scores (used in stream mode for saving).
  std::map<std::string, cv::Mat> latest_images_;
  std::map<std::string, double> latest_scores_;

  bool InitializeDevice() {
    try {
      auto infos = dai::Device::getAllAvailableDevices();
      if (infos.empty()) {
        std::cerr << "No devices found!\n";
        return false;
      }
      device_ = std::make_unique<dai::Device>(infos[0]);
      return true;
    } catch (const std::exception &e) {
      std::cerr << "Create device failed: " << e.what() << "\n";
      return false;
    }
  }

  bool InitializePipeline() {
    try {
      pipeline_ = std::make_unique<dai::Pipeline>();

      auto sync = pipeline_->create<dai::node::Sync>();
      auto xout = pipeline_->create<dai::node::XLinkOut>();
      xout->setStreamName("sync_out");
      sync->out.link(xout->input);

      for (const auto &[name, socket] : cams_) {
        auto mono = pipeline_->create<dai::node::MonoCamera>();
        mono->setResolution(
            dai::MonoCameraProperties::SensorResolution::THE_720_P);
        mono->setBoardSocket(socket);
        mono->setFps(30);

        // IMPORTANT: use [] here (not .at)
        mono->out.link(sync->inputs[name]);

        mono->initialControl.setFrameSyncMode(
            name == "CAM_A" ? dai::CameraControl::FrameSyncMode::OUTPUT
                            : dai::CameraControl::FrameSyncMode::INPUT);
        mono->initialControl.setManualExposure(10000, 400);
      }

      device_->startPipeline(*pipeline_);
      return true;
    } catch (const std::exception &e) {
      std::cerr << "Start pipeline failed: " << e.what() << "\n";
      return false;
    }
  }

  // ---- Focus math ----
  static double Brenner(const cv::Mat &image) {
    if (image.empty())
      return 0.0;

    cv::Mat gray;
    if (image.channels() == 3) {
      cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
      gray = image;
    }

    cv::Mat f32;
    gray.convertTo(f32, CV_32F);

    // Vectorized 2-pixel differences.
    cv::Mat fx = f32.colRange(2, f32.cols) - f32.colRange(0, f32.cols - 2);
    cv::Mat fy = f32.rowRange(2, f32.rows) - f32.rowRange(0, f32.rows - 2);
    fx = fx.mul(fx);
    fy = fy.mul(fy);

    const double sumx = cv::sum(fx)[0];
    const double sumy = cv::sum(fy)[0];
    const double denom =
        static_cast<double>(gray.rows) * static_cast<double>(gray.cols);
    return (sumx + sumy) / std::max(1.0, denom);
  }

  static cv::Rect CenterRoi(const cv::Mat &img) {
    const int w = std::max(1, img.cols / 3);
    const int h = std::max(1, img.rows / 3);
    const int x = std::max(0, (img.cols - w) / 2);
    const int y = std::max(0, (img.rows - h) / 2);
    return cv::Rect(x, y, std::min(w, img.cols - x), std::min(h, img.rows - y));
  }

  static double BrennerRoi(const cv::Mat &image) {
    if (image.empty())
      return 0.0;
    const cv::Rect roi = CenterRoi(image);
    return Brenner(image(roi));
  }

  // ---- First-frame capture ----
  std::optional<std::map<std::string, cv::Mat>> CaptureFirstFrames() {
    try {
      std::map<std::string, cv::Mat> frames;
      auto q = device_->getOutputQueue("sync_out", /*maxSize=*/4,
                                       /*blocking=*/false);

      const auto start = std::chrono::steady_clock::now();
      const auto deadline = start + std::chrono::milliseconds(opts_.timeout_ms);

      while (std::chrono::steady_clock::now() < deadline) {
        bool timed_out = false;
        auto grp = q->get<dai::MessageGroup>(std::chrono::milliseconds(200),
                                             timed_out);
        if (!grp)
          continue;

        for (const auto &[name, _] : cams_) {
          if (frames.count(name))
            continue;
          auto f = grp->get<dai::ImgFrame>(name);
          if (f)
            frames[name] = f->getCvFrame();
        }
        if (frames.size() == cams_.size())
          break;
      }

      // Ensure all keys exist (empty if missing).
      for (const auto &[name, _] : cams_) {
        if (!frames.count(name))
          frames[name] = cv::Mat();
      }
      return frames;
    } catch (const std::exception &e) {
      std::cerr << "CaptureFirstFrames error: " << e.what() << "\n";
      return std::nullopt;
    }
  }

  // ---- Stream helpers ----
  void StreamOnce(bool save_next) {
    auto q = device_->getOutputQueue("sync_out", 1, false);
    auto grp = q->tryGet<dai::MessageGroup>();
    if (!grp)
      return;

    cv::Mat display(kDisplayHeight, kDisplayWidth, CV_8UC3,
                    cv::Scalar(0, 0, 0));

    {
      std::ostringstream oss;
      oss << "Brenner on ROI (center 1/3) | IN if ROI >= " << std::fixed
          << std::setprecision(0) << opts_.threshold;
      cv::putText(display, oss.str(), {10, 25}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
                  {255, 255, 255}, 2);
    }

    const int grid[4][2] = {{0, 30}, {640, 30}, {0, 390}, {640, 390}};
    int idx = 0;

    for (const auto &[name, _] : cams_) {
      cv::Mat img;
      if (auto f = grp->get<dai::ImgFrame>(name))
        img = f->getCvFrame();

      latest_images_[name] = img.clone();
      const double score = BrennerRoi(img);
      latest_scores_[name] = score;
      const bool in_focus = score >= opts_.threshold;

      cv::Mat resized;
      if (!img.empty()) {
        cv::resize(img, resized, cv::Size(kCellW, kCellH));
      } else {
        resized = cv::Mat(kCellH, kCellW, CV_8UC1, cv::Scalar(0));
      }

      cv::Mat color;
      if (resized.channels() == 1) {
        cv::cvtColor(resized, color, cv::COLOR_GRAY2BGR);
      } else {
        color = resized;
      }

      // ROI rectangle in display coordinates (center 1/3 of 640x360).
      const cv::Rect roi_disp(213, 120, 214, 120);
      cv::rectangle(color, roi_disp, cv::Scalar(255, 255, 0), 2);

      // Labels.
      const cv::Scalar col =
          in_focus ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
      const std::string label =
          name + std::string(" - ") + (in_focus ? "IN FOCUS" : "OUT OF FOCUS");
      cv::putText(color, label, {10, 30}, cv::FONT_HERSHEY_SIMPLEX, 1.0, col,
                  2);

      {
        std::ostringstream oss;
        oss << "ROI Brenner: " << std::fixed << std::setprecision(1) << score;
        cv::putText(color, oss.str(), {10, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.7,
                    {255, 255, 255}, 2);
      }

      // Simple bar.
      const int bar_x = 10, bar_y = 100, bar_w = 200, bar_h = 10;
      cv::rectangle(color, {bar_x, bar_y}, {bar_x + bar_w, bar_y + bar_h},
                    {50, 50, 50}, -1);
      const double max_v = opts_.threshold * 1.5;
      const int fill =
          std::clamp(static_cast<int>((score / max_v) * bar_w), 0, bar_w);
      cv::rectangle(color, {bar_x, bar_y}, {bar_x + fill, bar_y + bar_h}, col,
                    -1);
      const int marker = static_cast<int>((opts_.threshold / max_v) * bar_w);
      cv::line(color, {bar_x + marker, bar_y - 2},
               {bar_x + marker, bar_y + bar_h + 2}, {0, 255, 255}, 2);

      color.copyTo(
          display(cv::Rect(grid[idx][0], grid[idx][1], kCellW, kCellH)));
      ++idx;
    }

    if (save_next)
      SaveImages();
    cv::imshow("OAK Focus (stream)", display);
  }

  void SaveImages() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ts;
    ts << std::put_time(std::localtime(&t), "%Y%m%d_%H%M%S");

    for (const auto &[name, img] : latest_images_) {
      if (img.empty())
        continue;
      const double score =
          latest_scores_.count(name) ? latest_scores_.at(name) : 0.0;
      std::ostringstream fn;
      fn << "focus_" << name << "_" << ts.str() << "_brenner_roi_"
         << static_cast<int>(score) << ".png";
      cv::imwrite(fn.str(), img);
      std::cout << "Saved: " << fn.str() << "\n";
    }
  }
};

} // namespace oak_focus

// ----------------- main & CLI -----------------
int main(int argc, char **argv) {
  oak_focus::Options opts;

  for (int i = 1; i < argc; ++i) {
    const std::string arg(argv[i]);
    if (arg == "--stream") {
      opts.stream = true;
    } else if (arg.rfind("--threshold=", 0) == 0) {
      try {
        opts.threshold = std::stod(arg.substr(12));
      } catch (...) {
        std::cerr << "Invalid threshold.\n";
        return 2;
      }
    } else if (arg.rfind("--timeout-ms=", 0) == 0) {
      try {
        opts.timeout_ms = std::stoi(arg.substr(13));
      } catch (...) {
        std::cerr << "Invalid timeout.\n";
        return 2;
      }
    } else if (arg == "--help" || arg == "-h") {
      std::cout << "Usage: " << argv[0]
                << " [--stream] [--threshold=N] [--timeout-ms=M]\n";
      return 0;
    } else {
      std::cerr << "Unknown option: " << arg << "\n";
      return 2;
    }
  }

  oak_focus::FocusChecker checker(opts);
  if (!checker.Initialize())
    return 2;
  return opts.stream ? checker.RunStream() : checker.RunCheckOnce();
}
