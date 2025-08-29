// depthai_camera_test.cc
//
// DepthAI 4-camera sync test.
// For each camera, builds a pipeline with that camera as the frame-sync master,
// starts the device, waits for a grouped message, and reports success/failure.
// On failure, prints a concise line to stderr: "FAILED: CAM_B[,CAM_...]".
//
// Style notes:
// - Functions: UpperCamelCase
// - Variables/params: lower_snake_case
// - Constants: kConstantCase
// - Avoid non-const globals; file-scope state in anonymous namespace
// - Prefer '\n' over std::endl unless flush is required

#include <algorithm>
#include <chrono>
#include <deque>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <opencv2/opencv.hpp>

#include <depthai/build/version.hpp>
#include <depthai/depthai.hpp>

namespace oak_sync {

// ========================= Constants & Types =========================

enum ExitCodeCreateDevice {
  kSuccess = 0,
  kMultipleOrNoDevice = 1,
  kDeviceInitFailed = 2
};

constexpr int kTimeoutMs = 500;
constexpr int kTestFps = 30;
constexpr int kBetweenTestsMs = 200;

// Camera name -> socket mapping (iteration order is key-sorted).
const std::map<std::string, dai::CameraBoardSocket> kNameToSocket = {
    {"CAM_A", dai::CameraBoardSocket::CAM_A},
    {"CAM_B", dai::CameraBoardSocket::CAM_B},
    {"CAM_C", dai::CameraBoardSocket::CAM_C},
    {"CAM_D", dai::CameraBoardSocket::CAM_D},
};

namespace {
// File-scope device info cache (set once by GetDeviceInfo()).
dai::DeviceInfo device_info;
bool device_info_valid = false;
}  // namespace

// ========================= Helpers =========================

ExitCodeCreateDevice GetDeviceInfo() {
  std::cout << "[GetDeviceInfo] Getting device information...\n";

  auto device_info_vec = dai::Device::getAllAvailableDevices();
  std::cout << "[GetDeviceInfo] Found " << device_info_vec.size()
            << " available device(s)\n";

  for (size_t idx = 0; idx < device_info_vec.size(); ++idx) {
    std::cout << "[GetDeviceInfo] Device " << idx << ": "
              << device_info_vec[idx].getMxId() << "\n";
  }

  if (device_info_vec.size() != 1) {
    std::cerr << "[GetDeviceInfo] ERROR: Expected 1 device, found "
              << device_info_vec.size() << "\n";
    return kMultipleOrNoDevice;
  }

  device_info = device_info_vec.front();
  device_info_valid = true;
  std::cout << "[GetDeviceInfo] Device info stored successfully\n";
  return kSuccess;
}

ExitCodeCreateDevice CreateDeviceWithPipeline(
    std::unique_ptr<dai::Device>& device,
    const std::map<std::string, dai::CameraBoardSocket>& name_socket,
    int fps,
    const std::string& sync_master) {
  std::cout << "[CreateDeviceWithPipeline] Creating device with pipeline...\n";
  std::cout << "[CreateDeviceWithPipeline] Sync master: " << sync_master << "\n";

  if (!device_info_valid) {
    std::cerr << "[CreateDeviceWithPipeline] ERROR: Device info not valid\n";
    return kDeviceInitFailed;
  }

  cv::setNumThreads(1);

  // Build the pipeline.
  dai::Pipeline pipeline;
  pipeline.setXLinkChunkSize(0);
  std::cout << "[CreateDeviceWithPipeline] Pipeline created, XLink chunk size set to 0\n";

  auto sync = pipeline.create<dai::node::Sync>();
  auto x_out = pipeline.create<dai::node::XLinkOut>();
  x_out->setStreamName("msgOut");
  sync->out.link(x_out->input);
  std::cout << "[CreateDeviceWithPipeline] Sync and XLinkOut nodes created and linked\n";

  for (const auto& cam_name_socket : name_socket) {
    const std::string& cam_name = cam_name_socket.first;
    std::cout << "[CreateDeviceWithPipeline] Configuring camera: " << cam_name << "\n";

    auto mono_cam = pipeline.create<dai::node::MonoCamera>();
    mono_cam->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    mono_cam->setBoardSocket(cam_name_socket.second);
    mono_cam->setFps(fps);

    // IMPORTANT: use [] to create Sync input port if missing.
    mono_cam->out.link(sync->inputs[cam_name]);

    const bool is_master = (cam_name == sync_master);
    mono_cam->initialControl.setFrameSyncMode(
        is_master ? dai::CameraControl::FrameSyncMode::OUTPUT
                  : dai::CameraControl::FrameSyncMode::INPUT);
    mono_cam->initialControl.setAutoExposureEnable();

    std::cout << "[CreateDeviceWithPipeline]   - Resolution: 800P\n";
    std::cout << "[CreateDeviceWithPipeline]   - Socket: "
              << static_cast<int>(cam_name_socket.second) << "\n";
    std::cout << "[CreateDeviceWithPipeline]   - Frame sync mode: "
              << (is_master ? "OUTPUT (master)" : "INPUT") << "\n";
    std::cout << "[CreateDeviceWithPipeline]   - Auto exposure enabled\n";
  }

  // Start the device with the pipeline.
  try {
    std::cout << "[CreateDeviceWithPipeline] Creating device with pipeline...\n";
    device = std::make_unique<dai::Device>(pipeline, device_info,
                                           dai::UsbSpeed::SUPER_PLUS);
    if (device == nullptr) {
      std::cerr << "[CreateDeviceWithPipeline] ERROR: Device creation failed (nullptr)\n";
      return kDeviceInitFailed;
    }
    std::cout << "[CreateDeviceWithPipeline] Device created and pipeline started successfully\n";
    return kSuccess;
  } catch (const std::exception& e) {
    std::cerr << "[CreateDeviceWithPipeline] ERROR: Device creation failed: " << e.what() << "\n";
    return kDeviceInitFailed;
  }
}

bool TestImageReception(std::unique_ptr<dai::Device>& device, int timeout_ms) {
  std::cout << "[TestImageReception] Starting image reception test...\n";

  auto msg_group = device->getOutputQueue("msgOut", 1, false);
  std::cout << "[TestImageReception] Got output queue\n";

  try {
    bool has_timed_out = false;
    auto msg_data = msg_group->get<dai::MessageGroup>(
        std::chrono::milliseconds(timeout_ms), has_timed_out);

    if (!has_timed_out && msg_data != nullptr) {
      std::cout << "[TestImageReception] msg_data not null\n";
      return true;
    } else if (has_timed_out) {
      std::cout << "[TestImageReception] Get timed out after " << timeout_ms << "ms\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "[TestImageReception] Failed to read msg_data: " << e.what() << "\n";
  }

  std::cout << "[TestImageReception] TIMEOUT: No images received after "
            << timeout_ms << "ms\n";
  return false;
}

}  // namespace oak_sync

// ========================= Main =========================

int main() {
  std::cout << "========================================\n";
  std::cout << "Starting DepthAI Camera Test (Optimized)\n";
  std::cout << "========================================\n";

  bool success[4] = {};

  // Cache device info once.
  const oak_sync::ExitCodeCreateDevice device_info_result = oak_sync::GetDeviceInfo();
  if (device_info_result != oak_sync::kSuccess) {
    std::cerr << "[main] Failed to get device info with code: " << device_info_result << "\n";
    return 1;
    }

  std::cout << "Testing " << oak_sync::kNameToSocket.size() << " cameras...\n";

  int idx = 0;
  for (const auto& cam_name_socket : oak_sync::kNameToSocket) {
    const std::string& cam_name = cam_name_socket.first;

    std::cout << "\n========================================\n";
    std::cout << "Testing Camera " << idx << ": " << cam_name << " as sync master\n";
    std::cout << "========================================\n";

    std::unique_ptr<dai::Device> device;
    const oak_sync::ExitCodeCreateDevice create_rc =
        oak_sync::CreateDeviceWithPipeline(device, oak_sync::kNameToSocket,
                                           oak_sync::kTestFps, cam_name);

    if (create_rc != oak_sync::kSuccess) {
      std::cerr << "[main] Device creation failed with code: " << create_rc << "\n";
      success[idx] = false;

      if (device) {
        std::cout << "[main] Closing device despite creation failure...\n";
        device->close();
      }

      ++idx;
      std::cout << "[main] Skipping to next camera...\n";
      std::this_thread::sleep_for(std::chrono::milliseconds(oak_sync::kBetweenTestsMs));
      continue;
    }

    // Small delay to let pipeline stabilize.
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Test image reception.
    std::cout << "[main] Testing image reception...\n";
    success[idx] = oak_sync::TestImageReception(device, oak_sync::kTimeoutMs);

    std::cout << "[main] Test complete for " << cam_name
              << " as master, result: " << (success[idx] ? "SUCCESS" : "FAILED") << "\n";

    // Close device before next test.
    std::cout << "[main] Closing device...\n";
    device->close();
    std::cout << "[main] Device closed\n";

    // Wait before next test.
    std::this_thread::sleep_for(std::chrono::milliseconds(oak_sync::kBetweenTestsMs));
    ++idx;
  }

  // Print results.
  std::cout << "\n========================================\n";
  std::cout << "Test Results Summary:\n";
  std::cout << "========================================\n";

  idx = 0;
  for (const auto& cam_name_socket : oak_sync::kNameToSocket) {
    std::cout << cam_name_socket.first << " as master: "
              << (success[idx] ? "SUCCESS" : "FAILED") << "\n";
    ++idx;
  }

  std::cout << "========================================\n";
  std::cout << "Test Complete\n";
  std::cout << "========================================\n";

  // Determine exit code and print failed cameras to stderr (for Ansible).
  int successful_count = 0;
  std::vector<std::string> failed_cameras;

  idx = 0;
  for (const auto& cam_name_socket : oak_sync::kNameToSocket) {
    if (success[idx]) {
      ++successful_count;
    } else {
      failed_cameras.push_back(cam_name_socket.first);
    }
    ++idx;
  }

  if (!failed_cameras.empty()) {
    std::cerr << "FAILED: ";
    for (size_t i = 0; i < failed_cameras.size(); ++i) {
      std::cerr << failed_cameras[i];
      if (i + 1 < failed_cameras.size()) std::cerr << ",";
    }
    std::cerr << "\n";
  }

  // Return 0 for success, 1 for failure.
  return (successful_count == static_cast<int>(oak_sync::kNameToSocket.size())) ? 0 : 1;
}

