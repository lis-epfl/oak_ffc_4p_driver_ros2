// Lean MCAP recorder for the 4 CAM_*/compressed topics.
// Bypasses rosbag2_cpp; writes via libmcap directly with a background drain
// thread fed from a std::deque guarded by a mutex.
//
// Note on shutdown: under systemd's KillMode=control-group, this process is
// SIGKILL'd before it can finish writing the mcap footer/index, so the file
// on disk is missing its summary section.  We don't try to fight that here —
// the data records themselves are intact, and `recover_mcap.py` (run from
// drones_postflight.yml) re-streams them into a well-formed mcap.  The
// destructor calls finalize() for the rare graceful-exit case (e.g. dev /
// debugging via Ctrl-C with rclcpp's default SIGINT handler), but it's not
// expected to run under systemctl stop.
//
// Build: see CMakeLists.txt addition. Run:
//   ros2 run oak_ffc_4p_driver_ros2 oak_compressed_recorder \
//       --ros-args -p output_dir:=/tmp/bag -p chunk_mb:=8


#include <mcap/writer.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace fs = std::filesystem;
using sensor_msgs::msg::CompressedImage;

struct PendingMsg {
  uint16_t channel_id;
  uint64_t log_time_ns;
  uint64_t pub_time_ns;
  std::vector<uint8_t> data;
};

class CompressedRecorder : public rclcpp::Node {
public:
  CompressedRecorder() : Node("oak_compressed_recorder") {
    output_dir_ = declare_parameter<std::string>("output_dir", "/home/lis/oak_exp/bags/custom");
    chunk_mb_   = declare_parameter<int>("chunk_mb", 8);
    auto topics = declare_parameter<std::vector<std::string>>(
        "topics",
        {"/oak_ffc_4p_driver_node/CAM_A/compressed",
         "/oak_ffc_4p_driver_node/CAM_B/compressed",
         "/oak_ffc_4p_driver_node/CAM_C/compressed",
         "/oak_ffc_4p_driver_node/CAM_D/compressed"});

    fs::create_directories(output_dir_);
    std::string out_path = output_dir_ + "/recording.mcap";
    file_.open(out_path, std::ios::binary);
    if (!file_.is_open()) {
      RCLCPP_FATAL(get_logger(), "cannot open %s for write", out_path.c_str());
      throw std::runtime_error("open failed");
    }

    mcap::McapWriterOptions opts("ros2");
    opts.compression = mcap::Compression::None;
    opts.chunkSize = static_cast<uint64_t>(chunk_mb_) * 1024 * 1024;
    writer_.open(file_, opts);

    // Register schema once. CompressedImage is small; we hard-code its IDL ref
    // so consumers (foxglove, mcap-cli) understand the messages.
    mcap::Schema schema(
        "sensor_msgs/msg/CompressedImage", "ros2msg",
        "std_msgs/Header header\nstring format\nuint8[] data\n"
        "================================================================================\n"
        "MSG: std_msgs/Header\nbuiltin_interfaces/Time stamp\nstring frame_id\n"
        "================================================================================\n"
        "MSG: builtin_interfaces/Time\nint32 sec\nuint32 nanosec\n");
    writer_.addSchema(schema);
    schema_id_ = schema.id;

    rclcpp::QoS qos = rclcpp::QoS(rclcpp::KeepLast(20)).best_effort();
    for (const auto &t : topics) {
      mcap::Channel ch(t, "cdr", schema_id_);
      writer_.addChannel(ch);
      uint16_t cid = ch.id;
      auto sub = create_subscription<CompressedImage>(
          t, qos,
          [this, cid](const CompressedImage::ConstSharedPtr msg) {
            this->on_message(cid, msg);
          });
      subs_.push_back(sub);
      RCLCPP_INFO(get_logger(), "subscribed to %s, channel_id=%u", t.c_str(), cid);
    }

    drain_thread_ = std::thread([this]() { drain_loop(); });
    stats_thread_ = std::thread([this]() { stats_loop(); });
    RCLCPP_INFO(get_logger(), "recording to %s, chunk_size=%dMB", out_path.c_str(), chunk_mb_);
  }

  // Best-effort shutdown for the rare graceful-exit case (Ctrl-C in dev,
  // explicit rclcpp::shutdown() in tests).  Under systemctl stop this never
  // runs to completion before SIGKILL — recover_mcap.py handles that path.
  ~CompressedRecorder() {
    stop_.store(true);
    queue_cv_.notify_all();
    if (drain_thread_.joinable()) drain_thread_.join();
    if (stats_thread_.joinable()) stats_thread_.join();
    writer_.close();
    file_.close();
  }

private:
  static constexpr size_t QUEUE_HARD_CAP = 4096;  // ~1 GB worst case at 260 KB/msg

  void on_message(uint16_t channel_id, const CompressedImage::ConstSharedPtr &msg) {
    received_.fetch_add(1, std::memory_order_relaxed);
    // Serialize the message via rclcpp::Serialization
    rclcpp::Serialization<CompressedImage> serializer;
    rclcpp::SerializedMessage smsg;
    serializer.serialize_message(msg.get(), &smsg);

    PendingMsg pm;
    pm.channel_id = channel_id;
    auto stamp_ns = static_cast<uint64_t>(msg->header.stamp.sec) * 1000000000ULL +
                    msg->header.stamp.nanosec;
    pm.pub_time_ns = stamp_ns;
    pm.log_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                         std::chrono::system_clock::now().time_since_epoch())
                         .count();
    pm.data.assign(smsg.get_rcl_serialized_message().buffer,
                   smsg.get_rcl_serialized_message().buffer + smsg.size());
    {
      std::lock_guard<std::mutex> lk(queue_mtx_);
      if (queue_.size() >= QUEUE_HARD_CAP) {
        overflow_.fetch_add(1, std::memory_order_relaxed);
        return;  // hard drop to avoid unbounded RAM growth
      }
      queue_.push_back(std::move(pm));
    }
    queue_cv_.notify_one();
  }

  void drain_loop() {
    std::deque<PendingMsg> local;
    while (!stop_.load() || !queue_empty()) {
      {
        std::unique_lock<std::mutex> lk(queue_mtx_);
        queue_cv_.wait_for(lk, std::chrono::milliseconds(50),
                           [this] { return !queue_.empty() || stop_.load(); });
        local.swap(queue_);
      }
      while (!local.empty()) {
        PendingMsg &pm = local.front();
        mcap::Message m;
        m.channelId = pm.channel_id;
        m.sequence = static_cast<uint32_t>(written_.load());
        m.logTime = pm.log_time_ns;
        m.publishTime = pm.pub_time_ns;
        m.dataSize = pm.data.size();
        m.data = reinterpret_cast<const std::byte*>(pm.data.data());
        auto status = writer_.write(m);
        if (!status.ok()) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                                "mcap write error: %s", status.message.c_str());
        }
        written_.fetch_add(1, std::memory_order_relaxed);
        local.pop_front();
      }
    }
  }

  bool queue_empty() {
    std::lock_guard<std::mutex> lk(queue_mtx_);
    return queue_.empty();
  }

  void stats_loop() {
    auto last = std::chrono::steady_clock::now();
    size_t last_recv = 0, last_writ = 0, last_over = 0;
    while (!stop_.load()) {
      std::this_thread::sleep_for(std::chrono::seconds(2));
      auto now = std::chrono::steady_clock::now();
      double dt = std::chrono::duration<double>(now - last).count();
      size_t r = received_.load(), w = written_.load(), o = overflow_.load();
      size_t qsz;
      { std::lock_guard<std::mutex> lk(queue_mtx_); qsz = queue_.size(); }
      RCLCPP_INFO(get_logger(),
                  "[stats] recv=%zu (+%.1f/s) write=%zu (+%.1f/s) overflow=%zu (+%zu) qsz=%zu",
                  r, (r-last_recv)/dt, w, (w-last_writ)/dt, o, (o-last_over), qsz);
      last_recv = r; last_writ = w; last_over = o; last = now;
    }
  }

  // ROS state
  std::vector<rclcpp::Subscription<CompressedImage>::SharedPtr> subs_;
  std::string output_dir_;
  int chunk_mb_;

  // mcap state
  std::ofstream file_;
  mcap::McapWriter writer_;
  uint16_t schema_id_ {0};

  // queue
  std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  std::deque<PendingMsg> queue_;
  std::atomic<bool> stop_ {false};
  std::thread drain_thread_;
  std::thread stats_thread_;

  // counters
  std::atomic<size_t> received_ {0};
  std::atomic<size_t> written_ {0};
  std::atomic<size_t> overflow_ {0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CompressedRecorder>();
  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
