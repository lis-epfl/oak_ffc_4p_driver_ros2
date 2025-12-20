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
  kDeviceInitFailed = 2,
  kDeviceNotFound = 3
};

constexpr int kTimeoutMs = 500;      // Increased timeout for image reception
constexpr int kTestFps = 30;
constexpr int kBetweenTestsMs = 200; // Increased delay to allow device reset

// Camera name -> socket mapping (iteration order is key-sorted).
const std::map<std::string, dai::CameraBoardSocket> kNameToSocket = {
    {"CAM_A", dai::CameraBoardSocket::CAM_A},
    {"CAM_B", dai::CameraBoardSocket::CAM_B},
    {"CAM_C", dai::CameraBoardSocket::CAM_C},
    {"CAM_D", dai::CameraBoardSocket::CAM_D},
};

// ========================= Helpers =========================

// Get the MXID of the single connected device
std::string GetTargetDeviceMxId() {
  std::cout << "[GetTargetDeviceMxId] Scanning for devices...\n";
  auto device_info_vec = dai::Device::getAllAvailableDevices();

  if (device_info_vec.empty()) {
    std::cerr << "[GetTargetDeviceMxId] No devices found!\n";
    return "";
  }
  if (device_info_vec.size() > 1) {
    std::cerr << "[GetTargetDeviceMxId] Warning: Multiple devices found. Using the first one.\n";
  }

  std::string mxId = device_info_vec[0].getDeviceId();
  std::cout << "[GetTargetDeviceMxId] Found device: " << mxId << "\n";
  return mxId;
}

// Find specific device info by MXID (refreshes state)
bool FindDeviceByMxId(const std::string& mxId, dai::DeviceInfo& outInfo) {
    // Retry a few times if device is re-enumerating
    for(int i = 0; i < 5; i++) {
        auto devices = dai::Device::getAllAvailableDevices();
        for (const auto& d : devices) {
            if (d.getDeviceId() == mxId) {
                outInfo = d;
                return true;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    return false;
}

ExitCodeCreateDevice CreateDeviceWithPipeline(
    std::shared_ptr<dai::Device>& device,
    std::shared_ptr<dai::Pipeline>& pipeline,
    std::shared_ptr<dai::MessageQueue>& outputQueue,
    const std::map<std::string, dai::CameraBoardSocket>& name_socket,
    int fps,
    const std::string& sync_master,
    const std::string& targetMxId) {

  std::cout << "[CreateDeviceWithPipeline] Finding device " << targetMxId << "...\n";

  dai::DeviceInfo currentDevInfo;
  if (!FindDeviceByMxId(targetMxId, currentDevInfo)) {
      std::cerr << "[CreateDeviceWithPipeline] ERROR: Device " << targetMxId << " not found (maybe still rebooting?)\n";
      return kDeviceNotFound;
  }

  cv::setNumThreads(1);

  // Initialize Device first (V3 Requirement)
  try {
    std::cout << "[CreateDeviceWithPipeline] Connecting to device...\n";
    device = std::make_shared<dai::Device>(currentDevInfo, dai::UsbSpeed::SUPER_PLUS);
    if (device == nullptr) {
      std::cerr << "[CreateDeviceWithPipeline] ERROR: Device creation failed (nullptr)\n";
      return kDeviceInitFailed;
    }
  } catch (const std::exception& e) {
    std::cerr << "[CreateDeviceWithPipeline] ERROR: Device creation failed: " << e.what() << "\n";
    return kDeviceInitFailed;
  }

  // Build the pipeline passing the device
  pipeline = std::make_shared<dai::Pipeline>(device);
  pipeline->setXLinkChunkSize(0);

  auto sync = pipeline->create<dai::node::Sync>();

  for (const auto& cam_name_socket : name_socket) {
    const std::string& cam_name = cam_name_socket.first;

    // Note: Using MonoCamera for simplicity/compatibility in this checker
    auto mono_cam = pipeline->create<dai::node::MonoCamera>();
    mono_cam->setResolution(dai::MonoCameraProperties::SensorResolution::THE_800_P);
    mono_cam->setBoardSocket(cam_name_socket.second);
    mono_cam->setFps(fps);

    mono_cam->out.link(sync->inputs[cam_name]);

    const bool is_master = (cam_name == sync_master);
    mono_cam->initialControl.setFrameSyncMode(
        is_master ? dai::CameraControl::FrameSyncMode::OUTPUT
                  : dai::CameraControl::FrameSyncMode::INPUT);
    mono_cam->initialControl.setAutoExposureEnable();
  }

  // Create queue directly from node
  outputQueue = sync->out.createOutputQueue();

  // Start the pipeline
  try {
    std::cout << "[CreateDeviceWithPipeline] Starting pipeline...\n";
    pipeline->start();
    return kSuccess;
  } catch (const std::exception& e) {
    std::cerr << "[CreateDeviceWithPipeline] ERROR: Pipeline start failed: " << e.what() << "\n";
    return kDeviceInitFailed;
  }
}

bool TestImageReception(std::shared_ptr<dai::MessageQueue>& msg_group, int timeout_ms) {
  std::cout << "[TestImageReception] Starting image reception test...\n";

  if(!msg_group) {
      std::cerr << "[TestImageReception] Queue is null\n";
      return false;
  }

  try {
    bool has_timed_out = false;
    auto msg_data = msg_group->get<dai::MessageGroup>(
        std::chrono::milliseconds(timeout_ms), has_timed_out);

    if (!has_timed_out && msg_data != nullptr) {
      std::cout << "[TestImageReception] msg_data received successfully\n";
      return true;
    } else if (has_timed_out) {
      std::cout << "[TestImageReception] Get timed out after " << timeout_ms << "ms\n";
    }
  } catch (const std::exception& e) {
    std::cerr << "[TestImageReception] Failed to read msg_data: " << e.what() << "\n";
  }

  return false;
}

}  // namespace oak_sync

// ========================= Main =========================

int main() {
  std::cout << "========================================\n";
  std::cout << "Starting DepthAI Camera Test (Optimized)\n";
  std::cout << "========================================\n";

  bool success[4] = {};

  // Get target device ID once
  std::string targetMxId = oak_sync::GetTargetDeviceMxId();
  if (targetMxId.empty()) {
      return 1;
  }

  std::cout << "Testing " << oak_sync::kNameToSocket.size() << " cameras...\n";

  int idx = 0;
  for (const auto& cam_name_socket : oak_sync::kNameToSocket) {
    const std::string& cam_name = cam_name_socket.first;

    std::cout << "\n========================================\n";
    std::cout << "Testing Camera " << idx << ": " << cam_name << " as sync master\n";
    std::cout << "========================================\n";

    // Scope for device/pipeline/queue to ensure destruction
    {
        std::shared_ptr<dai::Device> device;
        std::shared_ptr<dai::Pipeline> pipeline;
        std::shared_ptr<dai::MessageQueue> queue;

        const oak_sync::ExitCodeCreateDevice create_rc =
            oak_sync::CreateDeviceWithPipeline(device, pipeline, queue, oak_sync::kNameToSocket,
                                            oak_sync::kTestFps, cam_name, targetMxId);

        if (create_rc == oak_sync::kSuccess) {
            // Small delay to let pipeline stabilize
            std::this_thread::sleep_for(std::chrono::milliseconds(50));

            // Test image reception
            success[idx] = oak_sync::TestImageReception(queue, oak_sync::kTimeoutMs);
            std::cout << "[main] Test complete for " << cam_name
                    << " as master, result: " << (success[idx] ? "SUCCESS" : "FAILED") << "\n";
        } else {
            std::cerr << "[main] Device creation failed with code: " << create_rc << "\n";
            success[idx] = false;
        }

        // Clean up explicitly
        std::cout << "[main] Cleaning up resources...\n";
        queue.reset();
        pipeline.reset();
        if (device) {
            device->close();
            device.reset();
        }
    } // End of scope ensures destructors run

    std::cout << "[main] Resources released. Waiting " << oak_sync::kBetweenTestsMs << "ms for device reset...\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(oak_sync::kBetweenTestsMs));
    ++idx;
  }

  // Print results
  std::cout << "\n========================================\n";
  std::cout << "Test Results Summary:\n";
  std::cout << "========================================\n";

  idx = 0;
  int successful_count = 0;
  std::vector<std::string> failed_cameras;

  for (const auto& cam_name_socket : oak_sync::kNameToSocket) {
    std::cout << cam_name_socket.first << " as master: "
              << (success[idx] ? "SUCCESS" : "FAILED") << "\n";

    if (success[idx]) {
      ++successful_count;
    } else {
      failed_cameras.push_back(cam_name_socket.first);
    }
    ++idx;
  }

  std::cout << "========================================\n";

  if (!failed_cameras.empty()) {
    std::cerr << "FAILED: ";
    for (size_t i = 0; i < failed_cameras.size(); ++i) {
      std::cerr << failed_cameras[i];
      if (i + 1 < failed_cameras.size()) std::cerr << ",";
    }
    std::cerr << "\n";
  }

  return (successful_count == static_cast<int>(oak_sync::kNameToSocket.size())) ? 0 : 1;
}
