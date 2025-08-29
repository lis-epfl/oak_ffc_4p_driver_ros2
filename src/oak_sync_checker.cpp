#include <chrono>
#include <depthai/build/version.hpp>
#include <depthai/depthai.hpp>
#include <deque>
#include <iostream>
#include <map>
#include <sstream>
#include <thread>
#include <vector>

// Exit codes for the application
enum ExitCodeCreateDevice {
  kSuccess = 0,
  kMultipleOrNoDevice = 1,
  kDeviceInitFailed = 2
};

std::map<std::string, dai::CameraBoardSocket> name_socket = {
    {"CAM_A", dai::CameraBoardSocket::CAM_A},
    {"CAM_B", dai::CameraBoardSocket::CAM_B},
    {"CAM_C", dai::CameraBoardSocket::CAM_C},
    {"CAM_D", dai::CameraBoardSocket::CAM_D},
};

// Global device info to reuse across recreations
dai::DeviceInfo g_device_info;
bool g_device_info_valid = false;

ExitCodeCreateDevice GetDeviceInfo() {
  std::cout << "[GetDeviceInfo] Getting device information..." << std::endl;

  auto device_info_vec = dai::Device::getAllAvailableDevices();
  std::cout << "[GetDeviceInfo] Found " << device_info_vec.size()
            << " available device(s)" << std::endl;

  for (size_t i = 0; i < device_info_vec.size(); i++) {
    std::cout << "[GetDeviceInfo] Device " << i << ": "
              << device_info_vec[i].getMxId() << std::endl;
  }

  if (device_info_vec.size() != 1) {
    std::cout << "[GetDeviceInfo] ERROR: Expected 1 device, found "
              << device_info_vec.size() << std::endl;
    return kMultipleOrNoDevice;
  }

  g_device_info = device_info_vec.front();
  g_device_info_valid = true;
  std::cout << "[GetDeviceInfo] Device info stored successfully" << std::endl;
  return kSuccess;
}

ExitCodeCreateDevice CreateDeviceWithPipeline(
    std::unique_ptr<dai::Device> &device,
    std::map<std::string, dai::CameraBoardSocket> name_socket,
    int fps,
    std::string sync_master) {

  std::cout << "[CreateDeviceWithPipeline] Creating device with pipeline..." << std::endl;
  std::cout << "[CreateDeviceWithPipeline] Sync master: " << sync_master << std::endl;

  if (!g_device_info_valid) {
    std::cout << "[CreateDeviceWithPipeline] ERROR: Device info not valid" << std::endl;
    return kDeviceInitFailed;
  }

  cv::setNumThreads(1);

  // Create pipeline first
  dai::Pipeline pipeline;
  pipeline.setXLinkChunkSize(0);
  std::cout << "[CreateDeviceWithPipeline] Pipeline created, XLink chunk size set to 0"
            << std::endl;

  auto sync = pipeline.create<dai::node::Sync>();
  auto xOut = pipeline.create<dai::node::XLinkOut>();
  xOut->setStreamName("msgOut");
  sync->out.link(xOut->input);
  std::cout << "[CreateDeviceWithPipeline] Sync and XLinkOut nodes created and linked"
            << std::endl;

  for (const auto &cam_name_socket : name_socket) {
    std::cout << "[CreateDeviceWithPipeline] Configuring camera: "
              << cam_name_socket.first << std::endl;

    auto mono_cam = pipeline.create<dai::node::MonoCamera>();
    dai::MonoCameraProperties::SensorResolution mono_resolution;
    mono_resolution = dai::MonoCameraProperties::SensorResolution::THE_800_P;

    mono_cam->setResolution(mono_resolution);
    mono_cam->setBoardSocket(cam_name_socket.second);
    mono_cam->setFps(fps);
    mono_cam->out.link(sync->inputs[cam_name_socket.first]);

    bool is_master = (cam_name_socket.first == sync_master);
    mono_cam->initialControl.setFrameSyncMode(
        is_master ? dai::CameraControl::FrameSyncMode::OUTPUT
                  : dai::CameraControl::FrameSyncMode::INPUT);

    std::cout << "[CreateDeviceWithPipeline]   - Resolution: 800P" << std::endl;
    std::cout << "[CreateDeviceWithPipeline]   - Socket: "
              << static_cast<int>(cam_name_socket.second) << std::endl;
    std::cout << "[CreateDeviceWithPipeline]   - Frame sync mode: "
              << (is_master ? "OUTPUT (master)" : "INPUT") << std::endl;

    mono_cam->initialControl.setAutoExposureEnable();
    std::cout << "[CreateDeviceWithPipeline]   - Auto exposure enabled" << std::endl;
  }

  // Create device with pipeline
  const auto usb_speed = dai::UsbSpeed::SUPER_PLUS;
  auto open_vino_version = dai::OpenVINO::Version::VERSION_2021_4;

  try {
    std::cout << "[CreateDeviceWithPipeline] Creating device with pipeline..." << std::endl;
    device = std::make_unique<dai::Device>(pipeline, g_device_info, usb_speed);

    if (device == nullptr) {
      std::cout << "[CreateDeviceWithPipeline] ERROR: Device creation failed (nullptr)"
                << std::endl;
      return kDeviceInitFailed;
    }

    std::cout << "[CreateDeviceWithPipeline] Device created and pipeline started successfully"
              << std::endl;
    return kSuccess;

  } catch (const std::exception &e) {
    std::cout << "[CreateDeviceWithPipeline] ERROR: Device creation failed: "
              << e.what() << std::endl;
    return kDeviceInitFailed;
  }
}

bool TestImageReception(std::unique_ptr<dai::Device> &device, int timeout_ms) {
  std::cout << "[TestImageReception] Starting image reception test..."
            << std::endl;

  auto const msg_grp = device->getOutputQueue("msgOut", 1, false);
  std::cout << "[TestImageReception] Got output queue" << std::endl;

  try {
    bool has_timed_out = false;
    auto msg_data = msg_grp->get<dai::MessageGroup>(
        std::chrono::milliseconds(timeout_ms), has_timed_out);

    if (!has_timed_out && msg_data != nullptr) {
      std::cout << "[TestImageReception] msg_data not null" << std::endl;
      return true;
    } else if (has_timed_out) {
      std::cout << "[TestImageReception] Get timed out after " << timeout_ms
                << "ms" << std::endl;
    }
  } catch (const std::exception &e) {
    std::cout << " [TestImageReception] Failed to read msg_data: " << e.what()
              << std::endl;
  }

  std::cout << "[TestImageReception] TIMEOUT: No images received after "
            << timeout_ms << "ms" << std::endl;
  return false;
}

int main() {
  std::cout << "========================================" << std::endl;
  std::cout << "Starting DepthAI Camera Test (Optimized)" << std::endl;
  std::cout << "========================================" << std::endl;

  int timeout_ms = 500;
  int test_fps = 30;
  int waiting_between_tests_ms = 200;

  bool success[4];

  // Get device info once
  ExitCodeCreateDevice device_info_result = GetDeviceInfo();
  if (device_info_result != kSuccess) {
    std::cout << "[main] Failed to get device info with code: " << device_info_result
              << std::endl;
    return 1;
  }

  std::cout << "Testing " << name_socket.size() << " cameras..." << std::endl;

  int i = 0;
  for (const auto &cam_name_socket : name_socket) {
    std::cout << "\n========================================" << std::endl;
    std::cout << "Testing Camera " << i << ": " << cam_name_socket.first
              << " as sync master" << std::endl;
    std::cout << "========================================" << std::endl;

    std::unique_ptr<dai::Device> device;

    // Create device with pipeline configured for this sync master
    ExitCodeCreateDevice exit_device = CreateDeviceWithPipeline(
        device, name_socket, test_fps, cam_name_socket.first);

    if (exit_device != kSuccess) {
      std::cout << "[main] Device creation failed with code: " << exit_device
                << std::endl;
      success[i] = false;

      if (device) {
        std::cout << "[main] Closing device despite creation failure..."
                  << std::endl;
        device->close();
      }

      i++;
      std::cout << "[main] Skipping to next camera..." << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(waiting_between_tests_ms));
      continue;
    }

    // Small delay to let pipeline stabilize
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Test image reception
    std::cout << "[main] Testing image reception..." << std::endl;
    success[i] = TestImageReception(device, timeout_ms);

    std::cout << "[main] Test complete for " << cam_name_socket.first
              << " as master, result: " << (success[i] ? "SUCCESS" : "FAILED")
              << std::endl;

    // Close device before next test
    std::cout << "[main] Closing device..." << std::endl;
    device->close();
    std::cout << "[main] Device closed" << std::endl;

    // Wait before next test
    std::this_thread::sleep_for(std::chrono::milliseconds(waiting_between_tests_ms));
    i++;
  }

  // Print results
  std::cout << "\n========================================" << std::endl;
  std::cout << "Test Results Summary:" << std::endl;
  std::cout << "========================================" << std::endl;

  for (int i = 0; i < 4; i++) {
    std::cout << "Cam " << i << " as master: " << (success[i] ? "SUCCESS" : "FAILED")
              << std::endl;
  }

  std::cout << "========================================" << std::endl;
  std::cout << "Test Complete" << std::endl;
  std::cout << "========================================" << std::endl;

  // Determine exit code and print failed cameras to stderr
  int successful_count = 0;
  std::vector<std::string> failed_cameras;

  for (int i = 0; i < 4; i++) {
    if (success[i]) {
      successful_count++;
    } else {
      // Map index to camera name
      if (i == 0)
        failed_cameras.push_back("CAM_A");
      else if (i == 1)
        failed_cameras.push_back("CAM_B");
      else if (i == 2)
        failed_cameras.push_back("CAM_C");
      else if (i == 3)
        failed_cameras.push_back("CAM_D");
    }
  }

  // Print failed cameras to stderr for easy parsing by Ansible
  if (!failed_cameras.empty()) {
    std::cerr << "FAILED: ";
    for (size_t i = 0; i < failed_cameras.size(); i++) {
      std::cerr << failed_cameras[i];
      if (i < failed_cameras.size() - 1)
        std::cerr << ",";
    }
    std::cerr << std::endl;
  }

  // Return 0 for success, 1 for failure
  return (successful_count == 4) ? 0 : 1;
}
