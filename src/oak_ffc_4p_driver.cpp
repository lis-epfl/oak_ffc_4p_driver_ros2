#include "oak_ffc_4p_driver.hpp"
#include <future>  // [DBG] for std::async parallel encode experiment

namespace oak_ffc_4p_driver {

FFC4PDriver::FFC4PDriver()
    : rclcpp::Node("oak_ffc_4p_driver_node"),
      node_handle_(std::shared_ptr<FFC4PDriver>(this, [](auto *) {})),
      image_transport_(node_handle_) {

  // declare environment parameters
  DeclareRosParameters();

  // initialize parameters
  InitializeRosParameters();

  // check if the device is available and print all info
  bool device_detected = CreateDevice();

  // continue if device is detected
  if (device_detected) {
    // initialize pipeline
    InitializePipeline();

    // create a thread to stream the video
    streaming_thread_ = std::thread(&FFC4PDriver::StreamVideo, this);
  }
}

FFC4PDriver::~FFC4PDriver() { device_->close(); }

void FFC4PDriver::DeclareRosParameters() {
  declare_parameter("sync_master", "CAM_A");
  declare_parameter("resolution", "800");
  declare_parameter("fps", 30);
  declare_parameter("rgb", false);
  declare_parameter("auto_exposure_time", true);
  declare_parameter("exposure_time_us", 10000);
  declare_parameter("iso", 400);
  declare_parameter("image_info", true);
  declare_parameter("auto_awb", false);
  declare_parameter("awb_value", 4000);
  declare_parameter("sharpness_calibration_mode", false);
  declare_parameter("enable_upside_down", false);
  declare_parameter("max_size_fps", 10);
  declare_parameter("publish_cams_individually", false);
  declare_parameter("compress_images", false);
  declare_parameter("jpeg_quality", 80);
  declare_parameter("use_chip_mjpeg", false);  // [DBG] Option 2 toggle
}

void FFC4PDriver::InitializeRosParameters() {
  sync_master_ = get_parameter("sync_master").as_string();
  resolution_ = get_parameter("resolution").as_string();
  fps_ = get_parameter("fps").as_int();
  rgb_ = get_parameter("rgb").as_bool();
  auto_exposure_time_ = get_parameter("auto_exposure_time").as_bool();
  exposure_time_us_ = get_parameter("exposure_time_us").as_int();
  iso_ = get_parameter("iso").as_int();
  image_info_ = get_parameter("image_info").as_bool();
  auto_awb_ = get_parameter("auto_awb").as_bool();
  awb_value_ = get_parameter("awb_value").as_int();
  sharpness_calibration_mode_ =
      get_parameter("sharpness_calibration_mode").as_bool();
  enable_upside_down_ = get_parameter("enable_upside_down").as_bool();
  max_size_fps_ = get_parameter("max_size_fps").as_int();
  publish_cams_individually_ =
      get_parameter("publish_cams_individually").as_bool();
  compress_images_ = get_parameter("compress_images").as_bool();
  jpeg_quality_ = get_parameter("jpeg_quality").as_int();
  use_chip_mjpeg_ = get_parameter("use_chip_mjpeg").as_bool();  // [DBG] Option 2
}

bool FFC4PDriver::CreateDevice() {
  cv::setNumThreads(0);  // 0 = use all available cores
  RCLCPP_INFO(get_logger(), "FFC 4P Device Detecting\n");
  auto device_info_vec = dai::Device::getAllAvailableDevices();
  const auto usb_speed = dai::UsbSpeed::SUPER_PLUS;

  if (device_info_vec.size() != 1) {
    RCLCPP_ERROR(get_logger(), "Multiple devices or No device detected\n");
    return false;
  }
  // CHANGED: Removed OpenVINO version from Device constructor (moved to pipeline config)
  device_ = std::make_shared<dai::Device>(device_info_vec.front(), usb_speed);

  if (device_ == nullptr) {
    RCLCPP_ERROR(get_logger(), "device init failed\n");
    return false;
  }
  // print device infomation
  // CHANGED: getMxId() -> getDeviceId()
  std::cout << "===Connected to " << device_info_vec.front().getDeviceId()
            << std::endl;
  auto mx_id = this->device_->getDeviceId(); // CHANGED: getMxId() -> getDeviceId()
  auto cameras = this->device_->getConnectedCameras();
  auto usb_speed_dev = this->device_->getUsbSpeed();
  auto eeprom_data = this->device_->readCalibration2().getEepromData();
  std::cout << "   >>> MXID:" << mx_id << std::endl;
  std::cout << "   >>> Num of cameras:" << cameras.size() << std::endl;
  std::cout << "   >>> USB speed:" << usb_speed_dev << std::endl;
  if (eeprom_data.boardName != "") {
    std::cout << "   >>> Board name:" << eeprom_data.boardName << std::endl;
  }
  if (eeprom_data.productName != "") {
    std::cout << "   >>> Product name:" << eeprom_data.productName << std::endl;
  }
  RCLCPP_INFO(get_logger(), "FFC 4P Device detected!\n");

  return true;
}

void FFC4PDriver::InitializePipeline() {
  // Init and Start pipeline
  pipeline_ = std::make_unique<dai::Pipeline>(device_);

  // CHANGED: Set OpenVINO version here
  pipeline_->setOpenVINOVersion(dai::OpenVINO::Version::VERSION_2021_4);

  pipeline_->setXLinkChunkSize(0);
  auto sync = pipeline_->create<dai::node::Sync>();

  // Note: XLinkOut removed in v3 port

  for (const auto &cam_name_socket : name_socket_) {
    if (rgb_) { // RGB camera
      // Note: ColorCamera is deprecated, consider migrating to dai::node::Camera if needed
      auto rgb_cam = pipeline_->create<dai::node::ColorCamera>();
      dai::ColorCameraProperties::SensorResolution rgb_resolution;
      if (color_res_opts_.find(resolution_) == color_res_opts_.end()) {
        RCLCPP_ERROR(get_logger(), "Resolution %s not supported, use default\n",
                     resolution_.c_str());
        rgb_resolution = color_res_opts_["720"];
      } else {
        rgb_resolution = color_res_opts_[resolution_];
      }
      rgb_cam->setResolution(rgb_resolution);
      rgb_cam->setBoardSocket(cam_name_socket.second);
      rgb_cam->setInterleaved(false);
      rgb_cam->setFps(fps_);
      if (use_chip_mjpeg_) {
        // [DBG] Option 2: insert on-chip MJPEG encoder, ship compressed bytes
        auto enc = pipeline_->create<dai::node::VideoEncoder>();
        enc->setDefaultProfilePreset(fps_, dai::VideoEncoderProperties::Profile::MJPEG);
        enc->setQuality(jpeg_quality_);
        rgb_cam->video.link(enc->input);
        enc->bitstream.link(sync->inputs[cam_name_socket.first]);
      } else {
        rgb_cam->isp.link(sync->inputs[cam_name_socket.first]);
      }
      rgb_cam->initialControl.setFrameSyncMode(
          cam_name_socket.first == sync_master_
              ? dai::CameraControl::FrameSyncMode::OUTPUT
              : dai::CameraControl::FrameSyncMode::INPUT);
      if (auto_exposure_time_) {
        rgb_cam->initialControl.setAutoExposureEnable();
      } else {
        rgb_cam->initialControl.setManualExposure(exposure_time_us_, iso_);
      }
      if (!auto_awb_) {
        rgb_cam->initialControl.setManualWhiteBalance(awb_value_);
      }
    } else {
      // Note: MonoCamera is deprecated, consider migrating to dai::node::Camera if needed
      auto mono_cam = pipeline_->create<dai::node::MonoCamera>();
      dai::MonoCameraProperties::SensorResolution mono_resolution;
      if (mono_res_opts_.find(resolution_) == mono_res_opts_.end()) {
        RCLCPP_ERROR(get_logger(), "Resolution %s not supported, use default\n",
                     resolution_.c_str());
        mono_resolution = mono_res_opts_["720"];
      } else {
        mono_resolution = mono_res_opts_[resolution_];
      }
      mono_cam->setResolution(mono_resolution);
      mono_cam->setBoardSocket(cam_name_socket.second);
      mono_cam->setFps(fps_);
      mono_cam->out.link(sync->inputs[cam_name_socket.first]);
      mono_cam->initialControl.setFrameSyncMode(
          cam_name_socket.first == sync_master_
              ? dai::CameraControl::FrameSyncMode::OUTPUT
              : dai::CameraControl::FrameSyncMode::INPUT);
      if (auto_exposure_time_) {
        mono_cam->initialControl.setAutoExposureEnable();
      } else {
        mono_cam->initialControl.setManualExposure(exposure_time_us_, iso_);
      }
      if (!auto_awb_) {
        mono_cam->initialControl.setManualWhiteBalance(awb_value_);
      }
    }
  }

  // Create output queue directly from node
  output_queue_ = sync->out.createOutputQueue();

  // Start pipeline
  pipeline_->start();

  // create ros publisher
  if (sharpness_calibration_mode_) {
    RCLCPP_INFO(get_logger(), "Sharpness Calibration mode\n");
  }
  // create publishers for each cam
  else if (publish_cams_individually_) {
    for (const auto &cam_name_socket : name_socket_) {
      std::string node_name = get_name();

      if (compress_images_) {
        // Create compressed image publisher
        std::string compressed_topic_name =
            "/" + node_name + "/" + cam_name_socket.first + "/compressed";
        cam_compressed_pub_[cam_name_socket.first] =
            create_publisher<sensor_msgs::msg::CompressedImage>(
                compressed_topic_name, 1);
      } else {
        // Create raw image publisher using image_transport
        std::string topic_name =
            "/" + node_name + "/" + cam_name_socket.first + "/image_raw";
        cam_image_pub_[cam_name_socket.first] =
            image_transport_.advertise(topic_name, 1);
      }
    }
  } else {
    std::string node_name = get_name();

    if (compress_images_) {
      // Create compressed image publisher
      std::string compressed_topic_name = "/" + node_name + "/compressed";
      assembled_compressed_pub_ =
          create_publisher<sensor_msgs::msg::CompressedImage>(
              compressed_topic_name, 1);
    } else {
      // Create raw image publisher using image_transport
      std::string topic_name = "/" + node_name + "/image_raw";
      assembled_image_pub_ = image_transport_.advertise(topic_name, 1);
    }
  }

  if (compress_images_) {
    RCLCPP_INFO(get_logger(),
                "Publishing compressed images with JPEG quality: %d\n",
                jpeg_quality_);
  }
}

sensor_msgs::msg::CompressedImage::SharedPtr
FFC4PDriver::CompressImage(const cv::Mat &image, const std::string &encoding,
                           const std_msgs::msg::Header &header) {
  auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
  compressed_msg->header = header;

  // Set format based on encoding
  if (encoding == "bgr8" || encoding == "rgb8") {
    compressed_msg->format = "jpeg";
  } else if (encoding == "mono8") {
    compressed_msg->format = "jpeg";
  } else {
    compressed_msg->format = "jpeg";
  }

  // Compress the image
  std::vector<int> compression_params;
  compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
  compression_params.push_back(jpeg_quality_);

  std::vector<uchar> buffer;
  cv::imencode(".jpg", image, buffer, compression_params);

  compressed_msg->data = buffer;

  return compressed_msg;
}

void FFC4PDriver::StreamVideo() {

  int total_width = 0;
  int max_height = 0;
  int image_type = (rgb_) ? CV_8UC3 : CV_8UC1;
  std::string encoding = (rgb_) ? "bgr8" : "mono8";

  // === DEBUG TIMING ===
  using dbg_clk = std::chrono::steady_clock;
  struct PhaseStats {
    double sum_us = 0; double max_us = 0; int n = 0;
    void add(double v) { sum_us += v; if (v > max_us) max_us = v; ++n; }
    double avg_ms() const { return n ? (sum_us / n) / 1000.0 : 0; }
    double max_ms() const { return max_us / 1000.0; }
    void reset() { sum_us = 0; max_us = 0; n = 0; }
  };
  PhaseStats t_get, t_extract, t_assemble, t_jpeg, t_publish, t_iter, t_dev_dt;
  auto stats_window_start = dbg_clk::now();
  int dbg_frame_idx = 0;
  double last_dev_ts_s = -1;
  // === END DEBUG TIMING ===

  while (true) {
    if (image_info_) {
      std::cout << "--------------------" << '\n';
    }
    static cv_bridge::CvImage assembled_cv_img, cv_img;
    static cv::Mat assembled_cv_mat = cv::Mat::zeros(720, 5120, CV_8UC3);

    // [DBG] PHASE: wait for synced message group from OAK over USB
    auto dbg_t0 = dbg_clk::now();
    auto msg_data = output_queue_->get<dai::MessageGroup>();
    auto dbg_t1 = dbg_clk::now();
    t_get.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t1 - dbg_t0).count());

    if (msg_data == nullptr) {
      return;
    }

    auto arrival_timestamp = get_clock()->now();

    total_width = 0;
    for (const auto &cam_name_socket : name_socket_) {
      auto packet = msg_data->get<dai::ImgFrame>(cam_name_socket.first);
      if (packet) {
        // [DBG] track device-side cadence using master cam timestamp
        if (cam_name_socket.first == sync_master_) {
          double dev_ts_s = std::chrono::duration<double>(
              packet->getTimestampDevice().time_since_epoch()).count();
          if (last_dev_ts_s > 0) {
            t_dev_dt.add((dev_ts_s - last_dev_ts_s) * 1e6);
          }
          last_dev_ts_s = dev_ts_s;
        }
        // [DBG] OPTION 2: skip BGR conversion + bookkeeping; packet is MJPEG
        if (use_chip_mjpeg_) {
          continue;
        }
        // get the image
        auto cv_image = packet->getCvFrame();
        // get current time
        auto timestamp = std::chrono::steady_clock::now();

        // flip image if needed
        if (enable_upside_down_) {
          cv::flip(cv_image, cv_image, -1);
        }

        // ensure a deque exists for this camera
        auto &image_buffer = image_buffers_[cam_name_socket.first];

        // maintain max_size for the deque
        if (image_buffer.size() >= max_size_fps_) {
          image_buffer.pop_front();
        }

        // add the new image and timestamp pair
        image_buffer.emplace_back(cv_image, timestamp);

        // read the resolution of the image
        int img_width = cv_image.cols;
        int img_height = cv_image.rows;

        // update dimensions for the assembled image
        total_width += img_width;
        max_height = std::max(max_height, img_height);

        // [DBG] per-cam publish moved out of this loop and into the parallel
        // encode block below (when publish_cams_individually_ is true).

        if (image_info_) {
          double fps = CalculateFPS(image_buffer);
          std::cout << "Received " << cam_name_socket.first << ": "
                    << FormatDuration(
                           packet->getTimestampDevice().time_since_epoch())
                    << " fps: " << std::fixed << std::setprecision(2) << fps
                    << std::endl;
        }
      } else {
        RCLCPP_WARN(get_logger(), "Get %s frame failed\n",
                    cam_name_socket.first.c_str());
        return;
      }
    }

    // [DBG] end of extract phase
    auto dbg_t2 = dbg_clk::now();
    t_extract.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t2 - dbg_t1).count());

    if (sharpness_calibration_mode_) {
      for (const auto &cam_name_socket : name_socket_) {
        ShowImg(cam_name_socket.first, image_buffers_[cam_name_socket.first]);
      }
    } else if (!publish_cams_individually_ && compress_images_ && use_chip_mjpeg_) {
      // [DBG] OPTION 3: chip MJPEG -> host decode 4 in parallel -> assemble ->
      //                 reencode as single JPEG -> publish on assembled topic.
      auto dbg_t3a = dbg_clk::now();
      // Parallel decode of 4 chip-MJPEG packets
      struct DecJob { std::string name; std::future<cv::Mat> fut; };
      std::vector<DecJob> djobs;
      djobs.reserve(name_socket_.size());
      for (const auto &cam_name_socket : name_socket_) {
        auto packet = msg_data->get<dai::ImgFrame>(cam_name_socket.first);
        if (!packet) continue;
        auto raw = packet->getData();
        std::vector<uchar> bytes(raw.begin(), raw.end());
        djobs.push_back({cam_name_socket.first,
            std::async(std::launch::async, [bytes]() {
              return cv::imdecode(bytes, cv::IMREAD_COLOR);
            })});
      }
      std::map<std::string, cv::Mat> decoded;
      int local_total_w = 0, local_max_h = 0;
      for (auto &j : djobs) {
        cv::Mat m = j.fut.get();
        local_total_w += m.cols;
        local_max_h = std::max(local_max_h, m.rows);
        decoded[j.name] = std::move(m);
      }
      auto dbg_t3b = dbg_clk::now();
      // Assemble
      cv::Mat asm_mat = cv::Mat::zeros(local_max_h, local_total_w, CV_8UC3);
      int col = 0;
      for (const auto &cam_name_socket : name_socket_) {
        auto it = decoded.find(cam_name_socket.first);
        if (it == decoded.end()) continue;
        cv::Mat &m = it->second;
        m.copyTo(asm_mat(cv::Rect(col, 0, m.cols, m.rows)));
        col += m.cols;
      }
      auto dbg_t3c = dbg_clk::now();
      // Re-encode as single JPEG
      std_msgs::msg::Header header;
      header.stamp = arrival_timestamp;
      header.frame_id = "depth ai";
      auto compressed_msg = CompressImage(asm_mat, "bgr8", header);
      auto dbg_t3d = dbg_clk::now();
      assembled_compressed_pub_->publish(*compressed_msg);
      auto dbg_t3e = dbg_clk::now();
      static PhaseStats t_dec, t_asm, t_reenc, t_pub2;
      t_dec.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t3b - dbg_t3a).count());
      t_asm.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t3c - dbg_t3b).count());
      t_reenc.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t3d - dbg_t3c).count());
      t_pub2.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t3e - dbg_t3d).count());
      t_jpeg.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t3e - dbg_t3a).count());
      t_assemble.add(0);
      t_publish.add(0);
      static int op3_n = 0;
      if (++op3_n % 30 == 0) {
        RCLCPP_INFO(get_logger(),
          "  [OPT3] decode=%5.2fms  assemble=%5.2fms  reencode=%5.2fms  publish=%5.2fms",
          t_dec.avg_ms(), t_asm.avg_ms(), t_reenc.avg_ms(), t_pub2.avg_ms());
        t_dec.reset(); t_asm.reset(); t_reenc.reset(); t_pub2.reset();
      }
    } else if (publish_cams_individually_ && compress_images_ && use_chip_mjpeg_) {
      // [DBG] OPTION 2: chip already produced MJPEG bytes. Just publish them.
      auto dbg_t3 = dbg_clk::now();
      std_msgs::msg::Header header;
      header.stamp = arrival_timestamp;
      header.frame_id = "depth ai";
      for (const auto &cam_name_socket : name_socket_) {
        auto packet = msg_data->get<dai::ImgFrame>(cam_name_socket.first);
        if (!packet) continue;
        auto raw = packet->getData();  // returns dai::span by value
        auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        msg->header = header;
        msg->format = "jpeg";
        msg->data.assign(raw.begin(), raw.end());
        cam_compressed_pub_[cam_name_socket.first]->publish(*msg);
      }
      auto dbg_t4 = dbg_clk::now();
      t_jpeg.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t4 - dbg_t3).count());
      t_assemble.add(0);
      t_publish.add(0);
    } else if (publish_cams_individually_ && compress_images_) {
      // [DBG] OPTION 1: PARALLEL per-cam host JPEG encode via std::async.
      auto dbg_t3 = dbg_clk::now();
      std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};
      struct EncJob { std::string name; std::future<std::vector<uchar>> fut; };
      std::vector<EncJob> jobs;
      jobs.reserve(name_socket_.size());
      for (const auto &cam_name_socket : name_socket_) {
        cv::Mat img = image_buffers_[cam_name_socket.first].back().first;
        jobs.push_back({cam_name_socket.first,
            std::async(std::launch::async, [img, params]() {
              std::vector<uchar> buf;
              cv::imencode(".jpg", img, buf, params);
              return buf;
            })});
      }
      std_msgs::msg::Header header;
      header.stamp = arrival_timestamp;
      header.frame_id = "depth ai";
      for (auto &j : jobs) {
        auto buf = j.fut.get();
        auto msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
        msg->header = header;
        msg->format = "jpeg";
        msg->data = std::move(buf);
        cam_compressed_pub_[j.name]->publish(*msg);
      }
      auto dbg_t4 = dbg_clk::now();
      t_jpeg.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t4 - dbg_t3).count());
      t_assemble.add(0);
      t_publish.add(0);
    } else if (!publish_cams_individually_) {
      // prepare assembled image based on resolution and channel
      assembled_cv_mat = cv::Mat::zeros(max_height, total_width, image_type);
      int col_position = 0;
      for (const auto &cam_name_socket : name_socket_) {
        auto &cv_image = image_buffers_[cam_name_socket.first].back().first;
        int img_width = cv_image.cols;
        int img_height = cv_image.rows;

        // place the image in the correct position
        cv_image.copyTo(
            assembled_cv_mat(cv::Rect(col_position, 0, img_width, img_height)));
        col_position += img_width;
      }
      // [DBG] end of assemble phase
      auto dbg_t3 = dbg_clk::now();
      t_assemble.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t3 - dbg_t2).count());

      std_msgs::msg::Header header;
      header.stamp = arrival_timestamp;
      header.frame_id = "depth ai";

      if (compress_images_) {
        // Publish compressed assembled image
        auto compressed_msg = CompressImage(assembled_cv_mat, encoding, header);
        // [DBG] end of jpeg phase
        auto dbg_t4 = dbg_clk::now();
        t_jpeg.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t4 - dbg_t3).count());
        assembled_compressed_pub_->publish(*compressed_msg);
        // [DBG] end of publish phase
        auto dbg_t5 = dbg_clk::now();
        t_publish.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_t5 - dbg_t4).count());

        if (image_info_) {
          auto time_now = std::chrono::steady_clock::now();
          uint32_t latency_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  time_now - image_buffers_.begin()->second.back().second)
                  .count();
          std::cout << "latency for assembled image (compressed) in ms: "
                    << latency_us / 1000.0 << std::endl;
        }
      } else {
        // Publish raw assembled image
        assembled_cv_img.header = header;
        assembled_cv_img.encoding = encoding;
        assembled_cv_img.image = assembled_cv_mat;

        auto msg = assembled_cv_img.toImageMsg();

        if (image_info_) {
          auto time_now = std::chrono::steady_clock::now();
          uint32_t latency_us =
              std::chrono::duration_cast<std::chrono::microseconds>(
                  time_now - image_buffers_.begin()->second.back().second)
                  .count();
          std::cout << "latency for assembled image in ms: "
                    << latency_us / 1000.0 << std::endl;
        }

        assembled_image_pub_.publish(msg);
      }
    }

    // [DBG] end of full iteration
    auto dbg_iter_end = dbg_clk::now();
    t_iter.add(std::chrono::duration_cast<std::chrono::microseconds>(dbg_iter_end - dbg_t0).count());
    ++dbg_frame_idx;
    if (dbg_frame_idx >= 30) {
      double window_s = std::chrono::duration<double>(dbg_iter_end - stats_window_start).count();
      RCLCPP_INFO(get_logger(),
        "[TIMING] %d frames in %.2fs => %.2f Hz",
        dbg_frame_idx, window_s, dbg_frame_idx / window_s);
      RCLCPP_INFO(get_logger(),
        "  get_msg    avg=%6.2fms  max=%6.2fms   (USB/sync wait)",
        t_get.avg_ms(), t_get.max_ms());
      RCLCPP_INFO(get_logger(),
        "  extract    avg=%6.2fms  max=%6.2fms   (4x getCvFrame + flip + bookkeeping)",
        t_extract.avg_ms(), t_extract.max_ms());
      RCLCPP_INFO(get_logger(),
        "  assemble   avg=%6.2fms  max=%6.2fms   (zeros + 4x copyTo)",
        t_assemble.avg_ms(), t_assemble.max_ms());
      RCLCPP_INFO(get_logger(),
        "  jpeg       avg=%6.2fms  max=%6.2fms   (cv::imencode)",
        t_jpeg.avg_ms(), t_jpeg.max_ms());
      RCLCPP_INFO(get_logger(),
        "  publish    avg=%6.2fms  max=%6.2fms",
        t_publish.avg_ms(), t_publish.max_ms());
      RCLCPP_INFO(get_logger(),
        "  iter_total avg=%6.2fms  max=%6.2fms",
        t_iter.avg_ms(), t_iter.max_ms());
      RCLCPP_INFO(get_logger(),
        "  device_dt  avg=%6.2fms  max=%6.2fms   (master cam ts delta on OAK)",
        t_dev_dt.avg_ms(), t_dev_dt.max_ms());
      t_get.reset(); t_extract.reset(); t_assemble.reset();
      t_jpeg.reset(); t_publish.reset(); t_iter.reset(); t_dev_dt.reset();
      stats_window_start = dbg_iter_end;
      dbg_frame_idx = 0;
    }
  }
}

void FFC4PDriver::ShowImg(
    const std::string &cam_name,
    std::deque<std::pair<cv::Mat, SteadyTimePoint>> &image_buffer) {

  if (image_buffer.empty()) {
    return;
  }

  auto time_now = std::chrono::steady_clock::now();

  // get the last (most recent) image from the buffer
  cv::Mat &image = image_buffer.back().first;

  if (image.empty()) {
    return;
  } else {
    // compute clearness
    double clearness = Clearness(image);

    // compute FPS
    double fps = CalculateFPS(image_buffer);

    // compute latency
    uint32_t latency_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              time_now - image_buffer.back().second)
                              .count();

    // Prepare the info text to be displayed on the image
    std::stringstream info;
    info << cam_name << " clearness: " << clearness
         << "    image_delay ms: " << (latency_us / 1000) << "    FPS: " << fps;

    // Add the info text to the image
    cv::putText(image, info.str(), cv::Point(10, 30), cv::FONT_HERSHEY_PLAIN,
                1.5, cv::Scalar(0, 255, 0));

    // Display the image in an OpenCV window
    cv::imshow(cam_name, image);
    cv::waitKey(1);
  }
  return;
}

double FFC4PDriver::Clearness(cv::Mat &img) {
  // Clearness for focus
  if (img.empty()) {
    return 0.0f;
  } else {
    cv::Mat gray, imgSobel;
    cv::Rect2d roi(img.cols / 3, img.rows / 3, img.cols / 3, img.rows / 3);
    if (rgb_) {
      cv::rectangle(img, roi, cv::Scalar(255, 0, 0), 1);
      cv::cvtColor(img(roi), gray, cv::COLOR_BGR2GRAY);
    } else {
      cv::rectangle(img, roi, cv::Scalar(255), 1);
      gray = img(roi);
    }
    cv::Sobel(gray, imgSobel, CV_16U, 1, 1);
    return cv::mean(imgSobel)[0];
  }
}

double FFC4PDriver::CalculateFPS(
    const std::deque<std::pair<cv::Mat, SteadyTimePoint>> &buffer) const {
  if (buffer.size() < 2) {
    // Not enough data to compute FPS
    return 0.0;
  }
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
                      buffer.back().second - buffer.front().second)
                      .count();

  // avoid division by zero
  return (buffer.size() - 1) / (duration ? duration / 1e6 : 1.0);
}

std::string FFC4PDriver::FormatDuration(
    const std::chrono::steady_clock::duration duration) const {
  // first convert the duration to nanoseconds
  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(duration);

  // calculate hours, minutes, seconds, and milliseconds
  auto const hrs = std::chrono::duration_cast<std::chrono::hours>(ns);
  ns -= hrs;
  auto const mins = std::chrono::duration_cast<std::chrono::minutes>(ns);
  ns -= mins;
  auto const secs = std::chrono::duration_cast<std::chrono::seconds>(ns);
  ns -= secs;
  auto const millis = std::chrono::duration_cast<std::chrono::milliseconds>(ns);
  ns -= millis;
  auto const micros = std::chrono::duration_cast<std::chrono::microseconds>(ns);
  ns -= micros;

  // construct the formatted string
  std::ostringstream oss;
  oss << std::setw(2) << std::setfill('0') << hrs.count() << ":"  // hour
      << std::setw(2) << std::setfill('0') << mins.count() << ":" // minute
      << std::setw(2) << std::setfill('0') << secs.count() << "." // second
      << std::setw(3) << std::setfill('0') << millis.count() // milliseconds
      << std::setw(3) << std::setfill('0') << micros.count();

  return oss.str();
}

} // namespace oak_ffc_4p_driver
