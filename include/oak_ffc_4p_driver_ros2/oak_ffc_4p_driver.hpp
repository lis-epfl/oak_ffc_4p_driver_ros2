#ifndef OAK_FFC_4P_DRIVER_CLASS_H_
#define OAK_FFC_4P_DRIVER_CLASS_H_

#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <depthai/build/version.hpp>
#include <depthai/depthai.hpp>
#include <deque>
#include <image_transport/image_transport.hpp>
#include <iostream>
#include <map>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sstream>
#include <thread>

namespace oak_ffc_4p_driver {

class FFC4PDriver : public rclcpp::Node {
public:
  // constructor
  FFC4PDriver();

  // destructor
  ~FFC4PDriver();

private:
  // using for easier readability
  using SteadyTimePoint =
      std::chrono::time_point<std::chrono::steady_clock,
                              std::chrono::steady_clock::duration>;
  // configurations
  // mono cam resolution
  std::map<std::string, dai::MonoCameraProperties::SensorResolution>
      mono_res_opts_ = {
          {"400", dai::MonoCameraProperties::SensorResolution::THE_400_P},
          {"480", dai::MonoCameraProperties::SensorResolution::THE_480_P},
          {"720", dai::MonoCameraProperties::SensorResolution::THE_720_P},
          {"800", dai::MonoCameraProperties::SensorResolution::THE_800_P},
          {"1200", dai::MonoCameraProperties::SensorResolution::THE_1200_P},
      };

  // rgb cam resolution
  std::map<std::string, dai::ColorCameraProperties::SensorResolution>
      color_res_opts_ = {
          {"720", dai::ColorCameraProperties::SensorResolution::THE_720_P},
          {"800", dai::ColorCameraProperties::SensorResolution::THE_800_P},
          {"1080", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
          {"1200", dai::ColorCameraProperties::SensorResolution::THE_1200_P},
          {"4k", dai::ColorCameraProperties::SensorResolution::THE_4_K},
          {"5mp", dai::ColorCameraProperties::SensorResolution::THE_5_MP},
          {"12mp", dai::ColorCameraProperties::SensorResolution::THE_12_MP},
          {"48mp", dai::ColorCameraProperties::SensorResolution::THE_48_MP},
      };

  std::map<std::string, dai::CameraBoardSocket> name_socket_ = {
      {"CAM_A", dai::CameraBoardSocket::CAM_A},
      {"CAM_B", dai::CameraBoardSocket::CAM_B},
      {"CAM_C", dai::CameraBoardSocket::CAM_C},
      {"CAM_D", dai::CameraBoardSocket::CAM_D},
  };

  /*-------------- methods ---------------*/
  // declare ros parameters
  void DeclareRosParameters();

  // initialize ros parameters
  void InitializeRosParameters();

  //  create the device
  bool CreateDevice();

  // initialize device/pipeline
  void InitializePipeline();

  // initialize device/pipeline
  void StreamVideo();

  // show image using opencv for calibration mode
  void ShowImg(const std::string &,
               std::deque<std::pair<cv::Mat, SteadyTimePoint>> &);

  // compute the clearness/sharpness in an image
  double Clearness(cv::Mat &);

  // compute the fps
  double
  CalculateFPS(const std::deque<std::pair<cv::Mat, SteadyTimePoint>> &) const;

  // format the duration for printing out
  std::string FormatDuration(const std::chrono::steady_clock::duration) const;

  // compress image to JPEG format
  sensor_msgs::msg::CompressedImage::SharedPtr
  CompressImage(const cv::Mat& image, const std::string& encoding,
                const std_msgs::msg::Header& header);

  /*-------------- member variables ---------------*/
  /** cameras configuration **/
  // which cam's clock acts as the master for syncing the 4 fisheye cameras
  std::string sync_master_;
  // resolution
  std::string resolution_;
  // frame per second
  int fps_;
  // whether to use color or mono
  bool rgb_;
  // auto exposure time
  bool auto_exposure_time_;
  // exposure time in microseconds
  int exposure_time_us_;
  // iso to control sensor sensitivity
  int iso_;
  // show image info or not
  bool image_info_;
  // whether to use auto white balance
  bool auto_awb_;
  // auto white balance value in case we are not using auto white balance
  int awb_value_;
  // enter sharpness calibration mode to calibrate focus and sharpness of the
  // image
  bool sharpness_calibration_mode_;
  // flip the images upside down of all cameras
  bool enable_upside_down_;
  // publish cams individually in addition to the assembled image
  bool publish_cams_individually_;
  // whether to publish compressed images
  bool compress_images_;
  // JPEG compression quality (0-100)
  int jpeg_quality_;

  /** device variables **/
  // pointer to the device
  std::unique_ptr<dai::Device> device_ = nullptr;
  // pointer to the pipeline
  std::unique_ptr<dai::Pipeline> pipeline_ = nullptr;

  // publisher for the image
  rclcpp::Node::SharedPtr node_handle_;
  image_transport::ImageTransport image_transport_;

  // Publishers for raw images (using image_transport)
  image_transport::Publisher assembled_image_pub_;
  std::map<std::string, image_transport::Publisher> cam_image_pub_;

  // Publishers for compressed images (direct publishers)
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr assembled_compressed_pub_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> cam_compressed_pub_;

  // thread variable to run the streaming
  std::thread streaming_thread_;

  // a map to store a deque of pairs for each camera
  std::map<std::string, std::deque<std::pair<cv::Mat, SteadyTimePoint>>>
      image_buffers_;
  // max size of deque to compute the fps
  int max_size_fps_;
};

} // namespace oak_ffc_4p_driver

#endif
