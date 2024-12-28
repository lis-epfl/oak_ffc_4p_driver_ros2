#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <opencv2/opencv.hpp>

class ImageLatencyNode : public rclcpp::Node {
public:
    ImageLatencyNode() : Node("image_latency_node") {
        // Declare and get the topic parameter
        declare_parameter<std::string>("topic", "/oak_ffc_4p_driver_node/image_raw");
        get_parameter("topic", topic_);

        // Create the image transport subscriber
        image_sub_ = image_transport::create_subscription(
            this, topic_,
            std::bind(&ImageLatencyNode::ImageCallback, this, std::placeholders::_1),
            "raw");

        RCLCPP_INFO(get_logger(), "Subscribed to topic: %s", topic_.c_str());
    }

private:
    void ImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        // Get the current time
        auto now = this->now();

        // Calculate latency
        auto timestamp = msg->header.stamp;
        auto latency = now - rclcpp::Time(timestamp);

        // Convert latency to milliseconds
        auto latency_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::nanoseconds(latency.nanoseconds())).count();

        RCLCPP_INFO(get_logger(), "ROS latency: %ld ms", latency_ms);

        // uncomment to show the transferred image
        /* cv::Mat frame = cv_bridge::toCvShare(msg, "bgr8")->image; */
        /* cv::imshow("cam_assembled", frame); */
        /* cv::waitKey(1); */
    }

    std::string topic_;
    image_transport::Subscriber image_sub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageLatencyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

