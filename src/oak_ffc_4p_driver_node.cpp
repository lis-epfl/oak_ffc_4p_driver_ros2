#include "oak_ffc_4p_driver.hpp"

int main(int argc, char *argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<oak_ffc_4p_driver::FFC4PDriver>());
  rclcpp::shutdown();
}
