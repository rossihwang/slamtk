// Copyright <2021> [Copyright rossihwang]

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <odom_logger/odom_logger.hpp>

using slamtk::OdomLogger;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<OdomLogger>("odom_logger_node", rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;
  return  0;
}