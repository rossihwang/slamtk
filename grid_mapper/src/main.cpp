// Copyright <2021> [Copyright rossihwang]

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <grid_mapper/grid_mapper.hpp>

using slamtk::GridMapper;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GridMapper>("grid_mapper_node", rclcpp::NodeOptions());
  node->configure();
  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;
  return 0;
}
