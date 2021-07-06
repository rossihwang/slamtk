// Copyright <2021> [Copyright rossihwang]

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <dataset_player/dataset_player.hpp>

using toolkit::DatasetPlayer;

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DatasetPlayer>("dataset_player_node", rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();

  node = nullptr;
  return  0;
}