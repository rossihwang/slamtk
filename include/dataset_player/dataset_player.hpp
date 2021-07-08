// Copyright <2021> [Copyright rossihwang@gmail.com]

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

namespace toolkit {

enum class DataType {
  COMMENT,
  PARAM,
  FLASER,
  ODOM,
  NONE
};

class DatasetPlayer : public rclcpp::Node {
 private:
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
  std::string scan_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::thread main_thread_;
  std::atomic<bool> canceled_;
  rclcpp::Time last_stamp_;
  std::string dataset_file_;
  rclcpp::Time last_ros_stamp_;
  double last_dataset_stamp_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // params
  double laser_front_laser_resolution_;
  double robot_front_laser_max_;
  
 public:
  
  DatasetPlayer(const std::string &name, rclcpp::NodeOptions const &options);
  ~DatasetPlayer();
  std::tuple<DataType, size_t> check_line_type(const std::string& line);
  std::tuple<bool, sensor_msgs::msg::LaserScan> parse_laser(const std::string& line, size_t pos);
  std::tuple<bool, nav_msgs::msg::Odometry> parse_odom(const std::string& line,  size_t pos);
  void create_parameter();
  rcl_interfaces::msg::SetParametersResult update_callback(const std::vector<rclcpp::Parameter>& parameters);
  template <typename T>
  size_t parse_as_vector(const std::string& line, size_t pos, size_t size, std::vector<T> *vec);
  void set_params();
  void parse_params(const std::string& line, size_t pos);
};

}  // namespace toolkit
