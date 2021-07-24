// Copyright <2021> [Copyright rossihwang@gmail.com]

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <slamtk/sort_queue.hpp>

namespace toolkits {

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
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_callback_;
  std::string scan_frame_;
  std::string base_frame_;
  std::string odom_frame_;
  std::thread main_thread_;
  std::atomic<bool> canceled_;
  
  double last_dataset_stamp_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  // CARMEN parameters
  double laser_front_laser_resolution_;
  double robot_front_laser_max_;

  // ROS parameters
  std::string dataset_file_;
  int playback_freq_;

  std::thread publish_thread_;
  std::mutex odom_queue_mtx_;
  std::mutex scan_queue_mtx_;
  SortQueue<nav_msgs::msg::Odometry> odom_queue_;
  SortQueue<sensor_msgs::msg::LaserScan> scan_queue_;
  
 public:
  DatasetPlayer(const std::string &name, rclcpp::NodeOptions const &options);
  ~DatasetPlayer();

 protected:
  std::tuple<DataType, size_t> check_line_type(const std::string& line);
  std::tuple<bool, sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry> parse_laser(const std::string& line, size_t pos);
  std::tuple<bool, nav_msgs::msg::Odometry> parse_odom(const std::string& line,  size_t pos);
  void create_parameter();
  // rcl_interfaces::msg::SetParametersResult set_parameter_handle(const std::vector<rclcpp::Parameter>& parameters);
  template <typename T>
  size_t parse_as_vector(const std::string& line, size_t pos, size_t size, std::vector<T> *vec);
  void set_params();
  void parse_params(const std::string& line, size_t pos);
  geometry_msgs::msg::Pose xyt_to_pose(double x, double y, double theta);
  void publish_data();
};

}  // namespace toolkits
