// Copyright <2021> [Copyright rossihwang@gmail.com]

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rcl_interfaces/msg/parameter.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>

namespace toolkits {

class OdomLogger: public rclcpp::Node {
 protected:
  std::thread main_thread_;
  std::atomic<bool> canceled_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  // Parameters
  std::string log_filename_;
  std::string base_frame_;
  std::string global_frame_;
  int log_freq_;  // Hz

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr set_param_callback_;

 public:
  OdomLogger(const std::string& name, rclcpp::NodeOptions const& options);
  ~OdomLogger();

 protected:
  void create_parameter();
  // rcl_interfaces::msg::SetParametersResult set_parameter_handle(const std::vector<rclcpp::Parameter>& parameters);
};

}  // namespace toolkits