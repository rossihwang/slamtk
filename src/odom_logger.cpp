// Copyright <2021> [Copyright rossihwang@gmail.com]

#include <fstream>
#include <slamtk/odom_logger.hpp>
#include <boost/format.hpp>
#include <Eigen/Geometry>

#include <rcl_interfaces/msg/parameter.hpp>

namespace toolkits {

using namespace Eigen;
using std::placeholders::_1;

OdomLogger::OdomLogger(const std::string& name, rclcpp::NodeOptions const& options) 
  : Node(name, options),
    canceled_(false) {
  
  create_parameter();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    this->get_node_base_interface(),
    this->get_node_timers_interface());
  tf_buffer_->setCreateTimerInterface(timer_interface);
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  main_thread_ = std::thread{[this]() -> void {
    rclcpp::WallRate rate(log_freq_);
    std::fstream log_file;

    log_file.open(log_filename_, std::ios::out);
    if (!log_file.is_open()) {
      throw std::runtime_error("Failed to open log file");
    }
    
    while (rclcpp::ok() && !canceled_.load()) {
      geometry_msgs::msg::PoseStamped ident;
      geometry_msgs::msg::PoseStamped odom_pose;
      rclcpp::Time stamp = now();
      ident.header.frame_id = base_frame_;
      ident.header.stamp = stamp;
      tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);

      try {
        tf_buffer_->transform(ident, odom_pose, global_frame_, tf2::durationFromSec(0.01));
      } catch (tf2::TransformException &e) {
        RCLCPP_ERROR(get_logger(), "%s", e.what());
        continue;
      }

      // format: ODOM x y theta tv rv accel ipc_timestamp ipc_hostname logger_timestamp
      std::stringstream ss;
      // ss << boost::format("ODOM %.6f %.6f %.6f 0.000000 0.000000 0.000000 %.6f nohost 0.000000") 
      //   % odom_pose.pose.position.x
      //   % odom_pose.pose.position.y
      //   % 0.0
      //   % (stamp.nanoseconds()/1000000000.0) << std::endl;
      Quaterniond q(odom_pose.pose.orientation.w,
                    odom_pose.pose.orientation.x,
                    odom_pose.pose.orientation.y,
                    odom_pose.pose.orientation.z);
      auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
      ss << boost::format("FLASER 0 0.000000 0.000000 0.000000 %.6f %.6f %.6f %.6f nohost 0.000000") 
        % odom_pose.pose.position.x
        % odom_pose.pose.position.y
        % euler(2)
        % (stamp.nanoseconds()/1000000000.0);
      log_file << ss.str() << std::endl;
      RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());
      
      rate.sleep();
    }
  }};

}

OdomLogger::~OdomLogger() {
  canceled_.store(true);
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
}

void OdomLogger::create_parameter() {
  log_filename_ = declare_parameter<std::string>("log_filename", "estimated_odom.txt");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_footprint");
  global_frame_ = declare_parameter<std::string>("global_frame", "map");
  log_freq_ = declare_parameter<int>("log_freq", 10);

  // set_param_callback_ = add_on_set_parameters_callback(std::bind(&OdomLogger::set_parameter_handle, this, _1));
}

// rcl_interfaces::msg::SetParametersResult OdomLogger::set_parameter_handle(const std::vector<rclcpp::Parameter>& parameters) {
//   rcl_interfaces::msg::SetParametersResult result;
//   result.successful = true;

//   // for (const rclcpp::Parameter &param: parameters) {
//   //   if (param.get_name() == "log_filename") {
//   //     log_filename_ = param.as_string();
//   //   } else if (param.get_name() == "base_frame") {
//   //     base_frame_ = param.as_string();
//   //   } else if (param.get_name() == "global_frame") {
//   //     global_frame_ = param.as_string();
//   //   }
//   // }
//   return result;
// }

}  // namespace toolkits