// Copyright <2021> [Copyright rossihwang@gmail.com]

#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <dataset_player/dataset_player.hpp>

namespace slamtk {

using Eigen::Quaterniond;
using Eigen::AngleAxisd;
using Eigen::Vector3d;
using namespace std::literals::chrono_literals;

DatasetPlayer::DatasetPlayer(const std::string &name, rclcpp::NodeOptions const &options) 
  : Node(name, options),
    scan_frame_("base_scan"),
    base_frame_("base_footprint"),
    odom_frame_("odom"),
    canceled_(false),
    last_dataset_stamp_(0.0),
    odom_queue_(100),
    scan_queue_(30),
    is_dataset_end_(false) {

  create_parameter();

  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  geometry_msgs::msg::TransformStamped base_scan_tf;
  base_scan_tf.header.stamp = rclcpp::Time(1000000000);
  base_scan_tf.header.frame_id = base_frame_;
  base_scan_tf.child_frame_id = scan_frame_;
  base_scan_tf.transform.translation.x = 0;
  base_scan_tf.transform.translation.y = 0;
  base_scan_tf.transform.translation.z = 0;
  base_scan_tf.transform.rotation.x = 0;
  base_scan_tf.transform.rotation.y = 0;
  base_scan_tf.transform.rotation.z = 0;
  base_scan_tf.transform.rotation.w = 1;
  static_broadcaster_->sendTransform(base_scan_tf);

  // set_params();

  publish_thread_ = std::thread(&DatasetPlayer::publish_data, this);

  main_thread_ = std::thread{[this]() -> void  {
    std::fstream dataset;
    dataset.open(dataset_file_, std::ios::in);
    if (!dataset.is_open()) {
      throw std::runtime_error("Failed to open " + dataset_file_);
    }
    std::string line;

    while (rclcpp::ok() && !canceled_.load()) {
      if (odom_queue_.full() || scan_queue_.full()) {
        RCLCPP_WARN(get_logger(), "wait space on queue");
      } else {

        getline(dataset, line);
        if (!dataset) {
          is_dataset_end_ = true;
          RCLCPP_WARN(get_logger(), "Reaching the end of file");
          break;
        }

        DataType type;
        size_t pos;
        std::tie(type, pos) = check_line_type(line);
        switch (type) {
          case DataType::ODOM:
            {
              bool state;
              nav_msgs::msg::Odometry odom;
              std::tie(state, odom) = parse_odom(line, pos);
              if (!state) {
                break;
              }
              {
                std::lock_guard<std::mutex> lk(odom_queue_mtx_);
                odom_queue_.push(odom, rclcpp::Time(odom.header.stamp).nanoseconds()/1e9);
              }
              break;
            }
          case DataType::FLASER:
            {
              bool state;
              sensor_msgs::msg::LaserScan scan;
              nav_msgs::msg::Odometry odom;
              std::tie(state, scan, odom)  = parse_laser(line, pos);
              if (!state) {
                break;
              }
              {
                std::lock_guard<std::mutex> lk(scan_queue_mtx_);
                scan_queue_.push(scan, rclcpp::Time(scan.header.stamp).nanoseconds()/1e9);
              }
              {
                std::lock_guard<std::mutex> lk(odom_queue_mtx_);
                odom_queue_.push(odom, rclcpp::Time(odom.header.stamp).nanoseconds()/1e9);
              }
              break;
            }
          case DataType::PARAM:
            // parse_params(line, pos);
            break;
          case DataType::COMMENT:
          default:
            std::cout << "unknown" << std::endl;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    dataset.close();
    return;
  }};
}

DatasetPlayer::~DatasetPlayer() {
  canceled_.store(true);
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

std::tuple<DataType, size_t> DatasetPlayer::check_line_type(const std::string& line) {
  size_t pos = 0;
  pos = line.find(" ");
  if (pos != std::string::npos) {
    auto type = line.substr(0, pos - 0);
    if (type == "#") {
      return std::make_tuple(DataType::COMMENT, pos);
    } else if (type == "PARAM") {
      return std::make_tuple(DataType::PARAM, pos);
    } else if (type == "ODOM") {
      return std::make_tuple(DataType::ODOM, pos);
    } else if (type == "FLASER") {
      return std::make_tuple(DataType::FLASER, pos);
    }
  }
  return std::make_tuple(DataType::NONE, 0);
}

std::tuple<bool, sensor_msgs::msg::LaserScan, nav_msgs::msg::Odometry> DatasetPlayer::parse_laser(const std::string& line, size_t pos) {
  // RCLCPP_INFO(get_logger(), "%s", line.c_str());

  size_t end = line.find(" ", pos + 1);
  int num_readings = std::stoi(line.substr(pos, end - pos));

  std::vector<float> ranges;
  end = parse_as_vector<float>(line, end, num_readings, &ranges);

  sensor_msgs::msg::LaserScan scan;
  nav_msgs::msg::Odometry odom;

  std::vector<double> vec;
  parse_as_vector<double>(line, end, 7, &vec);
  // for (auto v: vec) {
  //   std::cout << v << ", ";
  // }
  // std::cout << std::endl;
  
  if (vec[6] < last_dataset_stamp_) {
    // FIXME: so far we don't handle this situation
    RCLCPP_WARN(get_logger(), "timestamp in wrong order, last: %f, new: %f", last_dataset_stamp_, vec[6]);
    scan.header.stamp = rclcpp::Time(vec[6] * 1000000000);
  } else {
    scan.header.stamp = rclcpp::Time(vec[6] * 1000000000);
  }
  last_dataset_stamp_ = vec[6];
  
  scan.header.frame_id = scan_frame_;
  scan.angle_increment = 0.017453292519943295 * laser_front_laser_resolution_;
  scan.angle_min = -M_PI / 2;
  scan.angle_max = scan.angle_min + ((num_readings - 1) * 0.017453292519943295 * laser_front_laser_resolution_);
  
  scan.scan_time = 0.001;
  scan.range_min = 0.0;
  scan.range_max = robot_front_laser_max_;
  scan.ranges = ranges;
  // scan.intensities = ;

  odom.header.frame_id = odom_frame_;
  odom.header.stamp = scan.header.stamp;
  odom.child_frame_id = base_frame_;
  odom.pose.pose = xyt_to_pose(vec[3], vec[4], vec[5]);
  
  return std::make_tuple(true, scan, odom);
}

std::tuple<bool, nav_msgs::msg::Odometry> DatasetPlayer::parse_odom(const std::string& line, size_t pos) {
  // RCLCPP_INFO(get_logger(), "%s", line.c_str());
  
  std::vector<double> vec;
  parse_as_vector<double>(line, pos, 7, &vec);
  
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = odom_frame_;

  if (vec[6] < last_dataset_stamp_) {
    // FIXME: so far we don't handle this situation
    RCLCPP_WARN(get_logger(), "timestamp in wrong order, last: %f, new: %f", last_dataset_stamp_, vec[6]);
    odom.header.stamp = rclcpp::Time(vec[6] * 1000000000);
  } else {
    odom.header.stamp = rclcpp::Time(vec[6] * 1000000000);
  }
  last_dataset_stamp_ = vec[6];

  odom.child_frame_id = base_frame_;

  odom.pose.pose = xyt_to_pose(vec[0], vec[1], vec[2]);
  // odom.pose.covariance = 
  odom.twist.twist.linear.x = vec[3];
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = vec[4]; 
  // odom.twist.covariance =
  // RCLCPP_INFO(get_logger(), "position(%f, %f, %f), orientation(%f, %f, %f, %f)", 
  //       odom.pose.pose.position.x,
  //       odom.pose.pose.position.y,
  //       odom.pose.pose.position.z,
  //       odom.pose.pose.orientation.w,
  //       odom.pose.pose.orientation.x,
  //       odom.pose.pose.orientation.y,
  //       odom.pose.pose.orientation.z);
  
  return std::make_tuple(true, odom);
}

void DatasetPlayer::create_parameter() {
  dataset_file_ = declare_parameter<std::string>("dataset", "");
  playback_freq_ = declare_parameter<int>("playback_freq", 25);  // NOTE: recommand for 25, faster speed will lead to no buffered data for reordering


  auto get_filename = [](const std::string& s) -> std::string {
    char sep = '/';
    size_t i = s.rfind(sep, s.length());
    if (i != std::string::npos) {
      return(s.substr(i+1, s.length() - i));
    }
    return("");
  };

  // Some presets for known datasets
  std::string base = get_filename(dataset_file_);
  if (base == "mit-killian.clf") {
    laser_front_laser_resolution_ = 1.0;
    robot_front_laser_max_ = 51.060;
  } else if (base == "aces.clf") {
    laser_front_laser_resolution_ = 1.0;
    robot_front_laser_max_ = 50.0;
  } else if (base == "fr079.clf") {
    laser_front_laser_resolution_ = 1.0;
    robot_front_laser_max_ = 50.0;
  } else if (base == "intel.clf") {
    laser_front_laser_resolution_ = 1.0;
    robot_front_laser_max_ = 81.83;
  } else if (base == "mit-csail.clf") {
    laser_front_laser_resolution_ = 0.5;
    robot_front_laser_max_ = 81.91;
  } else {
    RCLCPP_ERROR(get_logger(), "Unknown dataset");
    exit(1);
  }
  // set_param_callback_ = add_on_set_parameters_callback(std::bind(&DatasetPlayer::set_parameter_handle, this, std::placeholders::_1));
}
  
// rcl_interfaces::msg::SetParametersResult DatasetPlayer::set_parameter_handle(const std::vector<rclcpp::Parameter>& parameters) {
//   rcl_interfaces::msg::SetParametersResult result;
//   result.successful = true;

//   // for (const rclcpp::Parameter &param:  parameters) {
//   // }
//   return result;
// }

template <typename T>
size_t DatasetPlayer::parse_as_vector(const std::string& line, size_t pos, size_t size, std::vector<T> *vec) {
  size_t prev = pos;
  size_t curr;
  size_t i = 0;
  
  vec->resize(size);  // improve speed
  while (1) {  
    curr = line.find(" ", prev + 1);
    // RCLCPP_INFO(get_logger(), "prev: %d, curr: %d", prev, curr);
    if (curr != std::string::npos) {
      // RCLCPP_INFO(get_logger(), "1: %s", line.substr(prev, curr - prev).c_str());
      (*vec)[i] = static_cast<T>(std::stod(line.substr(prev, curr - prev)));  // FIXME
    } else {  // last data of the line
      // RCLCPP_INFO(get_logger(), "2: %s", line.substr(prev, line.size() - prev).c_str());
      (*vec)[i] = static_cast<T>(std::stod(line.substr(prev, line.size() - prev)));
      break;
    }
    if (size <= ++i) {
      break;
    }
    prev = curr;
  }
  if (i < size) {
    RCLCPP_WARN(get_logger(), "data size is %d, expected %d", i, size);
  }
  return curr;
}

void DatasetPlayer::set_params() {
  laser_front_laser_resolution_ = 0.5;
  robot_front_laser_max_ = 50.0;
}

void DatasetPlayer::parse_params(const std::string& line, size_t pos) {
  size_t key_end = line.find(" ", pos + 1);
  std::string key = line.substr(pos + 1, key_end - pos - 1);  // FIXME: +1/-1 to strip the space

  if (key == "laser_front_laser_resolution") {
    size_t value_end = line.find(" ", key_end + 1);
    double value = std::stod(line.substr(key_end + 1,  value_end - key_end));
    laser_front_laser_resolution_ = value;
    RCLCPP_INFO(get_logger(), "%s, %f", key.c_str(), value);
  } else if (key == "robot_front_laser_max") {
    size_t value_end = line.find(" ", key_end + 1);
    double value = std::stod(line.substr(key_end + 1,  value_end - key_end));
    robot_front_laser_max_ = value;
    RCLCPP_INFO(get_logger(), "%s, %f", key.c_str(), value);
  }
}

geometry_msgs::msg::Pose DatasetPlayer::xyt_to_pose(double x, double y, double theta) {
  geometry_msgs::msg::Pose pose;

  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = 0;
  Quaterniond q(AngleAxisd(theta, Vector3d::UnitZ()));
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  return pose;
}

void DatasetPlayer::publish_data() {

  rclcpp::WallRate rate(playback_freq_);
  bool send_transform, send_scan;
  StampedData<nav_msgs::msg::Odometry> stamped_odom_data;
  StampedData<sensor_msgs::msg::LaserScan> stamped_scan_data;
  std::unique_lock<std::mutex> odom_lk(odom_queue_mtx_, std::defer_lock);
  std::unique_lock<std::mutex> scan_lk(scan_queue_mtx_, std::defer_lock);

  while (rclcpp::ok() && !canceled_.load()) {
    // RCLCPP_INFO(get_logger(), "%d %d", odom_queue_.size(), scan_queue_.size());
    send_transform = false;
    send_scan = false;

    odom_lk.lock();
    scan_lk.lock();
    if (0 < odom_queue_.size() && 0 < scan_queue_.size()) {
      if (odom_queue_.top().stamp <= scan_queue_.top().stamp) {
        stamped_odom_data = odom_queue_.top();
        odom_queue_.pop();
        send_transform = true;
      } else {
        stamped_scan_data = scan_queue_.top();
        scan_queue_.pop();
        send_scan = true;
      }
    } else if (0 < odom_queue_.size()) {
      stamped_odom_data = odom_queue_.top();
      odom_queue_.pop();
      send_transform = true;
    } else if (0 < scan_queue_.size()) {
      stamped_scan_data = scan_queue_.top();
      scan_queue_.pop();
      send_scan = true;
    } else if (is_dataset_end_) {
      RCLCPP_WARN(get_logger(), "Reaching the end of data queue");
      break;
    }
    scan_lk.unlock();
    odom_lk.unlock();

    if (send_transform) {
      RCLCPP_INFO(get_logger(), "send transform %f", stamped_odom_data.stamp);

      rosgraph_msgs::msg::Clock clock;
      clock.clock = stamped_odom_data.data.header.stamp;
      clock_pub_->publish(clock);

      // odom_pub_->publish(odom_data);
      geometry_msgs::msg::TransformStamped tf_stamped;
      tf_stamped.header = stamped_odom_data.data.header;
      tf_stamped.child_frame_id = stamped_odom_data.data.child_frame_id;
      tf_stamped.transform.translation.x = stamped_odom_data.data.pose.pose.position.x;
      tf_stamped.transform.translation.y = stamped_odom_data.data.pose.pose.position.y;
      tf_stamped.transform.translation.z = stamped_odom_data.data.pose.pose.position.z;
      tf_stamped.transform.rotation = stamped_odom_data.data.pose.pose.orientation;
      tf_broadcaster_->sendTransform(tf_stamped);
      
    } else if (send_scan) {
      RCLCPP_INFO(get_logger(), "publish scan %f", stamped_scan_data.stamp);
      
      rosgraph_msgs::msg::Clock clock;
      clock.clock = stamped_scan_data.data.header.stamp;
      clock_pub_->publish(clock);

      scan_pub_->publish(stamped_scan_data.data);
      
    }
    rate.sleep();
  }
}

}  // namespace slamtk