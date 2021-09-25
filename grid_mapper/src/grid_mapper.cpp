// Copyright <2021> [Copyright rossihwang]

#include <grid_mapper/grid_mapper.hpp>
#include <grid_mapper/visualization_utils.hpp>
#include <Eigen/Geometry>
#include <opencv2/highgui.hpp>

using namespace Eigen;
using namespace slamtk;

GridMapper::GridMapper(const std::string& name, rclcpp::NodeOptions const& options)
  : Node(name, options),
    odom_frame_("odom"),
    map_frame_("map"),
    base_frame_("base_link"),
    scan_frame_("base_scan"),
    scan_topic_("/scan"),
    map_topic_("/map"),
    resolution_(0.05),
    canceled_(false) {
  main_thread_ = std::thread(std::bind(&GridMapper::publish_map_thread, this));
}

GridMapper::~GridMapper() {
  canceled_.store(true);
  if (main_thread_.joinable()) {
    main_thread_.join();
  }
}

void GridMapper::configure() {
  set_ros_interfaces();
}

void GridMapper::set_ros_interfaces() {
  tf_ = std::make_unique<tf2_ros::Buffer>(get_clock(), tf2::durationFromSec(30));
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(get_node_base_interface(), get_node_timers_interface());
  tf_->setCreateTimerInterface(timer_interface);
  tfL_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
  tfB_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

  scan_filter_sub_ = std::make_unique<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(
    shared_from_this().get(), 
    scan_topic_,
    rmw_qos_profile_sensor_data);
  scan_filter_ = std::make_unique<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *scan_filter_sub_,
    *tf_,
    odom_frame_,
    1,
    shared_from_this());
  scan_filter_->registerCallback(std::bind(&GridMapper::scan_callback, this, std::placeholders::_1));

  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  map_meta_pub_ = create_publisher<nav_msgs::msg::MapMetaData>(map_topic_ + "_metadata", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

}

bool GridMapper::add_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, const cv::Vec3f& base_pose, std::weak_ptr<LaserMetaData> laser_meta_data) {

  RCLCPP_INFO(get_logger(), "add_scan");
  // 1. construct the ScanWithPose
  //    - Convert scan points to Euclidean w.r.t base frame
  //    - Associate to LaserMetaData
  //    - Compute bounding box
  size_t points_amount = (scan->angle_max - scan->angle_min) / scan->angle_increment;
  ScanWithPose data(base_pose, laser_meta_data, points_amount);
  cv::Vec3f laser_pose = laser_meta_data.lock()->laser_pose;

  for (size_t i = 0; i < points_amount; ++i) {
    float angle = laser_pose[2] + i * scan->angle_increment + scan->angle_min;
    float range = scan->ranges[i];
    float max_range = laser_meta_data.lock()->range_max;
    // clip the inf range to max range
    if (std::isinf(range)) {
      range = max_range;
    }
    bool is_hit = false;
    if (range < max_range) {
      is_hit = true;
    }

    float x = laser_pose[0] + range * std::cos(angle);  // FIXME: no relative rotation
    float y = laser_pose[1] + range * std::sin(angle);
    data.scan_points[i] = std::make_pair(cv::Point2f(x, y), is_hit);  // scan points w.r.t base frame

    // piggybacking with computing bounding box
    data.bbox.add_point(cv::Point2f(x, y));
  }

  data.bbox.show();  // DEBUG

  // 2. put scan in buffer
  scans_.push_back(data);

  return true;
}

bool GridMapper::add_laser(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, const cv::Vec3f& laser_pose) {
  frame_id_meta_lut_.insert(std::make_pair(scan->header.frame_id, std::make_shared<LaserMetaData>(scan, laser_pose)));
  return true;
}

bool GridMapper::clean_scans() {
  scans_.clear();
  return true;
}

bool GridMapper::build_map() {
  // 1. Estimate the bounding box size
  cv::Vec2f map_size;
  cv::Point2f map_offset;
  std::tie(map_size, map_offset) = occupancy_map_ptr_->compute_map_dimension(scans_);

  // 2. Build the occupancy grid map
  occupancy_map_ptr_ = std::make_unique<OccupancyGridMap>(map_size, map_offset, resolution_);
  occupancy_map_ptr_->create_from_scans(scans_);

  return true;
}

bool GridMapper::should_build_map() {
  RCLCPP_INFO(get_logger(), "scans size: %d", scans_.size());
  if (scans_.size() == 0) {
    return false;
  }
  return true;
}

void GridMapper::publish_odom_map_transform() {
}

void GridMapper::publish_map_thread() {
  
  rclcpp::Rate rate(std::chrono::seconds(5));
  while (rclcpp::ok() && !canceled_.load()) {
    
    if (should_build_map()) {
      build_map();

      // packing the data
      nav_msgs::msg::OccupancyGrid og_msg;

      to_nav_map(*occupancy_map_ptr_, &og_msg);
      // test
      // UNKNOWN = 0,
      // OCCUPIED = 100,
      // FREE = 255
      const cv::Mat in = occupancy_map_ptr_->get_map();
      cv::Mat out(in.size(), in.type());
      for (int i = 0; i < in.rows; ++i) {
        for (int j = 0; j < in.cols; ++j) {
          switch (in.at<uint8_t>(i, j)) {
            case 0:
              out.at<uint8_t>(in.rows-1-i, j) = 205;
              break;
            case 100:
              out.at<uint8_t>(in.rows-1-i, j) = 0;
              break;
            case 255:
              out.at<uint8_t>(in.rows-1-i, j) = 254;
              break;
          }
        }
      }
      cv::imwrite("local_map.pgm", out);
      og_msg.header.frame_id = map_frame_;
      og_msg.header.stamp = now();

      map_pub_->publish(std::move(std::make_unique<nav_msgs::msg::OccupancyGrid>(og_msg)));
      map_meta_pub_->publish(std::move(std::make_unique<nav_msgs::msg::MapMetaData>(og_msg.info)));
    }
    rate.sleep();
  }
}

void GridMapper::scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
  RCLCPP_INFO(get_logger(), "scan_callback");
  // 1. whether the laser is registered
  std::string frame_id = scan->header.frame_id;
  if (frame_id_meta_lut_.count(frame_id) == 0) {
    RCLCPP_INFO(get_logger(), "Registering new laser(%s)", frame_id.c_str());
    // frame_id_meta_lut_[frame_id] = LaserMetaData(scan);  // NOTE: this will call default constructor LaserMetaData()
    bool ret;
    cv::Vec3f scan_pose;
    std::tie(ret, scan_pose) = get_frame_pose(scan_frame_, base_frame_, scan->header.stamp);
    if (!ret) {
      RCLCPP_WARN(get_logger(), "get_frame_pose(%s, %s)", base_frame_.c_str(), map_frame_.c_str());
      return;
    }
    add_laser(scan, scan_pose);
    frame_id_meta_lut_[frame_id]->show();  // DEBUG
  }

  // 2. whether move enougth
  bool ret;
  cv::Vec3f base_pose;
  std::tie(ret, base_pose) = get_frame_pose(base_frame_, odom_frame_, scan->header.stamp);
  if (!ret) {
    RCLCPP_WARN(get_logger(), "get_frame_pose(%s, %s)", base_frame_.c_str(), odom_frame_.c_str());
    return;
  }

  if (!should_add_scan(base_pose)) {
    RCLCPP_WARN(get_logger(), "Not enogth move distance or angle");
    return;
  }
  // TODO: update last pose

  // 3. look up the laser meta data
  std::weak_ptr<LaserMetaData> laser_meta_data = frame_id_meta_lut_[frame_id];
  
  // 4. add the scan data to buffer
  add_scan(scan, base_pose, laser_meta_data);

}

std::tuple<bool, cv::Vec3f> GridMapper::get_frame_pose(const std::string& from_frame, const std::string& to_frame, const rclcpp::Time& stamp) {
  geometry_msgs::msg::PoseStamped ident;
  geometry_msgs::msg::PoseStamped global_pose;

  ident.header.frame_id = from_frame;
  ident.header.stamp = stamp;
  tf2::toMsg(tf2::Transform::getIdentity(), ident.pose);  // get global pose of the camera
  try {
    tf_->transform(ident, global_pose, to_frame);  // tf2::durationFromSec(0.05)
  } catch (tf2::TransformException& e) {
    RCLCPP_ERROR(get_logger(), "%s", e.what());
    return std::make_tuple(false, cv::Vec3f(0, 0, 0));
  }
  Quaternionf q(global_pose.pose.orientation.w, 0, 0, global_pose.pose.orientation.z);
  q.normalize();
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  return std::make_tuple(true, cv::Vec3f(global_pose.pose.position.x, global_pose.pose.position.y, euler(2)));
}

bool GridMapper::should_add_scan(const cv::Vec3f& current_pose) {
  return true;
}

// OccupancyGridMap
OccupancyGridMap::OccupancyGridMap(const cv::Vec2f& map_size, const cv::Point2f& map_offset, float resolution)
  : map_size_(map_size),
    map_offset_(map_offset),
    resolution_(resolution) {
  
  image_size_ = cv::Size(std::round(map_size_[0] / resolution_), std::round(map_size_[1] / resolution_));
  printf("map_size: (%f, %f), image_size: (%d, %d), resolution: %f\n", map_size_[0], map_size_[1], image_size_.width, image_size_.height, resolution_);
  map_ = cv::Mat(image_size_, CV_8UC1);
}

OccupancyGridMap::~OccupancyGridMap() {

}

std::tuple<cv::Vec2f, cv::Point2f> OccupancyGridMap::compute_map_dimension(const std::vector<ScanWithPose>& scans) {
  BoundingBox bbox;
  for (auto s: scans) {
    BoundingBox bbox_on_base;
    bbox_on_base = s.bbox + s.get_base_pose();
    bbox |= bbox_on_base;
  }
  // bbox.add_point(cv::Point2f(0, 0));  // Add the origin
  return std::make_tuple(cv::Vec2f(bbox.get_width(), bbox.get_height()), bbox.get_minimum());
}

void OccupancyGridMap::create_from_scans(const std::vector<ScanWithPose>& scans) {

  cv::Mat hits(image_size_, CV_16UC1);
  cv::Mat misses(image_size_, CV_16UC1);
  hits = 0;
  misses = 0;
  
  // 1. traverse the scans to construct the hit and miss maps
  for (auto s: scans) {
    for (auto p: s.scan_points) {
      // compute the start point(laser position)
      cv::Point2f start, end;
      cv::Vec3f laser_rel_base = s.get_meta_data().lock()->laser_pose;
      cv::Vec3f base_pose = s.get_base_pose();

      start.x = laser_rel_base[0] * std::cos(base_pose[2]) - laser_rel_base[1] * std::sin(base_pose[2]) + base_pose[0];
      start.y = laser_rel_base[0] * std::sin(base_pose[2]) + laser_rel_base[1] * std::cos(base_pose[2]) + base_pose[1];

      end.x = p.first.x * std::cos(base_pose[2]) - p.first.y * std::sin(base_pose[2]) + base_pose[0];
      end.y = p.first.x * std::sin(base_pose[2]) + p.first.y * std::cos(base_pose[2]) + base_pose[1];
      // ray trace
      cv::LineIterator it(misses, covert_point_m2i(start), covert_point_m2i(end));
      for (int i = 0; i < it.count; ++i, ++it) {
        misses.at<uint16_t>(it.pos()) += 1;
      }
      if (p.second) {  // is_hit
        hits.at<uint16_t>(covert_point_m2i(end)) += 1;
        misses.at<uint16_t>(covert_point_m2i(end)) -= 1;
      }
    }
  }
  
  cv::Mat misses_f, hits_f;
  misses.convertTo(misses_f, CV_32FC1);
  hits.convertTo(hits_f, CV_32FC1);

  // 2. compute the map from hits and misses by the counting model
  cv::Mat ratio;
  cv::Mat unknowns;
  cv::divide(hits_f, hits_f + misses_f, ratio);
  unknowns = hits | misses;
  
  int r, c;
  const float kOccupiedThresh = 0.3;
  if (unknowns.isContinuous() && map_.isContinuous() && ratio.isContinuous()) {
    c = unknowns.cols * unknowns.rows;
    r = 1;
  } else {
    c = unknowns.cols;
    r = unknowns.rows;
  }
  for (int i = 0; i < r; ++i) {
    auto map_ptr = map_.ptr<uint8_t>(i);
    auto unknowns_ptr = unknowns.ptr<uint16_t>(i);
    auto ratio_ptr = ratio.ptr<float>(i);
    for (int j = 0; j < c; ++j) {
      if (unknowns_ptr[j] == 0) {
        map_ptr[j] = static_cast<uint8_t>(GridStates::UNKNOWN);
      } else if (kOccupiedThresh < ratio_ptr[j]) {
        map_ptr[j] = static_cast<uint8_t>(GridStates::OCCUPIED);
      } else {
        map_ptr[j] = static_cast<uint8_t>(GridStates::FREE);
      }
    }
  }

  // for test constant map output
  // map_.setTo(static_cast<uint8_t>(GridStates::OCCUPIED));
}

cv::Point2i OccupancyGridMap::covert_point_m2i(const cv::Point2f& map_point) {
  cv::Point2f point = map_point - map_offset_;
  point /=resolution_;

  // printf("point: (%f, %f)\n", point.x, point.y);
  if (point.x < 0) {
    point.x = 0;
  }
  if (point.y < 0) {
    point.y = 0;
  }
  if (image_size_.width < point.x) {
    point.x = image_size_.width;
  }
  if (image_size_.height < point.y) {
    point.y = image_size_.height;
  }
  assert(0 <= point.x && 0 <= point.y && point.x <= image_size_.width && point.y <= image_size_.height);
  return cv::Point2i(point.x, point.y);  // FIXME: why not image_size_.height - point.y
}

// cv::Point2f OccupancyGridMap::covert_point_i2m(const cv::Point2i& image_point) {
//   cv::Point2i point(image_point.x, image_size_.height - image_point.y);
//   return cv::Point2f(point * resolution_);
// }
