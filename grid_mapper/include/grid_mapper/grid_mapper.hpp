// Copyright <2021> [Copyright rossihwang]

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer_interface.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/create_timer_ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <nav_msgs/msg/map_meta_data.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace slamtk {

enum class GridStates {
  UNKNOWN = 0,
  OCCUPIED = 100,
  FREE = 255
};

// NOTE: cv::boundingRect() doesn't allow negative inputs, thus we implement ours
// Used for estimate the image size of the map
class BoundingBox {
 protected:
  cv::Point2f minimum_;
  cv::Point2f maximum_;
 public:
  BoundingBox() 
    : minimum_(std::numeric_limits<float>::max(), std::numeric_limits<float>::max()),
      maximum_(std::numeric_limits<float>::min(), std::numeric_limits<float>::min()) {
    
  }
  float get_width() const {
    return maximum_.x - minimum_.x;
  }
  float get_height() const {
    return maximum_.y - minimum_.y;
  }
  cv::Point2f get_minimum() const {
    return minimum_;
  }
  cv::Point2f get_maximum() const {
    return maximum_;
  }
  void add_point(const cv::Point2f& point) {
    minimum_.x = (point.x < minimum_.x) ? point.x : minimum_.x;
    minimum_.y = (point.y < minimum_.y) ? point.y : minimum_.y;
    maximum_.x = (maximum_.x < point.x) ? point.x : maximum_.x;
    maximum_.y = (maximum_.y < point.y) ? point.y : maximum_.y;
  }
  void add_points(const std::vector<cv::Point2f>& points) {
    for (auto p: points) {
      add_point(p);
    }
  }
  void show(const std::string& tag="") {
    printf("Bounding box(%s): min: (%f, %f), max: (%f, %f)\n", tag.c_str(), minimum_.x, minimum_.y, maximum_.x, maximum_.y);
  }
  friend BoundingBox operator|(const BoundingBox& lhs, const BoundingBox& rhs) {
    BoundingBox bbox;
    bbox.minimum_.x = (rhs.minimum_.x < lhs.minimum_.x) ? rhs.minimum_.x : lhs.minimum_.x;
    bbox.minimum_.y = (rhs.minimum_.y < lhs.minimum_.y) ? rhs.minimum_.y : lhs.minimum_.y;
    bbox.maximum_.x = (lhs.maximum_.x < rhs.maximum_.x) ? rhs.maximum_.x : lhs.maximum_.x;
    bbox.maximum_.y = (lhs.maximum_.y < rhs.maximum_.y) ? rhs.maximum_.y : lhs.maximum_.y;
    return bbox;
  }
  BoundingBox& operator|=(const BoundingBox& rhs) {
    minimum_.x = (rhs.minimum_.x < minimum_.x) ? rhs.minimum_.x : minimum_.x;
    minimum_.y = (rhs.minimum_.y < minimum_.y) ? rhs.minimum_.y : minimum_.y;
    maximum_.x = (maximum_.x < rhs.maximum_.x) ? rhs.maximum_.x : maximum_.x;
    maximum_.y = (maximum_.y < rhs.maximum_.y) ? rhs.maximum_.y : maximum_.y;
    return *this;
  }
  // Shift the bounding box by the base pose
  friend BoundingBox operator+(const BoundingBox& lhs, const cv::Vec3f& base_pose) {
    // NOTE: during transform MUST take consider the four corners of the bounding box
    cv::Mat before = (cv::Mat_<float>(4, 3) << lhs.minimum_.x, lhs.minimum_.y, 1,  // bottom left
                                                lhs.minimum_.x, lhs.maximum_.y, 1, // top left
                                                lhs.maximum_.x, lhs.minimum_.y, 1,  // bottom right
                                                lhs.maximum_.x, lhs.maximum_.y, 1);  // top right
    before = before.t();  // 3 x 4
    cv::Mat trans = (cv::Mat_<float>(2, 3) << std::cos(base_pose[2]), -std::sin(base_pose[2]), base_pose[0],
                                              std::sin(base_pose[2]), std::cos(base_pose[2]), base_pose[1]);
    cv::Mat after = trans * before;
    double min_x, max_x, min_y, max_y;
    cv::minMaxLoc(after.row(0), &min_x, &max_x);
    cv::minMaxLoc(after.row(1), &min_y, &max_y);

    BoundingBox bbox;
    bbox.minimum_ = cv::Point2f(min_x, min_y);
    bbox.maximum_ = cv::Point2f(max_x, max_y);

    return bbox;
  }
};

struct LaserMetaData {
  float range_min;
  float range_max;
  float angle_min;
  float angle_max;  // redundant
  float angle_increment;
  cv::Vec3f laser_pose;  // w.r.t base frame

  LaserMetaData(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, const cv::Vec3f& laser_pose)
    : range_min(scan->range_min),
      range_max(scan->range_max),
      angle_min(scan->angle_min),
      angle_max(scan->angle_max),
      angle_increment(scan->angle_increment),
      laser_pose(laser_pose) {
  }
  void show() {
    printf("range: (%f, %f), angle: (%f, %f, %f), laser_pose: (%f, %f, %f)\n", 
      range_min, range_max,
      angle_min, angle_max, angle_increment,
      laser_pose[0], laser_pose[1], laser_pose[2]);
  }
};

class ScanWithPose {
 protected:
  cv::Vec3f base_pose_;
  std::string scan_frame_;
  std::weak_ptr<LaserMetaData> meta_data_;
 public:
  std::vector<std::pair<cv::Point2f, bool>> scan_points;  // as Euclidean w.r.t map frame
  BoundingBox bbox;
 public:
  ScanWithPose()
    : base_pose_(0, 0, 0) {
    
  }
  ScanWithPose(const cv::Vec3f& base_pose, std::weak_ptr<LaserMetaData> laser_meta_data, size_t points_amount)
    : base_pose_(base_pose),
      meta_data_(laser_meta_data) {
    scan_points.resize(points_amount);  // FIXME: may move outside
  }
  void set_base_pose(const cv::Vec3f& base_pose) {
    base_pose_ = base_pose;
  }
  cv::Vec3f get_base_pose() const {
    return base_pose_;
  }
  std::weak_ptr<LaserMetaData> get_meta_data() const {
    return meta_data_;
  }
};

class OccupancyGridMap {
 protected:
  cv::Mat map_;
  cv::Vec2f map_size_;
  cv::Size image_size_;
  cv::Point2f map_offset_;  // NOTE: the global frame w.r.t occupancy grid map frame(on the left bottom the image, usually no relative rotation)
  float resolution_;
 
 public:
  OccupancyGridMap(const cv::Vec2f& map_size, const cv::Point2f& map_offset, float resolution);
  ~OccupancyGridMap();
  std::tuple<cv::Vec2f, cv::Point2f> compute_map_dimension(const std::vector<ScanWithPose>& scans);
  cv::Mat get_map() const {
    return map_;
  }
  cv::Point2f get_map_offset() const {
    return map_offset_;
  }
  float get_resolution() const {
    return resolution_;
  }
  void create_from_scans(const std::vector<ScanWithPose>& scans);
  cv::Point2i covert_point_m2i(const cv::Point2f& map_point);
  // cv::Point2f covert_point_i2m(const cv::Point2i& image_point);

};

class GridMapper: public rclcpp::Node {
 protected:
  std::vector<ScanWithPose> scans_;
  int max_buffered_scans_;
  std::unordered_map<std::string, std::shared_ptr<LaserMetaData>> frame_id_meta_lut_;
  std::unordered_map<int, std::shared_ptr<ScanWithPose>> id_scan_lut_;

  std::string odom_frame_;
  std::string map_frame_;
  std::string base_frame_;
  std::string scan_frame_;
  std::string scan_topic_;
  std::string map_topic_;
  float resolution_;

  std::unique_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> scan_filter_sub_;
  std::unique_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> scan_filter_;

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<nav_msgs::msg::MapMetaData>::SharedPtr map_meta_pub_;

  std::unique_ptr<tf2_ros::Buffer> tf_;
  std::unique_ptr<tf2_ros::TransformListener> tfL_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tfB_;

  std::unique_ptr<OccupancyGridMap> occupancy_map_ptr_;

  std::atomic<bool> canceled_;
  std::thread main_thread_;

 public:
  GridMapper(const std::string& name, rclcpp::NodeOptions const& options);
  ~GridMapper();
  void configure();
  void set_ros_interfaces();
  bool add_scan(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, const cv::Vec3f& base_pose, std::weak_ptr<LaserMetaData> laser_meta_data);
  bool add_laser(sensor_msgs::msg::LaserScan::ConstSharedPtr scan, const cv::Vec3f& laser_pose);
  bool clean_scans();
  bool build_map();
  bool should_build_map();
  bool update_map();
  void publish_odom_map_transform();
  void publish_map_thread();
  void scan_callback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan);
  std::tuple<bool, cv::Vec3f> get_frame_pose(const std::string& from_frame, const std::string& to_frame, const rclcpp::Time& stamp);
  bool should_add_scan(const cv::Vec3f& current_pose);
  // void transform_2d_pose(const Vec3f& target, const Vec3f& ref);
};

}  // namespace slamtk