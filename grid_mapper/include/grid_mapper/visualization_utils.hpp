// Copyright <2021> [Copyright rossihwang]

#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <grid_mapper/grid_mapper.hpp>

using slamtk::OccupancyGridMap;
using slamtk::GridStates;

constexpr int map_index(int width, int x, int y) {
  return (width * y + x);
}

inline void to_nav_map(const OccupancyGridMap& map, nav_msgs::msg::OccupancyGrid* ros_map) {
  
  cv::Mat map_mat = map.get_map();
  uint32_t width = map_mat.cols;
  uint32_t height = map_mat.rows;
  cv::Vec2f offset = map.get_map_offset();

  printf("to_nav_map: width: %d, height: %d, offset: (%f, %f)\n", width, height, offset[0], offset[1]);

  if (ros_map->info.width != width
    || ros_map->info.height != height
    || ros_map->info.origin.position.x != offset[0]
    || ros_map->info.origin.position.y != offset[1]) {
    ros_map->info.origin.position.x = offset[0];
    ros_map->info.origin.position.y = offset[1];
    ros_map->info.width = width;
    ros_map->info.height = height;
    ros_map->info.resolution = map.get_resolution();
    ros_map->data.resize(ros_map->info.width * ros_map->info.height);
  }

  for (size_t i = 0; i < height; ++i) {
    for (size_t j = 0; j < width; ++j) {
      switch (static_cast<GridStates>(map_mat.at<uint8_t>(i, j))) {
        case GridStates::UNKNOWN:
          ros_map->data[map_index(map_mat.cols, j, i)] = -1;
          break;
        case GridStates::OCCUPIED:
          ros_map->data[map_index(map_mat.cols, j, i)] = 100;
          break;
        case GridStates::FREE:
          ros_map->data[map_index(map_mat.cols, j, i)] = 0;
          break;
      }
    }
  }
}