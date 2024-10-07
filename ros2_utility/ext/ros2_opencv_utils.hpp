#pragma once
// opencv
#include <opencv2/opencv.hpp>
// ros2
#include "../ros2_includes.hpp"

namespace ros2_utils {
  /**
   * @brief make_cv_mat
   *
   * @param grid_map nav_msgs::msg::OccupancyGrid
   * @return cv::Mat
   */
  inline cv::Mat make_cv_mat(const nav_msgs::msg::OccupancyGrid& grid_map) {
    cv::Mat img = cv::Mat::zeros(cv::Size(grid_map.info.width, grid_map.info.height), CV_8UC1);
    for (unsigned int y = 0; y < grid_map.info.height; y++) {
      for (unsigned int x = 0; x < grid_map.info.width; x++) {
        unsigned int i = x + (grid_map.info.height - y - 1) * grid_map.info.width;
        int intensity  = 205;
        if (grid_map.data[i] >= 0 && grid_map.data[i] <= 100) intensity = std::round((float)(100.0 - grid_map.data[i]) * 2.55);
        img.at<unsigned char>(y, x) = intensity;
      }
    }
    return img;
  }

} // namespace ros2_utils