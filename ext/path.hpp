#pragma once
// std
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <list>
#include <numeric>
#include <optional>
#include <vector>
// Eigen
#include <Eigen/Dense>

namespace ext {
  template <typename FLOATING_TYPE>
  struct Path;
  using Pathf = Path<float>;
  using Pathd = Path<double>;

  /**
   * @brief Pathクラス
   *
   */
  template <typename FLOATING_TYPE>
  struct Path {
    std::vector<Eigen::Vector3<FLOATING_TYPE>> points;

    Path() = default;
    Path(Path& path) : points(path.points) {}
    Path(Path&& path) { points = std::move(path.points); }
    Path& operator=(Path&& path) {
      points = std::move(path.points);
      return *this;
    }

    std::vector<FLOATING_TYPE> lengths() {
      FLOATING_TYPE len = 0;
      std::vector<FLOATING_TYPE> lens;
      if(points.size()<=1) return lens;
      lens.resize(points.size() - 1);
      for (size_t i = 0; i < points.size() - 1; i++)
        lens[i] = (points[i + 1].head(2)-points[i].head(2)).norm();
      return lens;
    }

    FLOATING_TYPE length() {
      auto lens = lengths();
      if(points.size()<=1) return -1;
      return std::accumulate(lens.begin(), lens.end(), 0.0);
    }

    Eigen::Vector3<FLOATING_TYPE> lerp(FLOATING_TYPE t) {
      if (t < 0.0) t = 0;
      FLOATING_TYPE passed_len     = 0;
      Eigen::Vector3<FLOATING_TYPE> point = {points.back().x(), points.back().y(), 0.0};
      for (size_t i = 0; i < points.size() - 1; i++) {
        FLOATING_TYPE l = (points[i + 1].head(2) - points[i].head(2)).norm();
        if ((passed_len + l) > t) {
          t -= passed_len;
          t /= l;
          point.x() = points[i].x() + (points[i + 1].x() - points[i].x()) * t;
          point.y() = points[i].y() + (points[i + 1].y() - points[i].y()) * t;
          break;
        }
        passed_len += l;
      }
      return point;
    }

    size_t get_nearest_point_index(const Eigen::Vector3<FLOATING_TYPE>& p) {
      std::vector<double> dist;
      for (const auto& path_p : points)
        dist.push_back((double)(p.head(2) - path_p.head(2)).norm());
      std::vector<double>::iterator iter = std::min_element(dist.begin(), dist.end());
      return std::distance(dist.begin(), iter);
    }

    size_t get_nearest_point_index(const Eigen::Vector2<FLOATING_TYPE>& p) {
      return get_nearest_point_index(Eigen::Vector3<FLOATING_TYPE>(p.x(), p.y(), 0));
    }
  };


  std::ostream& operator<<(std::ostream& os, const Pathf& path) {
    os << "size: " << path.points.size() << std::endl;
    for (size_t i = 0; i < path.points.size(); i++) {
      os << "[" << path.points[i].transpose() << "]" << std::endl;
    }
    return os;
  };
  std::ostream& operator<<(std::ostream& os, const Pathd& path) {
    os << "size: " << path.points.size() << std::endl;
    for (size_t i = 0; i < path.points.size(); i++) {
      os << "[" << path.points[i].transpose() << "]" << std::endl;
    }
    return os;
  };
} // namespace ext
#ifdef USE_ROS2_NAV_PATH
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2/utils.h>

namespace ros2_utils {
  /**
   * @brief  nav_msgs::msg::PathからPathに変換
   *
   * @param nav_msgs::msg::Path
   * @return Path<FLOATING_TYPE>
   */
  template <typename FLOATING_TYPE = double>
  ext::Path<FLOATING_TYPE> make_path(const nav_msgs::msg::Path& path_msg) {
    ext::Path<FLOATING_TYPE> path;
    for (const auto& pose : path_msg.poses) {
      path.points.push_back(Eigen::Vector3d(pose.pose.position.x, pose.pose.position.y, tf2::getYaw(pose.pose.orientation)));
    }
    return path;
  }
  /**
   * @brief  Pathからnav_msgs::msg::Pathに変換
   *
   * @param Path<FLOATING_TYPE>
   * @return nav_msgs::msg::Path
   */
  template <typename FLOATING_TYPE = double>
  nav_msgs::msg::Path make_nav_path(std_msgs::msg::Header header, const ext::Path<FLOATING_TYPE>& path) {
    nav_msgs::msg::Path path_msg;
    path_msg.header = header;
    for (const auto& point : path.points) {
      geometry_msgs::msg::PoseStamped pose_stamped;
      pose_stamped.header = header;
      pose_stamped.pose.position.x = point.x();
      pose_stamped.pose.position.y = point.y();
      pose_stamped.pose.orientation.z = std::sin(point.z() / 2);
      pose_stamped.pose.orientation.w = std::cos(point.z() / 2);
      path_msg.poses.push_back(pose_stamped);
    }
    return path_msg;
  }
} // namespace ros2_utils
#endif

