#pragma once
#include "ros2_includes.hpp"

namespace ros2_utils {
  /**
   * @brief make_point_maker
   *
   * @param header std_msgs::msg::Header
   * @param point geometry_msgs::msg::Point
   * @param id int
   * @param color std_msgs::msg::ColorRGBA
   * @param size double
   * @return visualization_msgs::msg::Marker
   */
  template <typename FLOATING_TYPE = double>
  inline visualization_msgs::msg::Marker make_point_maker(const std_msgs::msg::Header& header, const geometry_msgs::msg::Point& point, int id,
                                                          const std_msgs::msg::ColorRGBA& color, double size = 0.05) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns     = "point";
    marker.id     = id;
    marker.type   = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.lifetime           = rclcpp::Duration(0.0);
    marker.pose.orientation.w = 1;
    marker.pose.position.x    = point.x;
    marker.pose.position.y    = point.y;
    marker.pose.position.z    = point.z;
    marker.scale.x            = size; // x方向の直径
    marker.scale.y            = size; // y方向の直径
    marker.scale.z            = size; // z方向の直径
    marker.color              = color;
    return marker;
  }

  /**
   * @brief make_point_maker
   *
   * @param header std_msgs::msg::Header
   * @param point geometry_msgs::msg::Vector3
   * @param id int
   * @param color std_msgs::msg::ColorRGBA
   * @param size double
   * @return visualization_msgs::msg::Marker
   */
  template <typename FLOATING_TYPE = double>
  inline visualization_msgs::msg::Marker make_point_maker(const std_msgs::msg::Header& header, const geometry_msgs::msg::Vector3& point, int id,
                                                          const std_msgs::msg::ColorRGBA& color, double size = 0.05) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns     = "point";
    marker.id     = id;
    marker.type   = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.lifetime           = rclcpp::Duration(0.0);
    marker.pose.orientation.w = 1;
    marker.pose.position.x    = point.x;
    marker.pose.position.y    = point.y;
    marker.pose.position.z    = point.z;
    marker.scale.x            = size; // x方向の直径
    marker.scale.y            = size; // y方向の直径
    marker.scale.z            = size; // z方向の直径
    marker.color              = color;
    return marker;
  }

  /**
   * @brief make_circle_maker
   *
   * @param header std_msgs::msg::Header
   * @param point geometry_msgs::msg::Vector3
   * @param id int
   * @param color std_msgs::msg::ColorRGBA
   * @param size geometry_msgs::msg::Vector3
   * @return visualization_msgs::msg::Marker
   */
  inline visualization_msgs::msg::Marker make_circle_maker(const std_msgs::msg::Header& header, const geometry_msgs::msg::Vector3& point, int id,
                                                           const std_msgs::msg::ColorRGBA& color, const geometry_msgs::msg::Vector3& size) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns     = "point";
    marker.id     = id;
    marker.type   = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.lifetime           = rclcpp::Duration(0.0);
    marker.pose.orientation.w = 1;
    marker.pose.position.x    = point.x;
    marker.pose.position.y    = point.y;
    marker.pose.position.z    = point.z;
    marker.scale.x            = size.x; // x方向の直径
    marker.scale.y            = size.y; // y方向の直径
    marker.scale.z            = size.z; // z方向の直径
    marker.color              = color;
    return marker;
  }

  /**
   * @brief make_vector_maker
   *
   * @param header std_msgs::msg::Header
   * @param start geometry_msgs::msg::Vector3
   * @param v geometry_msgs::msg::Vector3
   * @param id int
   * @param color std_msgs::msg::ColorRGBA
   * @return visualization_msgs::msg::Marker
   */
  inline visualization_msgs::msg::Marker make_vector_maker(const std_msgs::msg::Header& header, const geometry_msgs::msg::Vector3& start,
                                                           const geometry_msgs::msg::Vector3& v, int id, const std_msgs::msg::ColorRGBA& color) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns     = "vector";
    marker.id     = id;
    marker.type   = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.lifetime           = rclcpp::Duration(0.0);
    marker.pose.orientation.w = 1;

    marker.scale.x = 0.05;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    geometry_msgs::msg::Point p;
    p.x = start.x;
    p.y = start.y;
    p.z = start.z;
    marker.points.push_back(p);
    p.x = start.x + v.x;
    p.y = start.y + v.y;
    p.z = start.z + v.z;
    marker.points.push_back(p);
    marker.color = color;
    return marker;
  }

  /**
   * @brief make_area_maker
   *
   * @param header std_msgs::msg::Header
   * @param pos geometry_msgs::msg::Vector3
   * @param size geometry_msgs::msg::Vector3
   * @param id int
   * @param color std_msgs::msg::ColorRGBA
   * @return visualization_msgs::msg::Marker
   */
  inline visualization_msgs::msg::Marker make_area_maker(const std_msgs::msg::Header& header, const geometry_msgs::msg::Vector3& pos,
                                                         const geometry_msgs::msg::Vector3& size, int id, const std_msgs::msg::ColorRGBA& color) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns     = "area";
    marker.id     = id;
    marker.type   = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.lifetime           = rclcpp::Duration(0.0);
    marker.pose.orientation.w = 1;
    marker.scale.x            = size.x;
    marker.scale.y            = size.y;
    marker.scale.z            = size.z;
    marker.pose.position.x    = pos.x;
    marker.pose.position.y    = pos.y;
    marker.pose.position.z    = pos.z;
    marker.color              = color;
    return marker;
  }

  /**
   * @brief make_empty_maker
   *
   * @param header std_msgs::msg::Header
   * @param ns std::string
   * @param id int
   * @return visualization_msgs::msg::Marker
   */
  inline visualization_msgs::msg::Marker make_empty_maker(const std_msgs::msg::Header& header, const std::string& ns, int id) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns     = ns;
    marker.id     = id;
    marker.type   = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
    // marker.lifetime           = rclcpp::Duration(0.0);
    marker.pose.orientation.w = 1;
    return marker;
  }

} // namespace ros2_utils