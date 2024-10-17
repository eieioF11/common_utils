#pragma once
// std
#include <iostream>
#include <optional>
// ros2
#include "ros2_includes.hpp"

namespace ros2_utils {
  /**
   * @brief make_header
   *
   * @param frame_name std::string
   * @param t rclcpp::Time
   * @return std_msgs::msg::Header
   */
  inline std_msgs::msg::Header make_header(const std::string& frame_name, const rclcpp::Time& t) {
    std_msgs::msg::Header header;
    header.frame_id = frame_name;
    header.stamp    = t;
    return header;
  }

  /**
   * @brief make_bool
   *
   * @param data bool
   * @return std_msgs::msg::Header
   */
  inline std_msgs::msg::Bool make_bool(bool data) {
    std_msgs::msg::Bool msg;
    msg.data = data;
    return msg;
  }

  /**
   * @brief make_string
   *
   * @param str std::string
   * @return std_msgs::msg::String
   */
  inline std_msgs::msg::String make_string(const std::string& str) {
    std_msgs::msg::String msg;
    msg.data = str;
    return msg;
  }

  /**
   * @brief make_float32
   *
   * @tparam FLOATING_TYPE
   * @param val FLOATING_TYPE
   * @return std_msgs::msg::Float32
   */
  template <typename FLOATING_TYPE = double>
  inline std_msgs::msg::Float32 make_float32(const FLOATING_TYPE& val) {
    std_msgs::msg::Float32 msg;
    msg.data = val;
    return msg;
  }

  /**
   * @brief make_color
   *
   * @param r double
   * @param g double
   * @param b double
   * @param a double
   * @return std_msgs::msg::ColorRGBA
   */
  inline std_msgs::msg::ColorRGBA make_color(double r, double g, double b, double a = 1.0) {
    std_msgs::msg::ColorRGBA ret;
    ret.r = r;
    ret.g = g;
    ret.b = b;
    ret.a = a;
    return ret;
  }

  // Quaternion
  /**
   * @brief make_geometry_quaternion
   *
   * @param x double
   * @param y double
   * @param z double
   * @param w double
   * @return geometry_msgs::msg::Quaternion
   */
  inline geometry_msgs::msg::Quaternion make_geometry_quaternion(double x, double y, double z, double w) {
    geometry_msgs::msg::Quaternion q;
    q.x = x;
    q.y = y;
    q.z = z;
    q.w = w;
    return q;
  }

  // pose
  /**
   * @brief make_geometry_pose
   *
   * @param x double
   * @param y double
   * @param z double
   * @param theta double
   * @return geometry_msgs::msg::Pose
   */
  inline geometry_msgs::msg::Pose make_geometry_pose(double x, double y, double z, double theta) {
    geometry_msgs::msg::Pose pose;
    pose.position.x    = x;
    pose.position.y    = y;
    pose.position.z    = z;
    pose.orientation.z = std::sin(theta / 2);
    pose.orientation.w = std::cos(theta / 2);
    return pose;
  }

  /**
   * @brief make_geometry_pose
   *
   * @param x double
   * @param y double
   * @param z double
   * @param q geometry_msgs::msg::Quaternion
   * @return geometry_msgs::msg::Pose
   */
  inline geometry_msgs::msg::Pose make_geometry_pose(double x, double y, double z, geometry_msgs::msg::Quaternion q) {
    geometry_msgs::msg::Pose pose;
    pose.position.x  = x;
    pose.position.y  = y;
    pose.position.z  = z;
    pose.orientation = q;
    return pose;
  }

  // transform
  /**
   * @brief make_geometry_transform
   *
   * @param x double
   * @param y double
   * @param z double
   * @param theta double
   * @return geometry_msgs::msg::Transform
   */
  inline geometry_msgs::msg::Transform make_geometry_transform(double x, double y, double z, double theta) {
    geometry_msgs::msg::Transform transform;
    transform.translation.x = x;
    transform.translation.y = y;
    transform.translation.z = z;
    transform.rotation.z    = sin(theta / 2);
    transform.rotation.w    = cos(theta / 2);
    return transform;
  }

  /**
   * @brief make_geometry_transform
   *
   * @param x double
   * @param y double
   * @param z double
   * @param q geometry_msgs::msg::Quaternion
   * @return geometry_msgs::msg::Transform
   */
  inline geometry_msgs::msg::Transform make_geometry_transform(double x, double y, double z, geometry_msgs::msg::Quaternion q) {
    geometry_msgs::msg::Transform transform;
    transform.translation.x = x;
    transform.translation.y = y;
    transform.translation.z = z;
    transform.rotation      = q;
    return transform;
  }

  // point
  /**
   * @brief make_geometry_point
   *
   * @param x double
   * @param y double
   * @param z double
   * @return geometry_msgs::msg::Point
   */
  inline geometry_msgs::msg::Point make_geometry_point(double x, double y, double z) {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  /**
   * @brief make_geometry_point32
   *
   * @param x double
   * @param y double
   * @param z double
   * @return geometry_msgs::msg::Point32
   */
  inline geometry_msgs::msg::Point32 make_geometry_point32(double x, double y, double z) {
    geometry_msgs::msg::Point32 p;
    p.x = x;
    p.y = y;
    p.z = z;
    return p;
  }

  // vector
  /**
   * @brief make_geometry_vector3
   *
   * @param x double
   * @param y double
   * @param z double
   * @return geometry_msgs::msg::Vector3
   */
  inline geometry_msgs::msg::Vector3 make_geometry_vector3(double x, double y, double z) {
    geometry_msgs::msg::Vector3 v;
    v.x = x;
    v.y = y;
    v.z = z;
    return v;
  }

  /**
   * @brief make_geometry_vector3
   *
   * @param v Eigen::VectorX<T>
   * @return geometry_msgs::msg::Vector3
   */
  template <typename T = double>
  inline geometry_msgs::msg::Vector3 make_geometry_vector3(const Eigen::VectorX<T>& v) {
    return make_geometry_vector3(v(0), v(1), v(2));
  }

  /**
   * @brief make_geometry_vector3
   *
   * @param v Eigen::Vector3<T>
   * @return geometry_msgs::msg::Vector3
   */
  template <typename T = double>
  inline geometry_msgs::msg::Vector3 make_geometry_vector3(const Eigen::Vector3<T>& v) {
    return make_geometry_vector3(v.x(), v.y(), v.z());
  }

  /**
   * @brief make_geometry_vector3
   *
   * @param v Eigen::Vector2<T>
   * @return geometry_msgs::msg::Vector3
   */
  template <typename T = double>
  inline geometry_msgs::msg::Vector3 make_geometry_vector3(const Eigen::Vector2<T>& v) {
    return make_geometry_vector3(v.x(), v.y(), 0.0);
  }

  // twist
  /**
   * @brief make_geometry_twist
   *
   * @param x double
   * @param y double
   * @param z double
   * @param angular_x double
   * @param angular_y double
   * @param angular_z double
   * @return geometry_msgs::msg::Twist
   */
  inline geometry_msgs::msg::Twist make_geometry_twist(double x, double y, double z, double angular_x, double angular_y, double angular_z) {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = x;
    twist.linear.y  = y;
    twist.linear.z  = z;
    twist.angular.x = angular_x;
    twist.angular.y = angular_y;
    twist.angular.z = angular_z;
    return twist;
  }

  /**
   * @brief stop
   *
   * @return geometry_msgs::msg::Twist
   */
  geometry_msgs::msg::Twist stop() {
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = 0.0;
    twist.linear.y  = 0.0;
    twist.linear.z  = 0.0;
    twist.angular.x = 0.0;
    twist.angular.y = 0.0;
    twist.angular.z = 0.0;
    return twist;
  }

  // accel
  /**
   * @brief make_geometry_accel
   *
   * @param x double
   * @param y double
   * @param z double
   * @param angular_x double
   * @param angular_y double
   * @param angular_z double
   * @return geometry_msgs::msg::Accel
   */
  inline geometry_msgs::msg::Accel make_geometry_accel(double x, double y, double z, double angular_x, double angular_y, double angular_z) {
    geometry_msgs::msg::Accel accel;
    accel.linear.x  = x;
    accel.linear.y  = y;
    accel.linear.z  = z;
    accel.angular.x = angular_x;
    accel.angular.y = angular_y;
    accel.angular.z = angular_z;
    return accel;
  }

} // namespace ros2_utils