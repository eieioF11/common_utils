#pragma once
// ros2
#include "ros2_includes.hpp"
#include "msg_utils.hpp"

namespace ros2_utils {

  /**
   * @brief make_transform
   *
   * @param msg geometry_msgs::msg::Transform
   * @return tf2::Transform
   */
  inline tf2::Transform make_transform(const geometry_msgs::msg::Transform& msg) {
    tf2::Transform tf;
    tf.setOrigin(tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));
    tf2::Quaternion q(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w);
    tf.setRotation(q);
    return tf;
  }

  // 座標変換
  /**
   * @brief lookup_transform
   *
   * @param buffer tf2_ros::Buffer
   * @param source_frame std::string 変換元のフレーム名
   * @param target_frame std::string 変換先のフレーム名
   * @param time rclcpp::Time
   * @param timeout rclcpp::Duration
   * @return std::optional<geometry_msgs::msg::TransformStamped>
   */
  inline std::optional<geometry_msgs::msg::TransformStamped> lookup_transform(const tf2_ros::Buffer& buffer, const std::string& source_frame,
                                                                              const std::string& target_frame,
                                                                              const rclcpp::Time& time    = rclcpp::Time(0),
                                                                              const tf2::Duration timeout = tf2::durationFromSec(0.0)) {
    std::optional<geometry_msgs::msg::TransformStamped> ret = std::nullopt;
    try {
      ret = buffer.lookupTransform(target_frame, source_frame, time, timeout);
    } catch (tf2::LookupException& ex) {
      std::cerr << "[ERROR]" << ex.what() << std::endl;
    } catch (tf2::ExtrapolationException& ex) {
      std::cerr << "[ERROR]" << ex.what() << std::endl;
    }
    return ret;
  }

  // オイラー角とクオータニオン変換
  /**
   * @brief EulerToQuaternion
   *
   * @param roll double
   * @param pitch double
   * @param yaw double
   * @return geometry_msgs::msg::Quaternion
   */
  inline geometry_msgs::msg::Quaternion EulerToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);
    return tf2::toMsg(tf_quat);
  }

  /**
   * @brief EulerToQuaternion
   *
   * @param euler geometry_msgs::msg::Vector3
   * @return geometry_msgs::msg::Quaternion
   */
  inline geometry_msgs::msg::Quaternion EulerToQuaternion(geometry_msgs::msg::Vector3 euler) { return EulerToQuaternion(euler.x, euler.y, euler.z); }

  /**
   * @brief QuaternionToEuler
   *
   * @param geometry_quat geometry_msgs::msg::Quaternion
   * @return geometry_msgs::msg::Vector3
   */
  inline geometry_msgs::msg::Vector3 QuaternionToEuler(const geometry_msgs::msg::Quaternion& geometry_quat) {
    tf2::Quaternion tf_quat;
    tf2::convert(geometry_quat, tf_quat);
    tf2::Matrix3x3 m(tf_quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return make_geometry_vector3(roll, pitch, yaw);
  }

} // namespace ros2_utils