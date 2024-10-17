#pragma once
// std
#include <iostream>
#include <optional>
#include "../../utility/math_utils.hpp"
// ros2
#include "../ros2_includes.hpp"
#include "../marker_utils.hpp"
// pcl
#include "../../pcl_utility/pcl_includes.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace ros2_utils {
  /**
   * @brief make_ros_pointcloud2
   *
   * @tparam POINT_TYPE pcl::PointXYZ
   * @param header std_msgs::msg::Header
   * @param cloud pcl::PointCloud<POINT_TYPE>
   * @return sensor_msgs::msg::PointCloud2
   */
  template <typename POINT_TYPE = pcl::PointXYZ>
  inline sensor_msgs::msg::PointCloud2 make_ros_pointcloud2(std_msgs::msg::Header header, const pcl::PointCloud<POINT_TYPE>& cloud) {
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header = header;
    return cloud_msg;
  }

  /**
   * @brief make_point_maker
   *
   * @tparam POINT_TYPE pcl::PointXYZ
   * @param header std_msgs::msg::Header
   * @param point POINT_TYPE
   * @param id int
   * @param color std_msgs::msg::ColorRGBA
   * @param size double
   * @return sensor_msgs::msg::PointCloud2
   */
  template <typename POINT_TYPE = pcl::PointXYZ>
  inline visualization_msgs::msg::Marker make_point_maker(std_msgs::msg::Header header, POINT_TYPE point, int id, std_msgs::msg::ColorRGBA color,
                                                          double size = 0.05) {
    return make_point_maker(header, common_utils::conversion_vector3<POINT_TYPE, geometry_msgs::msg::Point>(point), id, color, size);
  }

  /**
   * @brief transform_pcl_pointcloud
   *
   * @tparam POINT_TYPE pcl::PointXYZ
   * @param tf_buffer tf2_ros::Buffer
   * @param source_frame std::string 変換元のフレーム名
   * @param target_frame std::string 変換先のフレーム名
   * @param source_cloud pcl::PointCloud<POINT_TYPE>
   * @param time rclcpp::Time
   * @return std::optional<pcl::PointCloud<POINT_TYPE>>
   */
  template <typename POINT_TYPE = pcl::PointXYZ>
  inline std::optional<pcl::PointCloud<POINT_TYPE>> transform_pcl_pointcloud(const tf2_ros::Buffer& tf_buffer, std::string source_frame,
                                                                          std::string target_frame, const pcl::PointCloud<POINT_TYPE>& source_cloud,
                                                                          const rclcpp::Time& time = rclcpp::Time(0)) {
    std::optional<pcl::PointCloud<POINT_TYPE>> ret = std::nullopt;
    try {
      pcl::PointCloud<POINT_TYPE> target_cloud;
      geometry_msgs::msg::TransformStamped transform_stamped = tf_buffer.lookupTransform(target_frame, source_frame, time, tf2::durationFromSec(0.0));
      Eigen::Affine3f affine_conversion(tf2::transformToEigen(transform_stamped).affine().cast<float>());
      pcl::transformPointCloud(source_cloud, target_cloud, affine_conversion);
      ret = target_cloud;
    } catch (tf2::LookupException& ex) {
      std::cerr << "[ERROR]" << ex.what() << std::endl;
    } catch (tf2::ExtrapolationException& ex) {
      std::cerr << "[ERROR]" << ex.what() << std::endl;
    }
    return ret;
  }

  /**
   * @brief transform_pointcloud2
   *
   * @param tf_buffer tf2_ros::Buffer
   * @param target_frame std::string 変換先のフレーム名
   * @param source_cloud sensor_msgs::msg::PointCloud2
   * @param time rclcpp::Time
   * @return std::optional<sensor_msgs::msg::PointCloud2>
   */
  inline std::optional<sensor_msgs::msg::PointCloud2> transform_pointcloud2(const tf2_ros::Buffer& tf_buffer, std::string target_frame,
                                                                            const sensor_msgs::msg::PointCloud2& source_cloud,
                                                                            const rclcpp::Time& time = rclcpp::Time(0)) {
    std::optional<sensor_msgs::msg::PointCloud2> ret = std::nullopt;
    try {
      sensor_msgs::msg::PointCloud2 target_cloud;
      geometry_msgs::msg::TransformStamped transform_stamped =
          tf_buffer.lookupTransform(target_frame, source_cloud.header.frame_id, time, tf2::durationFromSec(0.0));
      Eigen::Affine3f affine_conversion(tf2::transformToEigen(transform_stamped).affine().cast<float>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr source_pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::PointCloud<pcl::PointXYZ>::Ptr target_pcl_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      pcl::fromROSMsg(source_cloud, *source_pcl_cloud);
      pcl::transformPointCloud(*source_pcl_cloud, *target_pcl_cloud, affine_conversion);
      pcl::toROSMsg(*target_pcl_cloud, target_cloud);
      target_cloud.header.frame_id = target_frame;
      target_cloud.header.stamp    = source_cloud.header.stamp;
      ret                          = target_cloud;
    } catch (tf2::LookupException& ex) {
      std::cerr << "[ERROR]" << ex.what() << std::endl;
    } catch (tf2::ExtrapolationException& ex) {
      std::cerr << "[ERROR]" << ex.what() << std::endl;
    }
    return ret;
  }

} // namespace ros2_utils