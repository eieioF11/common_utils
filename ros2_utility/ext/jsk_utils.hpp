#pragma once
// ros2
#include "../ros2_includes.hpp"
// jsr_rviz_plugin_msgs
#include <jsk_rviz_plugin_msgs/msg/overlay_menu.hpp>
#include <jsk_rviz_plugin_msgs/msg/pictogram.hpp>

namespace ros2_utils {
  /**
   * @brief make_overlay_menu
   *
   * @param title std::string
   * @param menu_names std::vector<std::string>
   * @param index int
   * @param action int32_t
   * @return jsk_rviz_plugin_msgs::msg::OverlayMenu
   */
  inline jsk_rviz_plugin_msgs::msg::OverlayMenu make_overlay_menu(std::string title, const std::vector<std::string>& menu_names, int index = 0,
                                                                  int32_t action = jsk_rviz_plugin_msgs::msg::OverlayMenu::ACTION_SELECT) {
    jsk_rviz_plugin_msgs::msg::OverlayMenu overlay_menu;
    overlay_menu.action        = action;
    overlay_menu.title         = title;
    overlay_menu.menus         = menu_names;
    overlay_menu.current_index = index;
    return overlay_menu;
  }

  /**
   * @brief make_pictogram
   *
   * @param header std_msgs::msg::Header
   * @param pose geometry_msgs::msg::Pose
   * @param color std_msgs::msg::ColorRGBA
   * @param character std::string
   * @param size double
   * @param ttl double
   * @param speed double
   * @param action int32_t
   * @param mode uint8_t
   * @return jsk_rviz_plugin_msgs::msg::Pictogram
   */
  inline jsk_rviz_plugin_msgs::msg::Pictogram make_pictogram(const std_msgs::msg::Header& header, const geometry_msgs::msg::Pose& pose,
                                                             const std_msgs::msg::ColorRGBA& color, const std::string& character, double size = 1.0,
                                                             double ttl = 0.0, double speed = 1.0,
                                                             int32_t action = jsk_rviz_plugin_msgs::msg::Pictogram::JUMP,
                                                             uint8_t mode   = jsk_rviz_plugin_msgs::msg::Pictogram::STRING_MODE) {
    jsk_rviz_plugin_msgs::msg::Pictogram pictogram;
    pictogram.header    = header;
    pictogram.pose      = pose;
    pictogram.color     = color;
    pictogram.action    = action;
    pictogram.mode      = mode;
    pictogram.character = character;
    pictogram.size      = size;
    pictogram.ttl       = ttl;
    pictogram.speed     = speed;
    return pictogram;
  }
} // namespace ros2_utils