#pragma once
// std
#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
// Eigen
#include <Eigen/Dense>

namespace ext {
  struct map_info_t {
    double resolution; // The map resolution [m/cell]
    uint32_t width;    // Map width [cells]
    uint32_t height;   // Map height [cells]

    // The origin of the map [m, m, rad]. This is the real-world pose of the
    // bottom left corner of cell (0,0) in the map.
    Eigen::Vector3d origin;
  };
  /**
   * @brief GridMap(OccupancyGrid)クラス
   *
   */
  class GridMap {
  public:
    static constexpr int8_t WALL_VALUE = 100;
    map_info_t info;
    std::vector<int8_t> data; // mapの内容が100だと壁(占有数の確率は[0,100]の範囲) -1は不明
    /*
    参考:nav_msgs/OccupancyGrid
    https://docs.ros.org/en/melodic/api/nav_msgs/html/msg/OccupancyGrid.html
    https://docs.ros2.org/foxy/api/nav_msgs/msg/OccupancyGrid.html
    */

    GridMap() = default;
    GridMap(const size_t& x, const size_t& y) { resize(x, y); }
    GridMap(const map_info_t& i, const std::vector<int8_t>& d) { set_map(i, d); }
    GridMap(const GridMap& map) : info(map.info), data(map.data) {}
    GridMap(GridMap&& map) {
      info = std::move(map.info);
      data = std::move(map.data);
    }
    GridMap& operator=(GridMap&& map) {
      info = std::move(map.info);
      data = std::move(map.data);
      return *this;
    }
    /**
     * @brief  値のセット
     *
     * @param map_info_t
     * @param std::vector<int8_t>
     */
    void set_map(const map_info_t& i, const std::vector<int8_t>& d) {
      info = i;
      data = d;
    }
    /**
     * @brief  壁判定
     *
     * @param val
     * @return bool
     */
    bool is_wall(const int8_t& val) const { return val > (WALL_VALUE - 1); }
    /**
     * @brief  cellの壁判定
     *
     * @param v
     * @return bool
     */
    bool is_wall(const Eigen::Vector2d& v) const { return is_wall(data[v.y() * col() + v.x()]); }
    /**
     * @brief  cellの壁判定
     *
     * @param x
     * @param y
     * @return bool
     */
    bool is_wall(const int& x, const int& y) const { return is_wall(data[y * col() + x]); }
    /**
     * @brief  サイズ変更
     *
     * @param x
     * @param y
     */
    void resize(const size_t& x, const size_t& y) { data.resize(x * y); }
    /**
     * @brief  実際の位置からグリッドマップ上での位置に変換
     *
     * @param Vector2d 実際の位置
     * @return Vector2d グリッドマップ上の位置
     */
    Eigen::Vector2d get_grid_pos(const Eigen::Vector2d& v) const { return get_grid_pos(v.x(), v.y()); }
    /**
     * @brief  実際の位置からグリッドマップ上での位置に変換
     *
     * @param x 実際のx
     * @param y 実際のy
     * @return Eigen::Vector2d グリッドマップ上の位置
     */
    Eigen::Vector2d get_grid_pos(const double& x, const double& y) const {
      double inv_resolution = 1.0 / info.resolution;
      Eigen::Vector2d v, index_origin;
      index_origin.x() = std::round(-info.origin.x() * inv_resolution);
      index_origin.y() = std::round(-info.origin.y() * inv_resolution);
      v.x()            = std::round((x)*inv_resolution + index_origin.x());
      v.y()            = std::round((y)*inv_resolution + index_origin.y());
      return v;
    }
    /**
     * @brief  グリッドマップ上の位置から実際の位置に変換
     *
     * @param Vector2d グリッドマップ上の位置
     * @return Eigen::Vector2d 実際の位置
     */
    Eigen::Vector2d grid_to_pos(const Eigen::Vector2d& v) const { return grid_to_pos(v.x(), v.y()); }
    /**
     * @brief  グリッドマップ上の位置から実際の位置に変換
     *
     * @param x グリッドマップ上のx
     * @param y グリッドマップ上のy
     * @return Eigen::Vector2d 実際の位置
     */
    Eigen::Vector2d grid_to_pos(const double& x, const double& y) const {
      Eigen::Vector2d pos, index_origin;
      index_origin.x() = std::round(-info.origin.x() / info.resolution);
      index_origin.y() = std::round(-info.origin.y() / info.resolution);
      pos.x()          = (x - index_origin.x()) * info.resolution;
      pos.y()          = (y - index_origin.y()) * info.resolution;
      return pos;
    }
    /**
     * @brief  マップ上に存在するか
     *
     * @param Eigen::Vector2d グリッドマップ上の位置
     * @return bool
     */
    bool is_contain(const Eigen::Vector2d& v) const {
      if (0 <= v.x() && v.x() < row() && 0 <= v.y() && v.y() < col())
        return true;
      else
        return false;
    }

    /**
     * @brief  行の最大取得
     *
     * @return size_t
     */
    size_t row() const { return info.height; }
    /**
     * @brief  列の最大取得
     *
     * @return size_t
     */
    size_t col() const { return info.width; }
    /**
     * @brief  cellに値を代入
     *
     * @param Vector2d v
     * @param int8_t value
     */
    void set(const double& x, const double& y, int8_t value) { data[y * col() + x] = value; }
    void set(const Eigen::Vector2d& v, int8_t value) { set(v.x(), v.y(), value); }

    int8_t at(const int x, const int y) const { return data[y * col() + x]; }
    int8_t operator()(const int x, const int y) const { return at(x, y); }
    int8_t& operator()(const int x, const int y) { return data[y * col() + x]; }

    int8_t at(const Eigen::Vector2d& v) const { return at(v.x(), v.y()); }
    int8_t operator()(const Eigen::Vector2d& v) const { return at(v.x(), v.y()); }
    int8_t& operator()(const Eigen::Vector2d& v) { return data[v.y() * col() + v.x()]; }
  };
} // namespace common_lib