#ifndef PLANNER_VISUALIZER_HPP_
#define PLANNER_VISUALIZER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>
#include <vector>
#include <memory>

#include "emplanner/reference_line/reference_line.hpp"

namespace emplanner {

class PlannerVisualizer {
public:
  explicit PlannerVisualizer(rclcpp::Node* node);
  
  // 可视化全局路径
  void visualizePathPoints(
      const std::vector<std::pair<double, double>>& path_points,
      const std::string& frame_id = "map",
      const std::array<float, 3>& color = {0.0, 1.0, 0.0});

  void visualizeMapPoints(
      const std::vector<MapPoint>& path_points,
      const std::string& frame_id = "map",
      const std::array<float, 3>& color = {0.3, 0.4, 0.3});

  // 可视化参考线
  void visualizeReferenceLine(
      const ReferenceLine& reference_line,
      const std::string& frame_id = "map",
      const std::array<float, 3>& color = {0.0, 0.0, 1.0});

    void visualizePlannedLine(
        const ReferenceLine& reference_line,
        const std::string& frame_id = "map",
        const std::array<float, 3>& color = {0.5, 0.5, 0.5});
  
  // 可视化起点和终点
  void visualizeStartGoal(
      const double start_x, const double start_y,
      const double goal_x, const double goal_y,
      const std::string& frame_id = "map");
  
  // 可视化障碍物
  void visualizeObstacles(
      const std::vector<std::pair<double, double>>& obstacle_points,
      const std::string& frame_id = "map",
      const std::array<float, 3>& color = {1.0, 0.0, 0.0});
  
  // 清除所有可视化标记
  void clear();

private:
  // 生成路径线条标记
  visualization_msgs::msg::Marker createPathMarker(
      const std::vector<std::pair<double, double>>& path_points,
      const std::string& ns,
      int id,
      const std::array<float, 3>& rgb_color,
      const std::string& frame_id,
      float line_width = 0.1);
  
  // 生成点标记
  visualization_msgs::msg::Marker createPointMarker(
      const double x, const double y,
      const std::string& ns,
      int id,
      const std::array<float, 3>& rgb_color,
      const std::string& frame_id,
      float scale = 0.3);

  rclcpp::Node* node_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  visualization_msgs::msg::MarkerArray current_markers_;
  int marker_id_;
};

} // namespace emplanner

#endif // PLANNER_VISUALIZER_HPP_