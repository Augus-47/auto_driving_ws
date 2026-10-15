#include "emplanner/common/planner_visualizer.hpp"

namespace emplanner {

PlannerVisualizer::PlannerVisualizer(rclcpp::Node* node)
: node_(node), marker_id_(0) {
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    "/planner_visualization", 10);
}

// 可视化规划的全局路径点
void PlannerVisualizer::visualizePathPoints(
    const std::vector<std::pair<double, double>>& path_points,
    const std::string& frame_id,
    const std::array<float, 3>& color) {
  
  if (path_points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "无法可视化空路径");
    return;
  }
  
  auto path_marker = createPathMarker(path_points, "global_path", marker_id_++, color, frame_id, 0.1);
  
  visualization_msgs::msg::MarkerArray marker_array;
  marker_array.markers.push_back(path_marker);
  
  // 添加路径点标记
  for (size_t i = 0; i < path_points.size(); i += 10) {  // 每10个点放置一个路径点
    auto point_marker = createPointMarker(
        path_points[i].first, path_points[i].second,
        "path_points", marker_id_++, {color[0], color[1], color[2]}, frame_id, 0.2);
    marker_array.markers.push_back(point_marker);
  }
  
  marker_pub_->publish(marker_array);
  current_markers_ = marker_array;
  
  RCLCPP_INFO(node_->get_logger(), "已发布全局路径可视化，包含 %zu 个点", path_points.size());
}

void PlannerVisualizer::visualizeMapPoints(
      const std::vector<MapPoint>& path_points,
      const std::string& frame_id,
      const std::array<float, 3>& color){

  if (path_points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "无法可视化空地图点");
    return;
  }

  std::vector<std::pair<double, double>> path_points_vec;
  path_points_vec.reserve(path_points.size());
  for (const auto& point : path_points) {
    path_points_vec.emplace_back(point.x, point.y);
  }

  // 创建标记数组
  visualization_msgs::msg::MarkerArray marker_array;
  auto path_marker = createPathMarker(path_points_vec, "map_path", 900000, color, frame_id, 0.1);
  marker_array.markers.push_back(path_marker);


  // 添加路径点标记
  for (size_t i = 0; i < path_points_vec.size(); i += 1) {  // 每10个点放置一个路径点
    auto point_marker = createPointMarker(
        path_points_vec[i].first, path_points_vec[i].second,
        "map_path", 910000, {color[0], color[1], color[2]}, frame_id, 0.2);
    marker_array.markers.push_back(point_marker);
  }
  
  marker_pub_->publish(marker_array);
  current_markers_ = marker_array;
  
  RCLCPP_INFO(node_->get_logger(), "已发布全局路径可视化，包含 %zu 个点", path_points.size());

}

// 可视化参考线
void PlannerVisualizer::visualizeReferenceLine(
    const ReferenceLine& reference_line,
    const std::string& frame_id,
    const std::array<float, 3>& color){
  
  auto ref_line = reference_line.get_reference_points();
  if (ref_line.empty()) {
    RCLCPP_WARN(node_->get_logger(), "无法可视化空路径");
    return;
  }

  // 删除之前的标记
  // 首先删除旧的参考线标记
  visualization_msgs::msg::MarkerArray delete_markers;
  for (auto& marker : current_markers_.markers) {
    if (marker.ns == "ref_line" || marker.ns == "ref_points") {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      delete_markers.markers.push_back(marker);
    }
  }

  // 如果有标记需要删除，先发布删除消息
  if (!delete_markers.markers.empty()) {
    marker_pub_->publish(delete_markers);
    
    // 从current_markers_中移除已删除的标记
    auto it = std::remove_if(current_markers_.markers.begin(), current_markers_.markers.end(),
      [](const auto& marker) {
        return marker.ns == "ref_line" || marker.ns == "ref_points";
      });
    current_markers_.markers.erase(it, current_markers_.markers.end());
  }

  std::vector<std::pair<double, double>> path_points;
  auto n = ref_line.size();
  path_points.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    path_points[i].first = ref_line[i].x;
    path_points[i].second = ref_line[i].y;
  }

  // 创建新的参考线标记
  visualization_msgs::msg::MarkerArray new_markers;
  auto path_marker = createPathMarker(path_points, "ref_line", 10000, color, frame_id, 0.05);
  new_markers.markers.push_back(path_marker);
  
  
  // 添加路径点标记
  int point_id_base = 20000;  // 从1001开始标记ID
  for (size_t i = 0; i < path_points.size(); i += 1) {  // 每10个点放置一个路径点
    auto point_marker = createPointMarker(
        path_points[i].first, path_points[i].second,
        "ref_points", point_id_base + i, {color[0], color[1], color[2]}, frame_id, 0.1);
    new_markers.markers.push_back(point_marker);
  }
  
  // 发布新标记
  marker_pub_->publish(new_markers);
  // 更新当前标记列表
  for (const auto& marker : new_markers.markers) {
    current_markers_.markers.push_back(marker);
  }
  
  RCLCPP_INFO(node_->get_logger(), "已发布局部路径，包含 %zu 个点", path_points.size());
}


// 可视化规划结果
void PlannerVisualizer::visualizePlannedLine(
    const ReferenceLine& reference_line,
    const std::string& frame_id,
    const std::array<float, 3>& color){
  
  auto ref_line = reference_line.get_reference_points();
  if (ref_line.empty()) {
    RCLCPP_WARN(node_->get_logger(), "规划结果为空");
    return;
  }

  // 删除之前的标记
  // 首先删除旧的参考线标记
  visualization_msgs::msg::MarkerArray delete_markers;
  for (auto& marker : current_markers_.markers) {
    if (marker.ns == "planned_line" || marker.ns == "planned_points") {
      marker.action = visualization_msgs::msg::Marker::DELETE;
      delete_markers.markers.push_back(marker);
    }
  }

  // 如果有标记需要删除，先发布删除消息
  if (!delete_markers.markers.empty()) {
    marker_pub_->publish(delete_markers);
    
    // 从current_markers_中移除已删除的标记
    auto it = std::remove_if(current_markers_.markers.begin(), current_markers_.markers.end(),
      [](const auto& marker) {
        return marker.ns == "planned_line" || marker.ns == "planned_points";
      });
    current_markers_.markers.erase(it, current_markers_.markers.end());
  }

  std::vector<std::pair<double, double>> path_points;
  auto n = ref_line.size();
  path_points.resize(n);
  for (size_t i = 0; i < n; i++)
  {
    path_points[i].first = ref_line[i].x;
    path_points[i].second = ref_line[i].y;
  }

  // 创建新的参考线标记
  visualization_msgs::msg::MarkerArray new_markers;
  auto path_marker = createPathMarker(path_points, "planned_line", 5000, color, frame_id, 0.05);
  new_markers.markers.push_back(path_marker);
  
  
  // 添加路径点标记
  int point_id_base = 8000;  // 从8001开始标记ID
  for (size_t i = 0; i < path_points.size(); i += 10) {  // 每10个点放置一个路径点
    auto point_marker = createPointMarker(
        path_points[i].first, path_points[i].second,
        "planned_points", point_id_base + i/10, {color[0], color[1], color[2]}, frame_id, 0.1);
    new_markers.markers.push_back(point_marker);
  }
  
  // 发布新标记
  marker_pub_->publish(new_markers);
  // 更新当前标记列表
  for (const auto& marker : new_markers.markers) {
    current_markers_.markers.push_back(marker);
  }
  
  RCLCPP_INFO(node_->get_logger(), "已发布局部路径，包含 %zu 个点", path_points.size());
}


// 可视化起点和终点
void PlannerVisualizer::visualizeStartGoal(
    const double start_x, const double start_y,
    const double goal_x, const double goal_y,
    const std::string& frame_id) {
  
  visualization_msgs::msg::MarkerArray marker_array;
  
  // 起点标记 - 绿色大球
  auto start_marker = createPointMarker(
      start_x, start_y, "waypoints", marker_id_++, {0.0, 1.0, 0.0}, frame_id, 0.5);
  marker_array.markers.push_back(start_marker);
  
  // 终点标记 - 红色大球
  auto goal_marker = createPointMarker(
      goal_x, goal_y, "waypoints", marker_id_++, {1.0, 0.0, 0.0}, frame_id, 0.5);
  marker_array.markers.push_back(goal_marker);
  
  marker_pub_->publish(marker_array);
  
  // 合并到当前标记中
  for (const auto& marker : marker_array.markers) {
    current_markers_.markers.push_back(marker);
  }
  
  RCLCPP_INFO(node_->get_logger(), "已发布起点和终点可视化");
}

void PlannerVisualizer::visualizeObstacles(
    const std::vector<std::pair<double, double>>& obstacle_points,
    const std::string& frame_id,
    const std::array<float, 3>& color) {
  
  if (obstacle_points.empty()) {
    RCLCPP_WARN(node_->get_logger(), "无法可视化空障碍物点");
    return;
  }
  
  visualization_msgs::msg::MarkerArray marker_array;
  
  for (const auto& point : obstacle_points) {
    auto obstacle_marker = createPointMarker(
        point.first, point.second, "obstacles", marker_id_++, color, frame_id, 0.25);
    marker_array.markers.push_back(obstacle_marker);
  }
  
  marker_pub_->publish(marker_array);
  
  // 合并到当前标记中
  for (const auto& marker : marker_array.markers) {
    current_markers_.markers.push_back(marker);
  }
  
  RCLCPP_INFO(node_->get_logger(), "已发布障碍物可视化，包含 %zu 个点", obstacle_points.size());
}

void PlannerVisualizer::clear() {
  visualization_msgs::msg::MarkerArray clear_markers;
  
  for (auto& marker : current_markers_.markers) {
    marker.action = visualization_msgs::msg::Marker::DELETE;
    clear_markers.markers.push_back(marker);
  }
  
  if (!clear_markers.markers.empty()) {
    marker_pub_->publish(clear_markers);
    current_markers_.markers.clear();
    RCLCPP_INFO(node_->get_logger(), "已清除所有可视化标记");
  }
  
  marker_id_ = 0;
}

visualization_msgs::msg::Marker PlannerVisualizer::createPathMarker(
    const std::vector<std::pair<double, double>>& path_points,
    const std::string& ns,
    int id,
    const std::array<float, 3>& rgb_color,
    const std::string& frame_id,
    float line_width) {
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.scale.x = line_width;  // 线宽
  
  marker.color.r = rgb_color[0];
  marker.color.g = rgb_color[1];
  marker.color.b = rgb_color[2];
  marker.color.a = 1.0;
  
  for (const auto& point : path_points) {
    geometry_msgs::msg::Point p;
    p.x = point.first;
    p.y = point.second;
    p.z = 0.1;  // 略高于地平面
    marker.points.push_back(p);
  }
  
  marker.pose.orientation.w = 1.0;
  marker.lifetime = rclcpp::Duration(0, 0);  // 永久显示
  
  return marker;
}

visualization_msgs::msg::Marker PlannerVisualizer::createPointMarker(
    const double x, const double y,
    const std::string& ns,
    int id,
    const std::array<float, 3>& rgb_color,
    const std::string& frame_id,
    float scale) {
  
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = node_->now();
  marker.ns = ns;
  marker.id = id;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.1;  // 略高于地平面
  
  marker.pose.orientation.w = 1.0;
  
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = scale;
  
  marker.color.r = rgb_color[0];
  marker.color.g = rgb_color[1];
  marker.color.b = rgb_color[2];
  marker.color.a = 1.0;
  
  marker.lifetime = rclcpp::Duration(0, 0);  // 永久显示
  
  return marker;
}

} // namespace emplanner