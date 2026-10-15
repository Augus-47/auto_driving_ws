#include "emplanner/reference_line/reference_line_provider.hpp"
#include <iostream>

ReferenceLineProvider::ReferenceLineProvider() {}


// 角度归一化到[-pi, pi]
double normalize_angle(double angle) {
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

// 提供参考线
// 参数(路径点， 定位信息， 前一时刻参考线)
// 返回(当前时刻参考线)
void ReferenceLineProvider::Provide(
    const std::vector<MapPoint> &routing_path_points,
    const LocalizationInfo &localzation_info,
    const ReferenceLine &pre_reference_line, ReferenceLine &reference_line){
  
  // 检查输入路径有效性
  if (routing_path_points.size() < 2) {
    return;
  }

  // 1. 当前路径，位置
  auto path_points = routing_path_points;
  MapPoint host_xy{localzation_info.x, localzation_info.y};
  

  // 2. 匹配点索引
  int host_match_point_index = -1;
  if(pre_reference_line.get_reference_points().empty()){
    host_match_point_index = FindMatchIndex(path_points, host_xy, 0, 10);
  } else {
    int pre_match_point_index = pre_reference_line.get_match_point_index();
    host_match_point_index = FindMatchIndex(path_points, host_xy, pre_match_point_index, 5);
  }

  // 3. 局部参考线, 车辆在局部参考线中的匹配点索引
  std::vector<MapPoint> raw_reference_points;
  int host_match_index = -1; 
  GetReferenceLine(path_points, host_match_point_index, raw_reference_points, host_match_index);

  // 4. 参考线平滑
  bool is_smooth = false;
  std::vector<MapPoint> smoothed_reference_points;
  if(is_smooth){
    smoother_ = ReferenceLineSmoother(smoother_config_);
    smoother_.Smooth(raw_reference_points, smoothed_reference_points);
  }
  smoothed_reference_points = raw_reference_points; // 如果不平滑，直接使用原始参考线
  

  // 5. 为参考线添加其他属性
  // 5.1 添加heading，kappa
  RoutingPathToFrenetPath(smoothed_reference_points, &reference_line);

  // 5.2 设置匹配点
  reference_line.set_match_point_index(host_match_index);
  reference_line.set_host_match_point(reference_line.get_reference_points()[host_match_index]);

  // 5.3 计算并设置投影点
  ReferencePoint project_point;
  CalculateProjectPoint(reference_line.get_reference_points(), host_match_index, host_xy, project_point);
  reference_line.set_host_project_point(project_point);

  smoothed_reference_line_ = reference_line;
}

// 1.寻找匹配点(返回匹配点索引,最近点)
int ReferenceLineProvider::FindMatchIndex(
    const std::vector<MapPoint> &routing_path_points, const MapPoint &map_points,
    const int index_start_search, const int increase_count_limit) {

  int index = -1;
  
  auto frenet_path_points = routing_path_points;
  int size = frenet_path_points.size();
  int increase_count = 0;
  
  // 遍历全局路径，寻找匹配点(距离最近)
  double min_distance = DBL_MAX;
  for(int i = index_start_search; i < size; i++){
    double distance = std::hypot(map_points.x - frenet_path_points[i].x, map_points.y - frenet_path_points[i].y);
    if(distance < min_distance){
      min_distance = distance;
      index = i;// 匹配点索引
      increase_count = 0;
    } else {
      increase_count++;
    }

    // 如果连续增加超过限制，则停止搜索，认为匹配点为当前点
    if(increase_count > increase_count_limit)
      break;
  }

  return index;

}

void ReferenceLineProvider::referce_to_map_point(const ReferenceLine &reference_line, std::vector<MapPoint> &routing_path_points)
{
  // 将参考线转换为地图点格式
  std::vector<MapPoint> map_points;
  const auto &reference_points = reference_line.get_reference_points();
  map_points.reserve(reference_points.size());

  for (const auto &ref_point : reference_points) {
    MapPoint map_point;
    map_point.x = ref_point.x;
    map_point.y = ref_point.y;
    map_points.push_back(map_point);
  }

  // 更新全局路径点
  routing_path_points = map_points;
} 

// 2.根据自车位置的匹配点，截取参考线
void ReferenceLineProvider::GetReferenceLine(
    const std::vector<MapPoint> &routing_path_points,
    const int host_match_point_index, // 车辆在全局path的索引
    std::vector<MapPoint> &raw_reference_points,
    int &host_match_index) {
  
  std::cout << "匹配点索引 :" << host_match_point_index << std::endl;
  int kPreviewPoints = 2;  // 已经走过的点数
  int kTotalPoints = 20;   // 总截取点数16*0.5=8m
  
  // 计算起始点索引
  int start_index = 0;
  host_match_index = -1;//参考线截取后，车辆在新的参考线中的索引
  int len = routing_path_points.size();
  
  // 匹配点后方不够 一定点数，从头开始截取
  if (host_match_point_index < kPreviewPoints) {
    start_index = 0;
    host_match_index = host_match_point_index;
  }else {
    start_index = host_match_point_index - kPreviewPoints;
    host_match_index = kPreviewPoints;
  }

  // 截取路径点
  int end_index = std::min(start_index + kTotalPoints, len);
  raw_reference_points.assign(
      routing_path_points.begin() + start_index,
      routing_path_points.begin() + end_index);
}

// 5.1 由全局路径转换参考线，即(x,y)->(x,y,heading,kappa)
void ReferenceLineProvider::RoutingPathToFrenetPath(
    const std::vector<MapPoint> &routing_path_points,
    ReferenceLine *frenet_path) {
  auto points = routing_path_points;
  int size = points.size();

  if (size < 2) {
    return; //路径点数小于2，无法计算参考线
  }

  std::vector<ReferencePoint> reference_points;

  // 计算相邻点之间的距离和heading
  std::vector<double> ds(size - 1);
  std::vector<double> heading(size - 1);
  for (int i = 0; i < size - 1; i++) {
    double dx = points[i + 1].x - points[i].x;
    double dy = points[i + 1].y - points[i].y;
    ds[i] = std::hypot(dx, dy);
    heading[i] = std::atan2(dy, dx);
  }

  // 处理每个点的属性
  for (int i = 0; i < size; i++) {
    ReferencePoint rp;
    rp.x = points[i].x;
    rp.y = points[i].y;
    rp.index = i;

    // 处理航向
    if (i == 0) {
      rp.heading = heading[i]; // 起点使用第一个航向
    } else if (i == size - 1) {
      rp.heading = heading[i - 1]; // 终点使用最后一个航向
    } else { // 中间点使用前后航向的平均值
      rp.heading = (heading[i] + heading[i - 1]) / 2;
    }

    // 处理曲率 
    if (i > 0 && i < size - 1) {
      // 角度归一化
      double delta_heading = normalize_angle(heading[i] - heading[i - 1]);
      rp.kappa = std::sin(delta_heading) / ((ds[i - 1] + ds[i]) / 2.0);
    } else {
      rp.kappa = 0.0;// 起点和终点为0
    }

    // 添加参考点
    reference_points.push_back(rp);
  }

  // 设置参考点
  frenet_path->set_reference_points(reference_points);
}

// 5.3 计算投影点
void ReferenceLineProvider::CalculateProjectPoint(
    const std::vector<ReferencePoint> &reference_points,
    const int host_match_index,
    const MapPoint &map_points,
    ReferencePoint &project_point) {

  // 计算匹配点的方向向量
  ReferencePoint match_points = reference_points[host_match_index];
  double dir_x = cos(match_points.heading);
  double dir_y = sin(match_points.heading);

  // 计算待投影点与匹配点的向量差
  double dx = map_points.x - match_points.x;
  double dy = map_points.y - match_points.y;

  // 计算投影点
  double ds = dx * dir_x + dy * dir_y;
  project_point.x = match_points.x + ds * dir_x;
  project_point.y = match_points.y + ds * dir_y;
  project_point.heading = match_points.heading + match_points.kappa * ds;
  project_point.kappa = match_points.kappa;
}

const ReferenceLine ReferenceLineProvider::get_raw_reference_line() const {
  return raw_reference_line_;
}
const ReferenceLine ReferenceLineProvider::get_smoothed_reference_line() const {
  return smoothed_reference_line_;
}
