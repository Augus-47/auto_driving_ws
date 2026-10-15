#pragma once

#include "reference_line_smoother.hpp"
#include <float.h>
#include <math.h>
#include <memory>

/**
 * 参考线提供者类
 */
class ReferenceLineProvider {
public:

  explicit ReferenceLineProvider();

  // 类的主功能函数，由全局路径，定位信息，上一时刻的参考线信息，生成新的参考线
  void Provide(const std::vector<MapPoint> &routing_path_points,
               const LocalizationInfo &localzation_info,
               const ReferenceLine &pre_reference_line,
               ReferenceLine &reference_line);
  
  // 获取参考线
  const ReferenceLine get_raw_reference_line() const;
  const ReferenceLine get_smoothed_reference_line() const;
  
  // 给定自车位置与参考线 寻找匹配点，投影点
  static int FindMatchIndex(
    const std::vector<MapPoint> &routing_path_points, const MapPoint &map_points,
    const int index_start_search, const int increase_count_limit);
  
  // 参考线转为地图点格式
  static void referce_to_map_point(
    const ReferenceLine &reference_line,
    std::vector<MapPoint> &routing_path_points);

  // 根据自车位置和参考线，计算投影点
  static void CalculateProjectPoint(
    const std::vector<ReferencePoint> &reference_points,
    const int host_match_index,
    const MapPoint &map_points,
    ReferencePoint &project_point);

private:
  
  // 根据匹配点获取局部参考线
  void GetReferenceLine(
    const std::vector<MapPoint> &routing_path_points,
    const int host_match_point_index, // 车辆在全局path的索引
    std::vector<MapPoint> &raw_reference_points,
    int &host_match_index);

  // 全局路径转换参考线格式，即(x,y)->(x,y,heading,kappa)
  void RoutingPathToFrenetPath(
    const std::vector<MapPoint> &routing_path_points,
    ReferenceLine *frenet_path);

  

  ReferenceLine frenet_path_;           // 参考线
  std::vector<MapPoint> pre_points_;    // 用于存储上一时刻，传入该模块的待处理点
  bool is_first_run_ = false;           //是否是首次运行
  ReferenceLine raw_reference_line_;    // 传入的原始参考线
  ReferenceLine smoothed_reference_line_; // 平滑后的参考线
  ReferencePoint host_project_point_;   // 自车投影点
  ReferencePoint host_match_point_;     // 自车匹配点  

  ReferenceLineSmootherConfig smoother_config_; //平滑配置
  ReferenceLineSmoother smoother_; //平滑器
};
