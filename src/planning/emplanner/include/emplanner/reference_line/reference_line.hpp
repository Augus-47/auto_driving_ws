#pragma once
#include "emplanner/common/pnc_point.hpp"
#include <vector>

/**
 * 参考线类
 * 主要实现参考线属性的设置和获取
 * 包含：参考线点，当前匹配点，当前投影点，当前匹配点索引
 */
class ReferenceLine {
public:
  ReferenceLine() = default;
  ~ReferenceLine() = default;
  
  void set_reference_points(std::vector<ReferencePoint> reference_points);
  void set_match_point_index(int index);
  void set_host_match_point(ReferencePoint host_match_point);
  void set_host_project_point(ReferencePoint host_project_point);

  const std::vector<ReferencePoint> get_reference_points() const;
  const int get_match_point_index() const;
  const ReferencePoint get_host_project_point() const;
  const ReferencePoint get_host_match_point() const;

private:
  std::vector<ReferencePoint> reference_points_;    //参考点类
  int match_point_index_;                           //自车匹配点索引
  ReferencePoint host_project_point_;               //自车投影点
  ReferencePoint host_match_point_;                 //自车匹配点
};