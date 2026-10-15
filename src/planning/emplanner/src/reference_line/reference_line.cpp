#include "emplanner/reference_line/reference_line.hpp"

void ReferenceLine::set_reference_points(std::vector<ReferencePoint> reference_points) {
  reference_points_ = reference_points;
}

const std::vector<ReferencePoint> ReferenceLine::get_reference_points() const {
  return reference_points_;
}

void ReferenceLine::set_match_point_index(int index) {
  match_point_index_ = index;
}

// 设置投影点
void ReferenceLine::set_host_project_point(ReferencePoint host_project_point) {
  host_project_point_ = host_project_point;
}

// 设置匹配点
void ReferenceLine::set_host_match_point(ReferencePoint host_match_point) {
  host_match_point_ = host_match_point;
}

const int ReferenceLine::get_match_point_index() const {
  return match_point_index_;
}

const ReferencePoint ReferenceLine::get_host_project_point() const {
  return host_project_point_;
}

const ReferencePoint ReferenceLine::get_host_match_point() const {
  return host_match_point_;
}