#ifndef VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_
#define VEHICLE_INFO_UTILS__VEHICLE_INFO_HPP_
// 替换为这些标准C++头文件
#include <cmath>     // 提供数学函数 (sin, cos, sqrt等)
#include <cstring>   // 提供memcpy
#include <algorithm> // 提供std::min, std::max等
#include <cstddef>

struct VehicleInfo
{
  double wheel_radius_m{};       //<! should be positive
  double wheel_width_m{};        //<! should be positive
  double wheel_base_m{};         //<! should be positive
  double wheel_tread_m{};        //<! should be positive
  double front_overhang_m{};     //<! should be positive
  double rear_overhang_m{};      //<! should be positive
  double left_overhang_m{};      //<! should be positive
  double right_overhang_m{};     //<! should be positive
  double vehicle_height_m{};     //<! should be positive
  double max_steer_angle_rad{};  //<! should be positive

  double vehicle_length_m{};
  double vehicle_width_m{};
  double min_longitudinal_offset_m{};
  double max_longitudinal_offset_m{};
  double min_lateral_offset_m{};
  double max_lateral_offset_m{};
  double min_height_offset_m{};
  double max_height_offset_m{};

  static constexpr size_t FrontLeftIndex = 0;   //<! the point index of front-left edge == 0
  static constexpr size_t FrontRightIndex = 1;  //<! the point index of front-right > front-left
  static constexpr size_t RearRightIndex = 3;   //<! the point index of rear-right > front-right
  static constexpr size_t RearLeftIndex = 4;    //<! the point index of rear-left > rear-right

  double calcMaxCurvature() const;
  double calcCurvatureFromSteerAngle(const double steer_angle) const;
  double calcSteerAngleFromCurvature(const double curvature) const;


  /// Create vehicle info from base parameters
  VehicleInfo createVehicleInfo(
    const double wheel_radius_m, const double wheel_width_m, const double wheel_base_m,
    const double wheel_tread_m, const double front_overhang_m, const double rear_overhang_m,
    const double left_overhang_m, const double right_overhang_m, const double vehicle_height_m,
    const double max_steer_angle_rad);
};



#endif
