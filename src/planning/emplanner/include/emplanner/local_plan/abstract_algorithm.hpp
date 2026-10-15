#ifndef ABSTRACT_ALGORITHM_HPP_
#define ABSTRACT_ALGORITHM_HPP_

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>

#include "emplanner/common/vehicle_info.hpp"

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <limits>
#include <vector>
#include <cmath>

// 角度标准化
inline double normalizeRadian(const double rad, const double min_rad = -3.14159265358979323846)
{
  auto pi = 3.14159265358979323846;
  const auto max_rad = min_rad + 2 * pi;

  const auto value = std::fmod(rad, 2 * pi);
  if (min_rad <= value && value < max_rad) {
    return value;
  }

  return value - std::copysign(2 * pi, value);
}

/**
 * @brief 坐标变换函数 - 将给定的位姿通过变换矩阵进行转换
 * @param pose 需要转换的位姿
 * @param transform 变换矩阵
 * @return 转换后的位姿
 */
geometry_msgs::msg::Pose transformPose(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform);

/**
 * @brief 角度离散化函数 - 将连续角度值离散化为指定大小的索引
 * @param theta 连续角度值(弧度)
 * @param theta_size 离散化后的角度总数
 * @return 离散化后的角度索引
 */
int discretizeAngle(const double theta, const int theta_size);

/**
 * @brief 三维索引结构体(x,y,theta)，表示位置和方向
 */
struct IndexXYT
{
  int x;     // X方向网格索引
  int y;     // Y方向网格索引
  int theta; // 角度方向索引

  bool operator==(const IndexXYT & index) const{
    return (x == index.x && y == index.y && theta == index.theta);
  }
};

/**
 * @brief 二维索引结构体(x,y)，表示位置
 */
struct IndexXY
{
  int x;  // X方向网格索引
  int y;  // Y方向网格索引
};

/**
 * @brief 将位姿转换为栅格地图上的三维索引
 * @param costmap 代价地图
 * @param pose_local 局部坐标系下的位姿
 * @param theta_size 角度离散化大小
 * @return 对应的三维索引
 */
IndexXYT pose2index(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local,
  const int theta_size);

/**
 * @brief 将三维索引转换为位姿
 * @param costmap 代价地图
 * @param index 三维索引
 * @param theta_size 角度离散化大小
 * @return 对应的位姿
 */
geometry_msgs::msg::Pose index2pose(
  const nav_msgs::msg::OccupancyGrid & costmap, const IndexXYT & index, const int theta_size);

/**
 * @brief 全局坐标系转局部坐标系
 * @param costmap 代价地图
 * @param pose_global 全局坐标系下的位姿
 * @return 局部坐标系下的位姿
 */
geometry_msgs::msg::Pose global2local(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_global);

/**
 * @brief 局部坐标系转全局坐标系
 * @param costmap 代价地图
 * @param pose_local 局部坐标系下的位姿
 * @return 全局坐标系下的位姿
 */
geometry_msgs::msg::Pose local2global(
  const nav_msgs::msg::OccupancyGrid & costmap, const geometry_msgs::msg::Pose & pose_local);

/**
 * @brief 车辆形状参数结构体
 * 包含车辆几何尺寸和运动学特性参数
 */
struct VehicleShape
{
  double length;       // 车辆长度 X [m]
  double width;        // 车辆宽度 Y [m]
  double base_length;  // 轴距 [m]
  double max_steering; // 最大转向角 [rad]
  double base2back;    // 后轴到车辆后部的距离 [m]
  double min_dimension; // 车辆最小尺寸 [m]
  double max_dimension; // 车辆最大尺寸 [m]

  VehicleShape() = default;

  VehicleShape(double length, double width, double base_length, double max_steering, double base2back)
  : length(length),width(width),base_length(base_length),max_steering(max_steering),base2back(base2back){
    setMinMaxDimension();
  }

  /**
   * @brief 构造函数 - 通过车辆信息对象初始化
   * @param vehicle_info 车辆信息
   * @param margin 安全边距 [m]
   */
  explicit VehicleShape(
    const VehicleInfo & vehicle_info, const double margin = 0.0)
  : length(vehicle_info.vehicle_length_m + margin),
    width(vehicle_info.vehicle_width_m + margin),
    base_length(vehicle_info.wheel_base_m),
    max_steering(vehicle_info.max_steer_angle_rad),
    base2back(vehicle_info.rear_overhang_m + margin / 2.0)
  {
    setMinMaxDimension();
  }

  /**
   * @brief 设置车辆最小和最大尺寸
   * 用于碰撞检测和路径规划
   */
  void setMinMaxDimension()
  {
    const auto base2front = length - base2back;
    if (base2back <= base2front) {
      min_dimension = std::min(0.5 * width, base2back);
      max_dimension = std::hypot(base2front, 0.5 * width);
    } else {
      min_dimension = std::min(0.5 * width, base2front);
      max_dimension = std::hypot(base2back, 0.5 * width);
    }
  }
};

struct PlannerCommonParam
{
  // 基础配置
  double time_limit;  // 规划时间限制 [msec]

  // 搜索配置
  int theta_size;                  // 角度离散化表大小 [-]
  double curve_weight;             // 曲线移动代价权重 [-]
  double reverse_weight;           // 后退移动代价权重 [-]
  double direction_change_weight;  // 方向变化代价权重 [-]
  double lateral_goal_range;       // 到达目标的横向误差阈值 [m]
  double longitudinal_goal_range;  // 到达目标的纵向误差阈值 [m]
  double angle_goal_range;         // 到达目标的角度误差阈值 [deg]
  double max_turning_ratio;        // 最大转向范围系数 [-]
  int turning_steps;               // 转向步数 [-]

  // 代价地图配置
  int obstacle_threshold;  // 障碍物阈值 [-]
};

struct PlannerWaypoint
{
  geometry_msgs::msg::PoseStamped pose; // 路径点位姿
  bool is_back = false;                 // 是否为后退点
};

struct PlannerWaypoints
{
  std_msgs::msg::Header header;        // 消息头
  std::vector<PlannerWaypoint> waypoints; // 路径点数组

  double compute_length() const;
};

/**
 * @brief 欧氏距离变换数据结构体
 * 存储到最近障碍物的距离和角度信息
 */
struct EDTData
{
  double distance; // 到最近障碍物的距离 [m]
  double angle;    // 到最近障碍物的角度 [rad]
};

class AbstractPlanningAlgorithm
{
public:
  AbstractPlanningAlgorithm(
    const PlannerCommonParam & planner_common_param, const rclcpp::Clock::SharedPtr & clock,
    const VehicleShape & collision_vehicle_shape)
  : planner_common_param_(planner_common_param),
    collision_vehicle_shape_(collision_vehicle_shape),
    clock_(clock)
  {
    planner_common_param_.turning_steps = std::max(planner_common_param_.turning_steps, 1);
    collision_vehicle_shape_.max_steering *= planner_common_param_.max_turning_ratio;
    is_collision_table_initialized = false;
  }

  AbstractPlanningAlgorithm(
    const PlannerCommonParam & planner_common_param,
    const VehicleInfo & vehicle_info,
    const rclcpp::Clock::SharedPtr & clock, const double margin = 0.0)
  : planner_common_param_(planner_common_param),
    collision_vehicle_shape_(vehicle_info, margin),
    clock_(clock)
  {
    planner_common_param_.turning_steps = std::max(planner_common_param_.turning_steps, 1);
    collision_vehicle_shape_.max_steering *= planner_common_param_.max_turning_ratio;
  }

  virtual void setMap(const nav_msgs::msg::OccupancyGrid & costmap);
  
  /**
   * @brief 规划从起点到终点的路径
   * @param start_pose 起始位姿
   * @param goal_pose 目标位姿
   * @return 规划是否成功
   */
  virtual bool makePlan(
    const geometry_msgs::msg::Pose & start_pose, const geometry_msgs::msg::Pose & goal_pose) = 0;
  
  /**
   * @brief 检查轨迹上是否存在障碍物
   * @param trajectory 位姿轨迹
   * @return 是否存在障碍物
   */
  virtual bool hasObstacleOnTrajectory(const geometry_msgs::msg::PoseArray & trajectory) const;
  
  /**
   * @brief 获取规划的路径点
   * @return 规划的路径点集合
   */
  const PlannerWaypoints & getWaypoints() const { return waypoints_; }
  
  /**
   * @brief 获取指定位姿到最近障碍物的距离
   * @param pose 查询位姿
   * @return 到最近障碍物的距离 [m]
   */
  double getDistanceToObstacle(const geometry_msgs::msg::Pose & pose) const;

  /**
   * @brief 虚析构函数
   */
  virtual ~AbstractPlanningAlgorithm() {}

protected:
  /**
   * @brief 计算给定角度索引的碰撞检测索引集合
   * @param theta_index 角度索引
   * @param indexes 输出的碰撞检测索引集合
   * @param vertex_indexes_2d 输出的车辆顶点索引集合
   */
  void computeCollisionIndexes(
    int theta_index, std::vector<IndexXY> & indexes,
    std::vector<IndexXY> & vertex_indexes_2d) const;
  
  /**
   * @brief 检测是否超出地图边界
   * @param base_index 基准索引
   * @return 是否超出边界
   */
  bool detectBoundaryExit(const IndexXYT & base_index) const;
  
  /**
   * @brief 检测给定索引位置是否发生碰撞
   * @param base_index 基准索引
   * @return 是否碰撞
   */
  bool detectCollision(const IndexXYT & base_index) const;
  
  /**
   * @brief 检测给定位姿是否发生碰撞
   * @param base_pose 基准位姿
   * @return 是否碰撞
   */
  bool detectCollision(const geometry_msgs::msg::Pose & base_pose) const;

  /**
   * @brief 计算欧氏距离变换地图
   * 用于快速查询到障碍物的距离
   */
  void computeEDTMap();

  /**
   * @brief 检查索引是否超出地图范围
   * @param index 需要检查的索引
   * @return 是否超出范围
   */
  template <typename IndexType>
  inline bool isOutOfRange(const IndexType & index) const
  {
    if (index.x < 0 || static_cast<int>(costmap_.info.width) <= index.x) {
      return true;
    }
    if (index.y < 0 || static_cast<int>(costmap_.info.height) <= index.y) {
      return true;
    }
    return false;
  }

  /**
   * @brief 检查索引是否在安全边界内
   * 
   * 判断给定的索引是否处于地图的安全边界内。
   * 所谓安全边界，就是距离地图边缘至少有 nb_of_margin_cells_ 个单元格的区域。
   * 
   * @param index 需要检查的索引
   * @return 如果索引在安全边界内，返回true；否则返回false
   */
  template <typename IndexType>
  inline bool isWithinMargin(const IndexType & index) const
  {
    if (
      index.x < nb_of_margin_cells_ ||
      static_cast<int>(costmap_.info.width) - index.x < nb_of_margin_cells_) {
      return false;
    }
    if (
      index.y < nb_of_margin_cells_ ||
      static_cast<int>(costmap_.info.height) - index.y < nb_of_margin_cells_) {
      return false;
    }
    return true;
  }

  /**
   * @brief 判断给定索引是否为障碍物
   * 
   * 根据障碍物表格，判断指定索引位置是否是障碍物
   * 
   * @param index 需要检查的索引
   * @return 是否为障碍物
   */
  template <typename IndexType>
  inline bool isObs(const IndexType & index) const
  {
    return is_obstacle_table_[indexToId(index)];
  }

  /**
   * @brief 获取给定索引位置的欧氏距离变换数据
   * @param index 需要查询的索引
   * @return EDT数据(距离和角度)
   */
  template <typename IndexType>
  inline EDTData getObstacleEDT(const IndexType & index) const
  {
    return edt_map_[indexToId(index)];
  }

  /**
   * @brief 将二维索引转换为一维索引
   * 
   * 根据二维索引计算一维索引，使用公式: y * width + x
   * 
   * @param index 二维索引
   * @return 对应的一维索引
   */
  template <typename IndexType>
  inline int indexToId(const IndexType & index) const
  {
    return index.y * costmap_.info.width + index.x;
  }

  /**
   * @brief 计算从车辆基准点到边框的距离
   * 
   * 根据给定角度，计算从车辆基准点到车辆边框的距离
   * 
   * @param angle 角度 [rad]
   * @return 距离 [m]
   */
  inline double getVehicleBaseToFrameDistance(const double angle) const
  {
    const double normalized_angle = std::abs(normalizeRadian(angle));
    const double w = 0.5 * collision_vehicle_shape_.width;
    const double l_b = collision_vehicle_shape_.base2back;
    const double l_f = collision_vehicle_shape_.length - l_b;

    if (normalized_angle < atan(w / l_f)) return l_f / cos(normalized_angle);
    if (normalized_angle < M_PI_2) return w / sin(normalized_angle);
    if (normalized_angle < M_PI_2 + atan(l_b / w)) return w / cos(normalized_angle - M_PI_2);
    return l_b / cos(M_PI - normalized_angle);
  }

  PlannerCommonParam planner_common_param_;      // 规划器通用参数
  VehicleShape collision_vehicle_shape_;         // 用于碰撞检测的车辆形状

  // 父节点指针
  rclcpp::Clock::SharedPtr clock_;               // ROS时钟指针

  // 代价地图
  nav_msgs::msg::OccupancyGrid costmap_;         // 占用栅格地图

  // 碰撞索引缓存
  std::vector<std::vector<IndexXY>> coll_indexes_table_;    // 不同角度的碰撞检测索引表

  // 车辆顶点索引缓存
  std::vector<std::vector<IndexXY>> vertex_indexes_table_;  // 不同角度的车辆顶点索引表

  // 障碍物表格
  std::vector<bool> is_obstacle_table_;          // 标记每个栅格是否为障碍物

  // 欧氏距离变换地图(到最近障碍物单元格的距离和角度信息)
  std::vector<EDTData> edt_map_;                 // EDT地图数据

  // 代价地图坐标系下的位姿
  geometry_msgs::msg::Pose start_pose_;          // 起点位姿
  geometry_msgs::msg::Pose goal_pose_;           // 终点位姿

  // 碰撞表是否初始化
  bool is_collision_table_initialized;           // 碰撞表初始化标志

  // 结果路径
  PlannerWaypoints waypoints_;                   // 规划路径点

  int nb_of_margin_cells_;                       // 边界安全边距单元格数量
};

#endif  // AUTOWARE__FREESPACE_PLANNING_ALGORITHMS__ABSTRACT_ALGORITHM_HPP_
