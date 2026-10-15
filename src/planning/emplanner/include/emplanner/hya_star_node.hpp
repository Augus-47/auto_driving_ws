#pragma once

#include "emplanner/planner/a_star.hpp"
#include "emplanner/local_plan/astar_search.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "car_msg/msg/vehicle_state.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <nav2_map_server/map_io.hpp>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <memory>
#include <cmath>
#include <thread>

namespace emplanner {

/**
 * @brief EM规划器ROS2节点类
 * 该类封装了EM规划器的ROS2功能，处理规划请求和结果可视化
 */
class HyaStarNode : public rclcpp::Node {
public:

  explicit HyaStarNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~HyaStarNode() = default;

  void RunOnce();

private:

  // 加载发布全局地图
  void loadAndPublishMap(const std::string& map_yaml_filename);
  // 目标点，车辆状态回调函数
  void endPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void vehicleStateCallback(const car_msg::msg::VehicleState::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void occToMapData(const nav_msgs::msg::OccupancyGrid &occupancy_grid);
  
  bool globalPathPlanning();
  nav_msgs::msg::OccupancyGrid::SharedPtr getLatestMap();
  void loadParameters();// launch文件加载参数
  PlannerCommonParam getPlannerCommonParam();// 获取规划器公共参数
  geometry_msgs::msg::PoseStamped getCurrentPose();
  VehicleInfo getVehicleInfo(); // 获取车辆信息
  // 获取tf变换
  geometry_msgs::msg::TransformStamped getTransform(const std::string & from, const std::string & to);
  geometry_msgs::msg::Pose transform_pose(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform);
  geometry_msgs::msg::PoseStamped getNextTargetPose();

  

  // 订阅(当前位置（tf变换得到)， 终点位置， 车辆状态， 地图栅格数据)
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_pose_sub_;
  rclcpp::Subscription<car_msg::msg::VehicleState>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  
  // 发布(局部路径)(全局地图栅格数据)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  
  
  bool is_start_pose_set_ = false;
  bool is_end_pose_set_ = false;
  double current_speed_ = 0.0;
  
  
  // 定位信息
  geometry_msgs::msg::PoseStamped current_pose_;// 当前位置
  geometry_msgs::msg::PoseStamped end_pose_;// 终点位置
  nav_msgs::msg::OccupancyGrid::SharedPtr map_occupancy_grid_; // 地图栅格数据 
  nav_msgs::msg::OccupancyGrid::SharedPtr cur_occupancy_grid_; // 最新地图栅格数据
  
  // 添加TF监听相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_id_ = "map";      // 地图坐标系
  std::string lidar_frame_id_ = "lidar";  // 激光雷达坐标系
  
  // 组件实例
  std::unique_ptr<AStarPlanner> a_star_planner_;// 全局规划
  std::unique_ptr<AbstractPlanningAlgorithm> algo_; // 局部a*规划算法实例
  VehicleShape vehicle_shape_; // 车辆形状
  PlannerCommonParam planner_common_param_; // 规划器公共参数

  // 创建一个100ms的定时器
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time last_planning_time_;// 记录最新规划时间
  std::mutex map_mutex_;// 互斥锁保护地图数据
  std::string map_yaml_filename_; // 地图yaml文件名
  MapInfo map_info_; // 地图信息
  std::vector<std::vector<int>> map_data_; // 地图数据（栅格地图）
  

  // 全局路径
  std::vector<std::pair<double, double>> global_path_;

  // 辅助参数
  int planning_frequency_; // 规划频率
  bool is_collision_ = true;
  bool planning_in_progress_ = false; //是否正在规划
};

}  // namespace emplanner
