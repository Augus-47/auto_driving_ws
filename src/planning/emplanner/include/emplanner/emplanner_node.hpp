#pragma once

#include "emplanner/reference_line/reference_line_provider.hpp"
#include "emplanner/planner/emplanner.hpp"
#include "emplanner/common/plot.hpp"
#include "emplanner/planner/a_star.hpp"
#include "emplanner/common/planner_visualizer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "car_msg/msg/vehicle_state.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 用于tf2::fromMsg
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
class EmplannerNode : public rclcpp::Node {
public:

  explicit EmplannerNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~EmplannerNode() = default;


  void Initialize();
  void RunOnce(); // 执行一次局部路径规划

private:

  std::vector<MapPoint> CreatePath();    // 创建一个简单的路径点集合
  std::vector<MapPoint> GetGlobalPath(); // 获取全路全局路径
  bool getData();
  void loadAndPublishMap(const std::string& map_yaml_filename);
  void startPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void endPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void vehicleStateCallback(const car_msg::msg::VehicleState::SharedPtr msg);
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void occToMapData(const nav_msgs::msg::OccupancyGrid &occupancy_grid);
  bool globalPathPlanning();
  void SavePathToFile(const Vector2D &points, const std::string& filename);// 保存路径到文件并进行可视化
  nav_msgs::msg::OccupancyGrid::SharedPtr getLatestMap();
  void loadParameters();
  geometry_msgs::msg::PoseStamped getCurrentPose();
  void setLocalizationInfo(const geometry_msgs::msg::PoseStamped &pose);
  

  
  // 订阅(当前位置（tf变换得到)， 终点位置， 车辆状态， 地图栅格数据)
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr current_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr end_pose_sub_;
  rclcpp::Subscription<car_msg::msg::VehicleState>::SharedPtr vehicle_state_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  
  // 发布(局部路径)(全局地图栅格数据)
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  
  
  bool is_start_pose_set_ = false;
  bool is_end_pose_set_ = false;
  double current_speed_ = 0.0;
  
  // 参考线数据
  ReferenceLine pre_reference_line_;
  ReferenceLine reference_line_;
  
  // 轨迹数据
  Trajectory trajectory_;
  Trajectory pre_trajectory_;
  
  // 定位信息
  geometry_msgs::msg::PoseWithCovarianceStamped current_pose_;// 当前位置
  geometry_msgs::msg::PoseStamped end_pose_;// 终点位置
  LocalizationInfo localization_info_;// 定位信息（由位置转化而来）
  nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_; // 地图栅格数据 
  
  // 添加TF监听相关
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::string map_frame_id_ = "map";      // 地图坐标系
  std::string lidar_frame_id_ = "lidar";  // 激光雷达坐标系
  
  // 组件实例
  std::unique_ptr<ReferenceLineProvider> reference_line_provider_;// 参考线
  std::unique_ptr<EMPlanner> emplanner_;// em规划器实例
  std::unique_ptr<EMPlannerConfig> config_;
  std::unique_ptr<AStarPlanner> a_star_planner_;

  // 创建一个100ms的定时器
  rclcpp::TimerBase::SharedPtr timer_;
  std::mutex map_mutex_;// 互斥锁保护地图数据
  std::string map_yaml_filename_; // 地图yaml文件名
  MapInfo map_info_; // 地图信息
  std::vector<std::vector<int>> map_data_; // 地图数据（栅格地图）
  

  // 全局路径
  std::vector<std::pair<double, double>> global_path_;
  std::vector<MapPoint> global_path_points_; // 全局路径点集合

  // 可视化工具
  Plot plot_;
  std::unique_ptr<PlannerVisualizer> visualizer_;

  // 辅助参数
  int planning_frequency_; // 规划频率
  bool is_collision_ = true;
};

}  // namespace emplanner
