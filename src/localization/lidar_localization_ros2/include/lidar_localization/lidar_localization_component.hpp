#ifndef LIDAR_LOCALIZATION__LIDAR_LOCALIZATION_COMPONENT_HPP_
#define LIDAR_LOCALIZATION__LIDAR_LOCALIZATION_COMPONENT_HPP_

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>

#include <tf2/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <pcl_conversions/pcl_conversions.h>

#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"

#include "car_msg/msg/orin_pose.hpp"

#include <pclomp/ndt_omp.h>
#include <pclomp/ndt_omp_impl.hpp>
#include <pclomp/voxel_grid_covariance_omp.h>
#include <pclomp/voxel_grid_covariance_omp_impl.hpp>
#include <pclomp/gicp_omp.h>
#include <pclomp/gicp_omp_impl.hpp>

#include "lidar_localization/lidar_undistortion.hpp"

using namespace std::chrono_literals;

class PCLLocalization : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PCLLocalization(const rclcpp::NodeOptions & options);

  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &);
  CallbackReturn on_activate(const rclcpp_lifecycle::State &);
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);
  CallbackReturn on_error(const rclcpp_lifecycle::State & state);

  void initializeParameters();
  void initializePubSub();
  void initializeRegistration();

  void initialPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  void mapReceived(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void odomReceived(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void imuReceived(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
  void cloudReceived(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);

private:
  Eigen::Matrix4f tf2TransformToEigenMatrix(const tf2::Transform & tf) const;
  tf2::Transform eigenMatrixToTf2Transform(const Eigen::Matrix4f & mat) const;

  // 硬 reject：用于配准失败、fitness 太差等情况
  void handleReject(const std::string & reason);

  // 软 reject：用于走廊退化、静止漂移、后退误匹配等情况
  // 只拒绝当前 NDT 结果，但保持 TRACKING，不退回 INITIALIZING
  void handleSoftReject(const std::string & reason);

  bool computeAndPublishMapToOdom(
    const tf2::Transform & T_map_base,
    const rclcpp::Time & stamp);

  void publishLastMapToOdom(const rclcpp::Time & stamp);

private:
  // ==============================
  // TF
  // ==============================
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener tflistener_;

  // ==============================
  // 订阅器
  // ==============================
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::ConstSharedPtr
    initial_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::ConstSharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::ConstSharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::ConstSharedPtr imu_sub_;

  // ==============================
  // 发布器
  // ==============================
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<car_msg::msg::OrinPose>::SharedPtr
    orin_pose_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
    path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr
    initial_map_pub_;

  // ==============================
  // PCL
  // ==============================
  boost::shared_ptr<pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>> registration_;
  pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;

  // 当前 map 系下的最新位姿
  geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr corrent_pose_with_cov_stamped_ptr_;

  nav_msgs::msg::Path::SharedPtr path_ptr_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr last_scan_ptr_;

  // ==============================
  // map->odom 连续发布
  // ==============================
  rclcpp::TimerBase::SharedPtr map_odom_timer_;
  double map_odom_publish_rate_{20.0};
  double transform_tolerance_{0.1};

  bool map_odom_initialized_{false};
  tf2::Transform last_map_odom_;

  // ==============================
  // odom 状态
  // ==============================
  bool odom_initialized_{false};
  tf2::Transform last_odom_pose_;
  tf2::Transform current_odom_pose_;

  // 关键新增：
  // 记录“上一次成功接受 NDT 定位时”的 odom 位姿。
  // 之后预测时用：
  // odom_pose_at_last_accept_.inverse() * current_odom_pose_
  // 而不是最近两帧 odom 的差值。
  tf2::Transform odom_pose_at_last_accept_;
  bool odom_pose_at_last_accept_initialized_{false};

  // ==============================
  // 状态机
  // ==============================
  bool localized_{false};
  int consecutive_success_count_{0};
  int consecutive_reject_count_{0};

  bool map_recieved_{false};
  bool initialpose_recieved_{false};

  // ==============================
  // 话题参数
  // ==============================
  std::string cloud_topic_;
  std::string odom_topic_;
  std::string imu_topic_;
  std::string map_topic_;
  std::string initialpose_topic_;

  // ==============================
  // 坐标系参数
  // ==============================
  std::string global_frame_id_;
  std::string odom_frame_id_;
  std::string base_frame_id_;

  // ==============================
  // 配准参数
  // ==============================
  std::string registration_method_;
  double score_threshold_{2.0};
  double init_score_threshold_{3.0};
  double ndt_resolution_{1.0};
  double ndt_step_size_{0.1};
  double transform_epsilon_{0.01};
  double voxel_leaf_size_{0.2};
  int ndt_num_threads_{4};
  int ndt_max_iterations_{35};

  // ==============================
  // 点云范围参数
  // ==============================
  double scan_max_range_{100.0};
  double scan_min_range_{1.0};
  double scan_period_{0.1};

  // ==============================
  // 地图参数
  // ==============================
  bool use_pcd_map_{false};
  std::string map_path_;

  // ==============================
  // 初始位姿参数
  // ==============================
  bool set_initial_pose_{false};
  double initial_pose_x_{0.0};
  double initial_pose_y_{0.0};
  double initial_pose_z_{0.0};
  double initial_pose_qx_{0.0};
  double initial_pose_qy_{0.0};
  double initial_pose_qz_{0.0};
  double initial_pose_qw_{1.0};

  // ==============================
  // 先验/调试参数
  // ==============================
  bool use_odom_{false};
  bool use_imu_{false};
  bool enable_debug_{false};

  // ==============================
  // jump gate 参数
  // ==============================
  double reject_translation_threshold_{0.5};
  double reject_yaw_threshold_deg_{5.0};
  double reject_z_threshold_{0.3};
  double reject_roll_pitch_threshold_deg_{10.0};

  // ==============================
  // 状态机参数
  // ==============================
  int localization_success_frame_num_{3};
  int tracking_lost_frame_num_{5};

  // 是否允许 TRACKING 连续失败后自动回 INITIALIZING
  // 走廊退化场景建议 false
  bool allow_tracking_lost_reinit_{false};

  // ==============================
  // 新增：odom 一致性门控
  // ==============================
  bool enable_odom_consistency_gate_{true};
  double odom_consistency_trans_margin_{0.05};
  double odom_consistency_yaw_margin_deg_{1.0};

  // ==============================
  // 新增：静止保护
  // ==============================
  bool enable_static_ndt_reject_{true};
  double static_odom_trans_threshold_{0.01};
  double static_odom_yaw_threshold_deg_{0.3};
  double static_ndt_trans_threshold_{0.025};
  double static_ndt_yaw_threshold_deg_{0.8};

  // ==============================
  // 新增：后退保护
  // ==============================
  bool enable_backward_reject_{true};
  double backward_correction_threshold_{0.03};

  // ==============================
  // IMU 去畸变模块
  // ==============================
  LidarUndistortion lidar_undistortion_;
};

#endif  // LIDAR_LOCALIZATION__LIDAR_LOCALIZATION_COMPONENT_HPP_

