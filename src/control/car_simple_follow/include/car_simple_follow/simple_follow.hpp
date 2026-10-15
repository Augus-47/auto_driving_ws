#ifndef SINGLE_VEHICLE_CONTROLLER_HPP
#define SINGLE_VEHICLE_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <car_msg/msg/vehicle_state.hpp>
#include <car_msg/msg/orin_pose.hpp>
#include <vector>
#include <tuple>
#include <cmath>

class SingleVehicleController : public rclcpp::Node {
public:
  // 构造函数
  SingleVehicleController();
  void publish_vehicle_state(const car_msg::msg::VehicleState & msg){cmd_pub_->publish(msg);}

private:
  // 回调函数
  void vehicle_info_callback(const car_msg::msg::VehicleState::SharedPtr msg);
  void obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void utm_callback(const car_msg::msg::OrinPose::SharedPtr msg);
  void trajectory_callback(const nav_msgs::msg::Path::SharedPtr msg);
  void timer_callback();

  // 轨迹跟踪函数
  void set_trajectory(const std::vector<std::tuple<double, double, double, double>>& path_points);
  int find_target_point(double current_x, double current_y);
  std::tuple<car_msg::msg::VehicleState, bool> follow_trajectory();
  bool execute_trajectory();
  void readPositionData(std::string &file_path);

  // 发布器
  rclcpp::Publisher<car_msg::msg::VehicleState>::SharedPtr cmd_pub_;

  // 订阅器
  rclcpp::Subscription<car_msg::msg::VehicleState>::SharedPtr vehicle_info_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr avoidance_sub_;
  rclcpp::Subscription<car_msg::msg::OrinPose>::SharedPtr utm_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr trajectory_sub_;

  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;

  // TF监听器
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // 控制参数
  std::string robot_frame_id_;
  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double min_vel_theta_;
  double k_v_;
  double k_l_;
  double k_a_;
  double lookahead_distance_;
  double default_speed_;

  // 轨迹跟踪状态
  std::vector<std::tuple<double, double, double, double>> path_points_; // (x, y, yaw, velocity)
  int current_target_idx_;
  bool is_following_trajectory_;
  // 轨迹文件读取
  std::vector<geometry_msgs::msg::PoseStamped> allPathPoint_;  // 全部路径点
  nav_msgs::msg::Path GlobalENUPurSuitPoint; // 全局路径


  // 车辆状态
  double current_vel_x_;
  double current_yaw_;
  geometry_msgs::msg::Pose utm_pose_;
  bool avoid_object_;
};

#endif // SINGLE_VEHICLE_CONTROLLER_HPP