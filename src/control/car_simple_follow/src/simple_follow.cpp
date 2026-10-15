#include "car_simple_follow/simple_follow.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/utils.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>
#include <iomanip>

SingleVehicleController::SingleVehicleController()
: Node("single_vehicle_controller"),
  current_vel_x_(0.0),
  current_yaw_(0.0),
  current_target_idx_(0),
  is_following_trajectory_(false),
  avoid_object_(false)
{
  // 获取参数
  this->declare_parameter<std::string>("robot_frame_id", "base_link");
  this->declare_parameter<double>("max_vel_x", 1.0);
  this->declare_parameter<double>("min_vel_x", 0.05);
  this->declare_parameter<double>("max_vel_theta", 1.0);
  this->declare_parameter<double>("min_vel_theta", 0.05);
  this->declare_parameter<double>("k_v", 1.7);
  this->declare_parameter<double>("k_l", 0.5);
  this->declare_parameter<double>("k_a", 0.3);
  this->declare_parameter<double>("lookahead_distance", 2.0);
  this->declare_parameter<double>("default_speed", 0.5);
  this->declare_parameter<double>("wheelbase", 1.0);

  this->get_parameter("robot_frame_id", robot_frame_id_);
  this->get_parameter("max_vel_x", max_vel_x_);
  this->get_parameter("min_vel_x", min_vel_x_);
  this->get_parameter("max_vel_theta", max_vel_theta_);
  this->get_parameter("min_vel_theta", min_vel_theta_);
  this->get_parameter("k_v", k_v_);
  this->get_parameter("k_l", k_l_);
  this->get_parameter("k_a", k_a_);
  this->get_parameter("lookahead_distance", lookahead_distance_);
  this->get_parameter("default_speed", default_speed_);

  // 创建发布器
  cmd_pub_ = this->create_publisher<car_msg::msg::VehicleState>("/control_cmd", 1);

  // 创建订阅器
  vehicle_info_sub_ = this->create_subscription<car_msg::msg::VehicleState>(
    "/vehicle_info", 1, std::bind(&SingleVehicleController::vehicle_info_callback, this, std::placeholders::_1));

  avoidance_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "obstacle_detect", 1,
    std::bind(&SingleVehicleController::obstacle_callback, this, std::placeholders::_1));

  trajectory_sub_ = this->create_subscription<nav_msgs::msg::Path>(
    "/planned_trajectory", 1,
    std::bind(&SingleVehicleController::trajectory_callback, this, std::placeholders::_1));

  // 创建TF监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // 创建定时器
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),  // 10Hz
    std::bind(&SingleVehicleController::timer_callback, this));
}

// 速度回调
void SingleVehicleController::vehicle_info_callback(const car_msg::msg::VehicleState::SharedPtr msg)
{
  current_vel_x_ = msg->velocity;
}

// 障碍物回调
void SingleVehicleController::obstacle_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) {
    avoid_object_ = true;
    RCLCPP_WARN(this->get_logger(), "检测到障碍物!");
  } else {
    avoid_object_ = false;
  }
}

// 轨迹回调
void SingleVehicleController::trajectory_callback(const nav_msgs::msg::Path::SharedPtr msg)
{
  if (msg->poses.empty()) {
    RCLCPP_WARN(this->get_logger(), "接收到空轨迹");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "接收到新轨迹，共%ld个点", msg->poses.size());

  // 转换轨迹格式
  std::vector<std::tuple<double, double, double, double>> path_points;
  std::tuple<double, double, double> prev_pose = std::make_tuple(0.0, 0.0, 0.0);
  bool has_prev_pose = false;

  for (size_t i = 0; i < msg->poses.size(); ++i) {
    const auto& pose_stamped = msg->poses[i];
    
    // 提取位置
    double x = pose_stamped.pose.position.x;
    double y = pose_stamped.pose.position.y;
    
    // 提取朝向（四元数转欧拉角）
    tf2::Quaternion q;
    tf2::fromMsg(pose_stamped.pose.orientation, q);
    double yaw = tf2::getYaw(q);
    
    // 计算速度
    double velocity = default_speed_;
    
    // 根据路径曲率动态调整速度（可选）
    if (has_prev_pose) {
      // 计算点之间的距离
      double dist = std::hypot(
        x - std::get<0>(prev_pose),
        y - std::get<1>(prev_pose)
      );
      
      // 计算朝向变化
      double prev_yaw = std::get<2>(prev_pose);
      double yaw_change = std::abs(std::atan2(std::sin(yaw - prev_yaw), std::cos(yaw - prev_yaw)));
      
      // 根据曲率调整速度（转弯处速度降低）
      if (yaw_change > 0.1) {  // 如果转弯角度大于约5.7度
        // 转弯越大，速度越低
        double curvature_factor = std::max(0.3, 1.0 - yaw_change / M_PI);
        velocity = default_speed_ * curvature_factor;
      }
    }
    
    // 存储轨迹点 (x, y, yaw, velocity)
    path_points.emplace_back(x, y, yaw, velocity);
    prev_pose = std::make_tuple(x, y, yaw);
    has_prev_pose = true;
  }
  
  // 设置轨迹
  set_trajectory(path_points);
  // 自动开始跟踪
  RCLCPP_INFO(this->get_logger(), "开始跟踪新轨迹");
  is_following_trajectory_ = true;// 修改为true
}

// 设置转化格式后的轨迹
void SingleVehicleController::set_trajectory(const std::vector<std::tuple<double, double, double, double>>& path_points)
{
  path_points_ = path_points;
  current_target_idx_ = 0;
}

// 查找目标点，返回索引
int SingleVehicleController::find_target_point(double current_x, double current_y)
{
  if (path_points_.empty()) {
    return -1;
  }
  
  // 首先找到路径上最近的点
  double min_dist = std::numeric_limits<double>::max();
  int closest_idx = current_target_idx_;
  
  // 从当前索引开始搜索，避免回头
  int search_range = std::min(20, static_cast<int>(path_points_.size() - current_target_idx_));
  for (int i = current_target_idx_; i < current_target_idx_ + search_range; ++i) {
    if (i >= static_cast<int>(path_points_.size())) {
      break;
    }
    
    const auto& point = path_points_[i];
    double dist = std::hypot(std::get<0>(point) - current_x, std::get<1>(point) - current_y);
    if (dist < min_dist) {
      min_dist = dist;
      closest_idx = i;
    }
  }
  
  // 从最近点开始，沿路径累积距离
  double accumulated_dist = 0.0;
  int target_idx = closest_idx;
  
  // 向前累积距离直到达到前视距离
  for (int i = closest_idx; i < static_cast<int>(path_points_.size()) - 1; ++i) {
    const auto& p1 = path_points_[i];
    const auto& p2 = path_points_[i + 1];
    double segment_dist = std::hypot(
      std::get<0>(p2) - std::get<0>(p1),
      std::get<1>(p2) - std::get<1>(p1)
    );
    
    if (accumulated_dist + segment_dist > lookahead_distance_) {
      // 计算插值点的比例
      // double ratio = (lookahead_distance_ - accumulated_dist) / segment_dist;
      
      // 可以选择使用插值点或直接使用下一个点
      // 这里简单地选择下一个点
      target_idx = i + 1;
      break;
    }
    
    accumulated_dist += segment_dist;
    target_idx = i + 1;
  }
  
  // 更新当前目标点索引，确保不会回退
  current_target_idx_ = std::max(closest_idx, current_target_idx_);
  
  // 如果到达路径末尾，使用最后一个点
  if (target_idx >= static_cast<int>(path_points_.size())) {
    target_idx = path_points_.size() - 1;
  }
  
  return target_idx;
}

// 主函数，执行轨迹，返回<控制信息, 跟踪结束标志>
std::tuple<car_msg::msg::VehicleState, bool> SingleVehicleController::follow_trajectory()
{
  car_msg::msg::VehicleState cmd_vel;
  bool done = false;

  // 并没有在轨迹跟踪阶段，路径点为空，表示已经完成轨迹跟踪
  if (!is_following_trajectory_ || path_points_.empty()) {
    return std::make_tuple(cmd_vel, true);
  }

  // 安全检查 (原来)
  // if (avoid_object_) {
  //   RCLCPP_WARN(this->get_logger(), "检测到障碍物，停止移动");
  //   cmd_vel.velocity = 0.0;
  //   cmd_vel.steering = 0.0;
  //   return std::make_tuple(cmd_vel, false);
  // }
     // 优先处理障碍物检测 - 在函数最开头
// 新的
if (avoid_object_) {
    RCLCPP_WARN(this->get_logger(), "检测到障碍物，紧急制动！");
    cmd_vel.velocity = 0.0;
    cmd_vel.steering = 0.0;
    cmd_vel.brake = 1;  // ✅ 新增：设置制动标志
    cmd_vel.gears = 0;  // ✅ 新增：设置空挡
    return std::make_tuple(cmd_vel, false);
}
  
  try {
    // 获取当前位置和朝向
    double current_x, current_y, current_yaw;
    
    try {
      // 尝试通过TF获取位姿
      geometry_msgs::msg::TransformStamped transform_stamped =
        tf_buffer_->lookupTransform("map", robot_frame_id_, tf2::TimePointZero);
      current_x = transform_stamped.transform.translation.x;
      current_y = transform_stamped.transform.translation.y;
      tf2::Quaternion q;
      tf2::fromMsg(transform_stamped.transform.rotation, q);
      current_yaw = tf2::getYaw(q);
    } catch (const tf2::TransformException& ex) {
      // 如果TF获取失败，使用UTM位置和当前偏航角
      current_x = utm_pose_.position.x;
      current_y = utm_pose_.position.y;
      current_yaw = current_yaw_;
      RCLCPP_WARN(this->get_logger(), "无法获取TF变换: %s", ex.what());
    }
    
    // 找到目标点
    int target_idx = find_target_point(current_x, current_y);
    if (target_idx < 0) {
      return std::make_tuple(cmd_vel, true);
    }
    
    current_target_idx_ = target_idx;
    const auto& target_point = path_points_[target_idx];
    double target_x = std::get<0>(target_point);
    double target_y = std::get<1>(target_point);
    double target_yaw = std::get<2>(target_point);
    double target_v = std::get<3>(target_point);
    
    // 计算误差
    double e_linear_x = target_x - current_x;
    double e_linear_y = target_y - current_y;
    double e_angular_z = std::atan2(
      std::sin(target_yaw - current_yaw),
      std::cos(target_yaw - current_yaw)
    );
    
    // 计算局部坐标系中的误差
    double dx = e_linear_x * std::cos(current_yaw) + e_linear_y * std::sin(current_yaw);
    double dy = -e_linear_x * std::sin(current_yaw) + e_linear_y * std::cos(current_yaw);
    
    // 计算控制输出
    double vel_x = target_v + k_v_ * dx;
    
    // 计算转向角度而非角速度
    double steering_angle = 0.0;
    if (std::abs(vel_x) > 0.1) {
      // 计算前视点到车辆的距离
      double look_ahead_dist = std::hypot(dx, dy);
      
      // 获取车辆轴距 TODO
      double L = 0.8;
      this->get_parameter("wheelbase", L);
      
      // 计算车辆朝向与目标点连线的夹角
      double alpha = std::atan2(dy, dx);
      
      // 计算所需转向角
      steering_angle = std::atan2(2 * L * std::sin(alpha), look_ahead_dist);
      
      // 添加朝向误差的修正项
      steering_angle += k_a_ * std::sin(e_angular_z);
      
      // 限制转向角范围
      double max_steering_angle = 30.0 * M_PI / 180.0;  // 最大转向角30度
      steering_angle = std::max(-max_steering_angle, std::min(steering_angle, max_steering_angle));
    } else {
      // 低速时使用简化计算
      steering_angle = k_a_ * std::sin(e_angular_z);
    }
    
    // 限制速度
    if (std::abs(vel_x) < min_vel_x_) {
      vel_x = 0.0;
    } else {
      vel_x = std::max(-max_vel_x_, std::min(vel_x, max_vel_x_));
    }
    
    // 检查是否到达终点
    double distance_to_goal = std::hypot(
      std::get<0>(path_points_.back()) - current_x,
      std::get<1>(path_points_.back()) - current_y
    );
    
    if (target_idx == static_cast<int>(path_points_.size()) - 1 && distance_to_goal < 0.5) {
      RCLCPP_INFO(this->get_logger(), "轨迹跟踪完成");
      done = true;
      is_following_trajectory_ = false;
    } 
    cmd_vel.velocity = vel_x;
    cmd_vel.steering = steering_angle * 180 / M_PI;  // 转换为度数
    return std::make_tuple(cmd_vel, done);
  } catch (const std::exception& e) {
    RCLCPP_WARN(this->get_logger(), "轨迹跟踪出错: %s", e.what());
    return std::make_tuple(cmd_vel, false);
  }
}

bool SingleVehicleController::execute_trajectory()
{
  if (!is_following_trajectory_) {
    return true;
  }
  
  auto [cmd_vel, done] = follow_trajectory();
  
  // 发布控制命令
  cmd_pub_->publish(cmd_vel);
  RCLCPP_INFO(this->get_logger(), "发布控制命令: 线速度=%.2f, 转向角=%.2f", 
              cmd_vel.velocity, cmd_vel.steering);
  
  return done;
}

void SingleVehicleController::timer_callback()
{
  // 如果有轨迹，执行轨迹跟踪
  if (is_following_trajectory_) {
    bool done = execute_trajectory();
    
    // 如果轨迹执行完成，可以在这里添加后续操作
    if (done && !is_following_trajectory_) {
      RCLCPP_INFO(this->get_logger(), "轨迹执行完成");
      // 这里可以添加轨迹执行完成后的操作
    }
  } else {
    // 没有轨迹时，可以发送零速度命令或等待新轨迹
    car_msg::msg::VehicleState stop_cmd;
    stop_cmd.velocity = 0.0;
    stop_cmd.steering = 0.0;
    cmd_pub_->publish(stop_cmd);
  }
}

// 读取轨迹文件
void SingleVehicleController::readPositionData(std::string &file_path) {
        // 打开文件
  std::ifstream file(file_path);
  if (!file.is_open()) {
      std::cout << "Failed to open the file: " << std::endl;
      return;
  }
  // 轨迹读取
  double esp = 1.0;// 阈值
  double curcount = 0.0;

  std::string line;
  int i = 0;
  auto pose = geometry_msgs::msg::PoseStamped();
  auto lastPose = geometry_msgs::msg::PoseStamped();
  while (std::getline(file, line)) {
      size_t start = 0;
      size_t end = line.find(' ');
      std::vector<double> parts;
      while (end != std::string::npos) {
          parts.push_back(std::stod(line.substr(start, end - start)));
          start = end + 1;
          end = line.find(' ', start);
      }
      parts.push_back(std::stod(line.substr(start)));
      // 行数据大于等于7,赋值，每隔 1.0 加入全局路径点
      if (parts.size() >= 7) {
          pose.header.frame_id = "base_link";
          pose.pose.position.x = parts[0]; //x
          pose.pose.position.y = parts[1]; //y
          pose.pose.position.z = parts[2]; //z
          pose.pose.orientation.x = parts[3]; //qx
          pose.pose.orientation.y = parts[4]; //qy    
          pose.pose.orientation.z = parts[5]; //qz
          pose.pose.orientation.w = parts[6]; //qw
          
          allPathPoint_.push_back(pose);
          if(i == 0){
              GlobalENUPurSuitPoint.poses.push_back(pose);
              lastPose = pose;
          }
          // 当前点与上一个点的距离
          double dx = pose.pose.position.x - lastPose.pose.position.x;
          double dy = pose.pose.position.y - lastPose.pose.position.y;
          double distance = std::sqrt(dx * dx + dy * dy);
          curcount += distance;//当前累计距离
          // 累计距离大于0.5，加入全局
          if(curcount >= esp){
              GlobalENUPurSuitPoint.poses.push_back(pose);
              curcount = 0.0;
          }
          // 更新上一个点
          lastPose = pose;
          i++;
      }
  }
  // 将最后一个点加入全局路径点
  GlobalENUPurSuitPoint.poses.push_back(lastPose);
}



int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SingleVehicleController>();
  rclcpp::spin(node);
  
  // 关闭时发送停止命令
  auto stop_cmd = std::make_unique<car_msg::msg::VehicleState>();
  stop_cmd->velocity = 0.0;
  stop_cmd->steering = 0.0;
  node->publish_vehicle_state(*stop_cmd);
  
  rclcpp::shutdown();
  return 0;
}