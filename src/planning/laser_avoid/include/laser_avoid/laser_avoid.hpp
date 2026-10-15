#ifndef LASER_OBSTACLE_DETECTOR_HPP
#define LASER_OBSTACLE_DETECTOR_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

class LaserObstacleDetector : public rclcpp::Node {
public:
  // 构造函数
  LaserObstacleDetector();

private:

  struct {
    double front;
    double rear;
    double left;
    double right;
  } boundaries_;
  
  // 回调函数
  void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void publish_detection(bool result);

  // 参数
  double car_length_;        // 车长(m)
  double car_width_;         // 车宽(m)
  double lidar_position_x_; // 雷达偏移
  double lidar_position_y_;
  double alarm_ratio_;       // 危险点比例阈值
  int confirm_frames_;       // 确认帧数
  double filter_window_;
  int half_window_;
  std::vector<double> angle_array_;

  std::atomic<int> danger_counter_;

  double safe_distance_;
  double angle_range_;
  std::vector<double> detection_area_;
  int obstacle_points_threshold_;
  
  
  // 状态变量
  double min_distance_;
  double obstacle_direction_;
  bool obstacle_detected_;
  
  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  // 发布器
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr obstacle_pub_;
};

#endif // LASER_OBSTACLE_DETECTOR_HPP