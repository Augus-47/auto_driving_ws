#include "laser_avoid/laser_avoid.hpp"

LaserObstacleDetector::LaserObstacleDetector()
: Node("laser_obstacle_detector"),
  min_distance_(std::numeric_limits<double>::infinity()),
  obstacle_direction_(0.0),
  obstacle_detected_(false)
{
  // 声明和获取参数
    this->declare_parameter("car_length", 1.2);
    this->declare_parameter("car_width", 0.8);
    this->declare_parameter("lidar_position_x", 0.0);
    this->declare_parameter("lidar_position_y", 0.0);
    this->declare_parameter<int>("filter_window", 5);
    this->declare_parameter<double>("alarm_ratio", 0.05);
    this->declare_parameter<int>("confirm_frames", 2);

    this->get_parameter("car_length", car_length_);
    this->get_parameter("car_width", car_width_);
    this->get_parameter("lidar_position_x", lidar_position_x_);
    this->get_parameter("lidar_position_y", lidar_position_y_);
    this->get_parameter("lidar_position_y", filter_window_);
    this->get_parameter("lidar_position_y", alarm_ratio_);
    this->get_parameter("lidar_position_y", confirm_frames_);
    half_window_ = filter_window_ / 2;
    // 计算边界
    boundaries_.front = car_length_ / 2.0 - lidar_position_x_;
    boundaries_.rear = -car_length_ / 2.0 - lidar_position_y_;
    boundaries_.left = car_width_ / 2.0 - lidar_position_x_;
    boundaries_.right = -car_width_ / 2.0 - lidar_position_y_;


    this->declare_parameter<double>("safe_distance", 0.6);
    this->declare_parameter<double>("angle_range", 60.0);
    this->declare_parameter<std::vector<double>>("detection_area", {-30.0, 30.0});
    this->declare_parameter<int>("obstacle_points_threshold", 5);
    this->get_parameter("safe_distance", safe_distance_);
    this->get_parameter("angle_range", angle_range_);
    this->get_parameter("detection_area", detection_area_);
    this->get_parameter("obstacle_points_threshold", obstacle_points_threshold_);

    

  // 创建订阅器
  laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/scan", 10,std::bind(&LaserObstacleDetector::laser_callback, this, std::placeholders::_1));
  
  // 创建发布器
  obstacle_pub_ = this->create_publisher<std_msgs::msg::Bool>(
    "/obstacle_detect", 1);
  RCLCPP_INFO(this->get_logger(), "激光雷达避障节点初始化完成");
}

void LaserObstacleDetector::laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  // 获取激光雷达数据
  // 数据预处理
  std::vector<float> ranges(msg->ranges);
  std::vector<bool> valid_mask(ranges.size(), false);
  int valid_count = 0;

  for (size_t i = 0; i < ranges.size(); ++i) {
    if (ranges[i] >= msg->range_min && ranges[i] <= msg->range_max) {
      valid_mask[i] = true;
      valid_count++;
    }
  }

  if (valid_count < 10) {
    RCLCPP_WARN(get_logger(), "有效点数不足!");
    return;
  }

  // 向量化滑动窗口滤波
  std::vector<float> smoothed(ranges.size(), std::numeric_limits<float>::infinity());
  for (size_t i = 0; i < ranges.size(); ++i) {
    float min_val = std::numeric_limits<float>::infinity();
    for (int j = -half_window_; j <= half_window_; ++j) {
      int idx = (i + j + ranges.size()) % ranges.size();  // 循环索引
      min_val = std::min(min_val, ranges[idx]);
    }
    smoothed[i] = min_val;
  }

  // 角度数组生成
  if (angle_array_.empty() || angle_array_.size() != ranges.size()) {
    angle_array_.resize(ranges.size());
    for (size_t i = 0; i < ranges.size(); ++i) {
      angle_array_[i] = msg->angle_min + i * msg->angle_increment;
      // 标准化角度到[-pi, pi]
      angle_array_[i] = std::fmod(angle_array_[i] + M_PI, 2 * M_PI) - M_PI;
    }
  }

  // 向量化边界距离计算
  std::vector<double> cos_theta(angle_array_.size());
  std::vector<double> sin_theta(angle_array_.size());
  for (size_t i = 0; i < angle_array_.size(); ++i) {
    cos_theta[i] = std::cos(angle_array_[i]);
    sin_theta[i] = std::sin(angle_array_[i]);
  }

  // 各方向边界距离计算
  std::vector<double> front_dist(cos_theta.size(), std::numeric_limits<double>::infinity());
  std::vector<double> rear_dist(cos_theta.size(), std::numeric_limits<double>::infinity());
  std::vector<double> left_dist(sin_theta.size(), std::numeric_limits<double>::infinity());
  std::vector<double> right_dist(sin_theta.size(), std::numeric_limits<double>::infinity());


  for (size_t i = 0; i < cos_theta.size(); ++i) {
    // 前方边界
    if (cos_theta[i] > 1e-6) {
      front_dist[i] = boundaries_.front / cos_theta[i];
    }
    
    // 后方边界
    if (cos_theta[i] < -1e-6) {
      rear_dist[i] = boundaries_.rear / cos_theta[i];
    }
    
    // 左侧边界
    if (sin_theta[i] > 1e-6) {
      left_dist[i] = boundaries_.left / sin_theta[i];
    }
    
    // 右侧边界
    if (sin_theta[i] < -1e-6) {
      right_dist[i] = boundaries_.right / sin_theta[i];
    }
  }

  // 综合最小边界距离
  std::vector<double> boundary_dist(cos_theta.size(), std::numeric_limits<double>::infinity());
  for (size_t i = 0; i < boundary_dist.size(); ++i) {
    boundary_dist[i] = std::min({front_dist[i], rear_dist[i], left_dist[i], right_dist[i]});
  }
  
  // 有效安全距离计算
  int danger_count = 0;
  for (size_t i = 0; i < ranges.size(); ++i) {
    if (valid_mask[i] && (smoothed[i] - boundary_dist[i] <= safe_distance_)) {
      danger_count++;
    }
  }
  
  // 危险判断
  double danger_ratio = static_cast<double>(danger_count) / ranges.size();
  if (danger_ratio >= alarm_ratio_) {
    danger_counter_ = std::min(danger_counter_.load() + 1, confirm_frames_);
  } else {
    danger_counter_ = std::max(danger_counter_.load() - 1, 0);
  }
  // 触发急停
  if (danger_counter_.load() >= confirm_frames_) {
    publish_detection(true);
  }else{
    publish_detection(false);
  }
}

void LaserObstacleDetector::publish_detection(bool result)
{
  auto msg = std::make_unique<std_msgs::msg::Bool>();
  msg->data = result;
  obstacle_pub_->publish(std::move(*msg));
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LaserObstacleDetector>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}