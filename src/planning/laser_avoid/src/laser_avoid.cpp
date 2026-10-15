#include "laser_avoid/laser_avoid.hpp"

LaserObstacleDetector::LaserObstacleDetector()
: Node("laser_obstacle_detector"),
  min_distance_(std::numeric_limits<double>::infinity()),
  obstacle_direction_(0.0),
  obstacle_detected_(false)
{
  // 声明和获取参数
    this->declare_parameter("car_length", 0.93);
    this->declare_parameter("car_width", 0.7);
    this->declare_parameter("lidar_position_x", -0.15);
    this->declare_parameter("lidar_position_y", 0.0);
    this->declare_parameter<double>("safe_distance", 0.8);
    this->declare_parameter<double>("angle_range", 60.0);
    this->declare_parameter<std::vector<double>>("detection_area", {-30.0, 30.0});
    this->declare_parameter<int>("obstacle_points_threshold", 5);
    
    this->get_parameter("car_length", car_length_);
    this->get_parameter("car_width", car_width_);
    this->get_parameter("lidar_position_x", lidar_position_x_);
    this->get_parameter("lidar_position_y", lidar_position_y_);
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
  const auto& ranges = msg->ranges;
  double angle_min = msg->angle_min;// 最小角度，先按-180  ---   180度处理
  double angle_increment = msg->angle_increment;//扫描增加量

  double start_angle_rad = -150.0 * M_PI / 180.0; // 起始角度 -150度
  double end_angle_rad = 150.0 * M_PI / 180.0;   // 结束角度 150度(不考虑后方的60度)
  int start_angle_idx = static_cast<int>((start_angle_rad - angle_min) / angle_increment);
  int end_angle_idx = static_cast<int>((end_angle_rad - angle_min) / angle_increment);
  start_angle_idx = std::max(0, start_angle_idx); // 确保索引不小于0
  end_angle_idx = std::min(static_cast<int>(ranges.size()) - 1, end_angle_idx); // 确保索引不超过范围
  
  // 计算检测前方区域的索引范围(min_idx, max_idx)
  double min_angle_rad = detection_area_[0] * M_PI / 180.0;
  double max_angle_rad = detection_area_[1] * M_PI / 180.0;
  int min_idx = static_cast<int>((min_angle_rad - angle_min) / angle_increment);
  int max_idx = static_cast<int>((max_angle_rad - angle_min) / angle_increment);
  // 确保索引在有效范围内
  min_idx = std::max(0, min_idx);
  max_idx = std::min(static_cast<int>(ranges.size()) - 1, max_idx);
  
  // 提取前方检测区域内的距离数据
  std::vector<float> front_detection_ranges;
  for (int i = min_idx; i <= max_idx; ++i) {
    front_detection_ranges.push_back(ranges[i]);
  }

  // 提取两侧检测区域的距离数据
  std::vector<float> side_detection_ranges;
  for (int i = start_angle_idx; i < min_idx; ++i) {
    side_detection_ranges.push_back(ranges[i]);
  }
  for (int i = max_idx + 1; i <= end_angle_idx; ++i) {
    side_detection_ranges.push_back(ranges[i]);
  }
  
  // 过滤无效值(inf, nan)
  std::vector<float> front_valid_ranges;
  for (const auto& range : front_detection_ranges) {
    if (std::isfinite(range)) {
      front_valid_ranges.push_back(range);
    }
  }
  std::vector<float> side_valid_ranges;
  for (const auto& range : side_detection_ranges) {
    if (std::isfinite(range)) {
      side_valid_ranges.push_back(range);
    }
  }
  
  // 检测障碍物
  obstacle_detected_ = false; // 重置障碍物检测状态
  if (!front_valid_ranges.empty() || !side_valid_ranges.empty()) {
    
    // 找到最小距离
    if(!front_valid_ranges.empty()) {

      // 找到前方检测区域内的最小距离
      auto min_dist = *std::min_element(front_valid_ranges.begin(), front_valid_ranges.end());
      min_distance_ = min_dist;
      
      if(min_distance_ < safe_distance_){
        // 统计小于安全距离的点数
        int obstacle_points = 0;
        for (const auto& range : front_valid_ranges) {
          if (range < safe_distance_) {
            obstacle_points++;
          }
          // 如果小于安全距离的点数超过阈值，判定为有障碍物,计算障碍物距离方向
          if (obstacle_points >= obstacle_points_threshold_) {
            obstacle_detected_ = true;
            auto min_iter = std::min_element(front_detection_ranges.begin(), front_detection_ranges.end());//最小元素索引
            int min_dist_idx = std::distance(front_detection_ranges.begin(), min_iter) + min_idx;// 最近点索引
            double obstacle_angle = angle_min + min_dist_idx * angle_increment;// 弧度
            obstacle_direction_ = obstacle_angle * 180.0 / M_PI;// 方向
            RCLCPP_INFO(this->get_logger(), "检测到障碍物: 距离=%.2fm, 方向=%.1f°",min_dist, obstacle_direction_);
            break; // 找到障碍物后退出循环
          }
        }
      }
    }

    if(!obstacle_detected_ && !side_valid_ranges.empty()) {
      // 前方没有找到障碍物，两侧检测区域检查
      
      auto min_dist = *std::min_element(side_valid_ranges.begin(), side_valid_ranges.end());
      min_distance_ = min_dist;
      
      if(min_distance_ < 0.6){
        // 统计小于安全距离的点数
        int obstacle_points = 0;
        for (const auto& range : side_valid_ranges) {
          if (range < 0.4) {
            obstacle_points++;
          }
          // 如果小于安全距离的点数超过阈值，判定为有障碍物,计算障碍物距离方向
          if (obstacle_points >= obstacle_points_threshold_) {
            obstacle_detected_ = true;
            auto min_iter = std::min_element(side_detection_ranges.begin(), side_detection_ranges.end());//最小元素索引
            int min_dist_idx = std::distance(side_detection_ranges.begin(), min_iter) + min_idx;// 最近点索引
            double obstacle_angle = angle_min + min_dist_idx * angle_increment;// 弧度
            obstacle_direction_ = obstacle_angle * 180.0 / M_PI;// 方向
            RCLCPP_INFO(this->get_logger(), "检测到障碍物: 距离=%.2fm, 方向=%.1f°",min_dist, obstacle_direction_);
            break; // 找到障碍物后退出循环
          }
        }
      }
    }

  } else {
    min_distance_ = std::numeric_limits<double>::infinity();
    obstacle_detected_ = false;
  }
  publish_detection(obstacle_detected_);
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