#include "emplanner/emplanner_node.hpp"

namespace emplanner {

namespace plt = matplotlibcpp;

EmplannerNode::EmplannerNode(const rclcpp::NodeOptions& options)
    : Node("emplanner_node", options), plot_() {
  RCLCPP_INFO(this->get_logger(), "EmplannerNode构造中...");

  current_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/current_pose", 1, 
        std::bind(&EmplannerNode::startPoseCallback, this, std::placeholders::_1));
  
  end_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/end_pose", 1, 
        std::bind(&EmplannerNode::endPoseCallback, this, std::placeholders::_1));

  vehicle_state_sub_ = this->create_subscription<car_msg::msg::VehicleState>(
        "/vehicle_state", 1, 
        std::bind(&EmplannerNode::vehicleStateCallback, this, std::placeholders::_1));
  
  occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/current_lidar_occ_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),  // 使用持久订阅，确保获取最后一条消息
      std::bind(&EmplannerNode::mapCallback, this, std::placeholders::_1));

  // 发布地图栅格数据
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map", 1);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 1);

  // 获取参数，加载全局地图
  this->declare_parameter("map_yaml_filename", "/home/nvidia/lc/emplanner/src/emplanner/maps/map.yaml");
  this->get_parameter("map_yaml_filename", map_yaml_filename_);
  loadAndPublishMap(map_yaml_filename_);
  
  // 初始化配置
  config_ = std::make_unique<EMPlannerConfig>();
  reference_line_provider_ = std::make_unique<ReferenceLineProvider>();
  visualizer_ = std::make_unique<PlannerVisualizer>(this);
  emplanner_ = std::make_unique<EMPlanner>(*config_);
  a_star_planner_ = std::make_unique<AStarPlanner>();

  // 初始化 TF 缓冲区和监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  timer_ = this->create_wall_timer(std::chrono::milliseconds(planning_frequency_), 
                                  std::bind(&EmplannerNode::RunOnce, this));
  
  RCLCPP_INFO(this->get_logger(), "EmplannerNode构造完成");
}

// 从tf变换获取当前位姿
geometry_msgs::msg::PoseStamped EmplannerNode::getCurrentPose()
{
  geometry_msgs::msg::PoseStamped current_pose;
  
  try {
    // 获取地图坐标系到激光雷达坐标系的变换
    geometry_msgs::msg::TransformStamped transform_stamped = 
      tf_buffer_->lookupTransform(map_frame_id_, lidar_frame_id_, tf2::TimePointZero);
    
    // 设置时间戳和帧ID
    current_pose.header = transform_stamped.header;
    
    // 设置位置
    current_pose.pose.position.x = transform_stamped.transform.translation.x;
    current_pose.pose.position.y = transform_stamped.transform.translation.y;
    current_pose.pose.position.z = transform_stamped.transform.translation.z;
    
    // 设置朝向(四元数)
    current_pose.pose.orientation = transform_stamped.transform.rotation;          
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "获取变换失败: %s", ex.what());
  }
  
  return current_pose;
}

// 设置定位信息
void EmplannerNode::setLocalizationInfo(const geometry_msgs::msg::PoseStamped &pose) {
  localization_info_.x = pose.pose.position.x;
  localization_info_.y = pose.pose.position.y;
  auto& q = pose.pose.orientation;
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y); // 计算航向角
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  localization_info_.heading = std::atan2(siny_cosp, cosy_cosp);
  localization_info_.index = 0; // 索引初始化
  localization_info_.kappa = 0; // 曲率初始化
  localization_info_.v = current_speed_; // 当前速度
  localization_info_.vx = 0;
  localization_info_.vy = 0; // 车身坐标系速度
  localization_info_.a = 0; // 加速度初始化
  localization_info_.ax = 0;
  localization_info_.ay = 0; // 车身坐标系加速度
}

void EmplannerNode::Initialize() {}

void EmplannerNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // 使用互斥锁更新最新地图
  std::lock_guard<std::mutex> lock(map_mutex_);
  occupancy_grid_ = msg;
  RCLCPP_INFO(this->get_logger(), "收到新地图");
}

// 在需要时获取地图数据的方法
nav_msgs::msg::OccupancyGrid::SharedPtr EmplannerNode::getLatestMap() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return occupancy_grid_;
}

void EmplannerNode::occToMapData(const nav_msgs::msg::OccupancyGrid &occupancy_grid) {
  // 地图元数据
  map_info_.resolution = occupancy_grid.info.resolution;
  map_info_.width = occupancy_grid.info.width;
  map_info_.height = occupancy_grid.info.height;
  map_info_.origin_x = occupancy_grid.info.origin.position.x;
  map_info_.origin_y = occupancy_grid.info.origin.position.y;

  // 地图数据初始化
  map_data_.clear();
  map_data_.resize(map_info_.height, std::vector<int>(map_info_.width, 0));

  for (int y = 0; y < map_info_.height; ++y) {
      for (int x = 0; x < map_info_.width; ++x) {
          int index = y * map_info_.width + x;
          map_data_[y][x] = static_cast<int>(occupancy_grid.data[index]);
      }
  }

}

// 收到起点信息 进行初始化
void EmplannerNode::startPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
  current_pose_ = *msg;
  localization_info_.x = current_pose_.pose.pose.position.x;
  localization_info_.y = current_pose_.pose.pose.position.y;
  auto& q = current_pose_.pose.pose.orientation;
  double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
  double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
  localization_info_.heading = std::atan2(siny_cosp, cosy_cosp);
  localization_info_.index = 0;
  localization_info_.kappa = 0;
  localization_info_.v = current_speed_;
  localization_info_.vx = 0;
  localization_info_.vy = 0;
  localization_info_.a = 0;
  localization_info_.ax = 0;
  localization_info_.ay = 0;
  localization_info_.t = 0;
  is_start_pose_set_ = true;
}

// 收到终点信息 进行全局路径规划
void EmplannerNode::endPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  end_pose_ = *msg;
  is_end_pose_set_ = true;
  RCLCPP_INFO(this->get_logger(), "收到终点位置: x=%f, y=%f", end_pose_.pose.position.x, end_pose_.pose.position.y);
  auto result = globalPathPlanning();
}

void EmplannerNode::vehicleStateCallback(const car_msg::msg::VehicleState::SharedPtr msg) {
  current_speed_ = msg->velocity;
}

// TODO 当前位置信息获取优化
bool EmplannerNode::getData() {
  return true;
} 

// 全局路径规划
bool EmplannerNode::globalPathPlanning() {
  // 获取最新地图数据,定位数据
  auto current_pose = getCurrentPose();
  auto occupancy_grid = getLatestMap();
  if (!occupancy_grid) {
    RCLCPP_ERROR(this->get_logger(), "未获取到最新地图数据");
    return false;
  }

  // 将OccupancyGrid转换为二维地图数据
  occToMapData(*occupancy_grid);
  a_star_planner_->setMap(map_data_, map_info_);
  auto result = a_star_planner_->planPathWorld(current_pose.pose.position.x, current_pose.pose.position.y, end_pose_.pose.position.x, end_pose_.pose.position.y);

  if (result.empty()) {
    RCLCPP_ERROR(this->get_logger(), "全局路径规划失败");
    return false;
  }
  global_path_.clear();
  global_path_ = result;
  auto global_map_point = GetGlobalPath(); // 更新全局路径点
  global_path_points_.clear();
  global_path_points_ = global_map_point;
  visualizer_->clear();
  visualizer_->visualizeMapPoints(global_path_points_);
  RCLCPP_INFO(this->get_logger(), "全局路径规划成功，路径点数量: %zu", global_path_points_.size());

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = map_frame_id_;
  path_msg.header.stamp = this->now();
  path_msg.poses.clear();
  
  for (const auto& point : global_path_points_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point.x;
    pose.pose.position.y = point.y;
    pose.pose.position.z = 0.0; // 假设z轴为0
    pose.pose.orientation.w = 1.0; // 默认朝向
    path_msg.poses.push_back(pose);
  }
  path_pub_->publish(path_msg);
  
  return true;
}

// 执行一次局部路径规划
void EmplannerNode::RunOnce() {

  // 检查全局路径，碰撞检查
  if(global_path_points_.empty() || !is_collision_){
    return;
  }

  // 0. 当前位置
  auto current_pose = getCurrentPose();
  setLocalizationInfo(current_pose);

  RCLCPP_INFO(this->get_logger(), "开始执行一次局部路径规划...");

  // 1. 获取全局路径信息
  auto points = global_path_points_;
  
  uint64_t time = 0;
  
  // 2. 生成参考线，并可视化,并没有进行参考线平滑操作
  reference_line_provider_->Provide(points, localization_info_, pre_reference_line_, reference_line_);
  visualizer_->visualizeReferenceLine(reference_line_);
  
  
  // 3. 设置规划起点(这里之前使用的是上一时刻的轨迹点，我是否该调整为上一次的参考线点？)
  TrajectoryPoint plan_start_point;
  emplanner_->CalPlaningStartPoint(pre_reference_line_.get_reference_points(), localization_info_, &plan_start_point);
  RCLCPP_INFO(this->get_logger(), "简化版规划起点: x=%f, y=%f", plan_start_point.x, plan_start_point.y);
  pre_reference_line_ = reference_line_;

  // 4. 开始规划，可视化规划结果
  std::vector<ObstacleInfo> static_obstacles;
  ReferenceLine reference_line_planed;
  emplanner_->Plan(plan_start_point, reference_line_, localization_info_,
                  static_obstacles, &reference_line_planed);
  visualizer_->visualizePlannedLine(reference_line_planed);
  
  
  RCLCPP_INFO(this->get_logger(), "规划完成");
  is_collision_ = false;// 假设没有碰撞
}

void runAstarPlanner() {
  // 创建EmplannerNode实例
}

std::vector<MapPoint> EmplannerNode::GetGlobalPath() {
  std::vector<MapPoint> global_path;
  
  if (global_path_.empty()) {
    return global_path;
  }
  
  // 添加第一个点
  MapPoint last_added_point;
  last_added_point.x = global_path_[0].first;
  last_added_point.y = global_path_[0].second;
  global_path.push_back(last_added_point);
  
  // 距离阈值设置为0.5米
  const double distance_threshold = 0.5;
  
  // 遍历其余点，根据距离采样
  for (size_t i = 1; i < global_path_.size(); ++i) {
    double current_x = global_path_[i].first;
    double current_y = global_path_[i].second;
    
    // 计算与上一个添加点的欧几里得距离
    double dx = current_x - last_added_point.x;
    double dy = current_y - last_added_point.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    // 如果距离超过阈值，添加该点
    if (distance >= distance_threshold) {
      MapPoint new_point;
      new_point.x = current_x;
      new_point.y = current_y;
      global_path.push_back(new_point);
      last_added_point = new_point;
    }
  }
  
  // 确保添加最后一个点（如果不是最后添加的）
  size_t last_index = global_path_.size() - 1;
  if (global_path.back().x != global_path_[last_index].first || 
      global_path.back().y != global_path_[last_index].second) {
    MapPoint end_point;
    end_point.x = global_path_[last_index].first;
    end_point.y = global_path_[last_index].second;
    global_path.push_back(end_point);
  }
  
  return global_path;
}

std::vector<MapPoint> EmplannerNode::CreatePath() {
  std::vector<MapPoint> routing_path_points;
  routing_path_points.clear();
  
  MapPoint point;
  point.x = 0;
  point.y = 0;
  routing_path_points.push_back(point);
  
  double x = 0, y = 0;
  
  // 第1段
  for (int i = 0; i < 10; i++) {
    x = x + 1;
    y = y + 0;
    point.x = x;
    point.y = y;
    routing_path_points.push_back(point);
  }

  // 第2段
  for (int i = 0; i < 10; i++) {
    x = x;
    y = y + 1;
    point.x = x;
    point.y = y;
    routing_path_points.push_back(point);
  }
  
  // 第3段
  for (int i = 0; i < 10; i++) {
    x = x + 1;
    y = y;
    point.x = x;
    point.y = y;
    routing_path_points.push_back(point);
  }

  // 第4段
  for (int i = 0; i < 10; i++) {
    x = x;
    y = y - 1;
    point.x = x;
    point.y = y;
    routing_path_points.push_back(point);
  }

  // 第5段
  for (int i = 0; i < 20; i++) {
    x = x + 1;
    y = y;
    point.x = x;
    point.y = y;
    routing_path_points.push_back(point);
  }

  return routing_path_points;
}

void EmplannerNode::loadParameters() {
  
  this->declare_parameter("planning_frequency", 1000);
  this->declare_parameter("map_yaml_filename", "/home/nvidia/lc/emplanner/src/emplanner/maps/map.yaml");
  
  this->get_parameter("planning_frequency", planning_frequency_);
  this->get_parameter("map_yaml_filename", map_yaml_filename_);

  
  RCLCPP_INFO(this->get_logger(), "地图YAML文件路径: %s", map_yaml_filename_.c_str());
}

// 加载地图
void EmplannerNode::loadAndPublishMap(const std::string& map_yaml_filename){
  
  RCLCPP_INFO(this->get_logger(), "正在加载地图: %s", map_yaml_filename.c_str());
  try {
    // 创建地图消息
    auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    // 使用Nav2提供的工具加载地图
    nav2_map_server::loadMapFromYaml(map_yaml_filename, *map_msg);
    occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map_msg);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    map_pub_->publish(*map_msg);
    RCLCPP_INFO(this->get_logger(), "地图已成功加载和发布，尺寸: %dx%d", map_msg->info.width, map_msg->info.height);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "加载YAML文件失败: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "加载地图失败: %s", e.what());
  }
}

void EmplannerNode::SavePathToFile(const Vector2D &points, const std::string& filename) {
  plt::clf(); // 清除当前图形
  plt::plot(points.x, points.y);
  plt::title(filename);
  plt::xlabel("X");
  plt::ylabel("Y");
  plt::axis("equal"); // 设置坐标轴比例相等
  plt::save(filename); // 保存图像
}

} // namespace emplanner
