#include "emplanner/hya_star_node.hpp"

namespace emplanner {

HyaStarNode::HyaStarNode(const rclcpp::NodeOptions& options)
    : Node("HyaStarNode", options){

  end_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/end_pose", 1, 
        std::bind(&HyaStarNode::endPoseCallback, this, std::placeholders::_1));

  vehicle_state_sub_ = this->create_subscription<car_msg::msg::VehicleState>(
        "/vehicle_state", 1, 
        std::bind(&HyaStarNode::vehicleStateCallback, this, std::placeholders::_1));
  
  occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/current_lidar_occ_map", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local(),  // 使用持久订阅，确保获取最后一条消息
      std::bind(&HyaStarNode::mapCallback, this, std::placeholders::_1));

  // 发布地图栅格数据
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/global_map", 1);
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 1);

  // 车辆参数配置
  auto vehicle_info = getVehicleInfo();
  vehicle_shape_.length = vehicle_info.vehicle_length_m;
  vehicle_shape_.width = vehicle_info.vehicle_width_m;
  vehicle_shape_.base_length = vehicle_info.wheel_base_m;
  vehicle_shape_.max_steering = vehicle_info.max_steer_angle_rad;
  vehicle_shape_.base2back = vehicle_info.rear_overhang_m;
  // 扩展车辆形状以适应碰撞检测
  VehicleShape extended_vehicle_shape = vehicle_shape_;
  const double margin = 0.2; // 扩展边距 [m]
  extended_vehicle_shape.length += margin;
  extended_vehicle_shape.width += margin;
  extended_vehicle_shape.base2back += margin / 2;
  extended_vehicle_shape.setMinMaxDimension();

  // 初始化配置
  loadParameters();
  loadAndPublishMap(map_yaml_filename_);
  a_star_planner_ = std::make_unique<AStarPlanner>();
  planner_common_param_ = getPlannerCommonParam();
  algo_ = std::make_unique<AstarSearch>(planner_common_param_, vehicle_shape_, *this);

  // 初始化 TF 缓冲区和监听器
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);


  timer_ = this->create_wall_timer(std::chrono::milliseconds(planning_frequency_), 
                                  std::bind(&HyaStarNode::RunOnce, this));
  
  last_planning_time_ = this->now();
  RCLCPP_INFO(this->get_logger(), "EmplannerNode构造完成");
}

// 获取车辆信息
VehicleInfo HyaStarNode::getVehicleInfo() {
  VehicleInfo vehicle_info;
  const double wheel_radius_m = 0.15; // 轮胎半径
  const double wheel_width_m = 0.08; // 轮胎宽度
  const double wheel_base_m = 0.8; // 轴距
  const double wheel_tread_m = 0.05; 
  const double front_overhang_m = 0.1; // 前悬
  const double rear_overhang_m = 0.1; // 后悬
  const double left_overhang_m = 0.00; // 左悬
  const double right_overhang_m = 0.00; // 右悬
  const double vehicle_height_m = 0.8; // 车辆高度
  const double max_steer_angle_rad = M_PI / 5; // 最大转向角度

  vehicle_info.createVehicleInfo(wheel_radius_m, wheel_width_m,
    wheel_base_m, wheel_tread_m, front_overhang_m, rear_overhang_m,
    left_overhang_m, right_overhang_m, vehicle_height_m, max_steer_angle_rad); // 最大转向角度

  return vehicle_info;
}

// 从tf变换获取当前位姿
geometry_msgs::msg::PoseStamped HyaStarNode::getCurrentPose()
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
    return current_pose;
  }
  
  return current_pose;
}

// 实时occ反馈
void HyaStarNode::mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
  // 使用互斥锁更新最新地图
  std::lock_guard<std::mutex> lock(map_mutex_);
  cur_occupancy_grid_ = msg;
  RCLCPP_INFO(this->get_logger(), "收到新地图");
}

// 在需要时获取地图数据的方法
nav_msgs::msg::OccupancyGrid::SharedPtr HyaStarNode::getLatestMap() {
  std::lock_guard<std::mutex> lock(map_mutex_);
  return cur_occupancy_grid_;
}

// 全局occ转为二维mapdata
void HyaStarNode::occToMapData(const nav_msgs::msg::OccupancyGrid &occupancy_grid) {
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


// 收到终点信息 进行全局路径规划
void HyaStarNode::endPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
  end_pose_ = *msg;
  is_end_pose_set_ = true;
  RCLCPP_INFO(this->get_logger(), "收到终点位置: x=%f, y=%f", end_pose_.pose.position.x, end_pose_.pose.position.y);
  auto result = globalPathPlanning();
}

void HyaStarNode::vehicleStateCallback(const car_msg::msg::VehicleState::SharedPtr msg) {
  current_speed_ = msg->velocity;
}

// 全局路径规划,并发布
bool HyaStarNode::globalPathPlanning() {
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
  RCLCPP_INFO(this->get_logger(), "全局路径规划成功，路径点数量: %zu", global_path_.size());

  nav_msgs::msg::Path path_msg;
  path_msg.header.frame_id = map_frame_id_;
  path_msg.header.stamp = this->now();
  path_msg.poses.clear();
  
  for (const auto& point : global_path_) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_msg.header;
    pose.pose.position.x = point.first;
    pose.pose.position.y = point.second;
    pose.pose.position.z = 0.0; // 假设z轴为0
    pose.pose.orientation.w = 1.0; // 默认朝向
    path_msg.poses.push_back(pose);
  }
  path_pub_->publish(path_msg);
  
  return true;
}

// 将全局路径点转换为地图坐标系下的点
geometry_msgs::msg::Pose HyaStarNode::transform_pose(const geometry_msgs::msg::Pose & pose, const geometry_msgs::msg::TransformStamped & transform){
  geometry_msgs::msg::PoseStamped transformed_pose;
  geometry_msgs::msg::PoseStamped orig_pose;
  orig_pose.pose = pose;
  tf2::doTransform(orig_pose, transformed_pose, transform);
  return transformed_pose.pose;
}

geometry_msgs::msg::TransformStamped HyaStarNode::getTransform(const std::string & from, const std::string & to){
  geometry_msgs::msg::TransformStamped tf;
  try {
    tf = tf_buffer_->lookupTransform(from, to, rclcpp::Time(0), rclcpp::Duration::from_seconds(1.0));
  } catch (const tf2::TransformException & ex) {
    RCLCPP_ERROR(get_logger(), "%s", ex.what());
  }
  return tf;
}


// 执行一次局部路径规划
void HyaStarNode::RunOnce() {
  
  // auto is_collision_= isCollisionDetect();
  // 如果已经在规划，直接返回
  if (planning_in_progress_) {
    return;
  }
  
  // 获取当前时间
  rclcpp::Time current_time = this->now();
  double time_diff = (current_time - last_planning_time_).seconds();

  if(is_collision_){
    RCLCPP_INFO(this->get_logger(), "发生碰撞，重新规划");
  }else{
    if(time_diff < 3.0){
      return; // 没有碰撞，距离上次规划时间未达到一定阈值
    }
  }

  // 进入条件
  // 1. 有碰撞
  // 2. 没有碰撞，但是距离上次规划时间间隔达到一定程度
  
  // 0. 当前位置,地图
  auto current_pose = getCurrentPose();
  if(current_pose.header.frame_id.empty()){
    return;
  }
  
  auto current_occ = getLatestMap();
  if(!current_occ){
    RCLCPP_INFO(this->get_logger(), "没有收到当前occ map");
    return;
  }
  algo_->setMap(*current_occ);
  RCLCPP_INFO(this->get_logger(), "开始执行一次局部路径规划...");

  // 1. 根据全局路径获取下一个目标点
  // auto point = getNextTargetPose();
  geometry_msgs::msg::PoseStamped end_pose;
  end_pose.pose = end_pose_.pose;
  end_pose.header = end_pose_.header;
  
  
  // 2. 将起点终点信息转到当前occ坐标系上
  auto start_pose_in_map = transform_pose(current_pose.pose, getTransform(current_occ->header.frame_id, current_pose_.header.frame_id));
  auto end_pose_in_map = transform_pose(end_pose_.pose, getTransform(current_occ->header.frame_id, end_pose_.header.frame_id));

  // 3. 开始规划
  last_planning_time_ = this->now();
  planning_in_progress_ = true;
  if(!algo_->makePlan(start_pose_in_map, end_pose_in_map)){
    planning_in_progress_ = false;
    RCLCPP_INFO(this->get_logger(), "局部路径规划失败");
  }

  // 4. 规划成功,生成路径
  auto way_points = algo_->getWaypoints();
  planning_in_progress_ = false;
  RCLCPP_INFO(this->get_logger(), "局部路径规划成功，路径点数量");
}

void HyaStarNode::loadParameters() {
  
  this->declare_parameter("planning_frequency", 1000);
  this->declare_parameter("map_yaml_filename", "/home/nvidia/lc/emplanner/src/emplanner/maps/map.yaml");
  
  this->get_parameter("planning_frequency", planning_frequency_);
  this->get_parameter("map_yaml_filename", map_yaml_filename_);

  
  RCLCPP_INFO(this->get_logger(), "地图YAML文件路径: %s", map_yaml_filename_.c_str());
}

PlannerCommonParam HyaStarNode::getPlannerCommonParam(){
  
  PlannerCommonParam p;
  
  // search configs
  p.time_limit = declare_parameter<double>("time_limit");
  p.theta_size = declare_parameter<int>("theta_size");
  p.angle_goal_range = declare_parameter<double>("angle_goal_range");
  p.curve_weight = declare_parameter<double>("curve_weight");
  p.reverse_weight = declare_parameter<double>("reverse_weight");
  p.direction_change_weight = declare_parameter<double>("direction_change_weight");
  p.lateral_goal_range = declare_parameter<double>("lateral_goal_range");
  p.longitudinal_goal_range = declare_parameter<double>("longitudinal_goal_range");
  p.max_turning_ratio = declare_parameter<double>("max_turning_ratio");
  p.turning_steps = declare_parameter<int>("turning_steps");

  // costmap configs
  p.obstacle_threshold = declare_parameter<int>("obstacle_threshold");

  return p;
}


// 加载地图
void HyaStarNode::loadAndPublishMap(const std::string& map_yaml_filename){
  
  RCLCPP_INFO(this->get_logger(), "正在加载地图: %s", map_yaml_filename.c_str());
  try {
    // 创建地图消息
    auto map_msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    // 使用Nav2提供的工具加载地图
    nav2_map_server::loadMapFromYaml(map_yaml_filename, *map_msg);
    map_occupancy_grid_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(*map_msg);
    std::this_thread::sleep_for(std::chrono::seconds(5));
    map_pub_->publish(*map_msg);
    RCLCPP_INFO(this->get_logger(), "地图已成功加载和发布，尺寸: %dx%d", map_msg->info.width, map_msg->info.height);
  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "加载YAML文件失败: %s", e.what());
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "加载地图失败: %s", e.what());
  }
}

} // namespace emplanner


int main(int argc, char* argv[]) {
  // 初始化ROS2
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto node = std::make_shared<emplanner::HyaStarNode>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
} 
