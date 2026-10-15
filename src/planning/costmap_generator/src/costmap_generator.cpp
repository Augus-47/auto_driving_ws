#include "costmap_generator/costmap_generator.hpp"

#include <pcl_ros/transforms.hpp>
#include <rclcpp/clock.hpp>
#include <rclcpp/logging.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <tf2/time.h>
#include <tf2/utils.h>

#include <memory>
#include <string>
#include <vector>


// 获取当前位置（map to base_link)
geometry_msgs::msg::PoseStamped::ConstSharedPtr getCurrentPose(
  const tf2_ros::Buffer & tf_buffer, const rclcpp::Logger & logger,
  const rclcpp::Clock::SharedPtr clock)
{
  geometry_msgs::msg::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    return nullptr;
  }

  geometry_msgs::msg::PoseStamped::SharedPtr p(new geometry_msgs::msg::PoseStamped());
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return geometry_msgs::msg::PoseStamped::ConstSharedPtr(p);
}

// 点云转化
pcl::PointCloud<pcl::PointXYZ> getTransformedPointCloud(
  const sensor_msgs::msg::PointCloud2 & pointcloud_msg,
  const geometry_msgs::msg::Transform & transform)
{
  const Eigen::Matrix4f transform_matrix = tf2::transformToEigen(transform).matrix().cast<float>();

  sensor_msgs::msg::PointCloud2 transformed_msg;
  pcl_ros::transformPointCloud(transform_matrix, pointcloud_msg, transformed_msg);

  pcl::PointCloud<pcl::PointXYZ> transformed_pointcloud;
  pcl::fromROSMsg(transformed_msg, transformed_pointcloud);

  return transformed_pointcloud;
}


// 构造
CostmapGenerator::CostmapGenerator(const rclcpp::NodeOptions & node_options)
: Node("costmap_generator", node_options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)
{
  // 参数
  param_listener_ = std::make_shared<::costmap_generator_node::ParamListener>(this->get_node_parameters_interface());
  param_ = std::make_shared<::costmap_generator_node::Params>(param_listener_->get_params());


  // 使用系统默认QoS，更容易兼容大多数发布者
auto qos = rclcpp::SystemDefaultsQoS();

// 或使用传感器数据专用QoS
// auto qos = rclcpp::SensorDataQoS()
     
  points_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "~/input/points_no_ground", qos,
    std::bind(&CostmapGenerator::onPoints, this, std::placeholders::_1));
  
  pub_occupancy_grid_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("~/output/occupancy_grid", 1);

  // Timer
  const auto period_ns = rclcpp::Rate(param_->update_rate).period();
  timer_ = rclcpp::create_timer(this, get_clock(), period_ns, std::bind(&CostmapGenerator::onTimer, this));

  // Initialize
  initGridmap();


}


// 1. 数据更新，障碍物，点云
void CostmapGenerator::update_data(){
  // 仅在需要时从缓存获取点云数据
  if (!latest_points_) {
    RCLCPP_ERROR(this->get_logger(), "No point cloud data received");
    return;
  }
  points_ = latest_points_;
}

// 2. 当前位置
void CostmapGenerator::set_current_pose(){
  current_pose_ = getCurrentPose(tf_buffer_, this->get_logger(), this->get_clock());
}

// 0.定时执行
void CostmapGenerator::onTimer()
{
  
  update_data();      // 获取最新点云
  if(!points_){
    return;
  }
  
  set_current_pose(); // 当前位置

  // 设置地图中心
  geometry_msgs::msg::TransformStamped tf;
  try {
    // map to base_link
    tf = tf_buffer_.lookupTransform(param_->costmap_frame, param_->vehicle_frame, tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "%s", ex.what());
    return;
  }

  set_grid_center(tf);

  costmap_[LayerName::points] = generatePointsCostmap(points_, tf.transform.translation.z);

  costmap_[LayerName::combined] = generateCombinedCostmap();

  // 发布
  publishCostmap(costmap_, tf);
}

// 3. 设置map中心
void CostmapGenerator::set_grid_center(const geometry_msgs::msg::TransformStamped & tf)
{
  // 当前中心
  const auto cur_pos = costmap_.getPosition();
  // 现在需要设置的中心
  const grid_map::Position ref_pos(tf.transform.translation.x, tf.transform.translation.y);
  // 差距
  const auto disp = ref_pos - cur_pos;
  // 分辨率
  const auto resolution = costmap_.getResolution();
  // 偏差
  const grid_map::Position offset(std::round(disp.x() / resolution) * resolution, std::round(disp.y() / resolution) * resolution);
  costmap_.setPosition(cur_pos + offset);// 设置中心
}

// 初始化
void CostmapGenerator::initGridmap(){
  // 设置framed id
  costmap_.setFrameId(param_->costmap_frame);
  // gridmap 参数 （高宽，分辨率，初始位置0，0）
  costmap_.setGeometry(grid_map::Length(param_->grid_length_x, param_->grid_length_y), param_->grid_resolution,
                      grid_map::Position(param_->grid_position_x, param_->grid_position_y));

  // 图层（点云，结合）
  costmap_.add(LayerName::points, param_->grid_min_value);
  costmap_.add(LayerName::combined, param_->grid_min_value);
}

// 4. 根据 点云 产生 图层
grid_map::Matrix CostmapGenerator::generatePointsCostmap(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_points, const double vehicle_to_map_z){
  
  // 点云坐标系转化
  geometry_msgs::msg::TransformStamped points2costmap;
  try {
    points2costmap = tf_buffer_.lookupTransform(param_->costmap_frame, in_points->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {}
  const auto transformed_points = getTransformedPointCloud(*in_points, points2costmap.transform);

  // 配置z阈值
  const auto maximum_height_thres = param_->maximum_lidar_height_thres + vehicle_to_map_z;
  const auto minimum_height_thres = param_->minimum_lidar_height_thres + vehicle_to_map_z;

  // 点云图层产生
  grid_map::Matrix points_costmap = points2costmap_.makeCostmapFromPoints(
    maximum_height_thres, minimum_height_thres, param_->grid_min_value, param_->grid_max_value,
    costmap_, LayerName::points, transformed_points);

  return points_costmap;
}


// 图层结合
grid_map::Matrix CostmapGenerator::generateCombinedCostmap(){
  grid_map::GridMap combined_costmap = costmap_;
  combined_costmap[LayerName::combined].setConstant(param_->grid_min_value);
  // cwiseMax() 是 Eigen 库的逐元素最大值操作
  // 依次将 points、primitives 和 objects 图层的代价值合并到 combined 图层
  // 采用 "取最大值" 策略，确保综合代价反映最危险的障碍物
  combined_costmap[LayerName::combined] = combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::points]);
  return combined_costmap[LayerName::combined];
}

// 发布
void CostmapGenerator::publishCostmap(const grid_map::GridMap & costmap, const geometry_msgs::msg::TransformStamped & tf){
  // Set header
  std_msgs::msg::Header header;
  header.frame_id = param_->costmap_frame;
  header.stamp = this->now();

  nav_msgs::msg::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(
    costmap, LayerName::combined, param_->grid_min_value, param_->grid_max_value,
    out_occupancy_grid);
  out_occupancy_grid.header = header;
  out_occupancy_grid.info.origin.position.z = tf.transform.translation.z;
  pub_occupancy_grid_->publish(out_occupancy_grid);
}

// 添加onPoints回调函数，用于接收点云数据但不立即处理
void CostmapGenerator::onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  // 缓存最新的点云数据，但不立即处理
  latest_points_ = msg;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CostmapGenerator)
