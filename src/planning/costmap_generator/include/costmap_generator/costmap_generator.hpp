#ifndef COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
#define COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_

#include "costmap_generator/utils/points_to_costmap.hpp"

#include "costmap_generator_node_parameters.hpp"

#include <grid_map_ros/GridMapRosConverter.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <grid_map_msgs/msg/grid_map.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <vector>

class CostmapGenerator : public rclcpp::Node
{
public:
  explicit CostmapGenerator(const rclcpp::NodeOptions & node_options);

private:

  std::shared_ptr<::costmap_generator_node::ParamListener> param_listener_;
  std::shared_ptr<::costmap_generator_node::Params> param_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  sensor_msgs::msg::PointCloud2::ConstSharedPtr points_;

  grid_map::GridMap costmap_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr pub_occupancy_grid_;
  
  // 订阅者，点云
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  PointsToCostmap points2costmap_{};

  // 存储最新点云数据
  sensor_msgs::msg::PointCloud2::ConstSharedPtr latest_points_;

  // 图层名称
  struct LayerName
  {
    static constexpr const char * points = "points";
    static constexpr const char * combined = "combined";
  };

  void update_data();

  void set_current_pose();

  void set_grid_center(const geometry_msgs::msg::TransformStamped & tf);

  void onTimer();

  void initGridmap();

  void publishCostmap(
    const grid_map::GridMap & costmap, const geometry_msgs::msg::TransformStamped & tf);

  void onPoints(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg);

  grid_map::Matrix generatePointsCostmap(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr & in_points, const double vehicle_to_map_z);
  
  grid_map::Matrix generateCombinedCostmap();
};

#endif  // AUTOWARE__COSTMAP_GENERATOR__COSTMAP_GENERATOR_HPP_
