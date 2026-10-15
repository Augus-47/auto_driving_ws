#include "emplanner/emplanner_node.hpp"
#include <memory>

int main(int argc, char* argv[]) {
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建节点选项
  rclcpp::NodeOptions options;
  // 创建EmplannerNode实例
  auto node = std::make_shared<emplanner::EmplannerNode>(options);
  // 进入ROS2的事件循环
  rclcpp::spin(node);
  // 关闭ROS2
  rclcpp::shutdown();
  return 0;
} 