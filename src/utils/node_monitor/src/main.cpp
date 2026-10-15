#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include "node_monitor/node_monitor_ui.hpp"

int main(int argc, char* argv[])
{
  // 初始化ROS2
  rclcpp::init(argc, argv);
  
  // 初始化Qt应用
  QApplication app(argc, argv);
  
  // 创建并显示UI
  node_monitor_ros2::NodeMonitorUI window;
  window.show();
  
  // 运行Qt事件循环
  int result = app.exec();
  
  // 清理ROS2资源
  rclcpp::shutdown();
  
  return result;
}