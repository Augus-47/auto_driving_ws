#ifndef NODE_MONITOR_HPP_
#define NODE_MONITOR_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <string>
#include <vector>
#include <map>
#include <functional>

namespace node_monitor_ros2
{

// 节点状态枚举
enum class NodeStatus
{
  UNKNOWN = 0,
  RUNNING = 1,
  STOPPED = 2
};

// 状态回调函数类型定义
using StatusChangeCallback = std::function<void(const std::map<std::string, NodeStatus>&)>;

class NodeMonitor : public rclcpp::Node
{
public:
  // 构造函数
  explicit NodeMonitor();
  
  // 析构函数
  virtual ~NodeMonitor() = default;
  
  // 设置状态变化回调
  void setStatusChangeCallback(StatusChangeCallback callback);
  
  // 检查所有节点状态
  std::map<std::string, NodeStatus> checkNodes();

  // 添加要监控的节点
  void addNodeToMonitor(const std::string& node_name);
  
  // 移除监控的节点
  void removeNodeFromMonitor(const std::string& node_name);
  
  // 获取当前监控的节点列表
  std::vector<std::string> getMonitoredNodes() const;
  void setInitNodeList();

private:
  // 检查单个节点是否存在
  NodeStatus checkNodePresence(const std::string& node_name);
  
  // 定时器回调函数
  void timerCallback();
  void declareNodeParameters();// 声明节点列表参数
  void loadNodesFromParameters();// 从参数服务器加载节点列表
  
  // 需要监控的节点列表
  std::vector<std::string> nodes_to_monitor_;
  
  // 节点状态映射
  std::map<std::string, NodeStatus> node_status_;
  
  // 定时器
  rclcpp::TimerBase::SharedPtr timer_;
  
  // 状态变化回调
  StatusChangeCallback status_change_callback_;

  // 参数回调处理
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

} // namespace node_monitor_ros2

#endif // NODE_MONITOR_HPP_