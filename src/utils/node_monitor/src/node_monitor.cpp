#include "node_monitor/node_monitor.hpp"
#include <algorithm>

namespace node_monitor_ros2
{

NodeMonitor::NodeMonitor() : Node("node_monitor")
{

  declareNodeParameters();

  loadNodesFromParameters();
  
  // 初始化节点状态
  for (const auto& node : nodes_to_monitor_) {
    node_status_[node] = NodeStatus::UNKNOWN;
  }
  
  // 创建定时器，每秒检查一次节点状态
  timer_ = create_wall_timer(
    std::chrono::seconds(3),
    std::bind(&NodeMonitor::timerCallback, this));
  
  RCLCPP_INFO(get_logger(), "节点监控已启动...");
}

// 声明节点参数
void NodeMonitor::declareNodeParameters()
{
  // 声明参数
  rcl_interfaces::msg::ParameterDescriptor desc;
  desc.description = "要监控的ROS2节点列表";
  desc.read_only = false;
  
  this->declare_parameter("nodes_to_monitor", std::vector<std::string>{}, desc);
  
  // 设置参数回调
  param_callback_handle_ = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      
      for (const auto & param : parameters) {
        if (param.get_name() == "nodes_to_monitor" && 
            param.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
          // 更新节点列表
          nodes_to_monitor_ = param.as_string_array();
          
          // 重置节点状态
          node_status_.clear();
          for (const auto& node : nodes_to_monitor_) {
            node_status_[node] = NodeStatus::UNKNOWN;
          }
          
          RCLCPP_INFO(get_logger(), "节点监控列表已更新，现在监控 %zu 个节点", nodes_to_monitor_.size());
        }
      }
      
      return result;
    });
}

void NodeMonitor::loadNodesFromParameters()
{
  // 从参数加载节点列表
  if (this->has_parameter("nodes_to_monitor")) {
    nodes_to_monitor_ = this->get_parameter("nodes_to_monitor").as_string_array();
    
    if (nodes_to_monitor_.empty()) {
      // 如果参数为空，使用默认值
      setInitNodeList();
      this->set_parameter(rclcpp::Parameter("nodes_to_monitor", nodes_to_monitor_));
    }
  } else {
    setInitNodeList();
  }
}

void NodeMonitor::setStatusChangeCallback(StatusChangeCallback callback)
{
  status_change_callback_ = callback;
}

std::map<std::string, NodeStatus> NodeMonitor::checkNodes()
{
  bool status_changed = false;
  std::map<std::string, NodeStatus> status_info;
  
  for (const auto& node : nodes_to_monitor_)
  {
    NodeStatus current_status = checkNodePresence(node);
    status_info[node] = current_status;
    
    // 仅在状态变化时输出日志
    if (current_status != node_status_[node])
    {
      status_changed = true;
      if (current_status == NodeStatus::RUNNING)
      {
        RCLCPP_INFO(get_logger(), "节点 %s 已启动", node.c_str());
      }
      else
      {
        RCLCPP_ERROR(get_logger(), "节点 %s 已停止运行", node.c_str());
      }
      node_status_[node] = current_status;
    }
  }
  
  // 如果有状态变化且设置了回调，则通知UI
  if (status_changed && status_change_callback_)
  {
    status_change_callback_(node_status_);
  }
  
  return status_info;
}

// 检查节点状态
NodeStatus NodeMonitor::checkNodePresence(const std::string& node_name)
{
  try{
    // 获取已知节点列表
    auto node_names_and_namespaces = get_node_names();
    
    // 转换为与请求格式匹配的完整节点名称
    std::vector<std::string> full_node_names;
    for (const auto& node_info : node_names_and_namespaces)
    {
      std::string full_name = node_info;
      
      if (full_name[0] != '/') {
        full_name = "/" + full_name;
      }
      full_node_names.push_back(full_name);
    }
    
    // 检查节点是否在列表中
    if (std::find(full_node_names.begin(), full_node_names.end(), node_name) != full_node_names.end())
    {
      return NodeStatus::RUNNING;
    }
    return NodeStatus::STOPPED;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "查询节点 %s 状态时出错: %s", node_name.c_str(), e.what());
    return NodeStatus::STOPPED;
  }
}

void NodeMonitor::timerCallback()
{
  checkNodes();
}

void NodeMonitor::setInitNodeList(){

  nodes_to_monitor_ = {
    "/camera/camera",
    "/laser_avoid"
  };
}

} // namespace node_monitor_ros2