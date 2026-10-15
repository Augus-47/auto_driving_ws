#ifndef NODE_MONITOR_UI_HPP_
#define NODE_MONITOR_UI_HPP_

#include <QMainWindow>
#include <QTableWidget>
#include <QLabel>
#include <QTimer>
#include <memory>
#include "node_monitor/node_monitor.hpp"

namespace node_monitor_ros2
{

class NodeMonitorUI : public QMainWindow
{
  Q_OBJECT

public:
  // 构造函数
  NodeMonitorUI();
  
  // 析构函数
  virtual ~NodeMonitorUI();

private slots:
  // 更新状态
  void updateStatus();
  
  // 关闭事件处理
  void closeEvent(QCloseEvent *event) override;

private:
  // 初始化表格
  void initTable();
  
  // 更新表格
  void updateTable(const std::map<std::string, NodeStatus>& status);
  
  // Qt组件
  QTableWidget* table_;
  QLabel* title_label_;
  QTimer* timer_;
  
  // ROS2节点
  std::shared_ptr<NodeMonitor> node_monitor_;
  
  // 用于ROS2消息处理的线程
  std::thread ros_thread_;
  
  // 指示线程是否应该继续运行
  std::atomic<bool> keep_spinning_;
  
  // ROS消息处理函数
  void rosSpinThread();
};

} // namespace node_monitor_ros2

#endif // NODE_MONITOR_UI_HPP_