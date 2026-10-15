#include "node_monitor/node_monitor_ui.hpp"
#include <QVBoxLayout>
#include <QHeaderView>
#include <QTableWidgetItem>
#include <QFont>
#include <QCloseEvent>

namespace node_monitor_ros2
{

NodeMonitorUI::NodeMonitorUI()
: QMainWindow(),
  keep_spinning_(true)
{
  // 设置窗口标题和大小
  setWindowTitle("ROS2节点监控器");
  setGeometry(100, 100, 600, 400);
  
  // 创建中心部件和布局
  QWidget* central_widget = new QWidget();
  setCentralWidget(central_widget);
  QVBoxLayout* layout = new QVBoxLayout(central_widget);
  
  // 创建标题标签
  title_label_ = new QLabel("ROS2节点状态监控");
  title_label_->setAlignment(Qt::AlignCenter);
  QFont title_font;
  title_font.setPointSize(14);
  title_font.setBold(true);
  title_label_->setFont(title_font);
  layout->addWidget(title_label_);
  
  // 创建表格
  table_ = new QTableWidget();
  table_->setColumnCount(2);
  table_->setHorizontalHeaderLabels({"节点名称", "状态"});
  table_->horizontalHeader()->setSectionResizeMode(0, QHeaderView::Stretch);
  layout->addWidget(table_);
  
  // 初始化ROS2节点
  node_monitor_ = std::make_shared<NodeMonitor>();
  
  // 设置回调函数
  node_monitor_->setStatusChangeCallback(
    [this](const std::map<std::string, NodeStatus>& status) {
      updateTable(status);
    });
  
  // 初始化节点表格
  initTable();
  
  // 设置定时器更新表格
  timer_ = new QTimer(this);
  connect(timer_, &QTimer::timeout, this, &NodeMonitorUI::updateStatus);
  timer_->start(5000); // 每5秒更新一次UI
  
  // 创建ROS2消息处理线程
  ros_thread_ = std::thread(&NodeMonitorUI::rosSpinThread, this);
}

NodeMonitorUI::~NodeMonitorUI()
{
  // 停止ROS2消息处理线程
  keep_spinning_ = false;
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }
  
  // 清理UI资源
  delete timer_;
  // Qt自动管理其他UI组件的内存
}

void NodeMonitorUI::initTable()
{
  // 获取节点列表
  const auto& nodes = node_monitor_->checkNodes();
  
  // 设置表格行数
  table_->setRowCount(nodes.size());
  
  int row = 0;
  for (const auto& node_pair : nodes)
  {
    // 设置节点名称
    QTableWidgetItem* name_item = new QTableWidgetItem(QString::fromStdString(node_pair.first));
    table_->setItem(row, 0, name_item);
    
    // 设置初始状态为"未知"
    QTableWidgetItem* status_item = new QTableWidgetItem("未知");
    status_item->setTextAlignment(Qt::AlignCenter);
    table_->setItem(row, 1, status_item);
    
    row++;
  }
  
  // 首次更新状态
  updateStatus();
}

void NodeMonitorUI::updateStatus()
{
  // 获取最新节点状态
  auto status = node_monitor_->checkNodes();
  updateTable(status);
}

void NodeMonitorUI::updateTable(const std::map<std::string, NodeStatus>& status)
{
  int row = 0;
  for (const auto& node_pair : status)
  {
    QTableWidgetItem* status_item = table_->item(row, 1);
    if (!status_item) {
      continue;
    }
    
    QString status_text;
    QColor status_color;
    
    switch (node_pair.second)
    {
      case NodeStatus::RUNNING:
        status_text = "运行中";
        status_color = QColor(0, 200, 0); // 绿色
        break;
      case NodeStatus::STOPPED:
        status_text = "已停止";
        status_color = QColor(255, 0, 0); // 红色
        break;
      default:
        status_text = "未知";
        status_color = QColor(128, 128, 128); // 灰色
        break;
    }
    
    status_item->setText(status_text);
    status_item->setForeground(status_color);
    
    row++;
  }
}

void NodeMonitorUI::closeEvent(QCloseEvent* event)
{
  // 停止定时器
  timer_->stop();
  
  // 停止ROS2消息处理线程
  keep_spinning_ = false;
  if (ros_thread_.joinable()) {
    ros_thread_.join();
  }
  
  // 接受关闭事件
  event->accept();
}

void NodeMonitorUI::rosSpinThread()
{
  // 在单独的线程中处理ROS2消息
  while (keep_spinning_ && rclcpp::ok()) {
    rclcpp::spin_some(node_monitor_);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

} // namespace node_monitor_ros2