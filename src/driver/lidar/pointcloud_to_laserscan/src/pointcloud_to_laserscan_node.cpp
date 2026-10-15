/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2010-2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*
 * Author: Paul Bovbel
 */

#include "pointcloud_to_laserscan/pointcloud_to_laserscan_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>

#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/create_timer_ros.h"

namespace pointcloud_to_laserscan
{

PointCloudToLaserScanNode::PointCloudToLaserScanNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("pointcloud_to_laserscan", options)
{
  // =========================
  // 1. 读取参数
  // =========================

  // 目标坐标系：
  // 如果为空，则不在本节点里做 TF 变换，直接使用点云原始 frame
  target_frame_ = this->declare_parameter("target_frame", "");

  // 原代码默认 0.01（10ms）太小，车辆运动时很容易因为 TF 稍有延迟而丢帧
  // 这里默认改成 0.1（100ms），更稳一些
  tolerance_ = this->declare_parameter("transform_tolerance", 0.1);

  // 有些平台 std::thread::hardware_concurrency() 可能返回 0，这里做保护
  int default_queue_size = static_cast<int>(std::thread::hardware_concurrency());
  if (default_queue_size <= 0) {
    default_queue_size = 8;
  }
  input_queue_size_ = this->declare_parameter("queue_size", default_queue_size);

  // 这里是最关键的修复：
  // 原代码用 std::numeric_limits<double>::min()，这不是“最小负数”，而是“最小正数”
  // 会导致 z < 0 的点被错误过滤掉，车辆运动时非常容易造成动态障碍忽隐忽现
  min_height_ = this->declare_parameter("min_height", -std::numeric_limits<double>::max());
  max_height_ = this->declare_parameter("max_height",  std::numeric_limits<double>::max());

  angle_min_ = this->declare_parameter("angle_min", -M_PI);
  angle_max_ = this->declare_parameter("angle_max", M_PI);
  angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
  scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
  range_min_ = this->declare_parameter("range_min", 0.0);
  range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
  inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
  use_inf_ = this->declare_parameter("use_inf", true);

  // 发布 LaserScan
  pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("scan", rclcpp::SystemDefaultsQoS());

  // 打印关键参数，方便排查
  RCLCPP_INFO(this->get_logger(), "pointcloud_to_laserscan node started.");
  RCLCPP_INFO(this->get_logger(), "target_frame: %s", target_frame_.c_str());
  RCLCPP_INFO(this->get_logger(), "transform_tolerance: %.3f", tolerance_);
  RCLCPP_INFO(this->get_logger(), "queue_size: %d", input_queue_size_);
  RCLCPP_INFO(this->get_logger(), "min_height: %.3f, max_height: %.3f", min_height_, max_height_);
  RCLCPP_INFO(this->get_logger(), "angle_min: %.3f, angle_max: %.3f, angle_increment: %.6f",
              angle_min_, angle_max_, angle_increment_);
  RCLCPP_INFO(this->get_logger(), "range_min: %.3f, range_max: %.3f", range_min_, range_max_);

  using std::placeholders::_1;

  // =========================
  // 2. 根据是否设置 target_frame 决定是否启用 TF 过滤
  // =========================
  if (!target_frame_.empty()) {
    // 如果指定 target_frame，则需要等待 TF 可用
    tf2_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface());
    tf2_->setCreateTimerInterface(timer_interface);

    tf2_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf2_);

    message_filter_ = std::make_unique<MessageFilter>(
      sub_,
      *tf2_,
      target_frame_,
      input_queue_size_,
      this->get_node_logging_interface(),
      this->get_node_clock_interface());

    message_filter_->registerCallback(
      std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "Using TF message filter. cloud_in will be transformed to target_frame: %s",
                target_frame_.c_str());
  } else {
    // 如果没有 target_frame，则直接订阅点云并处理
    sub_.registerCallback(std::bind(&PointCloudToLaserScanNode::cloudCallback, this, _1));

    RCLCPP_INFO(this->get_logger(),
                "No target_frame set. Using input cloud frame directly.");
  }

  // =========================
  // 3. 启动订阅监听线程
  // 只有当有人订阅 /scan 时，才去订阅 cloud_in
  // =========================
  subscription_listener_thread_ = std::thread(
    std::bind(&PointCloudToLaserScanNode::subscriptionListenerThreadLoop, this));
}

PointCloudToLaserScanNode::~PointCloudToLaserScanNode()
{
  alive_.store(false);

  // 防止线程未启动或异常情况下直接 join 造成问题
  if (subscription_listener_thread_.joinable()) {
    subscription_listener_thread_.join();
  }
}

void PointCloudToLaserScanNode::subscriptionListenerThreadLoop()
{
  rclcpp::Context::SharedPtr context = this->get_node_base_interface()->get_context();
  const std::chrono::milliseconds timeout(100);

  while (rclcpp::ok(context) && alive_.load()) {
    int subscription_count =
      pub_->get_subscription_count() + pub_->get_intra_process_subscription_count();

    if (subscription_count > 0) {
      if (!sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "Detected subscriber(s) on /scan, starting subscription to cloud_in");

        rclcpp::SensorDataQoS qos;
        qos.keep_last(input_queue_size_);
        sub_.subscribe(this, "cloud_in", qos.get_rmw_qos_profile());
      }
    } else {
      if (sub_.getSubscriber()) {
        RCLCPP_INFO(
          this->get_logger(),
          "No subscribers on /scan, unsubscribing from cloud_in");
        sub_.unsubscribe();
      }
    }

    rclcpp::Event::SharedPtr event = this->get_graph_event();
    this->wait_for_graph_change(event, timeout);
  }

  // 退出前确保取消订阅
  sub_.unsubscribe();
}

void PointCloudToLaserScanNode::cloudCallback(
  sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg)
{
  if (!cloud_msg) {
    RCLCPP_WARN(this->get_logger(), "Received null cloud message.");
    return;
  }

  // =========================
  // 1. 构建输出 LaserScan
  // =========================
  auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
  scan_msg->header = cloud_msg->header;

  // 如果设置了 target_frame，则 LaserScan 的 frame_id 改为 target_frame
  if (!target_frame_.empty()) {
    scan_msg->header.frame_id = target_frame_;
  }

  scan_msg->angle_min = angle_min_;
  scan_msg->angle_max = angle_max_;
  scan_msg->angle_increment = angle_increment_;
  scan_msg->time_increment = 0.0;
  scan_msg->scan_time = scan_time_;
  scan_msg->range_min = range_min_;
  scan_msg->range_max = range_max_;

  if (scan_msg->angle_increment <= 0.0) {
    RCLCPP_ERROR(this->get_logger(), "angle_increment must be > 0.0");
    return;
  }

  if (scan_msg->angle_max <= scan_msg->angle_min) {
    RCLCPP_ERROR(this->get_logger(), "angle_max must be greater than angle_min");
    return;
  }

  // =========================
  // 2. 计算 LaserScan 的射线数量
  // =========================
  uint32_t ranges_size = static_cast<uint32_t>(
    std::ceil((scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment));

  if (ranges_size == 0) {
    RCLCPP_ERROR(this->get_logger(), "Computed ranges_size is 0, check angle parameters.");
    return;
  }

  // 对没有命中的角度，默认填充 inf 或 max_range + epsilon
  if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<float>::infinity());
  } else {
    scan_msg->ranges.assign(ranges_size, static_cast<float>(scan_msg->range_max + inf_epsilon_));
  }

  // =========================
  // 3. 如果有需要，先把点云变换到 target_frame
  // =========================
  if (scan_msg->header.frame_id != cloud_msg->header.frame_id) {
    try {
      auto cloud = std::make_shared<sensor_msgs::msg::PointCloud2>();
      tf2_->transform(*cloud_msg, *cloud, target_frame_, tf2::durationFromSec(tolerance_));
      cloud_msg = cloud;
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Transform failure from frame [%s] to [%s]: %s",
        cloud_msg->header.frame_id.c_str(),
        target_frame_.c_str(),
        ex.what());
      return;
    }
  }

  // =========================
  // 4. 遍历点云，投影到 LaserScan
  // =========================
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
       iter_y(*cloud_msg, "y"),
       iter_z(*cloud_msg, "z");
       iter_x != iter_x.end();
       ++iter_x, ++iter_y, ++iter_z)
  {
    // 4.1 过滤 NaN
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
      continue;
    }

    // 4.2 高度过滤
    if (*iter_z > max_height_ || *iter_z < min_height_) {
      continue;
    }

    // 4.3 距离过滤
    double range = std::hypot(*iter_x, *iter_y);

    if (range < range_min_ || range > range_max_) {
      continue;
    }

    // 4.4 角度过滤
    double angle = std::atan2(*iter_y, *iter_x);

    if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
      continue;
    }

    // 4.5 角度映射到 scan 数组索引
    int index = static_cast<int>((angle - scan_msg->angle_min) / scan_msg->angle_increment);

    // 边界保护，防止 index 因浮点误差越界
    if (index < 0 || index >= static_cast<int>(ranges_size)) {
      continue;
    }

    // 4.6 同一角度只保留最近的点
    if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = static_cast<float>(range);
    }
  }

  // =========================
  // 5. 发布 LaserScan
  // =========================
  pub_->publish(std::move(scan_msg));
}

}  // namespace pointcloud_to_laserscan

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(pointcloud_to_laserscan::PointCloudToLaserScanNode)

