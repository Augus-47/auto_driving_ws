#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion

from car_msg.msg import VehicleState


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')

        # 参数
        self.declare_parameter('wheelbase', 0.80)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'wheel_odom')
        self.declare_parameter('vehicle_state_topic', '/vehicle_info')

        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.vehicle_state_topic = self.get_parameter('vehicle_state_topic').get_parameter_value().string_value

        # 订阅车辆状态
        self.state_sub = self.create_subscription(
            VehicleState,
            self.vehicle_state_topic,
            self.state_callback,
            10
        )

        # 发布轮式里程计
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        # 里程计状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # 上一帧 ROS 时间
        self.last_time_ns = None

        self.get_logger().info(
            f'Wheel odometry node started. wheelbase={self.wheelbase:.3f} m, '
            f'odom_topic={self.odom_topic}, vehicle_state_topic={self.vehicle_state_topic}'
        )

    @staticmethod
    def yaw_to_quaternion(yaw: float) -> Quaternion:
        """平面 yaw 转四元数"""
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q

    @staticmethod
    def normalize_angle(angle: float) -> float:
        """把角度约束到 [-pi, pi]"""
        return math.atan2(math.sin(angle), math.cos(angle))

    def state_callback(self, msg: VehicleState):
        # 使用 ROS 当前时间
        now = self.get_clock().now()
        now_ns = now.nanoseconds

        # 第一帧只记时间，不积分
        if self.last_time_ns is None:
            self.last_time_ns = now_ns
            return

        dt = (now_ns - self.last_time_ns) * 1e-9
        if dt <= 0.0:
            self.get_logger().warn(f'Non-positive dt: {dt:.6f}, skip this frame.')
            return

        self.last_time_ns = now_ns

        # 读取车速和转角
        v = float(msg.velocity)        # m/s
        steering_deg = float(msg.steering)   # 前轮转角，单位：度
        # 例如限制前轮最大转角为 ±30 度
        steering_deg = max(-45.0, min(45.0, steering_deg))
        steering = math.radians(steering_deg)  # 转成弧度再参与计算

        # Ackermann 模型计算角速度
        if abs(steering) > 1e-6:
            omega = v * math.tan(steering) / self.wheelbase
        else:
            omega = 0.0
        
        self.get_logger().info(
    f"v={v:.3f}, steering_deg={steering_deg:.3f}, "
    f"steering_rad={steering:.4f}, omega={omega:.4f}, dt={dt:.4f}",
    throttle_duration_sec=3.0
)


        # 中值积分更新位姿
        half_yaw = self.yaw + 0.5 * omega * dt
        self.x += v * math.cos(half_yaw) * dt
        self.y += v * math.sin(half_yaw) * dt
        self.yaw += omega * dt
        self.yaw = self.normalize_angle(self.yaw)

        # 组装 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = now.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # 位姿
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation = self.yaw_to_quaternion(self.yaw)

        # 速度（base_link 下）
        odom_msg.twist.twist.linear.x = v
        odom_msg.twist.twist.linear.y = 0.0
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.x = 0.0
        odom_msg.twist.twist.angular.y = 0.0
        odom_msg.twist.twist.angular.z = omega

        # pose 协方差
        # x, y, yaw 相对可信；z, roll, pitch 对平面车不使用，给大协方差
        odom_msg.pose.covariance = [
            0.05, 0.0,  0.0,   0.0,   0.0,   0.0,
            0.0,  0.05, 0.0,   0.0,   0.0,   0.0,
            0.0,  0.0,  1e6,   0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,   1e6,   0.0,   0.0,
            0.0,  0.0,  0.0,   0.0,   1e6,   0.0,
            0.0,  0.0,  0.0,   0.0,   0.0,   0.1
        ]

        # twist 协方差
        # 只让 EKF 相信 vx 和 wz，其它没测的速度分量给大协方差
        odom_msg.twist.covariance = [
            0.05, 0.0,  0.0,   0.0,   0.0,   0.0,
            0.0,  1e6,  0.0,   0.0,   0.0,   0.0,
            0.0,  0.0,  1e6,   0.0,   0.0,   0.0,
            0.0,  0.0,  0.0,   1e6,   0.0,   0.0,
            0.0,  0.0,  0.0,   0.0,   1e6,   0.0,
            0.0,  0.0,  0.0,   0.0,   0.0,   0.05
        ]

        self.odom_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

