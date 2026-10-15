#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

# 消息类型导入
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from car_msg.msg import VehicleState  # 自定义消息
import tf2_ros

class AckermannOdomNode(Node):
    def __init__(self):
        super().__init__('ackermann_odom_node')

        # ==========================================
        # 1. 车辆参数配置 (请务必拿卷尺测量修改！)
        # ==========================================
        self.wheelbase = 0.74   # 轴距(前后轮轴心距离), 单位: 米
        self.frame_id = 'odom'
        self.child_frame_id = 'base_link'

        # ==========================================
        # 2. 订阅与发布
        # ==========================================
        # 订阅底层反馈 (注意 QoS 设置为 BEST_EFFORT 以匹配底层)
        qos_policy = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.sub_vehicle = self.create_subscription(
            VehicleState,
            '/vehicle_info',
            self.vehicle_callback,
            qos_policy
        )

        # 发布标准里程计
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        
        # 发布 TF 变换 (odom -> base_link)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # ==========================================
        # 3. 内部状态变量
        # ==========================================
        self.x = 0.0        # 位置 X (m)
        self.y = 0.0        # 位置 Y (m)
        self.th = 0.0       # 航向角 Yaw (rad)
        
        self.v = 0.0        # 当前线速度 (m/s)
        self.delta = 0.0    # 当前前轮转角 (rad)
        
        self.last_time = self.get_clock().now()
        
        # 创建定时器进行积分计算 (20Hz)
        self.create_timer(0.05, self.update_odometry)
        
        self.get_logger().info("里程计节点已启动！监听 /vehicle_info 中...")

    def vehicle_callback(self, msg):
        """
        回调函数：只负责更新当前的速度和角度状态
        """
        # --- 速度处理 ---
        # 假设 msg.velocity 单位是 mm/s (根据CAN协议)
        # 如果你发现推车 1米，Rviz里走了 1000米，请把这里除以 1000.0
        self.v = msg.velocity / 100.0

        # --- 角度处理 ---
        # 假设 msg.steering 单位是 度 (Degree)
        # 如果 CAN 协议里 21度=2100，而这里读到的是 21.0，说明驱动层除过100了
        # 我们需要将其转换为 弧度 (Radian) 供计算使用
        self.delta = msg.steering * (math.pi / 180.0)

    def update_odometry(self):
        """
        定时积分函数：计算位姿变化并发布
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        # 1. 阿克曼运动学模型
        # v_x = v (车身坐标系向前)
        # v_y = 0 (假设无侧滑)
        # omega = (v / L) * tan(delta)
        
        omega = (self.v / self.wheelbase) * math.tan(self.delta)

        # 2. 航位推算 (积分)
        # 简单的欧拉积分
        delta_x = self.v * math.cos(self.th) * dt
        delta_y = self.v * math.sin(self.th) * dt
        delta_th = omega * dt

        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # 3. 构建 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id

        # 填充位置
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        q = self.euler_to_quaternion(0, 0, self.th)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # 填充速度 (Twist)
        odom_msg.twist.twist.linear.x = self.v
        odom_msg.twist.twist.angular.z = omega

        # 发布
        self.pub_odom.publish(odom_msg)

        # 4. 发布 TF (这是 SLAM 必须的)
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.frame_id
        t.child_frame_id = self.child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_msg.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(t)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = AckermannOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
