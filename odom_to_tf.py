#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class OdomToTfNode(Node):
    def __init__(self):
        super().__init__('odom_to_tf_node')
        
        # 强制开启仿真时间，保证和 Bag 包时间线一致
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        # 初始化 TF 广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 订阅里程计话题
        self.odom_topic = '/odom'
        self.subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.get_logger().info(f"【已强制开启仿真时间】正在监听 {self.odom_topic} 话题，发布 odom -> base_link...")

    def odom_callback(self, msg):
        t = TransformStamped()

        # 直接使用 odom 消息原封不动的时间戳，不加任何缓冲
        t.header.stamp = msg.header.stamp
        
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation.x = msg.pose.pose.orientation.x
        t.transform.rotation.y = msg.pose.pose.orientation.y
        t.transform.rotation.z = msg.pose.pose.orientation.z
        t.transform.rotation.w = msg.pose.pose.orientation.w

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToTfNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
