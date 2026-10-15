#!/usr/bin/env python3
import math
import sys
import yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

class RouteRunner(Node):
    def __init__(self, route_yaml: str):
        super().__init__('route_runner_take_photo')
        self.route_yaml = route_yaml

        self.nav = BasicNavigator()

        # 触发拍照的服务客户端
        self.cli = self.create_client(Trigger, '/save_image')

    def load_poses(self):
        with open(self.route_yaml, 'r') as f:
            data = yaml.safe_load(f)

        poses = []
        stamp = self.get_clock().now().to_msg()
        for wp in data['waypoints']:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.header.stamp = stamp
            ps.pose.position.x = float(wp['x'])
            ps.pose.position.y = float(wp['y'])
            yaw = float(wp.get('yaw', 0.0))
            qx, qy, qz, qw = quat_from_yaw(yaw)
            ps.pose.orientation.x = qx
            ps.pose.orientation.y = qy
            ps.pose.orientation.z = qz
            ps.pose.orientation.w = qw
            poses.append(ps)
        return poses

    def call_save_image(self):
        if not self.cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().error("/save_image service not available")
            return

        req = Trigger.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.done() and future.result() is not None:
            resp = future.result()
            if resp.success:
                self.get_logger().info(f"Photo saved: {resp.message}")
            else:
                self.get_logger().warn(f"Photo failed: {resp.message}")
        else:
            self.get_logger().warn("Photo request timeout")

    def run(self):
        # 等 Nav2 active：你之前用的定位节点名
        self.nav.waitUntilNav2Active(navigator='bt_navigator', localizer='lidar_localization')

        poses = self.load_poses()
        self.get_logger().info(f"Sending {len(poses)} waypoints...")

        self.nav.followWaypoints(poses)

        last_wp = -1
        while not self.nav.isTaskComplete():
            fb = self.nav.getFeedback()
            if fb is not None:
                cur = fb.current_waypoint
                if cur != last_wp:
                    # 进入新 waypoint（到达上一个点/开始下一个点）时触发一次拍照
                    # 注意：cur=0 表示正在执行第1个点，你也可以选择在 cur==0 时不拍
                    self.get_logger().info(f"Now executing waypoint {cur+1}/{len(poses)} -> trigger photo")
                    self.call_save_image()
                    last_wp = cur

            rclpy.spin_once(self, timeout_sec=0.1)

        result = self.nav.getResult()
        self.get_logger().info(f"Nav result: {result}")
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info("Route succeeded.")
        else:
            self.get_logger().warn("Route finished with non-success (check Nav2 logs).")

def main():
    if len(sys.argv) < 2:
        print("Usage: run_route_take_photo.py /path/to/route.yaml")
        return

    rclpy.init()
    node = RouteRunner(sys.argv[1])
    try:
        node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

