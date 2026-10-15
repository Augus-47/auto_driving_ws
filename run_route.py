#!/usr/bin/env python3
import math
import sys
import yaml
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

def main():
    if len(sys.argv) < 2:
        print("Usage: run_route.py /path/to/route.yaml")
        return

    with open(sys.argv[1], 'r') as f:
        data = yaml.safe_load(f)

    rclpy.init()
    navigator = BasicNavigator()

    navigator.waitUntilNav2Active(navigator='bt_navigator', localizer='lidar_localization')

    goal_poses = []
    stamp = navigator.get_clock().now().to_msg()
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
        goal_poses.append(ps)

    print(f"Sending {len(goal_poses)} waypoints...")
    navigator.followWaypoints(goal_poses)

    while not navigator.isTaskComplete():
        fb = navigator.getFeedback()
        if fb is not None:
            print(f"Executing waypoint: {fb.current_waypoint + 1}/{len(goal_poses)}")

    result = navigator.getResult()
    print("Result:", result)
    if result == TaskResult.SUCCEEDED:
        print("Route succeeded.")
    else:
        print("Route finished with non-success (check Nav2 logs).")

    rclpy.shutdown()

if __name__ == '__main__':
    main()

