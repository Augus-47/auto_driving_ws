#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class ClickedPointRecorder(Node):
    def __init__(self):
        super().__init__('clicked_point_recorder')
        self.declare_parameter('topic', '/clicked_point')
        self.declare_parameter('out', 'route.yaml')
        self.topic = self.get_parameter('topic').value
        self.out = self.get_parameter('out').value

        self.pts = []
        self.create_subscription(PointStamped, self.topic, self.cb, 10)
        self.get_logger().info(
            f"Recording from {self.topic}. In RViz select 'Publish Point' and click points. Ctrl+C to save -> {self.out}"
        )

    def cb(self, msg: PointStamped):
        p = (float(msg.point.x), float(msg.point.y))
        self.pts.append(p)
        self.get_logger().info(f"Added point {len(self.pts)}: x={p[0]:.3f}, y={p[1]:.3f}")

    def save(self):
        wps = []
        n = len(self.pts)
        for i in range(n):
            x, y = self.pts[i]
            if n == 1:
                yaw = 0.0
            elif i < n - 1:
                nx, ny = self.pts[i + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                px, py = self.pts[i - 1]
                yaw = math.atan2(y - py, x - px)
            wps.append({'x': x, 'y': y, 'yaw': float(yaw)})

        with open(self.out, 'w') as f:
            yaml.safe_dump({'waypoints': wps}, f, sort_keys=False, allow_unicode=True)
        print(f"[clicked_point_recorder] Saved {len(wps)} waypoints to {self.out}")

def main():
    rclpy.init()
    node = ClickedPointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

