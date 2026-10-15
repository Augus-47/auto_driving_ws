#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image
from std_srvs.srv import Trigger

from cv_bridge import CvBridge

class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_service')

        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('save_dir', '/home/nvidia/waypoint_images')
        self.declare_parameter('prefix', 'wp')
        self.declare_parameter('ext', 'png')

        self.image_topic = self.get_parameter('image_topic').value
        self.save_dir = self.get_parameter('save_dir').value
        self.prefix = self.get_parameter('prefix').value
        self.ext = self.get_parameter('ext').value

        os.makedirs(self.save_dir, exist_ok=True)

        # 订阅相机：你的发布端是 RELIABLE，所以这里也用 RELIABLE 更稳
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.bridge = CvBridge()
        self.last_msg = None

        self.create_subscription(Image, self.image_topic, self.image_cb, qos)
        self.srv = self.create_service(Trigger, '/save_image', self.save_cb)

        self.get_logger().info(
            f"Listening: {self.image_topic} | Service: /save_image | Save dir: {self.save_dir}"
        )

    def image_cb(self, msg: Image):
        self.last_msg = msg

    def save_cb(self, request, response):
        _ = request
        if self.last_msg is None:
            response.success = False
            response.message = "No image received yet."
            return response

        # 文件名：wp_时间戳.png
        t = time.time()
        filename = f"{self.prefix}_{t:.3f}.{self.ext}"
        path = os.path.join(self.save_dir, filename)

        try:
            cv_img = self.bridge.imgmsg_to_cv2(self.last_msg, desired_encoding=self.last_msg.encoding)
            # 如果 encoding 是 bgr8，opencv 直接可写
            import cv2
            ok = cv2.imwrite(path, cv_img)
            if not ok:
                raise RuntimeError("cv2.imwrite returned False")
            response.success = True
            response.message = path
            self.get_logger().info(f"Saved image -> {path}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to save: {e}"
            self.get_logger().error(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ImageSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

