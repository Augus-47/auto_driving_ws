#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # 创建发布器
        self.publisher_ = self.create_publisher(CameraInfo, '/camera_info', 10)
        
        # 使用您的标定参数
        self.camera_info = CameraInfo()
        self.camera_info.header.frame_id = "camera_link"
        self.camera_info.width = 1280    # 您的图像宽度
        self.camera_info.height = 720    # 您的图像高度
        
        # 使用您的内参矩阵 [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        self.camera_info.k = [1280.25589, 0.0, 1004.17751,
                              0.0, 1242.76362, 463.92458, 
                              0.0, 0.0, 1.0]
        
        # 使用您的畸变参数 [k1, k2, p1, p2, k3]
        self.camera_info.d = [-0.314476, 0.081655, 0.004738, 0.010711, 0.0]
        
        self.camera_info.distortion_model = "plumb_bob"
        
        # 投影矩阵 (通常与K矩阵相同，第四行用于立体相机)
        self.camera_info.p = [1280.25589, 0.0, 1004.17751, 0.0,
                              0.0, 1242.76362, 463.92458, 0.0,
                              0.0, 0.0, 1.0, 0.0]
        
        # 定时发布 (匹配您的相机帧率)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz
        
    def timer_callback(self):
        self.camera_info.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
