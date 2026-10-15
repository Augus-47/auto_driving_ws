import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import time
import sys


class MultiCameraPublisher(Node):
    def __init__(self, camera_configs, timer_period):
        super().__init__('multi_camera_publisher')
        
        # 【修复】使用不同的属性名称避免冲突
        self._camera_configs = camera_configs
        self._cv_bridge = CvBridge()
        
        # 【修复】使用下划线前缀的属性名称
        self._image_publishers = []  # 改为_image_publishers
        self._camera_caps = []       # 改为_camera_caps
        self._last_times = []
        self._camera_names = []
        
        # 初始化所有摄像头
        for config in camera_configs:
            camera_id = config['camera_id']
            camera_name = config['camera_name']
            topic_name = config['topic_name']
            
            self.get_logger().info(f"正在初始化摄像头 {camera_id} ({camera_name})...")
            
            # 为每个摄像头创建发布器
            publisher = self.create_publisher(Image, topic_name, 10)
            self._image_publishers.append(publisher)
            self._camera_names.append(camera_name)
            
            # 打开摄像头
            cap = cv2.VideoCapture(camera_id)
            
            # 摄像头参数配置
            # 注意：这些 GMSL 相机只支持 30fps（离散值），请求 10fps 会导致
            # GStreamer 协商失败（"unable to start pipeline"）。
            w, h, fps = 1920, 1080, 30
            codec = cv2.VideoWriter_fourcc(*'UYVY')
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
            cap.set(cv2.CAP_PROP_FPS, fps)
            cap.set(cv2.CAP_PROP_FOURCC, codec)

            if not cap.isOpened():
                self.get_logger().error(f"摄像头 {camera_id} ({camera_name}) 未成功打开！")
                self._camera_caps.append(None)
                self._last_times.append(time.time())
                continue
            else:
                self.get_logger().info(f"摄像头 {camera_id} ({camera_name}) 已成功打开")
                # 测试摄像头格式
                self.test_camera_format(cap, camera_name)
            
            self._camera_caps.append(cap)
            self._last_times.append(time.time())
            
            # 摄像头间启动延迟，避免冲突
            time.sleep(1.0)
        
        # 【修复】使用不同的定时器列表名称
        self._camera_timers = []
        for i in range(len(self._camera_caps)):
            if self._camera_caps[i] is not None:
                # 使用lambda函数传递摄像头索引
                timer = self.create_timer(timer_period, lambda idx=i: self.camera_timer_callback(idx))
                self._camera_timers.append(timer)
                self.get_logger().info(f"为摄像头 {self._camera_names[i]} 创建定时器")
        
        self.get_logger().info("多摄像头发布器初始化完成")

    def test_camera_format(self, cap, camera_name):
        """测试摄像头格式"""
        ret, frame = cap.read()
        if ret:
            self.get_logger().info(f"摄像头 {camera_name} 帧信息: 形状={frame.shape}, 类型={frame.dtype}")
            if len(frame.shape) == 2:
                self.get_logger().info(f"摄像头 {camera_name} 检测到YUV格式(UYVY)，需要转换")
            else:
                self.get_logger().info(f"摄像头 {camera_name} 检测到BGR格式，无需转换")

    def uyvy_to_bgr(self, uyvy_frame):
        """将UYVY格式转换为BGR格式"""
        try:
            # UYVY转BGR
            bgr_frame = cv2.cvtColor(uyvy_frame, cv2.COLOR_YUV2BGR_UYVY)
            return bgr_frame
        except Exception as e:
            self.get_logger().error(f"UYVY转换失败: {str(e)}")
            return None

    def camera_timer_callback(self, camera_index):
        """每个摄像头的独立回调函数"""
        current_time = time.time()
        camera_name = self._camera_names[camera_index]
        
        # 计算帧率
        elapsed = current_time - self._last_times[camera_index]
        self._last_times[camera_index] = current_time
        self.get_logger().info(f'摄像头 {camera_name} 帧间隔: {elapsed:.4f}秒 ≈ {1/elapsed:.1f}Hz', 
                              throttle_duration_sec=2.0)

        cap = self._camera_caps[camera_index]
        if cap is None or not cap.isOpened():
            self.get_logger().warning(f"摄像头 {camera_name} 未打开")
            return

        ret, frame = cap.read()
        if not ret:
            self.get_logger().warning(f"读取摄像头 {camera_name} 图像失败")
            return

        try:
            # 添加摄像头标识到图像中（用于调试）
            debug_frame = frame.copy()
            if debug_frame.ndim == 3:
                # 在图像上添加文字标识
                text = f"{camera_name}"
                font_scale = 1.5
                thickness = 3
                color = (0, 255, 0)  # 绿色
                
                # 添加文字背景
                text_size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, thickness)[0]
                cv2.rectangle(debug_frame, (10, 10), (20 + text_size[0], 40 + text_size[1]), (0, 0, 0), -1)
                # 添加文字
                cv2.putText(debug_frame, text, (20, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)
            
            # 格式判断逻辑
            if debug_frame.ndim == 3 and debug_frame.shape[2] == 2:
                # 正确的UYVY格式（2通道）
                self.get_logger().info(f"摄像头 {camera_name} 检测到UYVY格式，进行转换...", 
                                      throttle_duration_sec=10.0)
                bgr_frame = self.uyvy_to_bgr(debug_frame)
                if bgr_frame is not None:
                    img_msg = self._cv_bridge.cv2_to_imgmsg(bgr_frame, 'bgr8')
                    # 设置frame_id以便在RViz2中区分
                    img_msg.header.frame_id = f"camera_{camera_name}"
                    self._image_publishers[camera_index].publish(img_msg)
                    self.get_logger().info(f'摄像头 {camera_name} 发布转换后的BGR图像成功', 
                                          throttle_duration_sec=5.0)
                else:
                    self.get_logger().error(f"摄像头 {camera_name} UYVY转换失败")
            elif debug_frame.ndim == 3 and debug_frame.shape[2] == 3:
                # BGR格式（3通道）
                img_msg = self._cv_bridge.cv2_to_imgmsg(debug_frame, 'bgr8')
                img_msg.header.frame_id = f"camera_{camera_name}"
                self._image_publishers[camera_index].publish(img_msg)
                self.get_logger().info(f'摄像头 {camera_name} 发布BGR图像成功', 
                                      throttle_duration_sec=5.0)
            else:
                # 其他格式（如单通道灰度图），尝试转换为BGR
                self.get_logger().warning(f"摄像头 {camera_name} 未知格式: {debug_frame.shape}，尝试转为BGR", 
                                         throttle_duration_sec=10.0)
                bgr_frame = cv2.cvtColor(debug_frame, cv2.COLOR_GRAY2BGR)
                img_msg = self._cv_bridge.cv2_to_imgmsg(bgr_frame, 'bgr8')
                img_msg.header.frame_id = f"camera_{camera_name}"
                self._image_publishers[camera_index].publish(img_msg)
                self.get_logger().info(f'摄像头 {camera_name} 发布转换后的BGR图像成功', 
                                      throttle_duration_sec=5.0)
                
        except Exception as e:
            self.get_logger().error(f'摄像头 {camera_name} 发布图像失败: {str(e)}')

    def destroy_node(self):
        # 释放所有摄像头资源
        for i, cap in enumerate(self._camera_caps):
            if cap is not None and cap.isOpened():
                cap.release()
                self.get_logger().info(f"释放摄像头 {self._camera_names[i]} 资源")
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    # 配置4个摄像头
    camera_configs = [
        {
            'camera_id': 0,
            'camera_name': 'front',
            'topic_name': 'camera_right/image_raw'
        },
        {
            'camera_id': 1, 
            'camera_name': 'rear',
            'topic_name': 'camera_front/image_raw'
        },
        {
            'camera_id': 2,
            'camera_name': 'left',
            'topic_name': 'camera_rear/image_raw'
        },
        {
            'camera_id': 3,
            'camera_name': 'right',
            'topic_name': 'camera_left/image_raw'
        }
    ]

    # 默认频率：30Hz
    desired_fps = 10.0
    if len(sys.argv) > 1:
        try:
            desired_fps = float(sys.argv[1])
        except ValueError:
            print("请输入合法频率（数字）")

    timer_period = 1.0 / desired_fps
    
    # 创建多摄像头发布器
    node = MultiCameraPublisher(camera_configs, timer_period)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
