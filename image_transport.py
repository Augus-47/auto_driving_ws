import os
import time
import requests
import cv2
import rclpy # 🌟 替换为 ROS 2 的 rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler

# ================= 基础配置 =================
IMAGE_DIR = "/home/nvidia/waypoint_images"
UPLOAD_URL = "http://192.168.62.151:8081/api/car/upload" 

# ================= MQTT 与 ROS 配置 =================
MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
MQTT_TOPIC = "cmd/Car305/capture"
ROS_IMAGE_TOPIC = "/image_raw"    

# 全局标记位：是否需要抓拍
REQUEST_CAPTURE = False 

# ================= 1. 文件夹监听与上传模块 =================
class ImageHandler(FileSystemEventHandler):
    def __init__(self, url):
        self.url = url

    def on_closed(self, event):
        if not event.is_directory and event.src_path.endswith(('.jpg', '.png', '.jpeg')):
            print(f"👀 Watchdog 发现新图片: {event.src_path}")
            self.upload_file(event.src_path)

    def upload_file(self, file_path):
        time.sleep(0.1) 
        try:
            with open(file_path, 'rb') as f:
                files = {'file': (os.path.basename(file_path), f, 'image/jpeg')}
                print(f"🚀 正在上传图片至后端...")
                response = requests.post(self.url, files=files, timeout=5)
                print(f"✅ 上传结果: {response.status_code} - {response.text}")
        except Exception as e:
            print(f"❌ 上传失败: {e}")

# ================= 2. ROS 2 节点处理 =================
class CameraCaptureNode(Node):
    def __init__(self):
        super().__init__('camera_capture_node')
        self.bridge = CvBridge()
        # 订阅相机的图像话题
        self.subscription = self.create_subscription(
            Image,
            ROS_IMAGE_TOPIC,
            self.image_callback,
            10 # 队列长度
        )
        self.get_logger().info(f"📷 正在监听相机话题: {ROS_IMAGE_TOPIC}")

    def image_callback(self, msg):
        global REQUEST_CAPTURE
        # 🌟 核心逻辑：只有当收到 MQTT 指令把标记位设为 True 时，才进行一次保存
        if REQUEST_CAPTURE:
            REQUEST_CAPTURE = False # 立刻复位，防止连拍
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                filename = f"realtime_capture_{int(time.time())}.jpg"
                save_path = os.path.join(IMAGE_DIR, filename)
                
                cv2.imwrite(save_path, cv_image)
                print(f"\n📸 抓拍成功！已保存至: {save_path} (即将触发自动上传)")
            except Exception as e:
                self.get_logger().error(f"❌ 图像保存失败: {e}")

# ================= 3. MQTT 接收模块 =================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print(f"🔗 已连接 MQTT 服务器，正在监听抓拍指令: {MQTT_TOPIC}")
        client.subscribe(MQTT_TOPIC)
    else:
        print(f"❌ 连接 MQTT 失败，返回码: {rc}")

def on_message(client, userdata, msg):
    print("\n🔔 收到前端抓拍指令！已通知相机抓取下一帧...")
    global REQUEST_CAPTURE
    REQUEST_CAPTURE = True # 收到指令，触发 ROS 节点截图

# ================= 主程序入口 =================
def main(args=None):
    if not os.path.exists(IMAGE_DIR):
        os.makedirs(IMAGE_DIR)

    # 1. 启动 Watchdog
    event_handler = ImageHandler(UPLOAD_URL)
    observer = Observer()
    observer.schedule(event_handler, IMAGE_DIR, recursive=False)
    observer.start()

    # 2. 启动 MQTT 
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "Car305_CameraCapture")
    client.on_connect = on_connect
    client.on_message = on_message
    try:
        client.connect(MQTT_BROKER, MQTT_PORT, 60)
        client.loop_start() 
    except Exception as e:
        print(f"❌ MQTT 启动失败: {e}")

    # 3. 初始化并运行 ROS 2 节点
    rclpy.init(args=args)
    capture_node = CameraCaptureNode()

    print("🚀 核心服务已全部启动 (ROS 2 版本)，等待指令中...\n" + "-"*50)

    try:
        rclpy.spin(capture_node) # 保持 ROS 2 节点运行
    except KeyboardInterrupt:
        print("\n🛑 收到退出指令，正在停止服务...")
    finally:
        capture_node.destroy_node()
        rclpy.shutdown()
        observer.stop()
        observer.join()
        client.loop_stop()
        print("👋 服务已安全关闭。")

if __name__ == "__main__":
    main()
