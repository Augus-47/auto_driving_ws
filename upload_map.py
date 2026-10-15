import os
import sys
import requests
import json
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt

# 如果你的 ROS2 版本是 Galactic/Humble，引入 Nav2 的地图加载服务
from nav2_msgs.srv import LoadMap

# ================= 配置区 =================
# 注意：小车本地保存 png 给前端，保存 pgm 给底层 Nav2 用
MAP_DIR = '/home/nvidia/ros2_ws/maps/'
MAP_PNG_PATH = os.path.join(MAP_DIR, 'map.png')
MAP_PGM_PATH = os.path.join(MAP_DIR, 'map.pgm') 
MAP_YAML_PATH = os.path.join(MAP_DIR, 'map.yaml')

SERVER_IP = '192.168.62.19'  # 你的 Java 后端 IP 1、192.168.0.87
SERVER_PORT = '8081'
UPLOAD_URL = f"http://{SERVER_IP}:{SERVER_PORT}/api/map/uploadMap"

MQTT_BROKER = "broker.emqx.io"
MQTT_PORT = 1883
CAR_NAME = "Car305"
# =========================================

class MapManagerNode(Node):
    def __init__(self):
        super().__init__('map_manager_node')
        self.get_logger().info("🌍 305Car 地图及配置双向同步管家已启动...")

        # 1. 初始化 ROS 2 的 Nav2 地图重载客户端
        self.load_map_client = self.create_client(LoadMap, '/map_server/load_map')

        # 2. 初始化 MQTT 客户端
        self.mqtt_client = mqtt.Client(client_id=f"{CAR_NAME}_Map_Manager_{os.getpid()}")
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        
        # 连接并开启 MQTT 后台监听线程
        self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        self.mqtt_client.loop_start()

    def on_connect(self, client, userdata, flags, rc):
        self.get_logger().info("✅ 成功连接到 MQTT 云端！开始监听地图指令...")
        # 订阅两个主题：一个是通用的命令主题(为了兼容 SYNC_MAP)，一个是专用的控制主题
        client.subscribe("car/cmd", qos=1) 
        client.subscribe(f"cmd/{CAR_NAME}/map_control", qos=1)

    def on_message(self, client, userdata, msg):
        topic = msg.topic
        payload = msg.payload.decode('utf-8')
        
        # --- 场景 1：前端点击了【从云端重置/同步】，要求小车上传本地地图 ---
        if payload == "SYNC_MAP":
            self.get_logger().info("📥 收到指令：上传本地地图至云端")
            self.upload_map()
            
        # --- 场景 2：前端点击了【下发新地图至小车】，要求小车下载并热更新 ---
        else:
            try:
                data = json.loads(payload)
                if data.get("action") == "reload_map":
                    download_url = data.get("download_url")
                    self.get_logger().info(f"📤 收到指令：云端地图有更新，开始下载...")
                    self.download_and_reload_map(download_url)
            except json.JSONDecodeError:
                pass # 忽略非 JSON 格式的无关消息

    # =========================================
    # 上传地图逻辑 (保留了你原来的代码核心)
    # =========================================
    def upload_map(self):
        # 1. ★ 新增核心逻辑：上传前，强制将小车最新的 PGM 转换为 PNG
        if os.path.exists(MAP_PGM_PATH):
            try:
                # 读取 ROS 的黑白地图 (PGM)
                img_pgm = cv2.imread(MAP_PGM_PATH, cv2.IMREAD_GRAYSCALE)
                # 覆盖保存为 PNG，供 Web 浏览器渲染
                cv2.imwrite(MAP_PNG_PATH, img_pgm)
                self.get_logger().info("🔄 已将本地最新 PGM 转换为 PNG，准备上传...")
            except Exception as e:
                self.get_logger().error(f"❌ PGM 转 PNG 失败: {e}")
        else:
            self.get_logger().error(f"❌ 找不到底层的 PGM 地图: {MAP_PGM_PATH}")
            return

        # 2. 检查生成的 PNG 和 YAML 是否就绪
        if not os.path.exists(MAP_PNG_PATH) or not os.path.exists(MAP_YAML_PATH):
            self.get_logger().error("❌ 本地找不到准备上传的地图图片或 yaml 配置文件！")
            return

        # 3. 发起 HTTP 上传
        try:
            with open(MAP_PNG_PATH, 'rb') as img_file, open(MAP_YAML_PATH, 'rb') as yaml_file:
                files = {
                    'imageFile': ('map.png', img_file, 'image/png'),
                    'yamlFile': ('map.yaml', yaml_file, 'text/plain')
                }
                
                form_data = {
                    'carId': CAR_NAME
                }
                # 注意这里加上了 data=form_data
                response = requests.post(UPLOAD_URL, files=files, data=form_data,timeout=10)

            if response.status_code == 200:
                self.get_logger().info("✅ 地图及配置成功同步至 Java 大屏端！")
            else:
                self.get_logger().error(f"❌ 上传失败！HTTP 状态码: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ 网络异常或上传报错: {e}")

   # =========================================
    # 下载地图并通知 Nav2 热重载
    # =========================================
    def download_and_reload_map(self, url):
        try:
            # 1. 极速下载图片
            response = requests.get(url, timeout=10)
            if response.status_code == 200:
                self.get_logger().info("✅ 成功从云端下载到新版本修编地图！")
                
                #  新增：把云端下发的 png 原封不动地覆盖本地的 map.png
                with open(MAP_PNG_PATH, 'wb') as f:
                    f.write(response.content)
                self.get_logger().info(f"✅ 已更新本地 PNG 预览图: {MAP_PNG_PATH}")
                
                # 2. 图像转换：PNG -> PGM
                nparr = np.frombuffer(response.content, np.uint8)
                img_png = cv2.imdecode(nparr, cv2.IMREAD_GRAYSCALE)
                
                # ★ 工业级改进：
                # 即使云端下发的文件名变了（带时间戳），我们本地依然统一保存为 map.pgm
                # 这样可以确保你本地的 map.yaml 永远有效，不需要频繁修改 yaml 文件
                cv2.imwrite(MAP_PGM_PATH, img_png)
                self.get_logger().info(f"✅ 已同步版本数据并覆盖本地底层地图: {MAP_PGM_PATH}")

                # 3. 呼叫 Nav2 服务进行热重载
                self.call_nav2_reload()
            else:
                self.get_logger().error(f"❌ 下载失败！HTTP 状态码: {response.status_code}")
        except Exception as e:
            self.get_logger().error(f"❌ 下载或图像转换报错: {e}")

    def call_nav2_reload(self):
        if not self.load_map_client.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn("⚠️ Nav2 map_server 服务未就绪，地图更新可能未生效，请检查导航栈是否开启。")
            return

        # 组装 ROS 2 Service 请求
        req = LoadMap.Request()
        req.map_url = MAP_YAML_PATH # 告诉 Nav2 去读这个 yaml，yaml 里会指向我们刚覆盖的 pgm
        
        future = self.load_map_client.call_async(req)
        future.add_done_callback(self.reload_callback)

    def reload_callback(self, future):
        try:
            response = future.result()
            # 这里的 response.result (不同版本的 ROS2 msg 结构略有差别，通常是 uint8)
            self.get_logger().info("🎉 Nav2 导航底层已成功热重载最新地图！避障规则已瞬间刷新！")
        except Exception as e:
            self.get_logger().error(f"❌ 呼叫 Nav2 服务失败: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapManagerNode()
    
    try:
        # 让节点持续运行，直到按下 Ctrl+C
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.mqtt_client.disconnect()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
