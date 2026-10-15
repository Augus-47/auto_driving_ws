import rclpy
from rclpy.node import Node
# 导入自定义状态消息
from car_msg.msg import VehicleState 
from sensor_msgs.msg import Image, Imu, PointCloud2
# 🌟 核心升级 1：导入标准的速度控制消息 Twist，用于毫秒级急停
from geometry_msgs.msg import Twist
import paho.mqtt.client as mqtt
import json
import time
import subprocess 
import os

class CarMqttBridge(Node):
    def __init__(self):
        super().__init__('car_mqtt_bridge')
        
        # --- 状态变量初始化 ---
        self.speed = 0.0
        self.electric = 0.0
        self.last_cam_time = 0
        self.last_lidar_time = 0
        self.last_imu_time = 0
        
        # --- 1. ROS 2 订阅者 ---
        self.create_subscription(VehicleState, '/vehicle_info', self.info_callback, 10)
        self.create_subscription(Image, '/image_raw', self.cam_callback, 10)
        self.create_subscription(PointCloud2, '/rslidar_points', self.lidar_callback, 10)
        self.create_subscription(Imu, '/imu_msg', self.imu_callback, 10)

        # 🌟 核心升级 1：在节点初始化时，就建好 /cmd_vel 的发布者通道（提前打通 DDS 链路）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- 2. MQTT 客户端设置 ---
        self.mqtt_client = mqtt.Client(client_id="Car305_car_stateBridge")
        
        # 🌟 核心升级 2：设置 MQTT 遗嘱 (LWT) 防猝死
        # 如果小车意外掉线，服务器会自动向 car/data 广播这条带 isOffline 标记的离线消息
        will_payload = json.dumps({
            "cname": "Car305",
            "speed": 0.0,
            "electric": 0.0,
            "camera": 0, "lidar": 0, "imu": 0,
            "isOffline": True 
        })
        self.mqtt_client.will_set("car/data", payload=will_payload, qos=1, retain=False)
        
        self.mqtt_client.on_message = self.on_mqtt_message
        
        try:
            self.mqtt_client.connect("broker.emqx.io", 1883)
            self.mqtt_client.subscribe("car/cmd")
            self.mqtt_client.subscribe("cmd/Car305/control")
            self.mqtt_client.loop_start()
            self.get_logger().info("MQTT 连接成功，开始监听指令，并已挂载安全遗嘱机制")
        except Exception as e:
            self.get_logger().error(f"MQTT 连接失败: {e}")
            
        # 创建一个 ROS 定时器，每隔 1.0 秒自动调用一次 report_status
        self.timer = self.create_timer(1.0, self.report_status)
        self.get_logger().info("已启动 1Hz 自动心跳上报机制")

    def info_callback(self, msg):
        try:
            self.speed = float(msg.velocity) 
            self.electric = float(msg.battary) 
        except AttributeError as e:
            self.get_logger().error(f"字段名错误: {e}")

    # 传感器回调（更新时间戳，用于判断设备在线状态）
    def cam_callback(self, msg): self.last_cam_time = time.time()
    def lidar_callback(self, msg): self.last_lidar_time = time.time()
    def imu_callback(self, msg): self.last_imu_time = time.time()

    # 上报最新状态
    def report_status(self):
        now = time.time()
        status_data = {
            "cname": "Car305",
            "speed": self.speed,       
            "electric": self.electric, 
            "camera": 1 if (now - self.last_cam_time) < 2.0 else 0,
            "lidar": 1 if (now - self.last_lidar_time) < 2.0 else 0,
            "imu": 1 if (now - self.last_imu_time) < 2.0 else 0,
            "time": time.strftime("%Y-%m-%d %H:%M:%S")
        }
        self.mqtt_client.publish("car/data", json.dumps(status_data))
        self.get_logger().info(f"已上报最新状态")

    # 处理后端或大屏发来的指令 
    def on_mqtt_message(self, client, userdata, msg):
        payload_str = msg.payload.decode()
        
        # 1. 旧版字符串查询指令
        if payload_str == "QUERY_STATUS":
            self.get_logger().info("收到查询指令，正在上报状态...")
            self.report_status()
            return
            
        # 2. 解析 JSON 控制指令
        try:
            cmd_data = json.loads(payload_str)
            target = cmd_data.get("target")   # camera, lidar, imu, chassis
            action = cmd_data.get("action")   # start, stop, estop, resume
            cname = cmd_data.get("cname")     # Car305

            if cname == "Car305":
                # 拦截最高优先级的底盘急停指令
                if action in ["estop", "resume"]:
                    self.get_logger().warn(f"收到最高优先级安全指令: {action.upper()}")
                    self.execute_sensor_cmd("chassis", action)
                
                # 正常的传感器启停指令
                elif target and action:
                    self.get_logger().info(f"收到控制指令 => 目标: {target}, 动作: {action}")
                    self.execute_sensor_cmd(target, action)
                
        except json.JSONDecodeError:
            pass 

    # 执行具体的硬件拉起/关闭/急停操作 
    def execute_sensor_cmd(self, target, action):
        WORKSPACE_DIR = '/home/nvidia/auto_driving' 
        
        try:
            # ==========================================
            # 🚨 底盘急停与恢复控制
            # ==========================================
            if target == "chassis":
                if action in ["stop", "estop"]:
                    self.get_logger().warn("🚨 [急停触发] 正在下发原生零速并切断底盘节点...")
                    
                    # 1. 原生发布全零速度 (无启动开销，延迟极低)
                    twist_msg = Twist()
                    twist_msg.linear.x = 0.0
                    twist_msg.linear.y = 0.0
                    twist_msg.angular.z = 0.0
                    self.cmd_vel_pub.publish(twist_msg)
                    
                    # 2. 给底层 STM32 电机驱动 0.1 秒时间消化刹车指令
                    time.sleep(0.1)
                    
                    # 3. 杀掉底盘节点，彻底切断后续控制流
                    subprocess.Popen(["pkill", "-f", "car_cmd_interface_node"])
                    self.get_logger().info("🛑 底盘驱动节点已强制关闭！")
                    
                elif action in ["start", "resume"]:
                    self.get_logger().info("✅ [急停解除] 正在重新启动底盘节点...")
                    start_cmd = "source install/setup.bash && ros2 run car_cmd_interface car_cmd_interface_node"
                    subprocess.Popen(start_cmd, shell=True, executable='/bin/bash', cwd=WORKSPACE_DIR)

            # ==========================================
            # 📷 原有传感器控制
            # ==========================================
            elif target == "camera":
                if action == "start":
                    self.get_logger().info("正在启动摄像头节点...")
                    start_cmd = "source install/setup.bash && ros2 run camera_driver camera_pub_node"
                    subprocess.Popen(start_cmd, shell=True, executable='/bin/bash', cwd=WORKSPACE_DIR)
                elif action == "stop":
                    self.get_logger().info("正在关闭摄像头节点...")
                    subprocess.Popen(["pkill", "-f", "camera_pub_node"])

            elif target == "lidar":
                if action == "start":
                    self.get_logger().info("正在启动雷达节点...")
                    start_cmd = "source install/setup.bash && ros2 launch rslidar_sdk start.py"
                    subprocess.Popen(start_cmd, shell=True, executable='/bin/bash', cwd=WORKSPACE_DIR)
                elif action == "stop":
                    self.get_logger().info("正在关闭雷达节点...")
                    subprocess.Popen(["pkill", "-f", "rslidar_sdk"])

            elif target == "imu":
                if action == "start":
                    self.get_logger().info("正在启动 IMU 节点...")
                    start_cmd = "source install/setup.bash && ros2 launch imu_node imu_node.launch.py"
                    subprocess.Popen(start_cmd, shell=True, executable='/bin/bash', cwd=WORKSPACE_DIR)
                elif action == "stop":
                    self.get_logger().info("正在关闭 IMU 节点...")
                    subprocess.Popen(["pkill", "-f", "imu_node"])
                    
        except Exception as e:
            self.get_logger().error(f"执行硬件操作时出错: {e}")

def main():
    rclpy.init()
    node = CarMqttBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
