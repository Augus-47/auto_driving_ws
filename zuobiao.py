import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Path  # 🌟 新增：导入路径消息类型
import math
import time
import json
import os
import yaml
import paho.mqtt.client as mqtt
#发实时坐标、巡检点、发轨迹
YAML_FILE_PATH = '/home/nvidia/ros2_ws/src/autopartol_robot/config/partol_config.yaml'

class PoseMqttManager(Node):
    def __init__(self):
        super().__init__('pose_mqtt_manager')
        
        # --- 1. 【上行：收 ROS 坐标 -> 发 MQTT】 ---
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/pcl_pose', 
            self.pose_callback, 
            10
        )

        # 🌟 2. 【新增上行：收 Nav2 规划路径 -> 发 MQTT】 ---
        # 注意：请确认你的 Nav2 全局路径话题是不是 /plan，如果不是请修改
        self.plan_sub = self.create_subscription(
            Path,
            '/plan', 
            self.plan_callback,
            10
        )
        
        # --- 3. MQTT 连接配置 ---
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, "Car305_Manager")
        self.mqtt_client.on_message = self.on_mqtt_message
        self.mqtt_client.on_connect = self.on_mqtt_connect
        
        self.mqtt_client.connect("broker.emqx.io", 1883, 60)
        self.mqtt_client.loop_start()

        # 时间节流控制
        self.last_pose_send_time = 0.0
        self.last_plan_send_time = 0.0  # 🌟 新增：控制路径发送频率

    # ---------------------------------------------------------
    # MQTT 核心回调
    # ---------------------------------------------------------
    def on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("✅ 已连接到 MQTT 服务器")
            self.mqtt_client.subscribe("car/pose/setRoute")
            self.get_logger().info("📡 已订阅指令话题: car/pose/setRoute")
            self.publish_current_route()
        else:
            self.get_logger().error(f"❌ 连接失败，错误码: {rc}")

    def on_mqtt_message(self, client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            points_list = payload.get('points', [])
            self.get_logger().info(f"📥 收到云端多点巡检路线，共 {len(points_list)} 站")
            
            flat_target_points = []
            for pt in points_list:
                flat_target_points.extend([float(pt['x']), float(pt['y']), float(pt['yaw'])])

            self.update_yaml_config(flat_target_points)
        except Exception as e:
            self.get_logger().error(f"❌ 路线指令解析失败: {e}")

    # ---------------------------------------------------------
    # YAML 读写与热更新逻辑
    # ---------------------------------------------------------
    def update_yaml_config(self, flat_points_array):
        try:
            with open(YAML_FILE_PATH, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)

            config_data['/partol_node']['ros__parameters']['target_points'] = flat_points_array

            with open(YAML_FILE_PATH, 'w', encoding='utf-8') as f:
                yaml.dump(config_data, f, default_flow_style=None, sort_keys=False, allow_unicode=True)
            
            self.get_logger().info(f"📄 成功覆盖本地 YAML！新路线阵列: {flat_points_array}")

            param_str = str(flat_points_array).replace(" ", "") 
            param_cmd = f"ros2 param set /partol_node target_points '{param_str}'"
            os.system(param_cmd)
            self.get_logger().info("🔥 已向 partol_node 注入最新巡检路线！")
            
            self.publish_current_route()
        except Exception as e:
            self.get_logger().error(f"❌ 修改 YAML 配置文件失败: {e}")

    def publish_current_route(self):
        try:
            with open(YAML_FILE_PATH, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
            
            params = config_data.get('/partol_node', {}).get('ros__parameters', {})
            flat_pts = params.get('target_points', [])
            
            points_list = []
            if flat_pts:
                for i in range(0, len(flat_pts), 3):
                    points_list.append({
                        "x": float(flat_pts[i]), 
                        "y": float(flat_pts[i+1]), 
                        "yaw": float(flat_pts[i+2])
                    })
            
            payload = {"cname": "Car305", "points": points_list}
            self.mqtt_client.publish("car/pose/routeInfo", json.dumps(payload), qos=1, retain=True)
            self.get_logger().info(f"📤 路线同步完毕，当前任务共 {len(points_list)} 个巡检点")

        except Exception as e:
            self.get_logger().error(f"❌ 读取本地路线同步失败: {e}")

    # ---------------------------------------------------------
    # 高频坐标上报逻辑
    # ---------------------------------------------------------
    def euler_from_quaternion(self, x, y, z, w):
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        return math.atan2(t3, t4)

    def pose_callback(self, msg):
        current_time = time.time()
        # 控制坐标发送频率 10Hz
        if current_time - self.last_pose_send_time < 0.1:
            return

        curr_x = msg.pose.pose.position.x
        curr_y = msg.pose.pose.position.y
        curr_yaw = self.euler_from_quaternion(
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        )

        payload = {
            "cname": "Car305",
            "x": round(curr_x, 3),
            "y": round(curr_y, 3),
            "yaw": round(curr_yaw, 4)
        }
        
        self.mqtt_client.publish("car/pose/update", json.dumps(payload), qos=0)
        self.last_pose_send_time = current_time

    # ---------------------------------------------------------
    # 🌟 新增：Nav2 全局规划路径上报逻辑
    # ---------------------------------------------------------
    def plan_callback(self, msg):
        current_time = time.time()
        # 降频：每秒最多向云端发送 1 次路径更新，防止大屏卡死
        if current_time - self.last_plan_send_time < 1.0:
            return

        total_points = len(msg.poses)
        if total_points == 0:
            return

        points = []
        # 降采样：动态计算步长。无论多长的路线，最多抽取约 60 个点发给大屏
        # 对于大屏 Canvas 画线来说，60 个点已经能描绘出非常平滑的弧线了
        step = max(1, total_points // 60)

        for i in range(0, total_points, step):
            pose = msg.poses[i].pose.position
            points.append({
                "x": round(pose.x, 3),
                "y": round(pose.y, 3)
            })

        # 确保终点一定在数组的最后，防止大屏上的线画不到头
        if total_points % step != 0:
            end_pose = msg.poses[-1].pose.position
            points.append({
                "x": round(end_pose.x, 3),
                "y": round(end_pose.y, 3)
            })

        payload = {
            "cname": "Car305",
            "path": points
        }

        # 发布到专用的导航路径频道
        self.mqtt_client.publish("car/nav2/plan", json.dumps(payload), qos=0)
        self.last_plan_send_time = current_time


def main(args=None):
    rclpy.init(args=args)
    node = PoseMqttManager()
    
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
