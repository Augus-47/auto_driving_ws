# auto_driving — 自动驾驶巡检车工作空间

基于 **ROS 2 Humble** 的小型阿克曼转向巡检车自动驾驶系统，运行于 NVIDIA Orin。整车通过 **SocketCAN（`can0`）** 与底盘通信。本文件介绍仓库内各文件/目录及其功能。

> 说明：`build/`、`install/`、`log/` 为 colcon 编译产物；`__pycache__/` 为 Python 缓存，均不在下文逐一列出。

---

## 一、源码包（`src/`）

colcon 工作空间的实际代码，按功能分模块。

### 消息定义 `src/msg/`
| 文件/包 | 功能 |
| --- | --- |
| `car_msg` | 全局共享自定义消息包，**几乎所有包都依赖它** |
| `car_msg/msg/VehicleState.msg` | 通用车辆指令/反馈类型：velocity、steering、brake、gears、led、battary |
| `car_msg/msg/OrinPose.msg` | 带时间戳的 `geometry_msgs/Pose`，由定位模块发布 |

### 传感器驱动 `src/driver/`
| 目录 | 功能 |
| --- | --- |
| `lidar/lidar_sdk_humble` | RoboSense 激光雷达 SDK，发布 `/rslidar_points` |
| `lidar/pointcloud_to_laserscan` | 点云转 2D 激光扫描（`/rslidar_points` → `/scan`），供避障使用 |
| `lidar/rs_to_velodyne` | RoboSense 点云格式转 Velodyne 格式 |
| `imu/ros2-imu-dirve-main` | IMU 驱动 |
| `camera/camera_raw` | 相机原始图像驱动 |
| `camera_driver` / `camera_driver_all` | 相机发布节点；`camera_driver_all` 多路相机发布（front/rear/left/right `image_raw`） |

### 定位与建图 `src/localization/`
| 目录 | 功能 |
| --- | --- |
| `lidar_localization_ros2` | **在线定位主路径**：基于 NDT 对预建 `.pcd` 地图做激光定位；订阅点云+IMU+odom+initialpose，发布位姿、`car_msg/OrinPose` 和路径（生命周期节点） |
| `funny_lidar_slam` | 实验性激光 SLAM 后端（建图用） |
| `lidarslam_ros2` | 激光 SLAM（生成 `pose_graph.g2o` 等） |
| `lightning-lm` | 实验性轻量定位/建图后端 |
| `rtabmap_ros` | RTAB-Map 视觉/激光 SLAM（建图用） |

### 感知 `src/perception/`
| 目录 | 功能 |
| --- | --- |
| `yolov5_ros2` | YOLOv5 2D 目标检测（`yolo_detect_2d.py`），订阅 `image_raw`，运行于 `cuda:0` |
| `yolo_ros` | YOLO ROS 封装（备选） |

### 规划 `src/planning/`
| 目录 | 功能 |
| --- | --- |
| `emplanner` | C++ EM-Planner + Hybrid-A\*；订阅当前/终点位姿与栅格地图，发布 `/planned_path`、`/global_map` |
| `costmap_generator` | 基于 grid_map/PCL 生成代价地图 |
| `laser_avoid` | 激光避障；订阅 `/scan`，发布 `std_msgs/Bool` 障碍标志 |
| `navigation_ros2` | 基于 Nav2 action 的目标点导航处理 |

### 控制 `src/control/`
| 目录 | 功能 |
| --- | --- |
| `car_simple_follow` | 路径跟踪（类纯追踪）；订阅 `/vehicle_info`、避障 `Bool`、`nav_msgs/Path`，发布 `car_msg/VehicleState` 到 **`/control_cmd`** |

### 车辆接口 `src/vehicle/`
| 目录 | 功能 |
| --- | --- |
| `car_cmd_interface` | **唯一操作 CAN 硬件的节点**：订阅 `/cmd_vel`(Twist)、`/angle`(Float32)、`/control_cmd`(VehicleState)，做阿克曼转向换算，写 `can0` CAN 帧，发布底盘反馈 `/vehicle_info`；含 `cmd_timeout`(默认 0.25s) 看门狗 |

### 启动编排 `src/launch/`
| 文件 | 功能 |
| --- | --- |
| `orin.launch.xml` | 模块化总入口，按开关包含各子模块（部分子文件为空桩） |
| `allin.launch.py` | **实际可用的整车 bring-up**：依次启动雷达→laserscan→定位→`nav2_map_server`(手动生命周期 configure/activate)→`car_cmd_interface` |
| `ekf_localization.launch.py` | EKF 融合定位启动 |
| `static_sensors_tf.launch.py` | 传感器静态 TF 发布 |
| `launch/<module>/*.launch.xml` | 各模块子启动文件（sensing/localization/perception/planning/control/vehicle） |
| `config/ekf_localization*.yaml` | robot_localization EKF 参数 |

### 工具 `src/utils/`
| 目录 | 功能 |
| --- | --- |
| `pcd2pgm` | 点云地图 `.pcd` 转 2D 栅格地图 `.pgm` |
| `node_monitor` | ROS2 节点监控器 + Qt 可视化界面（C++） |
| `bag_convert` | rosbag 数据格式转换 |

---

## 二、根目录脚本

独立运行的辅助脚本（`python3 xxx.py`），不经由包系统编译。

| 文件 | 功能 |
| --- | --- |
| `car_state.py` | ROS↔MQTT 桥接节点（`car_mqtt_bridge`）：订阅 `/vehicle_info`、相机、雷达、IMU 话题，将车辆状态上报 MQTT；接收 MQTT 急停指令发布 `/cmd_vel` |
| `car_v1.py` | 一键启动脚本：在 gnome-terminal 中开 4 个标签页分别运行 car_state / image_transport / upload_map / zuobiao |
| `zuobiao.py` | 坐标与路径 MQTT 上报（`pose_mqtt_manager`）：订阅 `/pcl_pose` 和 `/plan`，将实时位姿、巡检点、规划轨迹发送到 MQTT |
| `image_transport.py` | MQTT 触发拍照+HTTP 上传：监听 MQTT 拍照指令，订阅 `/image_raw` 抓帧，上传到后端服务器 |
| `image_saver_service.py` | 图像保存 ROS 服务节点：提供 `/save_image` Trigger 服务，将当前帧保存到本地目录 |
| `upload_map.py` | 地图管理节点：MQTT 接收地图下发→保存→调用 Nav2 `LoadMap` 服务热加载；也可上传本地地图到后端 |
| `run_route.py` | 读取 `route.yaml` 航点，用 Nav2 `BasicNavigator` 依次导航（配合 `route.yaml` 使用） |
| `run_route_take_photo.py` | 同上，到达航点后调用 `/save_image` 服务拍照 |
| `record_clicked_points.py` | 记录 RViz 中 Publish Point 点击的坐标，Ctrl+C 保存为 `route.yaml` |
| `odom.py` | 阿克曼里程计节点：订阅 `/vehicle_info`，积分计算并发布 `/odom` + TF |
| `wheel_odometry_node.py` | 轮速里程计节点：订阅 `/vehicle_info`，发布 `wheel_odom`（参数化轴距/frame） |
| `odom_to_tf.py` | 订阅 `/odom`，广播 `odom → base_link` TF（强制仿真时间，用于回放 bag） |
| `map_odom_republisher.py` | 订阅定位位姿，广播 `map → odom` TF |
| `pgm_to_png.py` | 栅格地图 `.pgm` 转 `.png`（用于前端展示） |
| `sync_node.cpp` | 多传感器时间同步节点（message_filters ApproximateTime 同步相机+雷达） |

---

## 三、配置文件

| 文件 | 功能 |
| --- | --- |
| `nav2_params.yaml` | Nav2 导航栈完整参数（controller/planner/costmap 等） |
| `slam_params.yaml` | SLAM 建图参数 |
| `route.yaml` | 巡检航点列表（x, y, yaw），供 `run_route.py` 使用 |
| `twist_to_vehiclecmd.yaml` | Twist→车辆指令映射配置（当前为空） |
| `static_tf.launch.py` | 传感器静态 TF 启动文件（根目录版本） |

---

## 四、地图与数据

| 文件/目录 | 功能 |
| --- | --- |
| `map.pcd` / `map.pgm` / `map.yaml` | **当前使用的地图**（点云 / 2D 栅格 / 元数据） |
| `map0~map11.*` | 历史地图版本（编号备份） |
| `map/`、`maps/`、`src/maps/` | 地图存放目录 |
| `pose_graph.g2o` | SLAM 生成的位姿图 |
| `bag0`、`bag1`、`bag*_pre_v2` | rosbag 录制数据 |
| `rosbag2_*` | 时间戳命名的 rosbag 录制 |
| `mapping_bag_01~05` | 建图用 rosbag 数据 |
| `data/`、`e2e_first/` | 数据集 / 端到端实验数据 |
| `forward.txt`、`reverse.txt`、`idle.txt` | 前进/倒车/怠速 CAN 指令序列样本 |

---

## 五、编译与运行

```bash
# 工作空间根目录编译
colcon build --symlink-install
# 仅编译单个包
colcon build --symlink-install --packages-select <pkg_name>
# 每个新终端都需 source
source install/setup.bash

# 运行节点 / 启动文件
ros2 run <pkg> <executable>
ros2 launch <pkg> <file>.launch.{py,xml}
```

### 数据流总览

```
传感器(雷达/IMU/相机) → 定位(NDT) → 感知(YOLOv5) → 规划(emplanner/Nav2) →
控制(car_simple_follow → /control_cmd  或  Nav2 → /cmd_vel) →
车辆接口(car_cmd_interface) → CAN(can0) → 底盘
                                  ↑
                          底盘反馈 /vehicle_info
```

> 注意：存在两个并行控制入口——Nav2 走 `/cmd_vel`，`car_simple_follow` 走 `/control_cmd`，二者最终都汇入 `car_cmd_interface`。
