# ROS2版本的导航包 (navigation_ros2)

这个包是ROS1版本的`navigation`包的ROS2移植版本。它提供了基于Nav2的导航功能和相关工具。

## 功能

- 提供基于Nav2的导航功能
- 包含从里程计到路径的转换节点
- 支持不同环境的导航配置（公园和小车环境）

## 安装依赖

确保你已经安装了以下依赖：

```bash
sudo apt install ros-$ROS_DISTRO-nav2-bringup
sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan
sudo apt install ros-$ROS_DISTRO-nav2-map-server
```

## 编译

```bash
cd ~/ros2_ws
colcon build --packages-select navigation_ros2
source install/setup.bash
```

## 使用方法

### 公园环境导航

```bash
ros2 launch navigation_ros2 move_base_park.launch.py
```

### 小车环境导航

```bash
ros2 launch navigation_ros2 move_base_lv.launch.py
```

## 参数配置

导航参数位于以下目录：
- 公园环境：`param/param_park/`
- 小车环境：`param/param_lv/`

您可以根据需要调整这些参数文件。

## 地图

- 公园环境地图：`map/park/park.yaml`
- 小车环境地图：`map/lv/lv.yaml`

## 注意事项

- 此包使用Nav2代替ROS1的move_base
- 为了兼容原始代码，保留了大部分原始参数格式，但在Nav2中的使用方式有所不同
- ROS2中TF已更改为TF2，因此静态变换发布器的用法也有所不同 