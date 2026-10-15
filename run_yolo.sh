#!/usr/bin/env bash
# 启动 YOLO 感知管线。
#
# yolo_ros 的节点用系统 Python 构建（shebang 指向 /usr/bin/python3），
# 但 torch / ultralytics 只装在 conda 的 yolo_ros 环境里。
# 所以只在“运行时”把 conda 的 site-packages 加到 PYTHONPATH，
# 千万不要在 colcon build 时加（会把 conda 的 setuptools 80.9.0 顶到前面，导致构建失败）。
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/nvidia/auto_driving/install/setup.bash

export PYTHONPATH="/home/nvidia/anaconda3/envs/yolo_ros/lib/python3.10/site-packages:${PYTHONPATH:-}"

# 默认模型文件在工作区根目录；如不在 CWD 会自动回退到绝对路径
cd /home/nvidia/auto_driving

exec ros2 launch yolo_bringup yolo.launch.py "$@"
