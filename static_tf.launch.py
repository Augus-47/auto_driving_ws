from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 发布 base_link→rslidar 的静态变换（欧拉角版本，无旋转）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            # 参数格式：x y z yaw pitch roll frame_id child_frame_id
            arguments=['0', '0', '1.0', '0', '0', '0', '1', 'base_link', 'rslidar']
        ),
        # 发布 map→odom 的静态变换（欧拉角版本，无旋转）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', '1', 'map', 'odom']
        ),
    ])
