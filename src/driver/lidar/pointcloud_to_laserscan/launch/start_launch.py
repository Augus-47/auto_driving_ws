from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='',
            description='Namespace prefix for topics (optional)'
        ),
        Node(
            package='pointcloud_to_laserscan',
            executable='pointcloud_to_laserscan_node',
            name='pointcloud_to_laserscan',
            remappings=[
                # 你现在实际点云是 /rslidar_points
                ('cloud_in', [LaunchConfiguration('scanner'), '/rslidar_points']),
                ('scan',     [LaunchConfiguration('scanner'), '/scan']),
            ],
            parameters=[{
                'queue_size': 1,

                # ✅ 推荐：先不设 target_frame，直接用点云自己的 frame_id 输出 scan
                # 如果你确实要设 target_frame，就取消下面两行注释，并把 frame 名改成真实存在的
                # 'target_frame': 'rslidar',
                # 'transform_tolerance': 0.1,

                'min_height': -0.4,
                'max_height': 1.0,
                'angle_min': -3.1415926,
                'angle_max': 3.1415926,
                'angle_increment': 0.003,
                'scan_time': 0.1,
                'range_min': 0.4,
                'range_max': 100.0,
                'use_inf': True,
                'inf_epsilon': 1.0,
            }],
        ),
    ])

