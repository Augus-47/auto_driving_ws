from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        # RTAB-Map 主节点
        Node(
            package='rtabmap_ros',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                # 加载刚才写的参数文件（rtabmap_launch 包里的相对路径）
                {'config_path': 'config/rslidar_mono.yaml'},
                'config/rslidar_mono.yaml',
            ],
            remappings=[
                # 相机
                ('rgb/image', '/camera/image_raw'),
                ('rgb/camera_info', '/camera/camera_info'),
                # 雷达点云（注意改成你真正在用的 topic）
                ('scan_cloud', '/rslidar_points'),
                # 如果有外部里程计，就把 odom remap 到对应的 topic
                # ('odom', '/your_odom_topic')
            ]
        ),
    ])

