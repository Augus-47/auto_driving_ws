from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 静态传感器挂载：rslidar 相对 base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rslidar_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'rslidar']
        ),

        # 测试用静态 odom -> base_link（如果没有里程计，可临时用）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base_link_static',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        # AMCL 或 SLAM 会发布 map -> odom，这里假设由 amcl 节点启动
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                # 你的 AMCL 配置文件路径
                'amcl_param_file': '/path/to/amcl_params.yaml'
            }]
        ),
    ])

