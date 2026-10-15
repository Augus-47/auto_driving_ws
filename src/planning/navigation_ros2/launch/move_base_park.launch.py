from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('navigation_ros2')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    
    # 参数文件路径
    nav_param_dir = os.path.join(pkg_share, 'param', 'param_park')
    map_file = os.path.join(pkg_share, 'map', 'park', 'park.yaml')
    
    
    # 使用nav2_bringup的launch文件
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ]),
        launch_arguments={
            'params_file': os.path.join(nav_param_dir, 'nav2_params.yaml'),
            'use_sim_time': 'false'
        }.items()
    )
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'frame_id': 'map'}]
    )
    
    odom_to_path_node = Node(
        package='navigation_ros2',
        executable='odom_to_path',
        name='odom_to_path',
        output='screen'
    )
    
    # 返回LaunchDescription对象
    return LaunchDescription([
        nav2_launch,
        map_server_node,
        odom_to_path_node,
    ])