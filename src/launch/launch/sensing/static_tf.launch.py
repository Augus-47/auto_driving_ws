from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='base_link_to_velodyne_top',
        #     arguments=['-0.0', '-0.0', '-0.0', '0.0', '0.0', '0.0', 'base_link', 'map']
        # ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_velodyne_top',
            arguments=['-0.310764', '0.1006876', '-0.606644', '0.0', '0.0', '0.0', 'base_link', 'velodyne_top']
        ),
    ])