from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # 声明节点列表参数
    



    nodes_to_monitor_arg = DeclareLaunchArgument(
        'nodes_to_monitor',
        default_value='[\"/pointcloud_to_laserscan\", \"/rs_to_velodyne\", \"/laser_obstacle_detector_node\", \"/vehicle_interface_node\", \"/lidar_localization\", \"/simple_follow_node\"]',
        description='要监控的节点列表，JSON数组格式'
    )

    # 获取节点列表参数
    nodes_to_monitor = LaunchConfiguration('nodes_to_monitor')


    return LaunchDescription([
        nodes_to_monitor_arg,
        Node(
            package='node_monitor',
            executable='monitor',
            name='node_monitor',
            parameters=[{
                'nodes_to_monitor': nodes_to_monitor
            }],
            output='screen',
            emulate_tty=True,
        )
    ])