from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emplanner_ros',
            executable='reference_line_test',
            name='reference_line_test',
            output='screen',
            emulate_tty=True
        )
    ]) 