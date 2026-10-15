import os

import launch
import launch.actions
import launch.events
import launch_ros
import launch_ros.events
import lifecycle_msgs.msg

from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = launch.LaunchDescription()

    localization_param_dir = launch.substitutions.LaunchConfiguration(
        "localization_param_dir",
        default=os.path.join(
            get_package_share_directory("lidar_localization_ros2"),
            "param",
            "localization.yaml",
        ),
    )

    # ==========================================
    # 1) 生命周期定位节点
    # ==========================================
    lidar_localization = LifecycleNode(
        name="lidar_localization",
        namespace="",
        package="lidar_localization_ros2",
        executable="lidar_localization_node",
        parameters=[localization_param_dir],
        output="screen",
    )

    # ==========================================
    # 2) 自动 configure
    # ==========================================
    configure_event = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    # ==========================================
    # 3) inactive -> activate
    # ==========================================
    activate_handler = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=lidar_localization,
            start_state="configuring",
            goal_state="inactive",
            entities=[
                launch.actions.LogInfo(msg="[lidar_localization] -> activating"),
                launch.actions.EmitEvent(
                    event=launch_ros.events.lifecycle.ChangeState(
                        lifecycle_node_matcher=launch.events.matches_action(lidar_localization),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # ==========================================
    # 4) 如果你没有 URDF / robot_state_publisher
    #    请自行补 base_link <-> 雷达frame 的静态 TF
    # ==========================================

    ld.add_action(lidar_localization)
    ld.add_action(activate_handler)
    ld.add_action(configure_event)

    return ld

