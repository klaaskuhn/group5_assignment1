from launch_ros.actions import Node, PushRosNamespace

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    group = GroupAction([
        PushRosNamespace(LaunchConfiguration('vehicle_name')),
        Node(executable='depth_calculator.py', package='depth_control'),
        Node(executable='depth_controller.py', package='depth_control'),
        Node(executable='depth_setpoint.py', package='depth_control'),
        Node(executable='depth_filter.py', package='depth_control')
    ])
    launch_description.add_action(group)
    return launch_description
