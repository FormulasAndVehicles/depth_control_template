from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
)
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    launch_description = LaunchDescription()
    arg = DeclareLaunchArgument('vehicle_name')
    launch_description.add_action(arg)

    action = (
        Node(
            executable='depth_calculator.py',
            package='depth_control',
            namespace=LaunchConfiguration('vehicle_name'),
        ),
    )
    launch_description.add_action(action)
    return launch_description
