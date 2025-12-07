from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="robot_dog_description",
                executable="cmd_vel_to_command",
                name="cmd_vel_to_command",
                output="screen",
            )
        ]
    )
