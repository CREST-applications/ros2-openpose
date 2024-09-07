from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="camera",
                executable="main",
            ),
            Node(
                package="display",
                executable="main",
            ),
            Node(
                package="pose",
                executable="main",
            ),
        ]
    )
