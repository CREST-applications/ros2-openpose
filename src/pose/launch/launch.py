from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pose",
                executable="camera",
            ),
            Node(
                package="pose",
                executable="display",
            ),
            Node(
                package="pose",
                executable="requester",
            ),
        ]
    )
