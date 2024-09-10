from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_transport",
                executable="republish",
                arguments=["raw"],
                remappings=[("/in", "/image_raw/compressed"), ("/out", "/camera")],
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
