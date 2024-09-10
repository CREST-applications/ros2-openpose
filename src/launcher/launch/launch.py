from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
            ),
            Node(
                package="image_transport",
                executable="republish",
                arguments=["raw"],
                remappings=[("in", "/image_raw"), ("out", "/image_raw/compressed")],
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
