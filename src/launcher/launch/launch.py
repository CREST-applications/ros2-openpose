from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="image_transport",
                executable="republish",
                arguments=["compressed"],
                remappings=[
                    ("/in/compressed", "/image_raw/compressed"),
                    ("/out/compressed", "/proxy/compressed"),
                ],
            ),
            Node(
                package="pose",
                executable="main",
                remappings=[("/camera", "/proxy/compressed")],
            ),
            Node(
                package="renderer",
                executable="main",
                remappings=[("/camera", "/proxy/compressed")],
            ),
            Node(
                package="display",
                executable="main",
            ),
        ]
    )
