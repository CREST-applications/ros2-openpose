from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            # Node(
            #     package="image_transport",
            #     executable="republish",
            #     arguments=["raw"],
            #     # remappings=[("/in", "/image_raw/compressed"), ("/out", "/camera")],
            # ),
            Node(
                package="image_transport",
                executable="republish",
                arguments=["compressed"],
                remappings=[("/in/compressed", "/image_raw/compressed")],
            ),
            Node(
                package="display",
                executable="main",
                remappings=[("/camera", "/image_raw/compressed")],
            ),
            Node(
                package="pose",
                executable="main",
                remappings=[("/camera", "/image_raw/compressed")],
            ),
        ]
    )
