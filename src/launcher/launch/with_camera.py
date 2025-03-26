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
                package="proxy",
                executable="main",
            ),
            Node(
                package="display",
                executable="main",
                # remappings=[("/camera", "/proxy")],
                remappings=[("/camera", "/delay")],
            ),
            Node(
                package="pose",
                executable="main",
                remappings=[("/camera", "/proxy")],
            ),
        ]
    )
