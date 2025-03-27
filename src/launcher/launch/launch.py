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
                arguments=["compressed"],
                remappings=[("/in/compressed", "/image_raw/compressed")],
            ),
            Node(
                package="pose",
                executable="main",
                remappings=[("/camera", "/out/compressed")],
            ),
            Node(
                package="renderer",
                executable="main",
                remappings=[("/camera", "/out/compressed")],
            ),
            Node(
                package="rqt_image_view",
                executable="rqt_image_view",
            ),
        ]
    )
