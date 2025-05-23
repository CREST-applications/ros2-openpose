import rclpy
import os

from . import requester


def main():
    rclpy.init()

    config = requester.Config(
        pleiades_host=os.environ["PLEIADES_HOST"],
        max_job=int(os.environ["MAX_JOB"]),
        max_fps=int(os.environ["POSE_FPS"]),
    )

    requester_node = requester.PoseRequester(config)

    rclpy.spin(requester_node)

    requester_node.destroy_node()
    rclpy.shutdown()
