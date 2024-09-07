import rclpy
import os

from . import requester


def main():
    rclpy.init()

    config = requester.Config(
        pleiades_host=os.environ["PLEIADES_HOST"],
    )

    requester_node = requester.Requester(config)

    rclpy.spin(requester_node)

    requester_node.destroy_node()
    rclpy.shutdown()
