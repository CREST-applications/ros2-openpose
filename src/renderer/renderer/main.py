import rclpy
import os

from .renderer import Renderer, Config


def main():
    config = Config(
        threshold=float(os.environ["POSE_THRESHOLD"]),
        # scale=2.0,
    )

    rclpy.init()
    renderer = Renderer(config)
    rclpy.spin(renderer)
    rclpy.shutdown()
