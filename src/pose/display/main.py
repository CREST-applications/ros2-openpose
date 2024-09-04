import rclpy

from .display import Display, Config


def main():
    config = Config()

    rclpy.init()
    camera = Display(config)
    rclpy.spin(camera)
    rclpy.shutdown()
