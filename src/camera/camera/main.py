import rclpy

from .camera import Camera, Config


def main():
    config = Config()

    rclpy.init()
    camera = Camera(config)
    rclpy.spin(camera)
    rclpy.shutdown()
