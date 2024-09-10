import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class Proxy(Node):
    def __init__(self):
        super().__init__("pub_node")

        self.__pub = self.create_publisher(CompressedImage, "/proxy", 1)
        self.create_subscription(CompressedImage, "/image_raw/compressed", self.__callback, 1)

        self.get_logger().info("Initialized")

    def __callback(self, image: CompressedImage):
        self.__pub.publish(image)


def main():
    rclpy.init()
    node = Proxy()
    rclpy.spin(node)
    rclpy.shutdown()
