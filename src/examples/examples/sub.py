import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class SubNode(Node):
    def __init__(self):
        super().__init__("sub_node")

        self.create_subscription(String, "/example", self.__callback, 10)

    def __callback(self, message: String):
        self.get_logger().info(f"Received from `/example`: {message.data}")


def main():
    rclpy.init()
    node = SubNode()
    rclpy.spin(node)
    rclpy.shutdown()
