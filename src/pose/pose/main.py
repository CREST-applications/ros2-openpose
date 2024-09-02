import rclpy
import rclpy.executors

from . import invoker, reciever


def main():
    rclpy.init()

    invoker_config = invoker.Config()
    reciever_config = reciever.Config()

    invoker_node = invoker.Invoker(invoker_config)
    reciever_node = reciever.Reciever(reciever_config)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(invoker_node)
    executor.add_node(reciever_node)

    executor.spin()

    executor.shutdown()
    rclpy.shutdown()
