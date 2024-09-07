import rclpy
import rclpy.executors

from . import _invoker, _reciever


def main():
    rclpy.init()

    invoker_config = _invoker.Config()
    reciever_config = _reciever.Config()

    invoker_node = _invoker.Invoker(invoker_config)
    reciever_node = _reciever.Reciever(reciever_config)

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(invoker_node)
    executor.add_node(reciever_node)

    executor.spin()

    executor.shutdown()
    rclpy.shutdown()
