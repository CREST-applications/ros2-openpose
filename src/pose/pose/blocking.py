# from rclpy.node import Node, Publisher
# from rclpy.impl.rcutils_logger import RcutilsLogger
# from pydantic import BaseModel
# from std_msgs.msg import String
# # from sensor_msgs.msg import Image
# from sensor_msgs.msg import CompressedImage
# from numpy import ndarray
# from cv_bridge import CvBridge
# from pymec import ClientBuilder, api
# import cv2
# import asyncio
# import time


# class Config(BaseModel):
#     pleiades_host: str
#     max_job: int
#     max_fps: int


# class PoseRequester(Node):
#     def __init__(self, config: Config):
#         super().__init__("pose_requester")

#         self.__config = config
#         self.__sub = self.create_subscription(
#             CompressedImage,
#             "/camera",
#             self.__callback,
#             1,
#         )
#         self.__pub = self.create_publisher(String, "/pose", 10)
#         self.__bridge = CvBridge()

#         self.get_logger().info("Initialized")

#     def __callback(self, msg: CompressedImage):
#         self.get_logger().debug("Received image")

#         input = self.__bridge.compressed_imgmsg_to_cv2(msg)
#         self.__runner.enqueue(input)

#         time.sleep(1 / self.__config.max_fps)
