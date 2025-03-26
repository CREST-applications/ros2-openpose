from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
from pydantic import BaseModel
from queue import Queue
import time

from .inner import Inner


class Config(BaseModel):
    threshold: float
    # scale: float


class Renderer(Node):
    def __init__(self, config: Config):
        super().__init__("renderer")
        self.create_subscription(
            CompressedImage, "/camera", self.__camera_callback, 1
        )
        self.create_subscription(String, "/pose", self.__pose_callback, 1)
        self.__pub = self.create_publisher(CompressedImage, "/rendered", 1)

        self.__cv_bridge = CvBridge()
        self.__inner = Inner(config.threshold)

        self.__pose_buffer: list[list[int]] = []

        self.__current_fps = 0.0
        self.__last = 0.0
        self.__request_history = Queue(maxsize=8)

        # self.__scale = config.scale

        self.get_logger().info("Initialized")

    def __camera_callback(self, image: CompressedImage):
        cv_image = self.__cv_bridge.compressed_imgmsg_to_cv2(image)
        self.__inner.draw(cv_image, self.__pose_buffer, self.__current_fps)
        compressed_image = self.__cv_bridge.cv2_to_compressed_imgmsg(cv_image)

        self.__pub.publish(compressed_image)
        self.get_logger().debug("Published: /rendered")

        # cv_image = cv2.resize(cv_image, None, fx=self.__scale, fy=self.__scale)
        # cv2.imshow("Display Node", cv_image)
        # cv2.waitKey(1)

    def __pose_callback(self, pose: String):
        self.get_logger().debug("Received: /pose")
        self.__pose_buffer = json.loads(pose.data)

        # Calculate FPS
        if self.__request_history.full():
            self.__request_history.get()

        self.__request_history.put(time.time() - self.__last)
        self.__last = time.time()

        mean = sum(self.__request_history.queue) / len(self.__request_history.queue)
        self.__current_fps = 1 / mean
