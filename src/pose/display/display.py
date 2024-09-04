from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
from pydantic import BaseModel
from queue import Queue
import time

from .renderer import Renderer


class Config(BaseModel):
    camera_topic: str = "/camera"
    pose_topic: str = "/pose"
    threshold: float = 0.5
    scale: float = 1.0


class Display(Node):
    def __init__(self, config: Config):
        super().__init__("display")
        self.create_subscription(Image, config.camera_topic, self.__camera_callback, 10)
        self.create_subscription(String, config.pose_topic, self.__pose_callback, 10)

        self.__cv_bridge = CvBridge()
        self.__renderer = Renderer(config.threshold)

        self.__pose_buffer: list[list[int]] = []

        self.__current_fps = 0.0
        self.__last = 0.0
        self.__request_history = Queue(maxsize=8)

        self.__threshold = config.threshold
        self.__scale = config.scale

        self.get_logger().info("Initialized")

    def __camera_callback(self, image: Image):
        cv_image = self.__cv_bridge.imgmsg_to_cv2(image)

        self.__renderer.draw(cv_image, self.__pose_buffer, self.__current_fps)

        cv2.imshow("Display Node", cv_image)
        cv2.waitKey(1)

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
