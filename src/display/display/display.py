from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import numpy as np
from pydantic import BaseModel

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

        self.__threshold = config.threshold
        self.__scale = config.scale

    def __camera_callback(self, image: Image):
        cv_image = self.__cv_bridge.imgmsg_to_cv2(image)

        self.__renderer.draw(cv_image, self.__pose_buffer)

        cv2.imshow("Display Node", cv_image)
        cv2.waitKey(1)

    def __pose_callback(self, pose: String):
        self.get_logger().info("Received: /pose")
        self.__pose_buffer = json.loads(pose.data)
