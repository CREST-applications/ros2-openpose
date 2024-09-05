from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pydantic import BaseModel
import cv2


class Config(BaseModel):
    camera_topic: str = "/camera"
    fps: int = 30
    device: int = 0
    scale: float = 1.0


class Camera(Node):
    def __init__(self, config: Config):
        super().__init__("camera")

        self.__config = config

        self.__pub = self.create_publisher(Image, self.__config.camera_topic, 10)
        self.create_timer(1 / config.fps, self.__callback)

        self.__capture = cv2.VideoCapture(self.__config.device)
        self.__cv_bridge = CvBridge()

        self.get_logger().info(f"Initialized with config: {self.__config}")

    def __callback(self):
        # Read the frame
        has_frame, frame = self.__capture.read()

        # Check if the frame is valid
        if not has_frame:
            self.get_logger().error("No frame")
            return

        # Convert the frame and publish it
        frame = cv2.resize(frame, None, fx=self.__config.scale, fy=self.__config.scale)
        image = self.__cv_bridge.cv2_to_imgmsg(frame)

        self.__pub.publish(image)
        self.get_logger().debug("published to `/camera`")
