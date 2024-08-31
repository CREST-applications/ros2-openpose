from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from pydantic import BaseModel
import cv2


class Config(BaseModel):
    camera_topic: str = "/camera"
    fps: int = 10
    device: int = 0


class Camera(Node):
    def __init__(self, config: Config):
        super().__init__("camera")
        self.__pub = self.create_publisher(Image, config.camera_topic, 1)
        self.create_timer(1 / config.fps, self.__callback)

        self.__capture = cv2.VideoCapture(config.device)
        self.__cv_bridge = CvBridge()

    def __callback(self):
        # Read the frame
        has_frame, frame = self.__capture.read()

        # Check if the frame is valid
        if not has_frame:
            self.get_logger().error("No frame")
            return

        # Convert the frame and publish it
        image = self.__cv_bridge.cv2_to_imgmsg(frame)

        self.__pub.publish(image)
        self.get_logger().info("published to `/camera`")
