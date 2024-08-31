from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode(Node):
    def __init__(
        self
    ):
        super().__init__("camera")
        self.__pub = self.create_publisher(Image, "/camera", 1)
        self.__timer = self.create_timer(1, self._timer_callback)
        self.__cv_bridge = CvBridge()

    def _timer_callback(self):
        """
        タイマーコールバック関数。
        カメラから画像を取得し、指定したトピックに画像を配信する。
        """

        # Read the frame
        has_frame, frame = self._capture.read()

        # Check if the frame is valid
        if not has_frame:
            self.get_logger().error("No frame")
            raise FrameNotAvailableError()

        # Convert the frame and publish it
        image = self.__cv_bridge.cv2_to_imgmsg(frame)

        self._camera_publisher.publish(image)
        self.get_logger().info("Publish: /camera")