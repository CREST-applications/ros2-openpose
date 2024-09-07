from rclpy.node import Node, Publisher
from pydantic import BaseModel
from std_msgs.msg import String
from sensor_msgs.msg import Image
from numpy import ndarray
from cv_bridge import CvBridge
from pymec import ClientBuilder, api
from pymec.client import Client
import cv2

from .runnner import Runner


class Ctx:
    def __init__(self, client: Client, lambda_id: str, publisher: Publisher):
        self.lambda_id = lambda_id
        self.client = client
        self.publisher = publisher


async def init(pub: Publisher) -> Ctx:
    # client = ClientBuilder().host("http://master.local/api/v0.5/").build()
    client = ClientBuilder().host("http://192.168.1.11/api/v0.5/").build()
    lambda_ = await client.request(
        api.lambda_.Create(data_id="1", runtime="openpose+gpu")
    )

    return Ctx(client, lambda_.lambda_id, pub)


async def invoke(ctx: Ctx, image: ndarray):
    client = ctx.client

    input = await client.request(
        api.data.Upload(data=cv2.imencode(".jpg", image)[1].tobytes())
    )
    job = await client.request(
        api.job.Create(lambda_id=ctx.lambda_id, data_id=input.data_id, tags=[])
    )

    ctx.publisher.publish(String(data=job.job_id))
    print("job created")


class Config(BaseModel):
    camera_topic: str = "/camera"
    job_topic: str = "/job"


class Invoker(Node):
    def __init__(self, config: Config):
        super().__init__("pose_invoker")

        self.create_subscription(Image, config.camera_topic, self.__callback, 10)
        self.__pub = self.create_publisher(String, config.job_topic, 10)

        self.__bridge = CvBridge()
        self.__runner = Runner(init(self.__pub), invoke)

        self.get_logger().info("Pose Invoker Node Initialized")

    def __callback(self, msg: Image):
        self.get_logger().info("Received image")

        input = self.__bridge.imgmsg_to_cv2(msg)
        self.__runner.enqueue(input)
