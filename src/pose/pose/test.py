from rclpy.node import Node, Publisher
from pydantic import BaseModel
from std_msgs.msg import String
from sensor_msgs.msg import Image
from numpy import ndarray
from cv_bridge import CvBridge
from pymec import ClientBuilder, api
from pymec.client import Client
import cv2
import asyncio
import rclpy

from .runnner import Runner


MAX_JOB = 6
lock = asyncio.Lock()


# class Ctx:
#     def __init__(self, client: Client, lambda_id: str, publisher: Publisher, ):
#         self.lambda_id = lambda_id
#         self.client = client
#         self.publisher = publisher
class Ctx:
    def __init__(self, client: Client, lambda_id: str, publisher: Publisher):
        self.client = client
        self.lambda_id = lambda_id
        self.publisher = publisher
        self.counter = 0


async def init(pub: Publisher) -> Ctx:
    # client = ClientBuilder().host("http://master.local/api/v0.5/").build()
    client = ClientBuilder().host("http://192.168.1.20/api/v0.5/").build()
    lambda_ = await client.request(
        api.lambda_.Create(data_id="1", runtime="openpose+gpu")
    )

    return Ctx(client, lambda_.lambda_id, pub)


async def invoke(ctx: Ctx, image: ndarray):
    async with lock:
        if ctx.counter > MAX_JOB:
            print("counter reached max")
            return

        ctx.counter += 1

    client = ctx.client

    input = await client.request(
        api.data.Upload(data=cv2.imencode(".jpg", image)[1].tobytes())
    )
    job = await client.request(
        api.job.Create(lambda_id=ctx.lambda_id, data_id=input.data_id, tags=[])
    )

    job_info = await client.request(
        api.job.Info(job_id=job.job_id, except_="Finished", timeout=10)
    )
    if job_info.output is None:
        return

    pose = await client.request(api.data.Download(data_id=job_info.output.data_id))

    ctx.publisher.publish(String(data=pose.data.decode()))
    print("job finished")

    async with lock:
        ctx.counter -= 1


class Config(BaseModel):
    camera_topic: str = "/camera"
    pose_topic: str = "/pose"


class Requester(Node):
    def __init__(self, config: Config):
        super().__init__("pose_requester")

        self.create_subscription(Image, config.camera_topic, self.__callback, 10)
        self.__pub = self.create_publisher(String, config.pose_topic, 10)

        self.__bridge = CvBridge()
        self.__runner = Runner(init(self.__pub), invoke)

        self.get_logger().info("Pose Invoker Node Initialized")

    def __callback(self, msg: Image):
        self.get_logger().info("Received image")

        input = self.__bridge.imgmsg_to_cv2(msg)
        self.__runner.enqueue(input)


def main():
    rclpy.init()

    requester = Requester(Config())

    rclpy.spin(requester)

    requester.destroy_node()
    rclpy.shutdown()
