from rclpy.node import Node, Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from pydantic import BaseModel
from std_msgs.msg import String
from sensor_msgs.msg import Image
from numpy import ndarray
from cv_bridge import CvBridge
from pymec import ClientBuilder, api
from pymec.client import Client
import cv2
import asyncio
import time

from .runnner import Runner


class Ctx:
    def __init__(
        self,
        client: Client,
        lambda_id: str,
        publisher: Publisher,
        logger: RcutilsLogger,
        max_job: int,
    ):
        self.client = client
        self.lambda_id = lambda_id
        self.publisher = publisher
        self.logger = logger
        self.counter = 0
        self.lock = asyncio.Lock()
        self.max_job = max_job


async def init(pub: Publisher, logger: RcutilsLogger, host: str, max_job: int) -> Ctx:
    client = ClientBuilder().host(host).build()
    lambda_ = await client.request(
        api.lambda_.Create(data_id="1", runtime="openpose+gpu")
    )

    return Ctx(client, lambda_.lambda_id, pub, logger, max_job)


async def invoke(ctx: Ctx, image: ndarray):
    async with ctx.lock:
        if ctx.counter > ctx.max_job:
            ctx.logger.warn("Counter reached max")
            return

        ctx.counter += 1

    client = ctx.client

    start = time.time()

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

    async with ctx.lock:
        ctx.counter -= 1

    # ctx.logger.info("Job Finished")
    end = time.time()
    # print(f"Time: {end - start}")
    ctx.logger.info(f"Elapsed: {end - start} s")

    if pose.data is None or len(pose.data) == 0:
        return

    ctx.publisher.publish(String(data=pose.data.decode()))


class Config(BaseModel):
    camera_topic: str = "/camera"
    pose_topic: str = "/pose"
    pleiades_host: str
    max_job: int = 4
    max_fps: int = 30


class Requester(Node):
    def __init__(self, config: Config):
        super().__init__("pose_requester")
        self.__config = config

        self.create_subscription(Image, config.camera_topic, self.__callback, 1)
        self.__pub = self.create_publisher(String, config.pose_topic, 10)

        self.__bridge = CvBridge()
        self.__runner = Runner(
            init(
                self.__pub,
                self.get_logger(),
                config.pleiades_host,
                config.max_job,
            ),
            invoke,
        )

        self.get_logger().info("Initialized")

    def __callback(self, msg: Image):
        self.get_logger().debug("Received image")

        input = self.__bridge.imgmsg_to_cv2(msg)
        self.__runner.enqueue(input)

        time.sleep(1 / self.__config.max_fps)
