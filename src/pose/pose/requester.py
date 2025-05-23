from rclpy.node import Node, Publisher
from rclpy.impl.rcutils_logger import RcutilsLogger
from pydantic import BaseModel
from std_msgs.msg import String
# from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from numpy import ndarray
from cv_bridge import CvBridge
from pleiades import Client, api
import cv2
import asyncio
import time

from .runnner import Runner


class Requester(Runner[ndarray]):
    def __init__(
        self,
        pleiades_host: str,
        publisher: Publisher,
        logger: RcutilsLogger,
        max_job: int,
    ):
        self.pleiades_host = pleiades_host
        self.publisher = publisher
        self.logger = logger
        self.counter = 0
        self.lock = asyncio.Lock()
        self.max_job = max_job

        super().__init__()

    async def async__init__(self) -> None:
        client = Client.builder().host(self.pleiades_host).build()
        lambda_ = await client.call_api(
            api.lambda_.Create(data_id="1", runtime="openpose+gpu")
        )

        self.client = client
        self.lambda_id = lambda_.lambda_id

    async def process(self, image: ndarray):
        async with self.lock:
            if self.counter > self.max_job:
                self.logger.warn("Counter reached max")
                return

            self.counter += 1

        start = time.time()
        pose = await self.run_job(image)
        self.logger.info(f"Elapsed: {time.time() - start} s")

        async with self.lock:
            self.counter -= 1

        if pose is None or len(pose) == 0:
            return

        self.publisher.publish(String(data=pose.decode()))

    async def run_job(self, image: ndarray) -> bytes:
        input = await self.client.call_api(
            api.data.Upload(data=cv2.imencode(".jpg", image)[1].tobytes())
        )
        job = await self.client.call_api(
            api.job.Create(lambda_id=self.lambda_id, input_id=input.data_id, tags=[])
        )
        job_info = await self.client.call_api(
            api.job.Info(job_id=job.job_id, except_="Finished", timeout=10)
        )
        if job_info.output is None:
            return

        pose = await self.client.call_api(
            api.data.Download(data_id=job_info.output.data_id)
        )

        return pose.data


class Config(BaseModel):
    pleiades_host: str
    max_job: int
    max_fps: int


class PoseRequester(Node):
    def __init__(self, config: Config):
        super().__init__("pose_requester")

        self.__config = config
        self.__sub = self.create_subscription(
            CompressedImage,
            "/camera",
            self.__callback,
            1,
        )
        self.__pub = self.create_publisher(String, "/pose", 10)
        self.__bridge = CvBridge()
        self.__runner = Requester(
            config.pleiades_host,
            self.__pub,
            self.get_logger(),
            config.max_job,
        )

        self.get_logger().info("Initialized")

    def __callback(self, msg: CompressedImage):
        self.get_logger().debug("Received image")

        input = self.__bridge.compressed_imgmsg_to_cv2(msg)
        self.__runner.enqueue(input)

        time.sleep(1 / self.__config.max_fps)
