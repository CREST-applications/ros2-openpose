from rclpy.node import Node, Publisher
from pydantic import BaseModel
from std_msgs.msg import String
from pymec import ClientBuilder, api
from pymec.client import Client

from .runnner import Runner


class Ctx:
    def __init__(self, client: Client, publisher: Publisher):
        self.client = client
        self.publisher = publisher


async def init(pub: Publisher) -> Ctx:
    # client = ClientBuilder().host("http://master.local/api/v0.5/").build()
    client = ClientBuilder().host("http://192.168.1.20/api/v0.5/").build()
    return Ctx(client, pub)


async def invoke(ctx: Ctx, job_id: str):
    client = ctx.client

    job_info = await client.request(
        api.job.Info(job_id=job_id, except_="Finished", timeout=10)
    )
    if job_info.output.data_id is None:
        return

    pose = await client.request(api.data.Download(data_id=job_info.output.data_id))

    ctx.publisher.publish(String(data=pose.data))


class Config(BaseModel):
    job_topic: str = "/job"
    pose_topic: str = "/pose"


class Reciever(Node):
    def __init__(self, config: Config):
        super().__init__("pose_reciever")

        self.create_subscription(String, config.job_topic, self.__callback, 10)
        self.__pub = self.create_publisher(String, config.pose_topic, 10)

        self.__runner = Runner(init(self.__pub), invoke)

        self.get_logger().info("Pose Reciever Node Initialized")

    async def __callback(self, msg: String):
        self.get_logger().info("Received job")
        self.__runner.enqueue(msg.data)
