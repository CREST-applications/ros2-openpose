import asyncio
import threading
import time
import logging
from janus import Queue
from typing import TypeVar, Generic


T = TypeVar("T")


class Runner(Generic[T]):
    def __init__(self):
        self.__queue: Queue[T] | None = None
        threading.Thread(target=asyncio.run, args=(self.__run(),), daemon=True).start()

    async def async__init__(self):
        pass

    async def process(self, item: T):
        pass

    async def __run(self):
        await self.async__init__()
        self.__queue = Queue(maxsize=16)

        logging.info("Runner initialized")

        while True:
            data = await self.__queue.async_q.get()
            asyncio.create_task(self.process(data))

    def enqueue(self, item: T):
        self.__wait_ready()
        self.__queue.sync_q.put(item)

    def __wait_ready(self):
        while self.__queue is None:
            logging.info("Waiting for queue to be initialized")
            time.sleep(1)


# class Runner:
#     def __init__(
#         self,
#         async_init_func: Awaitable,
#         async_func: Callable[[object, object], Awaitable],
#     ):
#         self.__queue: Queue | None = None
#         self.__ctx = None
#         self.__init_coroutine = async_init_func
#         self.__async_func = async_func

#         threading.Thread(target=asyncio.run, args=(self.run(),), daemon=True).start()

#     async def run(self):
#         self.__queue = Queue(maxsize=16)
#         self.__ctx = await self.__init_coroutine

#         logging.info("Runner initialized")

#         while True:
#             data = await self.__queue.async_q.get()
#             asyncio.create_task(self.__async_func(self.__ctx, data))

#     def enqueue(self, data):
#         while self.__queue is None:
#             logging.info("Waiting for queue to be initialized")
#             time.sleep(1)

#         self.__queue.sync_q.put(data)
