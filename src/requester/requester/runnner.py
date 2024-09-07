import asyncio
import threading
import time
import logging
from janus import Queue
from collections.abc import Awaitable, Callable


class Runner:
    def __init__(
        self,
        async_init_func: Awaitable,
        async_func: Callable[[object, object], Awaitable],
    ):
        self.__queue: Queue | None = None
        self.__ctx = None
        self.__init_coroutine = async_init_func
        self.__async_func = async_func

        threading.Thread(target=asyncio.run, args=(self.run(),), daemon=True).start()

    async def run(self):
        self.__queue = Queue(maxsize=16)
        self.__ctx = await self.__init_coroutine

        logging.info("Runner initialized")

        while True:
            data = await self.__queue.async_q.get()
            asyncio.create_task(self.__async_func(self.__ctx, data))

    def enqueue(self, data):
        while self.__queue is None:
            logging.info("Waiting for queue to be initialized")
            time.sleep(1)

        self.__queue.sync_q.put(data)
