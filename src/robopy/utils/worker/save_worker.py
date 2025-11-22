import concurrent.futures
import queue
import threading
from abc import ABC, abstractmethod
from concurrent.futures import Future
from logging import getLogger
from typing import Any, Generic, NamedTuple, TypeVar

import numpy as np
from numpy.typing import NDArray

T = TypeVar("T")

logger = getLogger(__name__)


class SaveTask(NamedTuple):
    """Data class representing a save task"""

    task_type: str
    data: Any
    save_path: str
    fps: int | None = None


class SaveWorker(ABC, Generic[T]):
    def __init__(self, worker_num: int = 2) -> None:
        self.worker_num = worker_num
        self._save_queue: queue.Queue[SaveTask | None] = queue.Queue()
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=worker_num)
        self._stop_event = threading.Event()
        self._background_thread = threading.Thread(target=self._background_saver, daemon=False)
        self._background_thread.start()
        self._futures: list[Future] = []

    @abstractmethod
    def save_arm_datas(
        self, leader_obs: NDArray[np.float32], follower_obs: NDArray[np.float32], path: str
    ) -> None:
        pass

    @abstractmethod
    def save_all_obs(self, obs: T, save_path: str, save_gif: bool) -> None:
        pass

    def enqueue_save_task(self, task: SaveTask) -> None:
        """Add a save task to the queue (non-blocking)"""
        self._save_queue.put(task)

    def wait_all_saved(self) -> None:
        """Wait for all save tasks to complete"""
        logger.info("Waiting for all queued tasks to complete...")
        self._save_queue.join()
        logger.info("All queued tasks marked as done")

    def shutdown(self) -> None:
        """Shutdown the save worker"""
        logger.info("Shutting down save worker...")

        # Wait for all tasks in the queue to complete
        self.wait_all_saved()

        # Send shutdown signal to background thread
        logger.info("Sending shutdown signal to background thread...")
        self._stop_event.set()
        self._save_queue.put(None)  # Shutdown signal

        # Wait for background thread to finish (timeout: 60 seconds)
        logger.info("Waiting for background thread to finish...")
        self._background_thread.join(timeout=60)

        if self._background_thread.is_alive():
            logger.warning("Background thread did not finish in time")
        else:
            logger.info("Background thread finished")

        # Shutdown ThreadPoolExecutor
        logger.info("Shutting down thread pool executor...")
        self._executor.shutdown(wait=True)
        logger.info("Save worker shutdown complete")

    def _background_saver(self) -> None:
        """Background worker thread that processes save tasks from the queue"""
        logger.info("Background saver thread started")

        while True:
            try:
                task = self._save_queue.get(timeout=1.0)
            except queue.Empty:
                if self._stop_event.is_set():
                    continue
                continue

            if task is None:  # Finish signal
                break

            try:
                future = self._process_task(task)
                if future:
                    self._futures.append(future)
            except Exception as e:
                logger.error(f"Error processing task {task.task_type}: {e}", exc_info=True)
            finally:
                self._save_queue.task_done()

        # Wait for all tasks to complete
        logger.info(f"Waiting for {len(self._futures)} tasks to complete...")
        for i, future in enumerate(concurrent.futures.as_completed(self._futures)):
            try:
                future.result()
            except Exception as e:
                logger.error(f"Error in background save task {i + 1}: {e}", exc_info=True)

        logger.info("Background saver thread finished successfully")

    @abstractmethod
    def _process_task(self, task: SaveTask) -> Future | None:
        """Process a single save task.

        Args:
            task (SaveTask): The task to process.

        Returns:
            Future | None: The future object representing the asynchronous execution of the task,
                           or None if no future is created.
        """
        pass
