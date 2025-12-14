import time
from logging import getLogger
from threading import Event, Lock, Thread
from typing import override

import numpy as np
from digit_interface import Digit
from numpy.typing import NDArray

from robopy.config.sensor_config.params_config import TactileParams

from ..common.sensor import Sensor

logger = getLogger(__name__)


class DigitSensor(Sensor[NDArray[np.float32]]):
    def __init__(self, config: TactileParams):
        self.config = config
        self.name = config.name
        self.fps = config.fps
        self.serial_num = config.serial_num
        self._is_connected = False
        self.digit = Digit(serial=self.serial_num, name=self.name)

        # Threading for async read
        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.new_frame_event: Event = Event()
        self.latest_frame: NDArray[np.float32] | None = None
        self.frame_ready = False  # Flag to track if initial frame is ready

    def connect(self) -> None:
        if self.digit is None:
            raise RuntimeError(f"Failed to initialize Digit sensor: {self.name}")
        self.digit.connect()
        self._is_connected = True

        if self.fps is not None:
            self.digit.set_fps(self.fps)

        # Start capture thread and wait for initial frame
        self._start_capture_thread()

        # Wait for initial frame to be available
        logger.info(f"Waiting for initial frame from Digit sensor {self.name}...")
        initial_timeout = 2.0  # 2 seconds for initial frame
        if not self.new_frame_event.wait(timeout=initial_timeout):
            logger.warning(f"Initial frame timeout for Digit sensor {self.name}")
        else:
            logger.info(f"Initial frame received from Digit sensor {self.name}")

        logger.info(f"Digit sensor {self.name} connected.")

    def disconnect(self) -> None:
        # Stop capture thread if running
        self._stop_capture_thread()

        if self.digit is not None:
            self.digit.disconnect()
        self._is_connected = False
        self.frame_ready = False  # Reset frame ready flag
        logger.info(f"Digit sensor {self.name} disconnected.")

    @override
    def read(self) -> NDArray[np.float32]:
        if not self.is_connected:
            raise RuntimeError(f"Digit sensor {self.name} is not connected.")
        frame = self.digit.get_frame().astype(np.float32)
        return frame

    @override
    def async_read(self, timeout_ms: float = 100) -> NDArray[np.float32]:
        """Read the latest frame asynchronously (non-blocking)."""
        if not self.is_connected:
            raise RuntimeError(f"Digit sensor {self.name} is not connected.")

        if self.thread is None or not self.thread.is_alive():
            self._start_capture_thread()
            # Wait a bit for thread to start capturing
            time.sleep(0.01)

        # Check if we already have a frame available
        with self.frame_lock:
            if self.latest_frame is not None and self.frame_ready:
                return self.latest_frame.copy()

        # Wait for new frame
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            thread_alive = self.thread is not None and self.thread.is_alive()
            # Try to get the latest frame even if timeout occurred
            with self.frame_lock:
                if self.latest_frame is not None:
                    logger.debug(f"Using cached frame for {self.name} after timeout")
                    return self.latest_frame.copy()

            raise TimeoutError(
                f"Timeout waiting for new frame from Digit sensor {self.name}. "
                f"Thread alive: {thread_alive}"
            )

        with self.frame_lock:
            frame = self.latest_frame
            # Don't clear the event immediately - keep it for potential quick successive calls
            if frame is not None:
                frame_copy = frame.copy()
            else:
                frame_copy = None

        if frame_copy is None:
            raise RuntimeError(f"No frame available from Digit sensor {self.name}.")

        return frame_copy

    def _start_capture_thread(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            self._stop_capture_thread()

        self.stop_event = Event()
        self.thread = Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info(f"Capture thread for Digit sensor {self.name} started.")

    def _stop_capture_thread(self) -> None:
        if self.stop_event:
            self.stop_event.set()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)

        self.thread = None
        self.stop_event = None
        logger.info(f"Capture thread for Digit sensor {self.name} stopped.")

    def _capture_loop(self) -> None:
        """Background thread loop for continuous frame capture."""
        if self.stop_event is None:
            raise RuntimeError("Stop event is not initialized.")

        logger.debug(f"Capture loop started for Digit sensor {self.name}")
        frame_count = 0

        while not self.stop_event.is_set():
            try:
                frame = self.digit.get_frame().astype(np.float32)

                if frame is not None:
                    with self.frame_lock:
                        self.latest_frame = frame
                        if not self.frame_ready:
                            self.frame_ready = True

                    # Set event to signal new frame availability
                    self.new_frame_event.set()
                    frame_count += 1

                    if frame_count % 100 == 0:  # Log every 100 frames
                        logger.debug(f"Digit sensor {self.name} captured {frame_count} frames")
                else:
                    logger.warning(f"Digit sensor {self.name} returned None frame")

            except Exception as e:
                if self._is_connected:
                    logger.error(f"Error in capture loop for Digit sensor {self.name}: {e}")
                # Add small delay to prevent tight error loop
                time.sleep(0.01)

        logger.debug(
            f"Capture loop stopped for Digit sensor {self.name}, total frames: {frame_count}"
        )

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def __repr__(self) -> str:
        return (
            f"DigitSensor(name={self.name}, serial_num={self.serial_num}, "
            f"fps={self.fps}, connected={self._is_connected})"
        )
