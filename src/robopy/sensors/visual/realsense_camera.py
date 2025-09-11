import logging
import time
from datetime import datetime, timezone
from threading import Event, Lock, Thread
from typing import Literal

import cv2
import numpy as np
from numpy.typing import NDArray

from robopy.config.sensor_config.visual_config.camera_config import CameraLog, RealsenseCameraConfig

from .camera import Camera

try:
    import pyrealsense2 as rs
except ImportError:
    rs = None

logger = logging.getLogger(__name__)


class RealsenseCamera(Camera):
    """Implementation class for Intel RealSense cameras using pyrealsense2

    This implementation focuses on threading to avoid blocking the main thread.
    The camera runs a background thread that continuously captures frames,
    allowing async_read() to return the latest frame without blocking.
    """

    def __init__(self, config: RealsenseCameraConfig) -> None:
        super().__init__()
        self.index = config.index
        self.name = config.name
        self.config = config if config is not None else RealsenseCameraConfig()
        self._is_connected = False
        self.log: CameraLog = {"timestamp_utc": 0.0, "delta_time": 0.0}

        if rs is None:
            raise ImportError(
                "pyrealsense2 is not installed. Please install it to use RealsenseCamera."
            )

        # RealSense pipeline objects
        self.rs_pipeline: rs.pipeline | None = None  # type: ignore
        self.rs_profile: rs.pipeline_profile | None = None  # type: ignore

        # Threading for async capture
        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.depth_lock: Lock = Lock()
        self.latest_color_frame: NDArray | None = None
        self.latest_depth_frame: NDArray | None = None
        self.new_frame_event: Event = Event()

        # Serial number for device identification (more reliable than index)
        self.serial_number: str | None = None

    def connect(self) -> None:
        """Connect to the RealSense camera and start background capture thread."""
        if self._is_connected:
            logger.warning(f"{self.name} is already connected.")
            return

        # Find camera by index or serial number
        self._find_camera_serial()

        # Initialize RealSense pipeline
        self.rs_pipeline = rs.pipeline()  # type: ignore
        rs_config = rs.config()  # type: ignore

        # Configure streams
        if self.serial_number:
            rs_config.enable_device(self.serial_number)  # type: ignore

        # Enable color stream
        if self.config.width and self.config.height and self.config.fps:
            rs_config.enable_stream(  # type: ignore
                rs.stream.color,  # type: ignore
                int(self.config.width),
                int(self.config.height),
                rs.format.rgb8,  # type: ignore
                int(self.config.fps),
            )
        else:
            rs_config.enable_stream(rs.stream.color)  # type: ignore

        # Enable depth stream if it's a depth camera
        if self.config.is_depth_camera:
            if self.config.width and self.config.height and self.config.fps:
                rs_config.enable_stream(  # type: ignore
                    rs.stream.depth,  # type: ignore
                    int(self.config.width),
                    int(self.config.height),
                    rs.format.z16,  # type: ignore
                    int(self.config.fps),
                )
            else:
                rs_config.enable_stream(rs.stream.depth)  # type: ignore

        try:
            # Start pipeline
            self.rs_profile = self.rs_pipeline.start(rs_config)  # type: ignore
            self._update_config_from_stream()
            self._is_connected = True

            # Start background capture thread
            self._start_capture_thread()

            logger.info(f"{self.name} connected successfully.")

        except Exception as e:
            self.rs_pipeline = None
            self.rs_profile = None
            raise ConnectionError(f"Failed to connect to RealSense camera: {e}")

    def disconnect(self) -> None:
        """Disconnect from the camera and stop background thread."""
        if not self._is_connected:
            logger.warning(f"{self.name} is not connected.")
            return

        # Stop background thread
        self._stop_capture_thread()

        # Stop pipeline
        if self.rs_pipeline:
            self.rs_pipeline.stop()  # type: ignore
            self.rs_pipeline = None
            self.rs_profile = None

        self._is_connected = False
        logger.info(f"{self.name} disconnected.")

    def get_observation(
        self, specific_color: Literal["rgb", "bgr"] | None = None
    ) -> NDArray[np.float32]:
        """Read frames from the camera synchronously (blocking).

        Args:
            specific_color: Color format override. If None, uses config.color_mode.

        Returns:
            NDArray: Captured frame in CHW format.
        """
        if not self._is_connected:
            raise OSError("Camera is not connected.")

        start_time = time.perf_counter()
        if specific_color is None:
            specific_color = self.config.color_mode
        self.log["timestamp_utc"] = datetime.now(timezone.utc).timestamp()

        # Wait for frames with timeout
        frames = self.rs_pipeline.wait_for_frames(timeout_ms=1000)  # type: ignore
        color_frame = frames.get_color_frame()  # type: ignore

        if not color_frame:
            raise OSError("Failed to capture color frame.")

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data(), dtype=np.float32)  # type: ignore

        # Convert color format if needed (RealSense outputs RGB by default)
        if specific_color == "bgr":
            color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

        # Validate resolution
        H, W, _ = color_image.shape
        if self.config.width and self.config.height:
            if H != self.config.height or W != self.config.width:
                raise OSError(
                    f"Camera resolution is {W}x{H}, but expected "
                    f"{self.config.width}x{self.config.height}."
                )

        end_time = time.perf_counter()
        self.log["delta_time"] = end_time - start_time

        # Convert HWC to CHW
        if color_image.shape[-1] == 3:
            color_image = color_image.transpose(2, 0, 1)
        color_image = color_image.astype("float32")

        return color_image

    def async_read(self, timeout_ms: float = 200) -> NDArray:
        """Read the latest available frame asynchronously (non-blocking).

        This method retrieves the most recent frame captured by the background
        thread without blocking for camera hardware.

        Args:
            timeout_ms: Maximum time to wait for a new frame.

        Returns:
            NDArray: Latest captured frame in CHW format.
        """
        if not self._is_connected:
            raise OSError("Camera is not connected.")

        if self.thread is None or not self.thread.is_alive():
            self._start_capture_thread()

        # Wait for new frame
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            thread_alive = self.thread is not None and self.thread.is_alive()
            raise TimeoutError(
                f"Timed out waiting for frame from {self.name} after {timeout_ms}ms. "
                f"Capture thread alive: {thread_alive}."
            )

        # Get latest frame
        with self.frame_lock:
            frame = self.latest_color_frame
            self.new_frame_event.clear()

        if frame is None:
            raise RuntimeError(f"Internal error: Event set but no frame available for {self.name}.")

        return frame

    def read_depth(self, timeout_ms: int = 1000) -> NDArray:
        """Read depth frame synchronously.

        Args:
            timeout_ms: Timeout for frame capture.

        Returns:
            NDArray: Depth map in millimeters.
        """
        if not self._is_connected:
            raise OSError("Camera is not connected.")

        if not self.config.is_depth_camera:
            raise RuntimeError("Depth stream is not enabled for this camera.")

        # Wait for frames
        frames = self.rs_pipeline.wait_for_frames(timeout_ms=timeout_ms)  # type: ignore
        depth_frame = frames.get_depth_frame()  # type: ignore

        if not depth_frame:
            raise OSError("Failed to capture depth frame.")

        # Convert to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())  # type: ignore

        return depth_image

    def async_read_depth(self, timeout_ms: float = 200) -> NDArray:
        """Read the latest depth frame asynchronously.

        Args:
            timeout_ms: Maximum time to wait for a new frame.

        Returns:
            NDArray: Latest depth frame.
        """
        if not self._is_connected:
            raise OSError("Camera is not connected.")

        if not self.config.is_depth_camera:
            raise RuntimeError("Depth stream is not enabled for this camera.")

        if self.thread is None or not self.thread.is_alive():
            self._start_capture_thread()

        # Wait for new frame (using same event as color for simplicity)
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(f"Timed out waiting for depth frame from {self.name}.")

        with self.depth_lock:
            depth_frame = self.latest_depth_frame

        if depth_frame is None:
            raise RuntimeError(f"No depth frame available for {self.name}.")

        return depth_frame

    def record(self) -> None:
        """Start recording (placeholder for Camera interface)."""
        # This could be implemented to save frames to disk
        logger.info(f"Recording started for {self.name}")

    def _find_camera_serial(self) -> None:
        """Find camera serial number by index."""
        context = rs.context()  # type: ignore
        devices = context.query_devices()  # type: ignore

        if len(devices) == 0:
            raise ConnectionError("No RealSense devices found.")

        if self.index >= len(devices):
            available_range = f"0-{len(devices) - 1}"
            raise ConnectionError(
                f"Camera index {self.index} not found. Available: {available_range}"
            )

        device = devices[self.index]
        self.serial_number = device.get_info(rs.camera_info.serial_number)  # type: ignore
        logger.info(f"Found camera at index {self.index} with serial: {self.serial_number}")

    def _update_config_from_stream(self) -> None:
        """Update config with actual stream settings."""
        if not self.rs_profile:
            return

        color_stream = self.rs_profile.get_stream(rs.stream.color).as_video_stream_profile()  # type: ignore

        if self.config.fps is None:
            self.config.fps = color_stream.fps()  # type: ignore
        if self.config.width is None:
            self.config.width = color_stream.width()  # type: ignore
        if self.config.height is None:
            self.config.height = color_stream.height()  # type: ignore

    def _capture_loop(self) -> None:
        """Background thread loop for continuous frame capture."""
        logger.debug(f"Capture loop started for {self.name}")
        frame_count = 0

        while not self.stop_event.is_set():  # type: ignore
            try:
                start_time = time.perf_counter()

                # Capture frames with timeout
                frames = self.rs_pipeline.wait_for_frames(timeout_ms=500)  # type: ignore

                # Process color frame
                color_frame = frames.get_color_frame()  # type: ignore
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())  # type: ignore

                    # Convert color format
                    if self.config.color_mode == "bgr":
                        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

                    # Convert to CHW format
                    if color_image.shape[-1] == 3:
                        color_image = color_image.transpose(2, 0, 1)

                    with self.frame_lock:
                        self.latest_color_frame = color_image

                # Process depth frame if available
                if self.config.is_depth_camera:
                    depth_frame = frames.get_depth_frame()  # type: ignore
                    if depth_frame:
                        depth_image = np.asanyarray(depth_frame.get_data())  # type: ignore
                        with self.depth_lock:
                            self.latest_depth_frame = depth_image

                # Signal new frame available
                self.new_frame_event.set()

                # Update logging info
                frame_count += 1
                if frame_count % 100 == 0:  # Log every 100 frames
                    capture_time = (time.perf_counter() - start_time) * 1000
                    logger.debug(
                        f"{self.name}: Captured frame {frame_count} in {capture_time:.1f}ms"
                    )

            except Exception as e:
                if self._is_connected:  # Only log if we expect to be connected
                    logger.warning(f"Error in capture loop for {self.name}: {e}")
                time.sleep(0.01)  # Small delay on error

    def _start_capture_thread(self) -> None:
        """Start background capture thread."""
        if self.thread is not None and self.thread.is_alive():
            return

        self.stop_event = Event()
        self.thread = Thread(target=self._capture_loop, name=f"{self.name}_capture", daemon=True)
        self.thread.start()
        logger.info(f"Capture thread started for {self.name}")

    def _stop_capture_thread(self) -> None:
        """Stop background capture thread."""
        if self.stop_event:
            self.stop_event.set()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)

        self.thread = None
        self.stop_event = None
        logger.info(f"Capture thread stopped for {self.name}")

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def __del__(self) -> None:
        """Destructor to ensure proper cleanup."""
        try:
            if self._is_connected:
                self.disconnect()
        except Exception as e:
            logger.error(f"Error during cleanup of {self.name}: {e}")
