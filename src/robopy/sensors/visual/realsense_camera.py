import logging
import time
from datetime import datetime, timezone
from threading import Event, Lock, Thread
from typing import Any, Literal, override

import cv2
import numpy as np
from numpy.typing import NDArray

from robopy.config.sensor_config.visual_config.camera_config import CameraLog, RealsenseCameraConfig

from .camera import Camera

try:
    import pyrealsense2 as rs  # type: ignore
except ImportError:
    rs = None

logger = logging.getLogger(__name__)


class RealsenseCamera(Camera):
    """Implementation class for Intel RealSense cameras using pyrealsense2

    This implementation uses threading to avoid blocking the main thread.
    The camera runs a background thread that continuously captures frames,
    allowing async_read() to return the latest frame without blocking.

    Based on LeRobot's implementation adapted for robopy architecture.
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
        self.new_depth_event: Event = Event()
        self.align: rs.align | None = None  # type: ignore

        # Serial number for device identification (more reliable than index)
        self.serial_number: str | None = None

        # Set capture dimensions considering rotation
        self.capture_width: int | float | None = None
        self.capture_height: int | float | None = None

        if self.config.width and self.config.height:
            self.capture_width, self.capture_height = self.config.width, self.config.height
        else:
            self.capture_width, self.capture_height = None, None

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.name})"

    def connect(self, warmup: bool = True) -> None:
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
            self.align = rs.align(rs.stream.color)  # type: ignore
            self._update_config_from_stream()
            self._is_connected = True

            # Warmup period
            if warmup:
                logger.debug(
                    f"""Warming up {self.name} for 
                        {self.config.warmup_s if hasattr(self.config, "warmup_s") else 1}
                        s...
                    """,
                )
                time.sleep(1)  # Basic warmup
                # Take a few frames to warm up the camera
                for _ in range(5):
                    try:
                        self._read_frame_sync(timeout_ms=1000)
                        time.sleep(0.1)
                    except Exception as e:
                        logger.warning(f"Warmup frame failed: {e}")

            # Start background capture thread
            # self._start_capture_thread()

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

    @override
    def read(self, specific_color: Literal["rgb", "bgr"] | None = None) -> NDArray[np.float32]:
        """Read frames from the camera synchronously (blocking).

        This method provides synchronous frame reading similar to LeRobot's read().
        For non-blocking access, use async_read().

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

        # Read frame synchronously
        color_image = self._read_frame_sync(timeout_ms=1000, color_mode=specific_color)

        end_time = time.perf_counter()
        self.log["delta_time"] = end_time - start_time

        if len(color_image.shape) == 3 and color_image.shape[0] != 3:
            logger.warning(
                f"Unexpected frame shape {color_image.shape} from {self.name}, "
                "expected 3 channels in CHW format."
            )
            color_image = color_image.transpose(2, 1, 0)  # Convert HWC to CHW

        return color_image

    @override
    def async_read(self, timeout_ms: float = 16) -> NDArray[np.float32]:
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

        if len(frame.shape) == 3 and frame.shape[0] != 3:
            logger.warning(
                f"Unexpected frame shape {frame.shape} from {self.name}, "
                "expected 3 channels in CHW format."
            )
            frame = frame.transpose(2, 1, 0)  # Convert HWC to CHW

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
        ret, aligned_frames = self.rs_pipeline.try_wait_for_frames(timeout_ms=timeout_ms)  # type: ignore

        if not ret or aligned_frames is None:
            raise OSError("Failed to capture depth frame.")

        try:
            aligned_frames = self.align.process(aligned_frames)  # type: ignore
        except Exception as e:
            print(e)
            raise OSError("Failed to align depth frame.") from e

        depth_frame = aligned_frames.get_depth_frame()  # type: ignore

        if not depth_frame:
            raise OSError("Failed to get depth frame from frameset.")

        # Convert to numpy array
        depth_image = np.asanyarray(depth_frame.get_data())  # type: ignore

        if depth_image is None:
            raise OSError("Failed to convert depth frame to numpy array.")

        # Ensure depth_image is not None before slicing
        if depth_image is not None:
            depth_image = depth_image[..., np.newaxis]  # Add channel dimension if needed
        else:
            raise OSError("Depth image is None after conversion.")
        depth_image = depth_image.transpose(2, 0, 1)  # Convert HWC to CHW
        depth_image = np.clip(depth_image, 0, self.config.max_depth)

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
        if not self.new_depth_event.wait(timeout=timeout_ms / 1000.0):
            raise TimeoutError(f"Timed out waiting for depth frame from {self.name}.")

        with self.depth_lock:
            depth_frame = self.latest_depth_frame
            self.new_depth_event.clear()

        if depth_frame is None:
            raise RuntimeError(f"No depth frame available for {self.name}.")

        depth_frame = depth_frame[..., np.newaxis]  # Add channel dimension if needed
        depth_frame = depth_frame.transpose(2, 0, 1)  # Convert HWC to CHW
        depth_frame = np.clip(depth_frame, 0, self.config.max_depth)

        return depth_frame

    def record(self) -> None:
        """Start recording (placeholder for Camera interface)."""
        logger.info(f"Recording started for {self.name}")

    @staticmethod
    def find_cameras() -> list[dict[str, Any]]:
        """Find available Intel RealSense cameras connected to the system.

        Returns:
            List of dictionaries containing camera information.
        """
        if rs is None:
            raise ImportError("pyrealsense2 is not installed.")

        found_cameras_info = []
        context = rs.context()  # type: ignore
        devices = context.query_devices()  # type: ignore

        for device in devices:
            camera_info = {
                "name": device.get_info(rs.camera_info.name),  # type: ignore
                "type": "RealSense",
                "serial_number": device.get_info(rs.camera_info.serial_number),  # type: ignore
                "firmware_version": device.get_info(rs.camera_info.firmware_version),  # type: ignore
                "usb_type_descriptor": device.get_info(rs.camera_info.usb_type_descriptor),  # type: ignore
                "physical_port": device.get_info(rs.camera_info.physical_port),  # type: ignore
                "product_id": device.get_info(rs.camera_info.product_id),  # type: ignore
                "product_line": device.get_info(rs.camera_info.product_line),  # type: ignore
            }

            # Get default stream profiles
            sensors = device.query_sensors()  # type: ignore
            for sensor in sensors:
                profiles = sensor.get_stream_profiles()  # type: ignore

                for profile in profiles:
                    if profile.is_video_stream_profile() and profile.is_default():  # type: ignore
                        vprofile = profile.as_video_stream_profile()  # type: ignore
                        stream_info = {
                            "stream_type": vprofile.stream_name(),  # type: ignore
                            "format": vprofile.format().name,  # type: ignore
                            "width": vprofile.width(),  # type: ignore
                            "height": vprofile.height(),  # type: ignore
                            "fps": vprofile.fps(),  # type: ignore
                        }
                        camera_info["default_stream_profile"] = stream_info
                        break

            found_cameras_info.append(camera_info)

        return found_cameras_info

    def _read_frame_sync(
        self, timeout_ms: int = 1000, color_mode: str | None = None
    ) -> NDArray[np.float32]:
        """Read a single frame synchronously from the camera.

        Args:
            timeout_ms: Timeout for frame capture.
            color_mode: Color mode override.

        Returns:
            NDArray: Processed frame in CHW format.
        """
        if color_mode is None:
            color_mode = self.config.color_mode

        # Wait for frames with timeout
        ret, aligned_frames = self.rs_pipeline.try_wait_for_frames(timeout_ms=timeout_ms)  # type: ignore

        if not ret or aligned_frames is None:
            raise OSError(f"Failed to capture frame from {self.name}")

        aligned_frames = self.align.process(aligned_frames)  # type: ignore

        try:
            color_frame = aligned_frames.get_color_frame()  # type: ignore
        except Exception as e:
            print(e)
            color_frame = None

        if not color_frame:
            raise OSError("Failed to get color frame from frameset.")

        # Convert to numpy array
        color_image = np.asanyarray(color_frame.get_data(), dtype=np.float32)  # type: ignore

        # Process the image
        processed_image = self._postprocess_image(color_image, color_mode)

        return processed_image

    def _postprocess_image(
        self, image: NDArray, color_mode: str | None = None
    ) -> NDArray[np.float32]:
        """Process raw image data according to configuration.

        Args:
            image: Raw image data from RealSense (RGB format).
            color_mode: Target color mode.

        Returns:
            NDArray: Processed image in CHW format.
        """
        if color_mode is None:
            color_mode = self.config.color_mode

        processed_image = image

        # Convert color format if needed (RealSense outputs RGB by default)
        if color_mode == "bgr":
            processed_image = cv2.cvtColor(processed_image, cv2.COLOR_RGB2BGR)

        # Validate resolution
        H, W, _ = processed_image.shape
        if self.config.width and self.config.height:
            if H != self.config.height or W != self.config.width:
                raise OSError(
                    f"Camera resolution is {W}x{H}, but expected "
                    f"{self.config.width}x{self.config.height}."
                )

        # Convert HWC to CHW
        if processed_image.shape[-1] == 3:
            processed_image = processed_image.transpose(2, 0, 1)

        processed_image = processed_image.astype("float32")

        return processed_image

    def _find_camera_serial(self) -> None:
        """Find camera serial number by index."""
        if rs is None:
            raise ImportError("pyrealsense2 is not installed.")

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

        # Update capture dimensions
        if self.config.width and self.config.height:
            self.capture_width, self.capture_height = self.config.width, self.config.height

    def _capture_loop(self) -> None:
        """Background thread loop for continuous frame capture.

        Similar to LeRobot's _read_loop but adapted for robopy's structure.
        """
        logger.debug(f"Capture loop started for {self.name}")
        frame_count = 0

        if self.stop_event is None:
            raise RuntimeError("Stop event is not initialized.")

        while not self.stop_event.is_set():
            try:
                start_time = time.perf_counter()

                # Capture frames with timeout
                ret, frames = self.rs_pipeline.try_wait_for_frames(timeout_ms=500)  # type: ignore

                if not ret or frames is None:
                    continue

                color = frames.get_color_frame()
                if not color:
                    continue  # 不完全なセットを捨てる
                if self.config.is_depth_camera:
                    depth = frames.get_depth_frame()
                    if not depth:
                        continue

                try:
                    aligned_frames = self.align.process(frames)  # type: ignore
                except Exception:
                    self.align = rs.align(rs.stream.color)  # type: ignore
                    continue

                # Process color frame
                color_frame = aligned_frames.first(rs.stream.color)  # type: ignore
                if color_frame:
                    color_image = np.asanyarray(color_frame.get_data())  # type: ignore
                    processed_image = self._postprocess_image(color_image)

                    with self.frame_lock:
                        self.latest_color_frame = processed_image

                # Signal new frame available
                self.new_frame_event.set()

                # Process depth frame if available
                if self.config.is_depth_camera:
                    depth_frame = aligned_frames.get_depth_frame()  # type: ignore
                    if depth_frame:
                        depth_image = np.asanyarray(depth_frame.get_data())  # type: ignore
                        with self.depth_lock:
                            self.latest_depth_frame = depth_image

                    # Signal new depth frame available
                    self.new_depth_event.set()

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

        logger.debug(f"Capture loop stopped for {self.name}")

    def _start_capture_thread(self) -> None:
        """Start background capture thread."""
        if self.thread is not None and self.thread.is_alive():
            self._stop_capture_thread()

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
            logger.error(f"Error during cleanup of {self.name}: {e}")

    def __repr__(self) -> str:
        return (
            f"RealsenseCamera(name={self.name}, index={self.index}, connected={self._is_connected})"
        )
