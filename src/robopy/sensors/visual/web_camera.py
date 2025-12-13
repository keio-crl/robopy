import math
import platform
import time
from datetime import datetime, timezone
from threading import Event
from typing import Literal

import cv2
import numpy as np
from cv2 import VideoCapture
from numpy.typing import NDArray
from rich import print as rprint

from robopy.config.sensor_config.visual_config.camera_config import CameraLog, WebCameraConfig
from robopy.config.types import OSType

from .camera import Camera
from .utils import find_camera_indices


class WebCamera(Camera[NDArray[np.float32]]):
    """Implementation class for cameras using OpenCV"""

    def __init__(
        self,
        camera_index: int,
        name: str,
        config: WebCameraConfig | None,
        **kwargs: object,
    ) -> None:
        """Initialize a WebCamera instance.

        Args:
            camera_index: Index or device id used by OpenCV to open the camera.
            name: Human readable name for the camera instance.
            config: Optional camera configuration. If None, a default
                WebCameraConfig will be used.
            **kwargs: Extra keyword arguments to override attributes of the
                provided `config` object (only attributes that exist on the
                config will be set).

        The constructor does not open the device; call `connect()` to open
        the camera and apply configured settings.
        """
        super().__init__()
        self.camera_index = camera_index
        self.name = name
        self.cap: VideoCapture | None = None
        self.config = config if config is not None else WebCameraConfig()
        self.log: CameraLog = {"timestamp_utc": 0.0, "delta_time": 0.0}
        self.stop_event: Event | None = None
        self._is_connected = False

        for key, value in kwargs.items():
            if hasattr(self.config, key):
                setattr(self.config, key, value)

        # Validation
        if self.config.is_realsense:
            err = (
                "WebCamera cannot be used as a RealSense camera. "
                "Please use RealsenseCamera class instead.",
            )
            raise ValueError(err)

    def connect(self) -> None:
        """Connect to the camera.

        Raises:
            OSError: Unsupported OS

        """
        # Open the camera
        if platform.system() is OSType.LINUX.value:
            temp_cap = cv2.VideoCapture(f"/dev/video{self.camera_index}")
        elif platform.system() is OSType.WINDOWS.value or platform.system() is OSType.MAC.value:
            temp_cap = cv2.VideoCapture(self.camera_index)
        else:
            err = f"Unsupported OS: {platform.system()}"
            raise OSError(err)

        # Validate the camera can be opened
        if not temp_cap.isOpened():
            available_indices = find_camera_indices()
            err = (
                f"Camera with index {self.camera_index} could not be opened. "
                f"Available camera indices are: {available_indices}"
            )
            del temp_cap
            raise OSError(err)
        self.cap = temp_cap
        self._check_set_actual_settings()
        self._is_connected = True

    def read(self, specific_color: Literal["rgb", "bgr"] | None = None) -> NDArray[np.float32]:
        """read frames from the camera and return them as a NumPy array.

        Args:
            specific_color (Literal[&quot;rgb&quot;, &quot;bgr&quot;] | None, optional):
                If &quot;rgb&quot;, convert BGR to RGB. If &quot;bgr&quot;, keep as BGR.
                If None, use the color_mode from the config. Defaults to None.

        Raises:
            OSError: Camera is not connected.
            OSError: Failed to read frame from camera.
            OSError: Camera resolution does not match the expected resolution.

        Returns:
            NDArray: The captured frame as a NumPy array in CHW format.
        """
        if self.cap is None or not self.cap.isOpened():
            err = "Camera is not connected."
            raise OSError(err)

        start_time = time.perf_counter()
        if specific_color is None:
            specific_color = self.config.color_mode
        self.log["timestamp_utc"] = datetime.now(timezone.utc).timestamp()

        ret, color_img = self.cap.read()
        if not ret:
            err = "Failed to read frame from camera."
            raise OSError(err)

        if specific_color == "rgb":
            color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

        H, W, _ = color_img.shape
        if self.config.width is not None and self.config.height is not None:
            if H != self.config.height or W != self.config.width:
                err = (
                    f"Camera resolution is {W}x{H}, but expected "
                    f"{self.config.width}x{self.config.height}."
                )
                raise OSError(err)
        end_time = time.perf_counter()
        self.log["delta_time"] = end_time - start_time

        # last check: convert HWC to CHW
        if color_img.shape[-1] == 3 or color_img.shape[-1] == 1:
            color_img = color_img.transpose(2, 0, 1)  # HWC to CHW
        img: NDArray[np.float32] = color_img.astype("float32")

        return img

    def async_read(self, timeout_ms: float = 100.0) -> NDArray[np.float32]:
        """Asynchronously read the latest frame from the camera.

        This method returns the most recent frame captured by the camera.
        If no new frame is available within the specified timeout, it returns None.

        Args:
            timeout_ms (float, optional): Maximum time to wait for a new frame in milliseconds.
                Defaults to 100.0 ms.

        Raises:
            OSError: Camera is not connected.

        Returns:
            NDArray[np.float32] | None: The latest frame as a NumPy array in CHW format,
                or None if no new frame is available within the timeout.
        """
        if self.cap is None or not self.cap.isOpened():
            err = "Camera is not connected."
            raise OSError(err)

        start_time = time.perf_counter()
        while True:
            ret, color_img = self.cap.read()
            if ret:
                if self.config.color_mode == "rgb":
                    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)

                H, W, _ = color_img.shape
                if self.config.width is not None and self.config.height is not None:
                    if H != self.config.height or W != self.config.width:
                        err = (
                            f"Camera resolution is {W}x{H}, but expected "
                            f"{self.config.width}x{self.config.height}."
                        )
                        raise OSError(err)

                # Convert HWC to CHW
                if color_img.shape[-1] == 3 or color_img.shape[-1] == 1:
                    color_img = color_img.transpose(2, 0, 1)  # HWC to CHW
                img: NDArray[np.float32] = color_img.astype("float32")

                self.log["timestamp_utc"] = datetime.now(timezone.utc).timestamp()
                end_time = time.perf_counter()
                self.log["delta_time"] = end_time - start_time

                return img

            elapsed_time_ms = (time.perf_counter() - start_time) * 1000.0
            if elapsed_time_ms >= timeout_ms:
                raise TimeoutError("No new frame available within the specified timeout.")

    def _check_set_actual_settings(self) -> None:
        """Apply requested camera settings and verify actual values.

        This method attempts to set frame rate, resolution, exposure and
        white balance according to `self.config`. After setting, it reads
        back the actual values from the camera and updates the
        `self.config` fields. If a requested value cannot be applied within
        a small tolerance, an OSError is raised to notify the caller.
        """
        if self.cap is None:
            err = "Camera is not connected."
            raise OSError(err)
        if self.config.fps is not None:
            self.cap.set(cv2.CAP_PROP_FPS, self.config.fps)
        if self.config.width is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.width)
        if self.config.height is not None:
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.height)
        if self.config.auto_exposure:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
        else:
            self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            if self.config.exposure is not None:
                self.cap.set(cv2.CAP_PROP_EXPOSURE, float(self.config.exposure))
        if self.config.auto_white_balance:
            rprint("Setting **auto** white balance")
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 1)

        else:
            rprint("Setting **manual** white balance")
            self.cap.set(cv2.CAP_PROP_AUTO_WB, 0)
            if self.config.white_balance is not None:
                self.cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(self.config.white_balance))

        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        actual_width = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)
        actual_height = self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        actual_exposure = self.cap.get(cv2.CAP_PROP_EXPOSURE)

        if self.config.fps is not None and not math.isclose(
            actual_fps,
            self.config.fps,
            rel_tol=1e-2,
        ):
            err = f"Warning: Unable to set FPS to {self.config.fps}. Actual FPS is {actual_fps}."
            raise OSError(err)
        if self.config.width is not None and not math.isclose(
            actual_width,
            self.config.width,
            rel_tol=1e-2,
        ):
            err = (
                f"Warning: Unable to set width to {self.config.width}. "
                f"Actual width is {actual_width}."
            )
            raise OSError(err)
        if self.config.height is not None and not math.isclose(
            actual_height,
            self.config.height,
            rel_tol=1e-2,
        ):
            err = (
                f"Warning: Unable to set height to {self.config.height}. "
                f"Actual height is {actual_height}."
            )
            raise OSError(err)
        self.config.fps = actual_fps
        self.config.width = actual_width
        self.config.height = actual_height
        self.config.exposure = actual_exposure

    def disconnect(self) -> None:
        """Disconnect and release camera resources.

        This will release the underlying OpenCV VideoCapture if it is open
        and mark the camera as not connected. It is safe to call repeatedly.
        """
        # release and mark disconnected
        self.cap.release() if self.cap is not None else None
        self._is_connected = False

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def __del__(self) -> None:
        """__del__ method to ensure proper cleanup."""
        self.disconnect()
        cv2.destroyAllWindows()

    def record(self) -> None:
        raise NotImplementedError("Recording not implemented for WebCamera.")
