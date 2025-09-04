from dataclasses import dataclass, field
from typing import Literal, TypedDict


@dataclass
class CameraConfig:
    """Base configuration class for cameras."""

    fps: int | float | None = None
    width: int | float | None = None
    height: int | float | None = None
    color_mode: Literal["rgb", "bgr"] = "rgb"
    auto_exposure: bool = False
    exposure: float | None = 190.0
    auto_white_balance: bool = False
    white_balance: float | None = 3300.0
    serial_no: str | None = None

    # These fields are set in subclasses
    is_depth_camera: bool = field(init=False, default=False)
    is_realsense: bool = field(init=False, default=False)


@dataclass
class WebCameraConfig(CameraConfig):
    """Configuration class for web cameras using OpenCV."""

    is_depth_camera: bool = False
    is_realsense: bool = False

    def __post_init__(self) -> None:
        """__post_init__ method to validate the configuration.

        Raises:
            ValueError: If color_mode is not 'rgb' or 'bgr'.

        """
        if self.color_mode is not Literal["rgb", "bgr"]:
            error_msg = (f" {self.color_mode} is not supported. Use 'rgb' or 'bgr'.",)
            raise ValueError(error_msg)
        print(f"OpenCVCameraConfig: {self}")


@dataclass
class RealsenseCameraConfig(CameraConfig):
    """Configuration class for RealSense cameras."""

    fps: int | float | None = 30
    is_depth_camera: bool = True
    is_realsense: bool = True
    min_depth: float = 100.0
    max_depth: float = 2000.0
    name: str = "main"

    def __post_init__(self) -> None:
        """__post_init__ method to validate the configuration.

        Raises:
            ValueError: If color_mode is not 'rgb' or 'bgr'.

        """
        if self.color_mode is not Literal["rgb", "bgr"]:
            error_msg = (f" {self.color_mode} is not supported. Use 'rgb' or 'bgr'.",)
            raise ValueError(error_msg)
        print(f"RealsenseCameraConfig: {self}")


class CameraLog(TypedDict):
    timestamp_utc: float
    delta_time: float
