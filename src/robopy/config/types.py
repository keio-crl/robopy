from enum import Enum, auto
from typing import List, TypedDict

from robopy.visual.realsense_camera import RealsenseCamera
from robopy.visual.web_camera import WebCamera


class OSType(Enum):
    """Enum for different operating systems."""

    WINDOWS = "Windows"
    LINUX = "Linux"
    MAC = "Darwin"
    OTHER = auto()


class Sensors(TypedDict):
    """Enum for different sensor types."""

    CAMERA: List[WebCamera | RealsenseCamera] | None
    TACTILE: str | None  # TODO implement TactileSensor class
