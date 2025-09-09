from dataclasses import dataclass
from enum import Enum, auto
from typing import List

from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.sensors.visual.web_camera import WebCamera


class OSType(Enum):
    """Enum for different operating systems."""

    WINDOWS = "Windows"
    LINUX = "Linux"
    MAC = "Darwin"
    OTHER = auto()


@dataclass
class Sensors:
    """Enum for different sensor types."""

    CAMERA: List[WebCamera | RealsenseCamera] | None
    TACTILE: str | None = None  # TODO implement TactileSensor class
