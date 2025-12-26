from abc import abstractmethod
from typing import Generic, TypeVar

from robopy.sensors.common.sensor import Sensor

T = TypeVar("T")


class Camera(Sensor[T], Generic[T]):
    """Abstract Base Class for Camera"""

    @abstractmethod
    def record(self) -> None:
        """Start recording video from the camera."""
        ...

    @abstractmethod
    def __del__(self) -> None:
        """Destructor to ensure proper cleanup"""
