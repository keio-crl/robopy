from abc import abstractmethod

from robopy.sensors.common.sensor import Sensor


class Camera(Sensor):
    """Abstract Base Class for Camera"""

    @abstractmethod
    def record(self) -> None:
        """Start recording video from the camera."""
        ...

    @abstractmethod
    def __del__(self) -> None:
        """Destructor to ensure proper cleanup"""
