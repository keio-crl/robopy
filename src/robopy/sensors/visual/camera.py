from abc import ABC, abstractmethod
from typing import Any

from numpy.typing import NDArray


class Camera(ABC):
    """Abstract Base Class for Camera"""

    @abstractmethod
    def connect(self) -> None:
        """Connect to the camera"""

    @abstractmethod
    def read(self) -> NDArray[Any]:
        """Read frames from the camera and return them as a NumPy array"""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the camera"""

    @abstractmethod
    def __del__(self) -> None:
        """Destructor to ensure proper cleanup"""
