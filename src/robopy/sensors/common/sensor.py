from abc import ABC, abstractmethod
from typing import Generic, TypeVar

T = TypeVar("T")


class Sensor(ABC, Generic[T]):
    """Abstract base class for all sensors."""

    @abstractmethod
    def connect(self) -> None:
        """Connect to the sensor."""
        ...

    @abstractmethod
    def disconnect(self) -> None:
        """Disconnect from the sensor."""
        ...

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Check if the sensor is connected."""
        ...

    @abstractmethod
    def read(self) -> T:
        """Get the current observation from the sensor."""
        ...
