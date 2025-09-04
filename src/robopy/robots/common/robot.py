from abc import ABC, abstractmethod
from typing import Generic, TypeVar

from robopy.config.types import SensorType

T = TypeVar("T")


class Robot(ABC, Generic[T]):
    """Abstract base class for leader-follower robotic systems"""

    @property
    @abstractmethod
    def leader(self) -> T:
        """Abstract property for the leader arm"""
        pass

    @property
    @abstractmethod
    def follower(self) -> T:
        """Abstract property for the follower arm"""
        pass

    @property
    @abstractmethod
    def sensors(self) -> dict[SensorType, T]:
        """Abstract property for sensors"""
        pass

    @abstractmethod
    def get_observation(self) -> dict:
        """Abstract method to get the current observation from the robot"""
        pass

    @abstractmethod
    def send_action(self, action: T) -> None:
        """Abstract method to send an action command to the robot"""
        pass

    @abstractmethod
    def connect(self) -> None:
        """Abstract method to connect to the robot"""
        pass

    @abstractmethod
    def disconnect(self) -> None:
        """Abstract method to disconnect from the robot"""
        pass
