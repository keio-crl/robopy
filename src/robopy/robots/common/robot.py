from abc import ABC, abstractmethod
from typing import Generic, TypeVar

T = TypeVar("T")


class Robot(ABC, Generic[T]):
    """Abstract base class for leader-follower robotic systems"""

    @property
    @abstractmethod
    def leader(self) -> T:
        """Abstract property for the leader arm"""

    @property
    @abstractmethod
    def follower(self) -> T:
        """Abstract property for the follower arm"""

    @abstractmethod
    def get_observation(self) -> T:
        """Abstract method to get the current observation from the robot"""

    @abstractmethod
    def send_action(self, action: T) -> None:
        """Abstract method to send an action command to the robot"""

    @abstractmethod
    def connect(self) -> None:
        """Abstract method to connect to the robot"""

    @abstractmethod
    def disconnect(self) -> None:
        """Abstract method to disconnect from the robot"""
