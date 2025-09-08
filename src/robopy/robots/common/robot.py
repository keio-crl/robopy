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
    def teleoperate(self) -> None:
        """Abstract method for teleoperation between leader and follower"""

    @abstractmethod
    def connect(self) -> None:
        """Abstract method to connect to the robot"""

    @abstractmethod
    def disconnect(self) -> None:
        """Abstract method to disconnect from the robot"""
