from abc import ABC, abstractmethod

from robopy.motor.dynamixel_bus import DynamixelBus


class Arm(ABC):
    """Abstract base class for robotic arms"""

    @property
    @abstractmethod
    def port(self) -> str:
        """Abstract property for port"""
        pass

    @property
    @abstractmethod
    def motors(self) -> DynamixelBus:
        """Abstract property for motors"""
        pass

    @property
    @abstractmethod
    def is_connected(self) -> bool:
        """Abstract property to check if the arm is connected"""
        pass

    @abstractmethod
    def connect(self) -> None:
        """Abstract method to connect the arm"""
        pass
