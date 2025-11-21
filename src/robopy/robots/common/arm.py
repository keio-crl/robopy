from abc import ABC, abstractmethod
from typing import Optional

from robopy.motor.dynamixel_bus import DynamixelBus


class Arm(ABC):
    """Abstract base class for robotic arms"""

    @property
    @abstractmethod
    def port(self) -> Optional[str]:
        """Abstract property for port"""
        pass

    @property
    @abstractmethod
    def motors(self) -> Optional[DynamixelBus]:
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

    @property
    @abstractmethod
    def motor_names(self) -> list[str]:
        """Abstract property to get the names of the motors"""
        pass

    @property
    @abstractmethod
    def motor_models(self) -> list[str]:
        """Abstract property to get the models of the motors"""
        pass
