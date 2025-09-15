"""Common robot modules."""

from .arm import Arm
from .composed import ComposedRobot
from .robot import Robot

__all__ = ["Arm", "ComposedRobot", "Robot"]
