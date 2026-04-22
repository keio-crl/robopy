"""End-effector pose representation for 5-DOF robots."""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class EEPose:
    """5-DOF end-effector pose.

    Attributes:
        x: End-effector X position in meters.
        y: End-effector Y position in meters.
        z: End-effector Z position in meters.
        pitch: Rotation about the Y-axis in radians.
        roll: Rotation about the X-axis in radians.
    """

    x: float
    y: float
    z: float
    pitch: float
    roll: float

    def to_array(self) -> NDArray[np.float64]:
        """Return (5,) numpy array [x, y, z, pitch, roll]."""
        return np.array([self.x, self.y, self.z, self.pitch, self.roll])

    @classmethod
    def from_array(cls, arr: NDArray[np.float64]) -> "EEPose":
        """Construct from a (5,) array."""
        if len(arr) != 5:
            raise ValueError(f"Expected 5 elements, got {len(arr)}")
        return cls(
            x=float(arr[0]),
            y=float(arr[1]),
            z=float(arr[2]),
            pitch=float(arr[3]),
            roll=float(arr[4]),
        )
