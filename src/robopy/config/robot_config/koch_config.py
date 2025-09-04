from dataclasses import dataclass
from typing import TypedDict

from numpy.typing import ArrayLike


@dataclass
class KochConfig:
    """Configuration class for Koch robot."""

    leader_port: str = "/dev/ttyUSB0"
    follower_port: str = "/dev/ttyUSB1"
    calibration_path: str = ".cache/koch.pkl"


class KochSensorRetuns(TypedDict):
    """Sensor returns class for Koch robot."""

    CAMERA: ArrayLike | None
    TACTILE: ArrayLike | None


class KochObservation(TypedDict):
    """Observation class for Koch robot."""

    leader: ArrayLike
    follower: ArrayLike
    sensors: KochSensorRetuns
