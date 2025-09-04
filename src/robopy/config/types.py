from enum import Enum, auto


class OSType(Enum):
    """Enum for different operating systems."""

    WINDOWS = "Windows"
    LINUX = "Linux"
    MAC = "Darwin"
    OTHER = auto()


class SensorType(Enum):
    """Enum for different sensor types."""

    CAMERA = "camera"
    TACTILE = "tactile"
    AUDIO = "audio"