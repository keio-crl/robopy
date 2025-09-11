from enum import Enum


class OSType(Enum):
    """Enum for different operating systems."""

    WINDOWS = "Windows"
    LINUX = "Linux"
    MAC = "Darwin"
    OTHER = "Other"
