from dataclasses import dataclass


@dataclass
class RakudaConfig:
    """Configuration class for Rakuda robot."""

    leader_port: str
    follower_port: str
    calibration_path: str
