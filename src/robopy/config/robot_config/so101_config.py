from dataclasses import dataclass, field
from typing import Dict

from robopy.config.sensor_config.visual_config.camera_config import (
    RealsenseCameraConfig,
    WebCameraConfig,
)


@dataclass
class So101SensorConfig:
    """Sensor configuration for SO-101 robot."""

    cameras: Dict[str, RealsenseCameraConfig | WebCameraConfig] = field(default_factory=dict)


@dataclass
class So101Config:
    """Configuration class for SO-101 robot.

    The SO-101 uses Feetech STS3215 motors with 6 joints per arm:
    shoulder_pan, shoulder_lift, elbow_flex, wrist_flex, wrist_roll, gripper.
    """

    follower_port: str
    calibration_path: str
    leader_port: str | None = None
    sensors: So101SensorConfig = field(default_factory=So101SensorConfig)


SO101_MOTOR_MAPPING: Dict[str, str] = {
    "shoulder_pan": "shoulder_pan",
    "shoulder_lift": "shoulder_lift",
    "elbow_flex": "elbow_flex",
    "wrist_flex": "wrist_flex",
    "wrist_roll": "wrist_roll",
    "gripper": "gripper",
}
