from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.config.sensor_config.params_config import CameraParams, TactileParams
from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig


@dataclass
class RakudaConfig:
    """Configuration class for Rakuda robot."""

    leader_port: str
    follower_port: str
    sensors: RakudaSensorParams | None = field(default=None)
    slow_mode: bool = False


@dataclass
class RakudaSensorParams:
    cameras: List[CameraParams] = field(default_factory=list)
    tactile: List[TactileParams] = field(default_factory=list)


@dataclass
class RakudaSensorConfigs:
    cameras: List[RealsenseCameraConfig]
    tactile: List[TactileParams]


@dataclass
class RakudaArmObs:
    leader: NDArray[np.float32]
    follower: NDArray[np.float32]


@dataclass
class RakudaSensorObs:
    cameras: Dict[str, NDArray[np.float32] | None]
    tactile: Dict[str, NDArray[np.float32] | None]


@dataclass
class RakudaObs:
    """
    Overall observation structure for Rakuda robot.
    arms: Observations from the robot arms (leader and follower).
    sensors: Observations from the sensors (cameras and tactile).
    """

    arms: RakudaArmObs
    sensors: RakudaSensorObs | None


RAKUDA_MOTOR_MAPPING: Dict[str, str] = {
    "torso_yaw": "torso_yaw",
    "head_yaw": "head_yaw",
    "head_pitch": "head_pitch",
    "r_arm_sh_pitch1": "r_arm_sh_pitch1",
    "r_arm_sh_roll": "r_arm_sh_roll",
    "r_arm_sh_pitch2": "r_arm_sh_pitch2",
    "r_arm_el_yaw": "r_arm_el_yaw",
    "r_arm_wr_roll": "r_arm_wr_roll",
    "r_arm_wr_yaw": "r_arm_wr_yaw",
    "r_arm_grip": "r_arm_grip",
    "l_arm_sh_pitch1": "l_arm_sh_pitch1",
    "l_arm_sh_roll": "l_arm_sh_roll",
    "l_arm_sh_pitch2": "l_arm_sh_pitch2",
    "l_arm_el_yaw": "l_arm_el_yaw",
    "l_arm_wr_roll": "l_arm_wr_roll",
    "l_arm_wr_yaw": "l_arm_wr_yaw",
    "l_arm_grip": "l_arm_grip",
}


@dataclass
class RAKUDA_CONTROLTABLE_VALUES:
    GRIP_OPEN_POSITION: int = 2500  # Open position for gripper
    GRIP_PID: Tuple[int, int, int] = (128, 32, 64)  # PID values for gripper control
    GRIP_PID_SLOW: Tuple[int, int, int] = (
        64,
        16,
        32,
    )  # PID values for gripper control in slow mode
    GRIP_GOAL_CURRENT: int = 30  # mA, goal current for gripper motors
    GRIP_MAX_POSITION: int = 2600  # Maximum position for gripper
    GRIP_OPERATING_MODE: int = (
        5  # Operating mode for gripper motors (Current-based position control)
    )
