from __future__ import annotations

from dataclasses import dataclass, field
from typing import Dict, List, TypedDict

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
    cameras: List[CameraParams] | None = field(default=None)
    tactile: List[TactileParams] | None = field(default=None)


@dataclass
class RakudaSensorConfigs:
    cameras: List[RealsenseCameraConfig]
    tactile: List[TactileParams]


class RakudaArmObs(TypedDict):
    leader: NDArray[np.float32]
    follower: NDArray[np.float32]


class RakudaSensorObs(TypedDict):
    cameras: Dict[str, NDArray[np.float32] | None]
    tactile: Dict[str, NDArray[np.float32] | None]


class RakudaObs(TypedDict):
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
