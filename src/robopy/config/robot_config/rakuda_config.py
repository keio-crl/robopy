from dataclasses import dataclass
from typing import Dict, TypedDict

from numpy.typing import NDArray


@dataclass
class RakudaConfig:
    """Configuration class for Rakuda robot."""

    leader_port: str
    follower_port: str
    slow_mode: bool = False


class RakudaArmObs(TypedDict):
    leader: NDArray
    follower: NDArray


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
