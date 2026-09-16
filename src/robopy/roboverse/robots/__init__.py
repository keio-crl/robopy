"""Robots this content pack offers to MetaSim.

``metasim.utils.setup_util.get_robot`` looks for ``<Name>Cfg`` in this module,
so a name only becomes usable once it is exported here.
"""

from __future__ import annotations

from .rakuda_cfg import (
    RAKUDA_ARM_JOINTS,
    RAKUDA_HEAD_JOINTS,
    RAKUDA_SERVO_BY_JOINT,
    RAKUDA_STAND_HEIGHT_M,
    RAKUDA_TORSO_JOINT,
    RakudaCfg,
)
from .rakuda_gripper_cfg import (
    RAKUDA_FINGER_JOINTS,
    RAKUDA_GRIPPER_JOINTS,
    RakudaGripperCfg,
    gripper_targets,
    pad_gap,
)

__all__ = [
    "RAKUDA_ARM_JOINTS",
    "RAKUDA_FINGER_JOINTS",
    "RAKUDA_GRIPPER_JOINTS",
    "RakudaGripperCfg",
    "gripper_targets",
    "pad_gap",
    "RAKUDA_HEAD_JOINTS",
    "RAKUDA_SERVO_BY_JOINT",
    "RAKUDA_STAND_HEIGHT_M",
    "RAKUDA_TORSO_JOINT",
    "RakudaCfg",
]
