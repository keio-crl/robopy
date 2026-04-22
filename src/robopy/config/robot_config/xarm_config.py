"""Configuration dataclasses for the xArm robot with GELLO teleoperation leader.

The structure mirrors :mod:`rakuda_config` / :mod:`koch_config` so that users can
configure an xArm system with the same style of dataclasses used for Rakuda or
Koch robots.

The xArm follower is driven through ``xarm-python-sdk`` over TCP/IP (default
192.168.1.240) while the leader is a GELLO Dynamixel controller whose joint
offsets and gripper calibration are expressed in :class:`GelloArmConfig`.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.config.sensor_config.params_config import (
    AudioParams,
    CameraParams,
    TactileParams,
)
from robopy.config.sensor_config.visual_config.camera_config import (
    RealsenseCameraConfig,
    WebCameraConfig,
)


@dataclass
class XArmWorkspaceBounds:
    """Cartesian workspace clipping bounds for the xArm (millimetres).

    The default values correspond to ``config/robot_area.yaml`` from the
    original xarm_modules repository.
    """

    min_x: float = 390.4
    max_x: float = 100000.0
    min_y: float = -257.5
    max_y: float = 314.0
    min_z: float = 25.0
    max_z: float = 10000.0


@dataclass
class GelloArmConfig:
    """GELLO Dynamixel controller calibration for the xArm7 leader.

    The defaults match the legacy ``PORT_CONFIG_MAP`` in
    ``xarm_modules/src/gello_agent.py`` so existing hardware continues to work.
    """

    joint_ids: Tuple[int, ...] = (1, 2, 3, 4, 5, 6, 7)
    joint_offsets: Tuple[float, ...] = (
        np.pi,
        np.pi,
        np.pi / 2,
        np.pi / 2,
        np.pi,
        np.pi / 2,
        np.pi,
    )
    joint_signs: Tuple[int, ...] = (1, 1, 1, 1, 1, 1, 1)
    gripper_id: int = 8
    gripper_open_deg: float = 202.0
    gripper_close_deg: float = 160.0
    motor_model: str = "xl330-m288"
    baudrate: int = 57600
    ema_alpha: float = 0.99

    def __post_init__(self) -> None:
        if not (len(self.joint_ids) == len(self.joint_offsets) == len(self.joint_signs)):
            raise ValueError(
                "joint_ids, joint_offsets and joint_signs must share the same length."
            )


GELLO_XARM7_DEFAULT = GelloArmConfig()


@dataclass
class XArmSensorParams:
    """Sensor parameter bundle shared with robopy's existing sensor abstractions."""

    cameras: List[CameraParams] = field(default_factory=list)
    tactile: List[TactileParams] = field(default_factory=list)
    audio: List[AudioParams] = field(default_factory=list)


@dataclass
class XArmSensorConfigs:
    """Resolved sensor configs consumed by :class:`robopy.robots.xarm.XArmRobot`."""

    cameras: List[RealsenseCameraConfig | WebCameraConfig]
    tactile: List[TactileParams]
    audio: List[AudioParams]


@dataclass
class XArmConfig:
    """Top-level configuration for an xArm7 + GELLO system."""

    follower_ip: str = "192.168.1.240"
    leader_port: str | None = None
    workspace_bounds: XArmWorkspaceBounds | None = None
    control_frequency: float = 50.0
    max_delta: float = 0.05
    cartesian_speed: int = 300
    cartesian_mvacc: int = 1000
    collision_sensitivity: int = 3
    gripper_open: int = 800
    gripper_close: int = 0
    gripper_speed: int = 3000
    start_joints: NDArray[np.float32] | None = None
    gello: GelloArmConfig = field(default_factory=GelloArmConfig)
    sensors: XArmSensorParams | None = None
    sim_mode: bool = False
    sim_host: str = "127.0.0.1"
    sim_port: int = 6000


@dataclass
class XArmArmObs:
    """Arm observation: leader / follower joint positions plus end-effector pose.

    Shapes:
        leader: (8,) -- 7 joints [rad] + gripper [0, 1]
        follower: (8,) -- 7 joints [rad] + gripper [0, 1]
        ee_pos_quat: (7,) -- xyz [m] + quaternion [x, y, z, w]
    """

    leader: NDArray[np.float32]
    follower: NDArray[np.float32]
    ee_pos_quat: NDArray[np.float32]


@dataclass
class XArmSensorObs:
    cameras: Dict[str, NDArray[np.float32] | None]
    tactile: Dict[str, NDArray[np.float32] | None]
    audio: Dict[str, NDArray[np.float32] | None]


@dataclass
class XArmObs:
    """Full observation bundle returned by :class:`XArmRobot` recording APIs."""

    arms: XArmArmObs
    sensors: XArmSensorObs | None


# Motor name convention used for the GELLO leader on the Dynamixel bus.
XARM_LEADER_MOTOR_NAMES: Tuple[str, ...] = (
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
    "joint_7",
    "gripper",
)
