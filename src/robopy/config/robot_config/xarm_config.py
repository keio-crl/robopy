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


@dataclass(frozen=True)
class XArmZFloorZone:
    """Raised z lower-bound region (millimetres, xArm base frame).

    Defines a planar (x, y) rectangle within which the effective ``min_z``
    is raised to ``min_z``. Use this to keep the end-effector above an
    obstacle (e.g. a drawer top) only when the gripper is over its footprint,
    while leaving the rest of the workspace unaffected.
    """

    min_x: float
    max_x: float
    min_y: float
    max_y: float
    min_z: float

    def __post_init__(self) -> None:
        if self.min_x > self.max_x or self.min_y > self.max_y:
            raise ValueError(
                f"Invalid XArmZFloorZone footprint: {self}. min must be <= max for x and y."
            )

    def contains(self, x: float, y: float) -> bool:
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


@dataclass
class XArmWorkspaceBounds:
    """Cartesian workspace clipping bounds for the xArm (millimetres).

    The default values correspond to ``config/robot_area.yaml`` from the
    original xarm_modules repository.

    Attributes:
        min_x/max_x/min_y/max_y/min_z/max_z: axis-aligned outer box. Each axis
            is clipped independently when sending an end-effector pose.
        z_floor_zones: optional list of planar (x, y) regions where the
            effective ``min_z`` is raised. The actual z lower bound at any
            point is ``max(min_z, max(zone.min_z for zone containing (x, y)))``.
            Use this to model obstacles (e.g. a drawer) that only occupy
            part of the reachable workspace.
    """

    min_x: float = 390.4
    max_x: float = 100000.0
    min_y: float = -300
    max_y: float = 314.0
    min_z: float = 25.0
    max_z: float = 10000.0
    z_floor_zones: Tuple[XArmZFloorZone, ...] = ()

    def effective_min_z(self, x: float, y: float) -> float:
        """Return the z lower bound applicable at planar position (x, y)."""
        z = self.min_z
        for zone in self.z_floor_zones:
            if zone.contains(x, y) and zone.min_z > z:
                z = zone.min_z
        return z


# Named workspace presets selectable via ``XArmConfig.restriction``.
# Values are millimetres in the xArm base frame. Add new entries below to
# expose additional task-specific restrictions.
XARM_WORKSPACE_PRESETS: Dict[str, XArmWorkspaceBounds] = {
    # Legacy default (xarm_modules/config/robot_area.yaml).
    "default": XArmWorkspaceBounds(),
    # Conservative box for drawer-style manipulation: in front of the base,
    # narrow lateral, low-to-mid height. Tune for your physical setup.
    "drawer": XArmWorkspaceBounds(
        z_floor_zones=(
            XArmZFloorZone(min_x=430.0, max_x=700.0, min_y=55.0, max_y=320.0, min_z=132.4),
        ),
    ),
}


def resolve_workspace_bounds(
    workspace_bounds: "XArmWorkspaceBounds | None",
    restriction: "List[str] | None",
) -> "XArmWorkspaceBounds | None":
    """Combine an explicit ``workspace_bounds`` with named ``restriction`` presets.

    All resolved boxes are intersected (max of mins, min of maxes) so the final
    box is the most restrictive that satisfies every input. Returns ``None`` if
    both inputs are empty (no clipping).

    Raises:
        ValueError: if a restriction name is unknown, or if the resulting
            intersection is empty (some axis has min > max).
    """
    boxes: List[XArmWorkspaceBounds] = []
    if workspace_bounds is not None:
        boxes.append(workspace_bounds)
    if restriction:
        for name in restriction:
            if name not in XARM_WORKSPACE_PRESETS:
                known = ", ".join(sorted(XARM_WORKSPACE_PRESETS))
                raise ValueError(
                    f"Unknown xArm workspace restriction preset: {name!r}. Known presets: {known}"
                )
            boxes.append(XARM_WORKSPACE_PRESETS[name])
    if not boxes:
        return None

    zones: List[XArmZFloorZone] = []
    for b in boxes:
        zones.extend(b.z_floor_zones)

    bounds = XArmWorkspaceBounds(
        min_x=max(b.min_x for b in boxes),
        max_x=min(b.max_x for b in boxes),
        min_y=max(b.min_y for b in boxes),
        max_y=min(b.max_y for b in boxes),
        min_z=max(b.min_z for b in boxes),
        max_z=min(b.max_z for b in boxes),
        z_floor_zones=tuple(zones),
    )
    if bounds.min_x > bounds.max_x or bounds.min_y > bounds.max_y or bounds.min_z > bounds.max_z:
        raise ValueError(
            f"Empty xArm workspace after intersection: {bounds}. Inputs do not overlap."
        )
    return bounds


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
            raise ValueError("joint_ids, joint_offsets and joint_signs must share the same length.")


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
    restriction: List[str] | None = None
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
