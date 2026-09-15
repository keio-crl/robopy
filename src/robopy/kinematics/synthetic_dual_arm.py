"""A small synthetic dual-arm URDF with a shared torso yaw.

The Rakuda CAD export was not available when this code was written, so the
implementation and its tests are exercised against this fixture instead.  It has
the same *topology* as Rakuda -- one shared torso yaw, six joints per arm, a
two-joint head, and fixed gripper frames whose names end in ``_dof`` -- and it
uses the same joint names, so swapping in the real model is a matter of pointing
at a different file.

What it is **not**: it is not Rakuda.  Its link lengths, axes and masses are
made up for testability.  Passing a test against this fixture says the software
is correct; it says nothing about the real machine's geometry or dynamics.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Sequence, Tuple

__all__ = [
    "SYNTHETIC_ARM_JOINTS",
    "SYNTHETIC_HEAD_JOINTS",
    "SYNTHETIC_TCP_FRAMES",
    "SYNTHETIC_TORSO_JOINT",
    "SyntheticLinkSpec",
    "synthetic_dual_arm_urdf",
    "write_synthetic_dual_arm_urdf",
]

SYNTHETIC_TORSO_JOINT = "torso_yaw_dof"

#: Arm joints from shoulder to wrist, matching the real Rakuda URDF names.
SYNTHETIC_ARM_JOINTS: Dict[str, Tuple[str, ...]] = {
    "left": (
        "shoulder_pitch_left_dof",
        "shoulder_roll_left_dof",
        "elbow_yaw_left_dof",
        "elbow_pitch_left_dof",
        "wrist_yaw_left_dof",
        "wrist_pitch_left_dof",
    ),
    "right": (
        "shoulder_pitch_right_dof",
        "shoulder_roll_right_dof",
        "elbow_yaw_right_dof",
        "elbow_pitch_right_dof",
        "wrist_yaw_right_dof",
        "wrist_pitch_right_dof",
    ),
}

SYNTHETIC_HEAD_JOINTS: Tuple[str, ...] = ("head_yaw_dof", "head_pitch_dof")

#: Fixed frames at each hand.  Named exactly like the real model's, including
#: the misleading ``_dof`` suffix on what are fixed joints.
SYNTHETIC_TCP_FRAMES: Dict[str, str] = {
    "left": "gripper_left_dof",
    "right": "gripper_right_dof",
}

#: Geometry of the fixture, in metres.  Chosen so that at ``q = 0`` every frame
#: sits at an exactly representable position, which makes the forward-kinematics
#: tests checkable by hand.
_TORSO_HEIGHT = 0.40
_SHOULDER_Z = 0.20
_SHOULDER_Y = 0.20
_UPPER_ARM = 0.25
_FOREARM = 0.20
_WRIST_TO_TCP = 0.10
_HEAD_Z = 0.30


@dataclass(frozen=True)
class SyntheticLinkSpec:
    """Nominal dimensions of the synthetic fixture, exposed for tests.

    Attributes:
        torso_height: ``root`` to torso-yaw axis, along +Z.
        shoulder_z: Torso-yaw frame to shoulder frame, along +Z.
        shoulder_y: Lateral offset of each shoulder (left is +Y, right is -Y).
        upper_arm: Shoulder to elbow, along -Z.
        forearm: Elbow to wrist, along -Z.
        wrist_to_tcp: Wrist to TCP frame, along -Z.
        head_z: Torso-yaw frame to head-yaw frame, along +Z.
    """

    torso_height: float = _TORSO_HEIGHT
    shoulder_z: float = _SHOULDER_Z
    shoulder_y: float = _SHOULDER_Y
    upper_arm: float = _UPPER_ARM
    forearm: float = _FOREARM
    wrist_to_tcp: float = _WRIST_TO_TCP
    head_z: float = _HEAD_Z

    def tcp_position_at_zero(self, side: str) -> Tuple[float, float, float]:
        """Position of a TCP frame in ``root`` when every joint is at zero."""
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'.")
        y = self.shoulder_y if side == "left" else -self.shoulder_y
        z = self.torso_height + self.shoulder_z - self.upper_arm - self.forearm - self.wrist_to_tcp
        return (0.0, y, z)


SYNTHETIC_SPEC = SyntheticLinkSpec()

_AXES: Dict[str, str] = {
    "pitch": "0 1 0",
    "roll": "1 0 0",
    "yaw": "0 0 1",
}


def _inertial(mass: float) -> str:
    # Plausible, clearly fictitious values.  They exist so that Pinocchio loads
    # the model without complaint; they are not a claim about any real machine.
    ixx = iyy = izz = round(mass * 1e-3, 9)
    return (
        "    <inertial>\n"
        '      <origin xyz="0 0 0" rpy="0 0 0"/>\n'
        f'      <mass value="{mass}"/>\n'
        f'      <inertia ixx="{ixx}" ixy="0" ixz="0" iyy="{iyy}" iyz="0" izz="{izz}"/>\n'
        "    </inertial>\n"
    )


def _pedestal_visual(height: float, radius: float = 0.06) -> str:
    """A visual-only column from the floor up to the root link's frame.

    The fixture stands on something, as the real machine does.  Without it the
    ``root`` link draws nothing at all and the viewer -- which puts its ground
    plane at the bottom of the base -- showed the robot hovering over the grid.

    No ``<collision>`` is emitted on purpose: this column exists to be looked
    at, and a collision shape here would change every collision-pair count
    measured against this fixture.
    """
    return (
        "    <visual>\n"
        "      <geometry>\n"
        f'        <cylinder radius="{radius}" length="{height:.6f}"/>\n'
        "      </geometry>\n"
        f'      <origin xyz="0 0 {height / 2:.6f}" rpy="0 0 0"/>\n'
        "    </visual>\n"
    )


def _link(name: str, *, mass: float, radius: float, length: float) -> str:
    geometry = (
        "      <geometry>\n"
        f'        <cylinder radius="{radius}" length="{length}"/>\n'
        "      </geometry>\n"
        f'      <origin xyz="0 0 {-length / 2:.6f}" rpy="0 0 0"/>\n'
    )
    return (
        f'  <link name="{name}">\n'
        f"{_inertial(mass)}"
        f"    <visual>\n{geometry}    </visual>\n"
        f"    <collision>\n{geometry}    </collision>\n"
        "  </link>\n"
    )


def _joint(
    name: str,
    *,
    joint_type: str,
    parent: str,
    child: str,
    xyz: Sequence[float],
    axis: str | None,
    lower: float | None = None,
    upper: float | None = None,
) -> str:
    parts = [
        f'  <joint name="{name}" type="{joint_type}">\n',
        f'    <origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="0 0 0"/>\n',
        f'    <parent link="{parent}"/>\n',
        f'    <child link="{child}"/>\n',
    ]
    if axis is not None:
        parts.append(f'    <axis xyz="{axis}"/>\n')
    if joint_type == "revolute":
        # Deliberately *not* effort="1" velocity="1": placeholder limits of that
        # shape are exactly what the audit warns about.
        lo = -2.0 if lower is None else lower
        hi = 2.0 if upper is None else upper
        parts.append(f'    <limit lower="{lo}" upper="{hi}" effort="6.0" velocity="3.0"/>\n')
    parts.append("  </joint>\n")
    return "".join(parts)


def _arm(side: str, spec: SyntheticLinkSpec, *, joint_named_child_links: bool) -> str:
    sign = 1.0 if side == "left" else -1.0
    joints = SYNTHETIC_ARM_JOINTS[side]
    out: list[str] = []
    gripper_link = SYNTHETIC_TCP_FRAMES[side] if joint_named_child_links else f"{side}_gripper_link"

    # shoulder pitch -- continuous, like the real model
    out.append(
        _joint(
            joints[0],
            joint_type="continuous",
            parent="torso_link",
            child=f"{side}_shoulder_pitch_link",
            xyz=(0.0, sign * spec.shoulder_y, spec.shoulder_z),
            axis=_AXES["pitch"],
        )
    )
    out.append(_link(f"{side}_shoulder_pitch_link", mass=0.4, radius=0.03, length=0.02))

    out.append(
        _joint(
            joints[1],
            joint_type="revolute",
            parent=f"{side}_shoulder_pitch_link",
            child=f"{side}_upper_arm_link",
            xyz=(0.0, 0.0, 0.0),
            axis=_AXES["roll"],
            lower=-2.6,
            upper=2.6,
        )
    )
    out.append(_link(f"{side}_upper_arm_link", mass=0.6, radius=0.035, length=spec.upper_arm))

    out.append(
        _joint(
            joints[2],
            joint_type="revolute",
            parent=f"{side}_upper_arm_link",
            child=f"{side}_elbow_yaw_link",
            xyz=(0.0, 0.0, -spec.upper_arm),
            axis=_AXES["yaw"],
            lower=-2.8,
            upper=2.8,
        )
    )
    out.append(_link(f"{side}_elbow_yaw_link", mass=0.2, radius=0.03, length=0.02))

    out.append(
        _joint(
            joints[3],
            joint_type="revolute",
            parent=f"{side}_elbow_yaw_link",
            child=f"{side}_forearm_link",
            xyz=(0.0, 0.0, 0.0),
            axis=_AXES["pitch"],
            lower=-2.4,
            upper=2.4,
        )
    )
    out.append(_link(f"{side}_forearm_link", mass=0.4, radius=0.03, length=spec.forearm))

    out.append(
        _joint(
            joints[4],
            joint_type="revolute",
            parent=f"{side}_forearm_link",
            child=f"{side}_wrist_yaw_link",
            xyz=(0.0, 0.0, -spec.forearm),
            axis=_AXES["yaw"],
            lower=-2.8,
            upper=2.8,
        )
    )
    out.append(_link(f"{side}_wrist_yaw_link", mass=0.15, radius=0.025, length=0.02))

    out.append(
        _joint(
            joints[5],
            joint_type="revolute",
            parent=f"{side}_wrist_yaw_link",
            child=f"{side}_hand_link",
            xyz=(0.0, 0.0, 0.0),
            axis=_AXES["pitch"],
            lower=-1.9,
            upper=1.9,
        )
    )
    out.append(_link(f"{side}_hand_link", mass=0.2, radius=0.025, length=spec.wrist_to_tcp))

    # Fixed gripper frame, named '*_dof' exactly like the real model's, to keep
    # the "a name is not a degree of freedom" case covered by the fixture. The
    # real export also gives the child *link* the same name as the joint, which
    # makes Pinocchio hold two frames of that name; the fixture reproduces that
    # by default so the ambiguity handling is exercised.
    out.append(
        _joint(
            SYNTHETIC_TCP_FRAMES[side],
            joint_type="fixed",
            parent=f"{side}_hand_link",
            child=gripper_link,
            xyz=(0.0, 0.0, -spec.wrist_to_tcp),
            axis=None,
        )
    )
    out.append(_link(gripper_link, mass=0.05, radius=0.02, length=0.01))
    return "".join(out)


def synthetic_dual_arm_urdf(
    spec: SyntheticLinkSpec | None = None,
    *,
    joint_named_child_links: bool = True,
) -> str:
    """Build the synthetic dual-arm URDF as a string.

    Args:
        spec: Optional geometry override.
        joint_named_child_links: Give the gripper and camera child links the
            same names as their fixed joints, as the real Rakuda export does.
            That produces two Pinocchio frames per name and is the case the
            frame resolver has to handle; set ``False`` for unique link names.

    Returns:
        A complete URDF document.  It references no external meshes, so it needs
        no ``package://`` resolution and can be written to any directory.
    """
    s = spec or SYNTHETIC_SPEC
    camera_link = "head_camera_link" if joint_named_child_links else "head_camera_body"
    parts: list[str] = [
        '<?xml version="1.0"?>\n',
        '<robot name="synthetic_dual_arm">\n',
        '  <link name="root">\n',
        _inertial(1.0),
        # The root frame is the mounting point, so the column is drawn from
        # z = 0 (the floor) up to it; it carries no joint and no collision.
        _pedestal_visual(s.torso_height),
        "  </link>\n",
        _joint(
            SYNTHETIC_TORSO_JOINT,
            joint_type="continuous",
            parent="root",
            child="torso_link",
            xyz=(0.0, 0.0, s.torso_height),
            axis=_AXES["yaw"],
        ),
        _link("torso_link", mass=2.0, radius=0.08, length=0.05),
        _arm("left", s, joint_named_child_links=joint_named_child_links),
        _arm("right", s, joint_named_child_links=joint_named_child_links),
        # Head: modelled, but never an IK decision variable.
        _joint(
            SYNTHETIC_HEAD_JOINTS[0],
            joint_type="revolute",
            parent="torso_link",
            child="head_yaw_link",
            xyz=(0.0, 0.0, s.head_z),
            axis=_AXES["yaw"],
            lower=-1.5,
            upper=1.5,
        ),
        _link("head_yaw_link", mass=0.3, radius=0.04, length=0.02),
        _joint(
            SYNTHETIC_HEAD_JOINTS[1],
            joint_type="revolute",
            parent="head_yaw_link",
            child="head_link",
            xyz=(0.0, 0.0, 0.0),
            axis=_AXES["pitch"],
            lower=-1.0,
            upper=1.0,
        ),
        _link("head_link", mass=0.5, radius=0.06, length=0.08),
        _joint(
            "head_camera_link",
            joint_type="fixed",
            parent="head_link",
            child=camera_link,
            xyz=(0.06, 0.0, 0.0),
            axis=None,
        ),
        _link(camera_link, mass=0.05, radius=0.01, length=0.01),
        "</robot>\n",
    ]
    return "".join(parts)


def write_synthetic_dual_arm_urdf(
    path: Path | str,
    spec: SyntheticLinkSpec | None = None,
    *,
    joint_named_child_links: bool = True,
) -> Path:
    """Write the synthetic URDF to ``path`` and return the path."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(
        synthetic_dual_arm_urdf(spec, joint_named_child_links=joint_named_child_links),
        encoding="utf-8",
    )
    return target
