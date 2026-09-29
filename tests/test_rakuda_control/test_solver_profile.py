"""One solver profile for the viewer, the VR simulation and the machine."""

from __future__ import annotations

import inspect
import time
from pathlib import Path

import numpy as np
import pytest

from robopy.kinematics.dual_arm_ik import (
    APPROACH_AXIS_COST,
    ROLL_POSTURE_COST,
    approach_axes_from_model,
    approach_axis_for,
    arm_roll_joints,
    teleop_solver_settings,
)

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from .test_vr_server import SOFT_LIMITS  # noqa: E402


def test_roll_joints_are_the_arm_yaws() -> None:
    joints = [
        "shoulder_pitch_left_dof",
        "shoulder_roll_left_dof",
        "elbow_yaw_left_dof",
        "elbow_pitch_left_dof",
        "wrist_yaw_left_dof",
        "wrist_pitch_left_dof",
        "wrist_yaw_right_dof",
        "head_yaw_dof",
    ]
    assert arm_roll_joints(joints) == [
        "elbow_yaw_left_dof",
        "wrist_yaw_left_dof",
        "wrist_yaw_right_dof",
    ]


def test_profile_is_position_only_with_a_roll_posture() -> None:
    settings = teleop_solver_settings(
        "torso_yaw_dof", arm_joints=["elbow_yaw_left_dof", "elbow_pitch_left_dof"]
    )
    assert settings["orientation_mode"] == "position_only"
    assert settings["task_priority_mode"] == "hierarchical"
    assert settings["limit_avoidance_enabled"] is True
    assert settings["posture_cost"] == {"elbow_yaw_left_dof": ROLL_POSTURE_COST}
    assert settings["posture_reference"] == {"elbow_yaw_left_dof": 0.0}
    assert "posture_cost" not in teleop_solver_settings("torso_yaw_dof")


def test_profile_with_approach_axes_aligns_them() -> None:
    axes = {"left": (0.0, 0.3, 0.95), "right": (0.0, -0.3, 0.95)}
    settings = teleop_solver_settings("torso_yaw_dof", approach_axes=axes)
    assert settings["orientation_mode"] == "axis_aligned"
    assert settings["orientation_cost"] == APPROACH_AXIS_COST
    assert settings["approach_axis_tcp"] == axes
    assert approach_axis_for(settings["approach_axis_tcp"], "right") == (0.0, -0.3, 0.95)
    assert approach_axis_for((0.0, 0.0, 1.0), "left") == (0.0, 0.0, 1.0)
    with pytest.raises(ValueError, match="names no 'left'"):
        approach_axis_for({"right": (0.0, 0.0, 1.0)}, "left")


def test_approach_axes_are_read_off_the_model(synthetic_urdf: Path) -> None:
    from robopy.kinematics.synthetic_dual_arm import SYNTHETIC_ARM_JOINTS
    from robopy.viewer.model_bundle import ModelBundle

    bundle = ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)
    axes = approach_axes_from_model(
        bundle.model,
        bundle.tcp_frames,
        {side: SYNTHETIC_ARM_JOINTS[side][-1] for side in ("left", "right")},
    )
    for side in ("left", "right"):
        assert np.linalg.norm(axes[side]) == pytest.approx(1.0)


def test_viewer_and_machine_build_the_same_profile(synthetic_urdf: Path) -> None:
    """The viewer's IKSetup and the control system's builder agree on the profile."""
    from robopy.kinematics.synthetic_dual_arm import SYNTHETIC_ARM_JOINTS
    from robopy.robots.rakuda import rakuda_control
    from robopy.viewer.model_bundle import ModelBundle
    from robopy.viewer.server import IKSetup

    bundle = ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)
    described = IKSetup(bundle).describe_config()
    axes = approach_axes_from_model(
        bundle.model,
        bundle.tcp_frames,
        {side: SYNTHETIC_ARM_JOINTS[side][-1] for side in ("left", "right")},
    )
    expected = teleop_solver_settings(
        "torso_yaw_dof",
        arm_joints=[*SYNTHETIC_ARM_JOINTS["left"], *SYNTHETIC_ARM_JOINTS["right"]],
        approach_axes=axes,
    )
    for key in ("task_priority_mode", "orientation_mode", "limit_avoidance_enabled"):
        assert described[key] == expected[key]
    assert described["orientation_mode"] == "axis_aligned"
    assert described["posture_cost"] == expected["posture_cost"]
    assert described["approach_axis_tcp"] == expected["approach_axis_tcp"]
    # The machine's builder starts from the same dictionary; control.ik is
    # applied on top by both.
    assert "teleop_solver_settings(" in inspect.getsource(rakuda_control.build_model_and_ik)


def test_the_roll_posture_brings_a_displaced_roll_back(synthetic_urdf: Path) -> None:
    """A roll joint left at 0.5 rad returns towards neutral while the hand holds still."""
    from robopy.control.types import DualArmTarget, TorsoPolicy
    from robopy.viewer.model_bundle import ModelBundle
    from robopy.viewer.server import IKSetup
    from robopy.vr.backend import SimulationBackend, TeleopCommand

    bundle = ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)
    setup = IKSetup(bundle, config_overrides={"max_state_age_s": 10.0})
    rolls = arm_roll_joints(bundle.joint_order)
    if not rolls:
        pytest.skip("the synthetic fixture has no roll joints")
    roll = next(j for j in rolls if "left" in j)
    backend = SimulationBackend(bundle, setup, initial_positions={roll: 0.5})
    hand = backend.hand_pose("left").copy()
    t = 0.0
    for _ in range(120):
        t += 1.0 / 60.0
        target = DualArmTarget(
            left_target=hand,
            right_target=None,
            left_enabled=True,
            right_enabled=False,
            torso_policy=TorsoPolicy.FIXED,
            created_ns=time.monotonic_ns(),
            expiry_ns=time.monotonic_ns() + 10**9,
        )
        report = backend.apply(TeleopCommand(arm_target=target, stamp_s=t))
        assert report.ik_commandable, report.ik_message
    after = backend.joint_positions()[roll]
    assert abs(after) < 0.25, after
    moved = float(np.linalg.norm(backend.hand_pose("left")[:3, 3] - hand[:3, 3]))
    assert moved < 0.02
