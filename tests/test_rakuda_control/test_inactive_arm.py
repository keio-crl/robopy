"""Single-arm IK must let the other TCP move with the shared torso."""

from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.control.types import DualArmTarget, InactiveArmPolicy, TorsoPolicy
from robopy.kinematics.dual_arm_ik import DualArmIK, DualArmIKConfig, DualArmIKStatus
from robopy.kinematics.synthetic_dual_arm import SYNTHETIC_ARM_JOINTS, SYNTHETIC_TORSO_JOINT
from robopy.kinematics.urdf_model import WholeBodyModel
from robopy.models import find_rakuda_model

from .test_dual_arm_ik import DT, _run, _state, _zero


def _single(model, positions, side, **kwargs):
    return DualArmTarget(
        left_enabled=side == "left",
        right_enabled=side == "right",
        **{f"{side}_target": model.frame_pose(model.q_from_positions(positions), f"{side}_tcp")},
        **kwargs,
    )


@pytest.mark.parametrize("side", ["left", "right"])
def test_manual_torso_carries_inactive_tcp_without_arm_compensation(
    dual_arm_ik, whole_body_model, side
):
    model = whole_body_model
    other = "right" if side == "left" else "left"
    start = _zero(model)
    start[f"elbow_pitch_{other}_dof"] = -0.4
    before = model.frame_pose(model.q_from_positions(start), f"{other}_tcp")
    target = _single(model, start, side, torso_policy=TorsoPolicy.MANUAL, torso_velocity_rad_s=0.3)
    # A conflicting posture reference must not pull the inactive arm home.
    dual_arm_ik.set_posture_reference(_zero(model))
    result = dual_arm_ik.solve_step(_state(model, start), target, DT)
    assert result.is_commandable, result.message
    for joint in SYNTHETIC_ARM_JOINTS[other]:
        assert result.joint_targets_rad[joint] == pytest.approx(start[joint], abs=1e-12)
        assert result.joint_velocities_rad_s[joint] == 0.0
    assert getattr(result, f"{other}_hold_residual_m") is None
    assert getattr(result, f"{other}_position_error_m") is None
    after = model.frame_pose(
        model.q_from_positions(start | result.joint_targets_rad), f"{other}_tcp"
    )
    expected = dict(start)
    expected[SYNTHETIC_TORSO_JOINT] += 0.3 * DT
    np.testing.assert_allclose(
        after, model.frame_pose(model.q_from_positions(expected), f"{other}_tcp"), atol=1e-12
    )
    assert np.linalg.norm(after[:3, 3] - before[:3, 3]) > 1e-4


@pytest.mark.parametrize("side", ["left", "right"])
def test_optimized_torso_remains_available_to_single_arm(dual_arm_ik, whole_body_model, side):
    model = whole_body_model
    other = "right" if side == "left" else "left"
    start = _zero(model)
    goal = start | {SYNTHETIC_TORSO_JOINT: 0.15}
    target = _single(model, goal, side, torso_policy=TorsoPolicy.OPTIMIZE)
    # Keep the active arm joints still to make the shared torso necessary.
    dual_arm_ik.config.max_joint_velocity_rad_s = {
        n: 1.0 if n == SYNTHETIC_TORSO_JOINT else 1e-9 for n in dual_arm_ik.active_joints
    }
    final, result = _run(dual_arm_ik, model, target, start=start)
    assert result.status is DualArmIKStatus.CONVERGED, result.message
    assert final[SYNTHETIC_TORSO_JOINT] > 0.14
    for joint in SYNTHETIC_ARM_JOINTS[other]:
        assert final[joint] == pytest.approx(start[joint], abs=1e-12)


@pytest.mark.parametrize("side", ["left", "right"])
def test_switching_from_world_hold_clears_old_target(dual_arm_ik, whole_body_model, side):
    model = whole_body_model
    other = "right" if side == "left" else "left"
    start = _zero(model)
    world = _single(model, start, side, inactive_arm_policy=InactiveArmPolicy.HOLD_WORLD)
    dual_arm_ik.solve_step(_state(model, start), world, DT)
    pushed = start | {f"elbow_pitch_{other}_dof": -0.4}
    follow = replace(world, inactive_arm_policy=InactiveArmPolicy.HOLD_JOINTS)
    result = dual_arm_ik.solve_step(_state(model, pushed), follow, DT)
    assert result.is_commandable
    assert result.joint_velocities_rad_s[f"elbow_pitch_{other}_dof"] == 0.0
    # Switching back without reset/rebaseline must capture where the TCP is now.
    result = dual_arm_ik.solve_step(_state(model, pushed), world, DT)
    assert getattr(result, f"{other}_hold_residual_m") == pytest.approx(0.0, abs=1e-12)


def test_reenabling_arm_restores_its_task(dual_arm_ik, whole_body_model):
    model = whole_body_model
    start = _zero(model)
    target = _single(model, start, "right")
    dual_arm_ik.solve_step(_state(model, start), target, DT)
    goal = start | {"shoulder_roll_left_dof": 0.2, "elbow_pitch_left_dof": -0.3}
    both = replace(
        target,
        left_enabled=True,
        left_target=model.frame_pose(model.q_from_positions(goal), "left_tcp"),
    )
    final, result = _run(dual_arm_ik, model, both, start=start)
    assert result.status is DualArmIKStatus.CONVERGED
    assert abs(final["elbow_pitch_left_dof"]) > 0.05
    assert result.left_position_error_m is not None


@pytest.mark.parametrize("torso", [TorsoPolicy.FIXED, TorsoPolicy.MANUAL])
def test_no_active_arms_handles_empty_qp(dual_arm_ik, whole_body_model, torso):
    start = _zero(whole_body_model)
    target = DualArmTarget(
        left_enabled=False,
        right_enabled=False,
        torso_policy=torso,
        torso_velocity_rad_s=0.1 if torso is TorsoPolicy.MANUAL else 0.0,
    )
    result = dual_arm_ik.solve_step(_state(whole_body_model, start), target, DT)
    assert result.is_commandable, result.message
    assert result.torso_velocity_rad_s == pytest.approx(target.torso_velocity_rad_s)
    for joints in SYNTHETIC_ARM_JOINTS.values():
        assert all(result.joint_velocities_rad_s[j] == 0.0 for j in joints)


def test_inactive_joint_limits_are_still_checked(dual_arm_ik, whole_body_model):
    start = _zero(whole_body_model)
    target = _single(whole_body_model, start, "right")
    start["elbow_pitch_left_dof"] = 10.0
    result = dual_arm_ik.solve_step(_state(whole_body_model, start), target, DT)
    assert result.status is DualArmIKStatus.INFEASIBLE
    assert "Held joint" in result.message


def test_inactive_arm_collision_constraints_are_still_checked(dual_arm_ik, whole_body_model):
    start = _zero(whole_body_model)
    target = _single(whole_body_model, start, "right")
    # The requested clearance is impossible when this arm is held. Removing
    # its task must not remove the model's distance constraints as well.
    dual_arm_ik.config.collision_safety_distance_m = 0.5
    dual_arm_ik.config.collision_activation_distance_m = 0.6
    result = dual_arm_ik.solve_step(_state(whole_body_model, start), target, DT)
    assert not result.is_commandable
    assert result.status is DualArmIKStatus.INFEASIBLE


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("bent", [False, True])
def test_committed_rakuda_urdf_single_arm_torso_motion(side, bent):
    rakuda = find_rakuda_model()
    if rakuda is None:
        pytest.skip("committed Rakuda model not present")
    model = WholeBodyModel.from_urdf(
        rakuda.convex_collision_urdf, package_dirs=rakuda.package_dirs, geometry_only=True
    )
    for s in ("left", "right"):
        model.add_fixed_frame(f"{s}_tcp", f"gripper_{s}_dof", np.eye(4))
    model.set_soft_limits(
        {
            "torso_yaw_dof": (-1.57, 1.57),
            "shoulder_pitch_left_dof": (-3.14, 3.14),
            "shoulder_pitch_right_dof": (-3.14, 3.14),
        }
    )
    arms = {s: [n for n in model.movable_joint_names if s in n] for s in ("left", "right")}
    ik = DualArmIK(
        model,
        left_frame="left_tcp",
        right_frame="right_tcp",
        torso_joint="torso_yaw_dof",
        left_arm_joints=arms["left"],
        right_arm_joints=arms["right"],
        head_joints=["head_yaw_dof", "head_pitch_dof"],
        config=DualArmIKConfig(compute_budget_s=5.0),
    )
    start = _zero(model)
    if bent:
        start.update({"elbow_pitch_left_dof": 0.8, "elbow_pitch_right_dof": -0.8})
    target = _single(model, start, side, torso_policy=TorsoPolicy.MANUAL, torso_velocity_rad_s=0.3)
    other = "right" if side == "left" else "left"
    current = dict(start)
    for _ in range(40):
        result = ik.solve_step(_state(model, current), target, DT)
        assert result.is_commandable, result.message
        current.update(result.joint_targets_rad)
    for joint in arms[other]:
        assert current[joint] == pytest.approx(start[joint], abs=1e-12)
    expected = start | {"torso_yaw_dof": current["torso_yaw_dof"]}
    after = model.frame_pose(model.q_from_positions(current), f"{other}_tcp")
    np.testing.assert_allclose(
        after, model.frame_pose(model.q_from_positions(expected), f"{other}_tcp"), atol=1e-12
    )
    before = model.frame_pose(model.q_from_positions(start), f"{other}_tcp")
    assert np.linalg.norm(after[:3, 3] - before[:3, 3]) > 0.01
