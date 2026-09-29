"""The approach-axis task: a direction on the sphere, free about itself.

Skipped without the ``kinematics`` extra (the pure-geometry cases run
regardless).
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from robopy.kinematics.axis_alignment_task import axis_rotation_error


class TestRotationError:
    def test_parallel_perpendicular_and_antiparallel(self) -> None:
        z = np.array([0.0, 0.0, 1.0])
        rotation, angle = axis_rotation_error(z, z)
        assert angle == 0.0 and np.allclose(rotation, 0.0)
        rotation, angle = axis_rotation_error(z, np.array([1.0, 0.0, 0.0]))
        assert angle == pytest.approx(math.pi / 2)
        assert np.allclose(rotation, [0.0, math.pi / 2, 0.0])  # z x x = +y
        rotation, angle = axis_rotation_error(z, -z)
        assert angle == pytest.approx(math.pi)
        assert np.linalg.norm(rotation) == pytest.approx(math.pi)
        assert abs(rotation @ z) < 1e-12  # about a perpendicular axis
        # Deterministic: asking again gives the same axis.
        again, _ = axis_rotation_error(z, -z)
        assert np.allclose(again, rotation)

    def test_the_angle_is_continuous_up_to_the_pole(self) -> None:
        z = np.array([0.0, 0.0, 1.0])
        angles = []
        for eps in np.linspace(1e-2, 1e-7, 8):
            d = np.array([math.sin(eps), 0.0, -math.cos(eps)])  # eps short of anti-parallel
            _, angle = axis_rotation_error(z, d)
            angles.append(angle)
        assert all(a < b for a, b in zip(angles, angles[1:]))
        assert angles[-1] == pytest.approx(math.pi, abs=1e-6)


pink = pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.control.types import DualArmTarget  # noqa: E402
from robopy.kinematics.axis_alignment_task import AxisAlignmentTask  # noqa: E402
from robopy.kinematics.dual_arm_ik import DualArmIKConfig, DualArmIKStatus  # noqa: E402

from .test_dual_arm_ik import DT, _run, _state, _zero  # noqa: E402
from .test_hierarchical_ik import bent, make_ik  # noqa: E402

AXIS = (0.0, 0.0, -1.0)  # the synthetic gripper points down its -Z at the zero pose


def rotate_about(T: np.ndarray, axis_world: np.ndarray, angle: float) -> np.ndarray:
    k = axis_world / np.linalg.norm(axis_world)
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    R = np.eye(3) + math.sin(angle) * K + (1 - math.cos(angle)) * (K @ K)
    out = T.copy()
    out[:3, :3] = R @ T[:3, :3]
    return out


class TestTask:
    def test_jacobian_matches_a_finite_difference_of_the_axis(self, whole_body_model) -> None:
        model = whole_body_model
        task = AxisAlignmentTask(model, "left_tcp", AXIS, cost=1.0)
        q = model.q_from_positions(bent(model))
        a0 = task.current_axis(q)
        J = task.compute_jacobian(q)
        rng = np.random.default_rng(1)
        for _ in range(5):
            v = np.zeros(model.nv)
            v[
                model.v_indices(
                    ["shoulder_roll_left_dof", "elbow_yaw_left_dof", "wrist_pitch_left_dof"]
                )
            ] = rng.normal(size=3) * 1e-6
            a1 = task.current_axis(model.integrate(q, v))
            # The axis moves by omega x a; the task rows are omega's part
            # perpendicular to a, so (J v) x a reproduces the change.
            predicted = np.cross(J @ v, a0)
            assert np.allclose(a1 - a0, predicted, atol=1e-10)

    def test_a_step_along_the_task_reduces_the_angle(self, whole_body_model) -> None:
        model = whole_body_model
        task = AxisAlignmentTask(model, "left_tcp", AXIS, cost=1.0)
        q = model.q_from_positions(bent(model))
        task.set_target_direction([0.3, 0.0, -1.0])
        before = task.angle_error(q)
        H, c = task.compute_qp_objective(q)
        columns = model.v_indices(list(model.movable_joint_names))
        step = np.zeros(model.nv)
        arm = model.v_indices(
            [
                "shoulder_roll_left_dof",
                "elbow_yaw_left_dof",
                "wrist_yaw_left_dof",
                "wrist_pitch_left_dof",
            ]
        )
        step[arm] = -np.linalg.lstsq(H[np.ix_(arm, arm)] + 1e-9 * np.eye(4), c[arm], rcond=None)[0]
        after = task.angle_error(model.integrate(q, 0.2 * step))
        assert after < before
        assert columns.size == model.nv

    def test_construction_errors(self, whole_body_model) -> None:
        with pytest.raises(ValueError, match="non-zero"):
            AxisAlignmentTask(whole_body_model, "left_tcp", (0, 0, 0), cost=1.0)
        task = AxisAlignmentTask(whole_body_model, "left_tcp", AXIS, cost=1.0)
        with pytest.raises(RuntimeError, match="no target"):
            task.compute_error(whole_body_model.neutral_q())
        with pytest.raises(ValueError, match="non-zero"):
            task.set_target_direction([0.0, 0.0, 0.0])


class TestSolverMode:
    def test_rotation_about_the_approach_axis_is_free(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        q0 = model.q_from_positions(start)
        T = model.frame_pose(q0, "left_tcp")
        axis_world = T[:3, :3] @ np.asarray(AXIS)
        # The same pose turned a quarter turn about its own approach axis.
        turned = rotate_about(T, axis_world, math.pi / 2)
        target = DualArmTarget(left_target=turned, right_enabled=False)
        aligned = make_ik(model, orientation_mode="axis_aligned", approach_axis_tcp=AXIS)
        result = aligned.solve_step(_state(model, start), target, DT)
        assert result.status is DualArmIKStatus.CONVERGED, result.message
        assert result.left_axis_error_rad == pytest.approx(0.0, abs=1e-6)
        assert result.left_orientation_error_rad == pytest.approx(math.pi / 2, abs=1e-3)
        # The step itself is (numerically) nothing: nothing was asked.
        assert max(abs(v) for v in result.joint_velocities_rad_s.values()) < 1e-6
        # In pose mode the same target is a real request.
        posed = make_ik(model, orientation_mode="pose")
        assert posed.solve_step(_state(model, start), target, DT).status is DualArmIKStatus.TRACKING

    def test_a_tilted_axis_target_is_reached_with_the_position(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        q0 = model.q_from_positions(start)
        T = model.frame_pose(q0, "left_tcp").copy()
        T[0, 3] += 0.02
        tilted = rotate_about(T, np.array([0.0, 1.0, 0.0]), 0.3)  # tip the axis 0.3 rad
        target = DualArmTarget(left_target=tilted, right_enabled=False)
        ik = make_ik(
            model, orientation_mode="axis_aligned", approach_axis_tcp=AXIS, orientation_cost=1.0
        )
        final, result = _run(ik, model, target, steps=400, start=start)
        assert result.status is DualArmIKStatus.CONVERGED, result.message
        assert result.left_position_error_m < 1e-3
        assert result.left_axis_error_rad < 1e-2
        q = model.q_from_positions(final)
        axis_now = model.frame_pose(q, "left_tcp")[:3, :3] @ np.asarray(AXIS)
        axis_goal = tilted[:3, :3] @ np.asarray(AXIS)
        assert math.acos(float(np.clip(axis_now @ axis_goal, -1, 1))) < 1e-2

    def test_an_antiparallel_target_is_not_called_converged(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        T = model.frame_pose(model.q_from_positions(start), "left_tcp")
        axis_world = T[:3, :3] @ np.asarray(AXIS)
        perpendicular = np.cross(axis_world, [1.0, 0.0, 0.0])
        flipped = rotate_about(T, perpendicular, math.pi)  # the axis now points the other way
        target = DualArmTarget(left_target=flipped, right_enabled=False)
        ik = make_ik(
            model, orientation_mode="axis_aligned", approach_axis_tcp=AXIS, orientation_cost=1.0
        )
        first = ik.solve_step(_state(model, start), target, DT)
        assert first.is_commandable and first.status is DualArmIKStatus.TRACKING
        assert first.left_axis_error_rad is not None and first.left_axis_error_rad > 3.0
        current = dict(start)
        for _ in range(80):
            result = ik.solve_step(_state(model, current), target, DT)
            assert result.is_commandable, result.message
            current.update(result.joint_targets_rad)
        assert result.left_axis_error_rad is not None
        assert result.left_axis_error_rad < first.left_axis_error_rad - 0.5  # it left the pole

    def test_mode_switches_and_missing_axis(self, whole_body_model) -> None:
        ik = make_ik(whole_body_model, orientation_mode="pose")
        with pytest.raises(ValueError, match="approach_axis_tcp"):
            ik.set_orientation_mode("axis_aligned")
        with pytest.raises(ValueError, match="orientation mode"):
            ik.set_orientation_mode("free")
        ik.set_orientation_mode("position_only")
        assert ik.orientation_mode == "position_only"
        with_axis = make_ik(whole_body_model, approach_axis_tcp=AXIS)
        with_axis.set_orientation_mode("axis_aligned")
        assert with_axis.orientation_mode == "axis_aligned"
        assert (
            DualArmIKConfig(
                orientation_mode="axis_aligned", approach_axis_tcp=AXIS
            ).approach_axis_tcp
            == AXIS
        )

    def test_zero_pose_is_still_convergence_free_of_orientation(self, whole_body_model) -> None:
        # position_only never judges orientation, whatever it reports.
        model = whole_body_model
        start = _zero(model)
        start["elbow_pitch_left_dof"] = -0.5
        T = model.frame_pose(model.q_from_positions(start), "left_tcp").copy()
        T[0, 3] += 0.03
        ik = make_ik(model, orientation_mode="position_only")
        _, result = _run(
            ik, model, DualArmTarget(left_target=T, right_enabled=False), steps=300, start=start
        )
        assert result.status is DualArmIKStatus.CONVERGED
        assert (
            result.left_orientation_error_rad is not None
            and result.left_orientation_error_rad > 0.0
        )
