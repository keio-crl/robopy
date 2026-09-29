"""The reference governor and the timed joint trajectory it drives.

The governor is pure numpy; the trajectory runs need the ``kinematics`` extra.
"""

from __future__ import annotations

import math
from typing import Dict, List

import numpy as np
import pytest

from robopy.kinematics.cartesian_trajectory import (
    PoseReference,
    TrajectoryLimits,
    rotation_exp,
    rotation_log,
)

LIMITS = TrajectoryLimits(
    max_linear_velocity_m_s=0.2,
    max_linear_acceleration_m_s2=1.0,
    max_angular_velocity_rad_s=1.0,
    max_angular_acceleration_rad_s2=4.0,
)
DT = 0.01


def pose(p, R=None):  # type: ignore[no-untyped-def]
    T = np.eye(4)
    T[:3, 3] = p
    if R is not None:
        T[:3, :3] = R
    return T


def run_to_arrival(ref: PoseReference, dt: float = DT, max_s: float = 10.0):  # type: ignore[no-untyped-def]
    samples = []
    t = 0.0
    while t < max_s:
        s = ref.advance(dt)
        t += dt
        samples.append((t, s))
        if s.arrived:
            break
    return samples


class TestGovernor:
    def test_from_rest_the_path_is_straight_and_within_the_ceilings(self) -> None:
        ref = PoseReference(pose([0.0, 0.0, 0.0]), LIMITS)
        ref.set_goal(pose([0.3, 0.1, 0.0]))
        samples = run_to_arrival(ref)
        assert samples[-1][1].arrived
        direction = np.array([0.3, 0.1, 0.0]) / np.linalg.norm([0.3, 0.1, 0.0])
        speeds = []
        previous = np.zeros(3)
        for index, (_, s) in enumerate(samples):
            p = s.pose[:3, 3]
            # Every point lies on the segment: no lateral deviation.
            lateral = p - (p @ direction) * direction
            assert np.linalg.norm(lateral) < 1e-9
            v = s.linear_velocity
            speeds.append(float(np.linalg.norm(v)))
            assert np.linalg.norm(v) <= LIMITS.max_linear_velocity_m_s + 1e-9
            # The landing sample absorbs the sub-step remainder; every other
            # step changes the velocity by at most a_max * dt.
            if index < len(samples) - 2:
                assert (
                    np.linalg.norm(v - previous) <= LIMITS.max_linear_acceleration_m_s2 * DT + 1e-9
                )
            previous = v
        assert max(speeds) == pytest.approx(LIMITS.max_linear_velocity_m_s, rel=1e-6)
        # 0.316 m at 0.2 m/s with 0.2 s ramps: about 1.8 s, not 350 ms.
        assert 1.6 < samples[-1][0] < 2.0
        assert np.allclose(ref.pose[:3, 3], [0.3, 0.1, 0.0])

    def test_rotation_follows_the_geodesic(self) -> None:
        R_goal = rotation_exp(np.array([0.0, 0.0, 1.2]))
        ref = PoseReference(pose([0.0, 0.0, 0.0]), LIMITS)
        ref.set_goal(pose([0.0, 0.0, 0.0], R_goal))
        samples = run_to_arrival(ref)
        assert samples[-1][1].arrived
        previous = np.zeros(3)
        for index, (_, s) in enumerate(samples):
            r = rotation_log(s.pose[:3, :3])
            # Always a rotation about the goal's axis, by a growing angle.
            assert abs(r[0]) < 1e-9 and abs(r[1]) < 1e-9 and r[2] >= -1e-12
            w = s.angular_velocity
            assert np.linalg.norm(w) <= LIMITS.max_angular_velocity_rad_s + 1e-9
            if index < len(samples) - 2:
                assert (
                    np.linalg.norm(w - previous)
                    <= LIMITS.max_angular_acceleration_rad_s2 * DT + 1e-9
                )
            previous = w
        assert np.allclose(ref.pose[:3, :3], R_goal, atol=1e-9)

    def test_retargeting_bends_the_path_without_stopping(self) -> None:
        ref = PoseReference(pose([0.0, 0.0, 0.0]), LIMITS)
        ref.set_goal(pose([0.5, 0.0, 0.0]))
        for _ in range(60):
            ref.advance(DT)
        v_before = ref.linear_velocity
        assert v_before[0] > 0.15  # cruising
        ref.set_goal(pose([0.3, 0.3, 0.0]))
        s = ref.advance(DT)
        # The velocity is continuous across the re-target (bounded change).
        assert (
            np.linalg.norm(s.linear_velocity - v_before)
            <= LIMITS.max_linear_acceleration_m_s2 * DT + 1e-9
        )
        assert np.linalg.norm(s.linear_velocity) > 0.1  # it did not stop
        samples = run_to_arrival(ref)
        assert samples[-1][1].arrived
        assert np.allclose(ref.pose[:3, 3], [0.3, 0.3, 0.0])

    def test_braking_brings_the_reference_to_rest(self) -> None:
        ref = PoseReference(pose([0.0, 0.0, 0.0]), LIMITS)
        ref.set_goal(pose([1.0, 0.0, 0.0]))
        for _ in range(40):
            ref.advance(DT)
        assert np.linalg.norm(ref.linear_velocity) > 0.1
        for _ in range(40):
            s = ref.advance(DT, brake=True)
        assert np.linalg.norm(s.linear_velocity) < 1e-9
        assert s.braking and not s.arrived
        distance, _ = ref.remaining()
        assert distance > 0.5

    def test_limits_are_validated(self) -> None:
        with pytest.raises(ValueError, match="max_linear_velocity_m_s"):
            TrajectoryLimits(0.0, 1.0, 1.0, 1.0)
        with pytest.raises(ValueError, match="lag_tolerance_m"):
            TrajectoryLimits(1.0, 1.0, 1.0, 1.0, lag_tolerance_m=0.0)
        with pytest.raises(ValueError, match="pose"):
            PoseReference(np.eye(3), LIMITS)
        ref = PoseReference(np.eye(4), LIMITS)
        assert ref.arrived and ref.advance(DT).arrived
        with pytest.raises(ValueError):
            ref.advance(0.0)

    def test_log_and_exp_round_trip(self) -> None:
        rng = np.random.default_rng(3)
        for _ in range(20):
            r = rng.normal(size=3)
            r *= rng.uniform(0.0, math.pi - 1e-3) / np.linalg.norm(r)
            assert np.allclose(rotation_log(rotation_exp(r)), r, atol=1e-9)
        assert np.allclose(rotation_log(np.eye(3)), 0.0)
        half_turn = rotation_exp(np.array([0.0, math.pi, 0.0]))
        assert np.linalg.norm(rotation_log(half_turn)) == pytest.approx(math.pi)


pink = pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.control.types import InactiveArmPolicy, TorsoPolicy  # noqa: E402
from robopy.kinematics.cartesian_trajectory import run_trajectory  # noqa: E402
from robopy.kinematics.dual_arm_ik import DualArmIKStatus  # noqa: E402

from .test_hierarchical_ik import GAIN_TAU, bent, left_target_ahead, make_ik  # noqa: E402

FRAMES = {"left": "left_tcp", "right": "right_tcp"}


def joint_speeds(samples) -> List[Dict[str, float]]:  # type: ignore[no-untyped-def]
    out = []
    for a, b in zip(samples, samples[1:]):
        dt = b.time_from_start_s - a.time_from_start_s
        out.append({k: (b.joints[k] - a.joints[k]) / dt for k in a.joints})
    return out


class TestRunTrajectory:
    def test_the_hand_follows_a_straight_reference_to_the_goal(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        goal = left_target_ahead(model, start, 0.08)
        ik = make_ik(
            model,
            task_priority_mode="hierarchical",
            orientation_mode="position_only",
            gain_time_constant_s=GAIN_TAU,
        )
        trajectory = run_trajectory(
            ik,
            start,
            {"left": goal},
            LIMITS,
            dt=0.02,
            max_duration_s=6.0,
            frames=FRAMES,
            torso_policy=TorsoPolicy.OPTIMIZE,
        )
        assert trajectory.status is DualArmIKStatus.CONVERGED, trajectory.message
        assert not trajectory.truncated
        first, last = trajectory.samples[0], trajectory.samples[-1]
        assert first.time_from_start_s == 0.0
        assert last.goal_error_m["left"] < 1e-3
        # 8 cm at 0.2 m/s with ramps: roughly 0.6 s, not compressed to 350 ms
        # nor stretched by a failing reference.
        assert 0.4 < trajectory.duration_s < 1.5
        # The path error against the straight reference stays small the whole
        # way, and the goal error decreases monotonically to within noise.
        p0 = first.tcp["left"][:3, 3]
        direction = (goal[:3, 3] - p0) / np.linalg.norm(goal[:3, 3] - p0)
        worst = 0.0
        for sample in trajectory.samples:
            p = sample.tcp["left"][:3, 3]
            lateral = (p - p0) - ((p - p0) @ direction) * direction
            worst = max(worst, float(np.linalg.norm(lateral)))
            assert sample.reference_error_m["left"] < 0.01
        assert worst < 2e-3
        # Joint velocities and accelerations respect the solver's own bounds.
        speeds = joint_speeds(trajectory.samples)
        v_max = ik.config.max_joint_velocity_rad_s
        a_max = ik.config.max_joint_acceleration_rad_s2
        assert isinstance(v_max, float) and isinstance(a_max, float)
        for v in speeds:
            assert max(abs(x) for x in v.values()) <= v_max + 1e-6
        for a, b in zip(speeds[1:], speeds[2:]):
            for k in a:
                assert abs(b[k] - a[k]) <= a_max * 0.02 + 1e-6
        # The undriven arm kept its joints.
        for name in trajectory.samples[-1].joints:
            if "right" in name and "head" not in name:
                assert trajectory.samples[-1].joints[name] == pytest.approx(start[name], abs=1e-12)

    def test_an_unreachable_goal_stalls_and_says_why(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        goal = left_target_ahead(model, start, 1.5)  # a metre and a half ahead
        ik = make_ik(model, orientation_mode="position_only", gain_time_constant_s=GAIN_TAU)
        trajectory = run_trajectory(
            ik,
            start,
            {"left": goal},
            TrajectoryLimits(1.0, 5.0, 1.0, 4.0, lag_tolerance_m=None),
            dt=0.02,
            max_duration_s=12.0,
            frames=FRAMES,
            settle_samples=20,
        )
        assert trajectory.status.is_stall, (trajectory.status, trajectory.message)
        assert trajectory.status in (
            DualArmIKStatus.LOCALLY_STALLED,
            DualArmIKStatus.LIMITS_BLOCKED,
        )
        assert "stopped improving" in trajectory.message
        assert trajectory.samples[-1].goal_error_m["left"] > 0.5
        assert not trajectory.truncated

    def test_a_goal_beyond_reach_stalls_while_the_reference_brakes(self, whole_body_model) -> None:
        # With lag braking the reference never "arrives" at an unreachable
        # goal: it waits near the hand. The run must still end as a stall
        # within the budget, not run the budget out as "tracking" (which the
        # page would keep continuing for ever).
        model = whole_body_model
        start = bent(model)
        goal = left_target_ahead(model, start, 0.4)
        ik = make_ik(model, orientation_mode="position_only", gain_time_constant_s=GAIN_TAU)
        trajectory = run_trajectory(
            ik,
            start,
            {"left": goal},
            LIMITS,  # lag tolerance 2 cm: the reference brakes for the hand
            dt=0.02,
            max_duration_s=8.0,
            frames=FRAMES,
        )
        assert trajectory.status.is_stall, (trajectory.status, trajectory.message)
        assert not trajectory.truncated
        assert trajectory.duration_s < 8.0
        assert any(s.braking for s in trajectory.samples)

    def test_the_duration_budget_truncates_and_can_be_resumed(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        goal = left_target_ahead(model, start, 0.08)
        # With the torso fixed this arm cannot quite reach 8 cm ahead of the
        # bent pose (it straightens out and stalls 6 mm short); the torso
        # makes the goal reachable, as in the first test.
        ik = make_ik(model, orientation_mode="position_only", gain_time_constant_s=GAIN_TAU)
        first = run_trajectory(
            ik,
            start,
            {"left": goal},
            LIMITS,
            dt=0.02,
            max_duration_s=0.2,
            frames=FRAMES,
            torso_policy=TorsoPolicy.OPTIMIZE,
        )
        assert first.truncated and first.status is DualArmIKStatus.TRACKING
        assert "continue" in first.message
        v_end = first.references["left"].linear_velocity
        assert np.linalg.norm(v_end) > 0.1
        second = run_trajectory(
            ik,
            first.final_joints,
            {"left": goal},
            LIMITS,
            dt=0.02,
            max_duration_s=6.0,
            frames=FRAMES,
            torso_policy=TorsoPolicy.OPTIMIZE,
            references=first.references,
        )
        assert second.status is DualArmIKStatus.CONVERGED, second.message
        # Resumed from the reference's velocity, not from rest.
        assert (
            np.linalg.norm(
                second.samples[1].reference["left"][:3, 3]
                - second.samples[0].reference["left"][:3, 3]
            )
            / 0.02
            > 0.1
        )

    def test_a_lagging_hand_brakes_the_reference(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        goal = left_target_ahead(model, start, 0.10)
        # A slow solver (tiny joint speed) behind a fast reference.
        slow = make_ik(
            model,
            orientation_mode="position_only",
            gain_time_constant_s=GAIN_TAU,
            max_joint_velocity_rad_s=0.05,
        )
        fast = TrajectoryLimits(1.0, 5.0, 1.0, 4.0, lag_tolerance_m=0.01)
        trajectory = run_trajectory(
            slow, start, {"left": goal}, fast, dt=0.02, max_duration_s=1.0, frames=FRAMES
        )
        assert any(s.braking for s in trajectory.samples)
        # The reference never runs away: it stays within the tolerance plus
        # what one braking step covers.
        assert max(s.reference_error_m["left"] for s in trajectory.samples) < 0.03
        # Without a tolerance the same reference runs ahead unchecked.
        slow.reset()
        loose = TrajectoryLimits(1.0, 5.0, 1.0, 4.0, lag_tolerance_m=None)
        ahead = run_trajectory(
            slow, start, {"left": goal}, loose, dt=0.02, max_duration_s=1.0, frames=FRAMES
        )
        assert max(s.reference_error_m["left"] for s in ahead.samples) > 0.05

    def test_both_hands_and_the_json_view(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        goals = {
            "left": left_target_ahead(model, start, 0.03),
            "right": model.frame_pose(model.q_from_positions(start), "right_tcp").copy(),
        }
        goals["right"][2, 3] += 0.03
        ik = make_ik(model, orientation_mode="position_only", gain_time_constant_s=GAIN_TAU)
        trajectory = run_trajectory(
            ik,
            start,
            goals,
            LIMITS,
            dt=0.02,
            max_duration_s=6.0,
            frames=FRAMES,
            torso_policy=TorsoPolicy.OPTIMIZE,
            inactive_arm_policy=InactiveArmPolicy.HOLD_JOINTS,
        )
        assert trajectory.status is DualArmIKStatus.CONVERGED, trajectory.message
        view = trajectory.describe()
        assert view["status"] == "converged" and view["n_samples"] == len(trajectory.samples)
        sample = view["samples"][-1]
        assert set(sample["tcp"]) == {"left", "right"} and set(sample["reference"]) == {
            "left",
            "right",
        }
        assert sample["goal_error_m"]["right"] < 1e-3
        pose_at, velocity = trajectory.reference_at(0.1, "left")
        assert pose_at.shape == (4, 4) and velocity.shape == (3,)
        with pytest.raises(ValueError):
            run_trajectory(ik, start, {}, LIMITS, dt=0.02, max_duration_s=1.0, frames=FRAMES)
