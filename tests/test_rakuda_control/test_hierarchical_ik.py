"""Task priority, secondary objectives and the step's time base in the dual-arm solver.

Skipped without the ``kinematics`` extra.  Everything runs on the synthetic
fixture: the numbers say the software behaves as specified, nothing about the
machine.
"""

from __future__ import annotations

import math
from typing import Any, Dict, List

import numpy as np
import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.control.types import DualArmTarget, TorsoPolicy  # noqa: E402
from robopy.kinematics.dual_arm_ik import (  # noqa: E402
    DualArmIK,
    DualArmIKConfig,
    DualArmIKStatus,
)
from robopy.kinematics.synthetic_dual_arm import (  # noqa: E402
    SYNTHETIC_ARM_JOINTS,
    SYNTHETIC_HEAD_JOINTS,
    SYNTHETIC_TORSO_JOINT,
)
from robopy.kinematics.urdf_model import WholeBodyModel  # noqa: E402

from .test_dual_arm_ik import DT, _run, _state, _zero  # noqa: E402

#: Error-correction time constant used where a target several centimetres away
#: is approached under the default velocity and acceleration bounds.  A
#: dead-beat gain (alpha = 1) drives every joint into velocity saturation on the
#: first step, and an acceleration bound then keeps it moving past the target:
#: the loop overshoots and can diverge, in the weighted mode as much as in the
#: hierarchical one (see test_a_dead_beat_gain_overshoots_under_an_acceleration_bound).
#: The streaming profiles give the solver a reference trajectory and a time
#: constant instead; these tests use the time constant.
GAIN_TAU = 0.05


def make_ik(model: WholeBodyModel, **overrides) -> DualArmIK:  # type: ignore[no-untyped-def]
    settings: Dict[str, Any] = dict(
        max_joint_step_rad=0.08, compute_budget_s=5.0, max_state_age_s=5.0
    )
    settings.update(overrides)
    return DualArmIK(
        model,
        left_frame="left_tcp",
        right_frame="right_tcp",
        torso_joint=SYNTHETIC_TORSO_JOINT,
        left_arm_joints=SYNTHETIC_ARM_JOINTS["left"],
        right_arm_joints=SYNTHETIC_ARM_JOINTS["right"],
        head_joints=SYNTHETIC_HEAD_JOINTS,
        config=DualArmIKConfig(**settings),
    )


def bent(model: WholeBodyModel) -> Dict[str, float]:
    start = _zero(model)
    start.update({"elbow_pitch_left_dof": -0.6, "elbow_pitch_right_dof": -0.6})
    return start


def left_target_ahead(model: WholeBodyModel, start: Dict[str, float], dx: float) -> np.ndarray:
    T = model.frame_pose(model.q_from_positions(start), "left_tcp").copy()
    T[0, 3] += dx
    return T


class TestPriority:
    def test_a_heavy_posture_weight_costs_accuracy_only_in_the_weighted_mode(
        self, whole_body_model
    ) -> None:
        # A posture reference that disagrees with the target: the weighted sum
        # settles on a compromise between the two, the hierarchy does not.
        model = whole_body_model
        start = bent(model)
        target = DualArmTarget(
            left_target=left_target_ahead(model, start, 0.05),
            right_enabled=False,
            torso_policy=TorsoPolicy.OPTIMIZE,
        )
        posture = dict(start, elbow_pitch_left_dof=-1.4, shoulder_roll_left_dof=0.6)
        results = {}
        for mode in ("weighted", "hierarchical"):
            ik = make_ik(
                model,
                task_priority_mode=mode,
                orientation_mode="position_only",
                posture_cost=3.0,
                posture_reference=posture,
            )
            _, result = _run(ik, model, target, steps=400, start=start)
            assert result.is_commandable, result.message
            results[mode] = result
        assert results["hierarchical"].status is DualArmIKStatus.CONVERGED
        assert results["hierarchical"].left_position_error_m < 1e-3
        assert (
            results["weighted"].left_position_error_m
            > 5 * results["hierarchical"].left_position_error_m
        )

    def test_the_second_stage_never_worsens_the_first(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        target = DualArmTarget(
            left_target=left_target_ahead(model, start, 0.06),
            right_enabled=False,
            torso_policy=TorsoPolicy.OPTIMIZE,
        )
        ik = make_ik(
            model,
            task_priority_mode="hierarchical",
            orientation_mode="position_only",
            posture_cost=2.0,
            joint_motion_cost={SYNTHETIC_TORSO_JOINT: 3.0},
            velocity_smoothing_cost=0.5,
            limit_avoidance_enabled=True,
            gain_time_constant_s=GAIN_TAU,
        )
        current = dict(start)
        seen = 0
        for _ in range(60):
            result = ik.solve_step(_state(model, current), target, DT)
            assert result.is_commandable, result.message
            assert result.stage1_residual is not None and result.task_residual_after is not None
            assert result.task_residual_after <= result.stage1_residual + 1e-6
            assert not result.hierarchical_fallback
            seen += 1
            current.update(result.joint_targets_rad)
            if result.status is DualArmIKStatus.CONVERGED:
                break
        assert seen > 1

    def test_secondary_preferences_choose_among_equally_good_solutions(
        self, whole_body_model
    ) -> None:
        # One hand, torso free: seven joints for three position rows. The
        # preferred posture decides where the redundancy goes, and the hand
        # still arrives.
        model = whole_body_model
        start = bent(model)
        target = DualArmTarget(
            left_target=left_target_ahead(model, start, 0.04),
            right_enabled=False,
            torso_policy=TorsoPolicy.OPTIMIZE,
        )
        finals = {}
        for name, reference in (
            ("neutral", start),
            ("rolled", dict(start, shoulder_roll_left_dof=0.5)),
        ):
            ik = make_ik(
                model,
                task_priority_mode="hierarchical",
                orientation_mode="position_only",
                posture_cost=1.0,
                posture_reference=reference,
                gain_time_constant_s=GAIN_TAU,
            )
            final, result = _run(ik, model, target, steps=400, start=start)
            assert result.status is DualArmIKStatus.CONVERGED, result.message
            finals[name] = final
        assert (
            finals["rolled"]["shoulder_roll_left_dof"]
            > finals["neutral"]["shoulder_roll_left_dof"] + 0.05
        )

    def test_a_torso_motion_cost_makes_the_arm_do_the_work(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        target = DualArmTarget(
            left_target=left_target_ahead(model, start, 0.03),
            right_enabled=False,
            torso_policy=TorsoPolicy.OPTIMIZE,
        )
        torso_use = {}
        for name, cost in (("free", None), ("expensive", {SYNTHETIC_TORSO_JOINT: 20.0})):
            # Bounds wide enough that the joints never saturate: the cost can
            # only redistribute motion the bounds leave free.
            ik = make_ik(
                model,
                task_priority_mode="hierarchical",
                orientation_mode="position_only",
                joint_motion_cost=cost,
                posture_cost=0.0,
                torso_regularisation=0.0,
                gain_time_constant_s=GAIN_TAU,
                max_joint_velocity_rad_s=100.0,
                max_joint_step_rad=1.0,
                max_joint_acceleration_rad_s2=None,
            )
            final, result = _run(ik, model, target, steps=300, start=start)
            assert result.status is DualArmIKStatus.CONVERGED, result.message
            torso_use[name] = abs(final[SYNTHETIC_TORSO_JOINT] - start[SYNTHETIC_TORSO_JOINT])
        assert torso_use["expensive"] < 0.5 * torso_use["free"]

    def test_limit_avoidance_guides_a_joint_in_the_band_and_nothing_else(
        self, whole_body_model
    ) -> None:
        model = whole_body_model
        # Park the left wrist yaw 5 cm... 0.05 rad inside its +/-2.8 limit and
        # give the hand nothing to do (its own pose as the target).
        start = bent(model)
        start["wrist_yaw_left_dof"] = 2.8 - 0.05
        hold = model.frame_pose(model.q_from_positions(start), "left_tcp")
        target = DualArmTarget(left_target=hold, right_enabled=False)
        ik = make_ik(
            model,
            task_priority_mode="hierarchical",
            orientation_mode="position_only",
            posture_cost=0.0,
            limit_avoidance_enabled=True,
            limit_avoidance_band_rad=0.2,
            limit_avoidance_cost=1.0,
        )
        current = dict(start)
        for _ in range(40):
            result = ik.solve_step(_state(model, current), target, DT)
            assert result.is_commandable, result.message
            current.update(result.joint_targets_rad)
        assert current["wrist_yaw_left_dof"] < start["wrist_yaw_left_dof"] - 0.02
        assert current["wrist_yaw_left_dof"] > 2.8 - 0.2 - 0.05  # eased out, not flung
        # A joint nowhere near a limit is left alone: no pull to the centre.
        assert current["shoulder_roll_left_dof"] == pytest.approx(
            start["shoulder_roll_left_dof"], abs=1e-3
        )
        assert result.left_position_error_m is not None and result.left_position_error_m < 2e-3
        assert min(result.limit_margin_rad.values()) > 0.0


class TestTimeBase:
    def test_a_dead_beat_gain_overshoots_under_an_acceleration_bound(
        self, whole_body_model
    ) -> None:
        # The reason the streaming profiles use a time constant (or a
        # reference trajectory): with alpha = 1 the first step saturates the
        # velocity bound and the acceleration bound then forbids stopping in
        # time, whatever the priority mode.  With a time constant the same
        # target converges.  Recorded so the behaviour is a known property,
        # not a surprise.
        model = whole_body_model
        start = bent(model)
        target = DualArmTarget(
            left_target=left_target_ahead(model, start, 0.03),
            right_enabled=False,
            torso_policy=TorsoPolicy.OPTIMIZE,
        )
        outcomes = {}
        for label, tau in (("dead_beat", None), ("time_constant", GAIN_TAU)):
            ik = make_ik(
                model,
                orientation_mode="position_only",
                posture_cost=0.0,
                torso_regularisation=0.0,
                gain_time_constant_s=tau,
            )
            _, result = _run(ik, model, target, steps=400, start=start)
            outcomes[label] = result
        assert outcomes["time_constant"].status is DualArmIKStatus.CONVERGED
        assert outcomes["dead_beat"].status is not DualArmIKStatus.CONVERGED

    def test_the_acceleration_bound_uses_the_previous_period(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        far = left_target_ahead(model, start, 0.5)
        target = DualArmTarget(left_target=far, right_enabled=False)
        a_max = 8.0
        ik = make_ik(model, orientation_mode="position_only", max_joint_acceleration_rad_s2=a_max)
        current = dict(start)
        previous: Dict[str, float] | None = None
        periods = [0.01, 0.02, 0.005, 0.03, 0.01, 0.02]
        for dt in periods:
            result = ik.solve_step(_state(model, current), target, dt)
            assert result.is_commandable, result.message
            if previous is not None:
                for name, v in result.joint_velocities_rad_s.items():
                    assert abs(v - previous[name]) <= a_max * dt + 1e-6, (name, dt)
            previous = dict(result.joint_velocities_rad_s)
            current.update(result.joint_targets_rad)

    def test_a_gain_time_constant_makes_the_correction_period_independent(
        self, whole_body_model
    ) -> None:
        model = whole_body_model
        start = bent(model)
        target = DualArmTarget(
            left_target=left_target_ahead(model, start, 0.02), right_enabled=False
        )
        tau = 0.1
        moved = {}
        for dt in (0.01, 0.02):
            ik = make_ik(
                model,
                orientation_mode="position_only",
                gain_time_constant_s=tau,
                max_joint_velocity_rad_s=100.0,
                max_joint_step_rad=1.0,
            )
            before = model.frame_pose(model.q_from_positions(start), "left_tcp")[:3, 3]
            result = ik.solve_step(_state(model, start), target, dt)
            after = model.frame_pose(
                model.q_from_positions(start | result.joint_targets_rad), "left_tcp"
            )[:3, 3]
            moved[dt] = float(np.linalg.norm(after - before))
        expected = (1 - math.exp(-0.02 / tau)) / (1 - math.exp(-0.01 / tau))
        assert moved[0.02] / moved[0.01] == pytest.approx(expected, rel=0.1)

    def test_singular_values_are_normalised_and_damping_ramps(self, whole_body_model) -> None:
        model = whole_body_model
        straight = _zero(model)  # the arm hangs fully extended: singular
        folded = bent(model)
        target_of = lambda start: DualArmTarget(  # noqa: E731
            left_target=left_target_ahead(model, start, 0.03), right_enabled=False
        )
        fast = dict(max_joint_velocity_rad_s=100.0, max_joint_step_rad=1.0)
        plain = make_ik(model, orientation_mode="position_only", **fast)
        sigma_straight = plain.solve_step(_state(model, straight), target_of(straight), DT)
        plain.reset()
        sigma_folded = plain.solve_step(_state(model, folded), target_of(folded), DT)
        assert sigma_straight.min_singular_value is not None
        assert sigma_folded.min_singular_value is not None
        assert sigma_straight.min_singular_value < 0.2 * sigma_folded.min_singular_value
        # With the ramp on, the step near the singularity is smaller, and the
        # damping does nothing where the arm is well conditioned.
        # Fully extended the arm is exactly singular (sigma_min ~ 0), so the
        # threshold is set from the well-conditioned pose instead.
        threshold = 0.5 * sigma_folded.min_singular_value
        damped = make_ik(
            model,
            orientation_mode="position_only",
            singularity_sigma_min=threshold,
            singularity_damping=1.0,
            **fast,
        )
        step_damped = damped.solve_step(_state(model, straight), target_of(straight), DT)
        norm = lambda r: float(  # noqa: E731
            np.linalg.norm(list(r.joint_velocities_rad_s.values()))
        )
        assert norm(step_damped) < norm(sigma_straight)
        damped.reset()
        step_folded = damped.solve_step(_state(model, folded), target_of(folded), DT)
        assert norm(step_folded) == pytest.approx(norm(sigma_folded), rel=1e-6)

    def test_a_continuous_joint_keeps_its_turn(self, synthetic_urdf) -> None:
        model = WholeBodyModel.from_urdf(synthetic_urdf)
        for side in ("left", "right"):
            model.add_fixed_frame(f"{side}_tcp", f"gripper_{side}_dof", np.eye(4))
        model.set_soft_limits(
            {
                SYNTHETIC_TORSO_JOINT: (-4.0, 4.0),
                "shoulder_pitch_left_dof": (-2.0, 2.0),
                "shoulder_pitch_right_dof": (-2.0, 2.0),
            }
        )
        ik = make_ik(model, orientation_mode="position_only")
        start = _zero(model)
        start[SYNTHETIC_TORSO_JOINT] = math.pi - 0.02  # decodes near the +pi seam
        # Ask the left hand to move so the torso turns further positive.
        q = model.q_from_positions(start | {SYNTHETIC_TORSO_JOINT: math.pi + 0.3})
        goal = model.frame_pose(q, "left_tcp")
        target = DualArmTarget(
            left_target=goal, right_enabled=False, torso_policy=TorsoPolicy.OPTIMIZE
        )
        current = dict(start)
        history: List[float] = []
        for _ in range(60):
            result = ik.solve_step(_state(model, current), target, DT)
            assert result.is_commandable, result.message
            current.update(result.joint_targets_rad)
            history.append(current[SYNTHETIC_TORSO_JOINT])
        assert max(abs(b - a) for a, b in zip(history, history[1:])) < 0.1  # no 2pi jump
        assert history[-1] > math.pi  # it crossed the seam and stayed on the same turn


class TestConfiguration:
    def test_invalid_settings_are_refused(self) -> None:
        with pytest.raises(ValueError, match="task_priority_mode"):
            DualArmIKConfig(task_priority_mode="lexicographic")
        with pytest.raises(ValueError, match="orientation_mode"):
            DualArmIKConfig(orientation_mode="euler")
        with pytest.raises(ValueError, match="approach_axis_tcp"):
            DualArmIKConfig(orientation_mode="axis_aligned")
        with pytest.raises(ValueError, match="non-zero"):
            DualArmIKConfig(approach_axis_tcp=(0.0, 0.0, 0.0))
        with pytest.raises(ValueError, match="gain_time_constant_s"):
            DualArmIKConfig(gain_time_constant_s=0.0)
        with pytest.raises(ValueError, match="limit_avoidance"):
            DualArmIKConfig(limit_avoidance_band_rad=0.0)

    def test_secondary_weights_name_only_active_joints(self, whole_body_model) -> None:
        with pytest.raises(ValueError, match="does not drive"):
            make_ik(whole_body_model, joint_motion_cost={"head_yaw_dof": 1.0})
        with pytest.raises(ValueError, match="non-negative"):
            make_ik(whole_body_model, posture_cost=-1.0)
        ik = make_ik(whole_body_model, posture_cost={"elbow_pitch_left_dof": 0.5})
        assert ik.config.posture_cost == {"elbow_pitch_left_dof": 0.5}

    def test_results_carry_the_new_diagnostics(self, whole_body_model) -> None:
        model = whole_body_model
        start = bent(model)
        ik = make_ik(model, orientation_mode="position_only", task_priority_mode="hierarchical")
        target = DualArmTarget(
            left_target=left_target_ahead(model, start, 0.02), right_enabled=False
        )
        result = ik.solve_step(_state(model, start), target, DT)
        assert result.is_commandable
        assert result.dt_s == DT and result.orientation_mode == "position_only"
        assert result.task_residual_before is not None and result.task_residual_after is not None
        assert result.task_residual_after < result.task_residual_before
        assert set(result.limit_margin_rad) == set(ik.active_joints)
        errors = result.errors()
        assert errors["left_position_m"] == result.left_position_error_m
        assert errors["left_axis_rad"] is None
        # Orientation is still reported in position_only mode, for display.
        assert result.left_orientation_error_rad is not None
