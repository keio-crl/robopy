"""The joint-space coupling control law."""

from __future__ import annotations

import numpy as np
import pytest

from robopy.control.bilateral import (
    BilateralController,
    BilateralGains,
    UnvalidatedGravityProvider,
    coupling_stored_energy,
)

from .conftest import make_joint_state

JOINTS = ("a", "b")


class _FixedGravity:
    """A gravity provider that returns a constant torque, bound to one side."""

    def __init__(self, side: str, torque: float, *, validated: bool = True) -> None:
        self._side = side
        self._torque = torque
        self._validated = validated

    @property
    def side(self) -> str:
        return self._side

    @property
    def validated(self) -> bool:
        return self._validated

    def gravity_torque_nm(self, joint_names, positions_rad):  # type: ignore[no-untyped-def]
        del positions_rad
        return np.full(len(joint_names), self._torque)


def _controller(**gain_overrides: object) -> BilateralController:
    gains = BilateralGains(
        stiffness_nm_per_rad=2.0,
        damping_nm_s_per_rad=0.1,
        ramp_time_s=0.0,
        velocity_filter_hz=None,
        max_torque_nm=100.0,
        max_torque_rate_nm_s=1e9,
    )
    for key, value in gain_overrides.items():
        setattr(gains, key, value)
    controller = BilateralController(JOINTS, gains, allow_uncompensated=True)
    controller.engage()
    return controller


class TestControlLaw:
    def test_zero_error_and_zero_velocity_difference_gives_zero_coupling(self) -> None:
        controller = _controller()
        state = make_joint_state(JOINTS, {"a": 0.3, "b": -0.2}, {"a": 1.0, "b": -1.0})
        output = controller.compute(state, state, 0.01)
        np.testing.assert_allclose(output.coupling_torque_nm, 0.0, atol=1e-12)
        np.testing.assert_allclose(output.leader_torque_nm, 0.0, atol=1e-12)
        np.testing.assert_allclose(output.follower_torque_nm, 0.0, atol=1e-12)

    def test_a_held_follower_pushes_toward_the_leader_and_resists_the_operator(self) -> None:
        controller = _controller()
        leader = make_joint_state(JOINTS, {"a": 0.5})
        follower = make_joint_state(JOINTS, {"a": 0.0})
        output = controller.compute(leader, follower, 0.01)

        # e = q_s - S q_m - b = -0.5, so u = K e is negative.
        assert output.position_error_rad[0] == pytest.approx(-0.5)
        assert output.coupling_torque_nm[0] == pytest.approx(-1.0)
        # tau_s = g_s - u drives the follower toward the leader ...
        assert output.follower_torque_nm[0] == pytest.approx(1.0)
        # ... and tau_m = g_m + S^T u is the reaction felt by the operator.
        assert output.leader_torque_nm[0] == pytest.approx(-1.0)
        assert output.follower_torque_nm[0] == pytest.approx(-output.leader_torque_nm[0])

    def test_damping_acts_on_the_velocity_difference(self) -> None:
        controller = _controller()
        leader = make_joint_state(JOINTS, velocities={"a": 2.0})
        follower = make_joint_state(JOINTS, velocities={"a": 0.0})
        output = controller.compute(leader, follower, 0.01)
        assert output.velocity_error_rad_s[0] == pytest.approx(-2.0)
        assert output.coupling_torque_nm[0] == pytest.approx(-0.2)

    def test_scale_enters_position_velocity_and_the_transposed_torque(self) -> None:
        controller = _controller(scale=2.0)
        leader = make_joint_state(JOINTS, {"a": 0.25}, {"a": 1.0})
        follower = make_joint_state(JOINTS, {"a": 0.5}, {"a": 2.0})
        output = controller.compute(leader, follower, 0.01)
        # S = 2: the follower moves twice the leader's angle, so e and e_dot are zero.
        assert output.position_error_rad[0] == pytest.approx(0.0)
        assert output.velocity_error_rad_s[0] == pytest.approx(0.0)

        controller.reset()
        controller.engage()
        follower_off = make_joint_state(JOINTS, {"a": 0.6}, {"a": 2.0})
        output = controller.compute(leader, follower_off, 0.01)
        assert output.position_error_rad[0] == pytest.approx(0.1)
        # tau_m = S^T u, so the leader torque is scaled by S while the follower's is not.
        assert output.leader_torque_nm[0] == pytest.approx(2.0 * output.coupling_torque_nm[0])
        assert output.follower_torque_nm[0] == pytest.approx(-output.coupling_torque_nm[0])

    def test_offset_shifts_the_zero_error_pose(self) -> None:
        controller = _controller(offset_rad=0.3)
        leader = make_joint_state(JOINTS, {"a": 0.0})
        follower = make_joint_state(JOINTS, {"a": 0.3})
        output = controller.compute(leader, follower, 0.01)
        assert output.position_error_rad[0] == pytest.approx(0.0)

    def test_stored_energy_matches_the_ideal_expression(self) -> None:
        controller = _controller()
        leader = make_joint_state(JOINTS, {"a": 0.4, "b": -0.2})
        follower = make_joint_state(JOINTS)
        output = controller.compute(leader, follower, 0.01)
        expected = 0.5 * 2.0 * (0.4**2 + 0.2**2)
        assert controller.stored_energy_j(output) == pytest.approx(expected)
        assert coupling_stored_energy(np.array([0.4, -0.2]), np.array([2.0, 2.0])) == pytest.approx(
            expected
        )


class TestLimitsAndRamping:
    def test_torque_saturation_is_applied_and_reported(self) -> None:
        controller = _controller(max_torque_nm=0.5)
        leader = make_joint_state(JOINTS, {"a": 5.0})
        output = controller.compute(leader, make_joint_state(JOINTS), 0.01)
        assert abs(output.follower_torque_nm[0]) == pytest.approx(0.5)
        assert output.limits.torque_saturated["a"] is True
        assert output.limits.any_active

    def test_rate_limit_bounds_the_step_between_cycles(self) -> None:
        controller = _controller(max_torque_rate_nm_s=1.0)
        leader = make_joint_state(JOINTS, {"a": 1.0})
        follower = make_joint_state(JOINTS)
        first = controller.compute(leader, follower, 0.01)
        second = controller.compute(leader, follower, 0.01)

        # The unlimited command would be 2.0 Nm; at 1 Nm/s over 10 ms each
        # cycle may move 0.01 Nm, counting from zero on the very first one.
        assert first.follower_torque_nm[0] == pytest.approx(0.01)
        assert second.follower_torque_nm[0] == pytest.approx(0.02)
        assert first.limits.rate_limited["a"] is True
        assert second.limits.rate_limited["a"] is True

    def test_the_coupling_ramps_in_rather_than_stepping(self) -> None:
        controller = BilateralController(
            JOINTS,
            BilateralGains(
                stiffness_nm_per_rad=2.0,
                damping_nm_s_per_rad=0.0,
                ramp_time_s=1.0,
                velocity_filter_hz=None,
                max_torque_rate_nm_s=1e9,
            ),
            allow_uncompensated=True,
        )
        leader = make_joint_state(JOINTS, {"a": 0.5})
        follower = make_joint_state(JOINTS)
        assert controller.compute(leader, follower, 0.01).coupling_scale == 0.0

        controller.engage()
        scales = [controller.compute(leader, follower, 0.1).coupling_scale for _ in range(12)]
        assert scales[0] == pytest.approx(0.1)
        assert scales == sorted(scales)
        assert scales[-1] == pytest.approx(1.0)

    def test_reset_clears_the_filter_ramp_and_previous_command(self) -> None:
        controller = _controller(velocity_filter_hz=5.0, max_torque_rate_nm_s=1.0)
        controller.compute(make_joint_state(JOINTS, {"a": 1.0}), make_joint_state(JOINTS), 0.01)
        controller.reset()
        assert not controller.is_engaged
        assert controller.coupling_scale == 0.0
        controller.engage()
        # The remembered torque is back at zero, so the first cycle after the
        # reset starts from zero and is rate limited from there rather than
        # continuing from whatever the previous mode had commanded.
        output = controller.compute(
            make_joint_state(JOINTS, {"a": 1.0}), make_joint_state(JOINTS), 0.01
        )
        assert output.follower_torque_nm[0] == pytest.approx(0.01)
        assert output.limits.rate_limited["a"] is True


class TestGuards:
    def test_a_follower_gravity_model_is_refused_as_the_leader_model(self) -> None:
        with pytest.raises(ValueError, match="not transferable"):
            BilateralController(
                JOINTS,
                leader_gravity=_FixedGravity("follower", 0.1),
                follower_gravity=_FixedGravity("follower", 0.1),
            )

    def test_a_missing_gravity_model_must_be_opted_out_of_explicitly(self) -> None:
        with pytest.raises(ValueError, match="drop it under its own weight"):
            BilateralController(JOINTS)

    def test_an_unvalidated_model_is_refused_without_the_opt_out(self) -> None:
        with pytest.raises(ValueError, match="validated=False"):
            BilateralController(
                JOINTS,
                leader_gravity=_FixedGravity("leader", 0.0, validated=False),
                follower_gravity=_FixedGravity("follower", 0.0, validated=False),
            )

    def test_gravity_is_added_to_each_side_separately(self) -> None:
        controller = BilateralController(
            JOINTS,
            BilateralGains(
                stiffness_nm_per_rad=0.0,
                damping_nm_s_per_rad=0.0,
                ramp_time_s=0.0,
                velocity_filter_hz=None,
                max_torque_nm=10.0,
                max_torque_rate_nm_s=1e9,
            ),
            leader_gravity=_FixedGravity("leader", 0.1),
            follower_gravity=_FixedGravity("follower", 0.7),
        )
        controller.engage()
        state = make_joint_state(JOINTS)
        output = controller.compute(state, state, 0.01)
        np.testing.assert_allclose(output.leader_torque_nm, 0.1)
        np.testing.assert_allclose(output.follower_torque_nm, 0.7)

    def test_asymmetric_feedback_needs_an_explicit_opt_in(self) -> None:
        gains = BilateralGains(leader_feedback_scale=0.5)
        with pytest.raises(ValueError, match="energy balance"):
            BilateralController(JOINTS, gains, allow_uncompensated=True)
        BilateralController(
            JOINTS, gains, allow_uncompensated=True, allow_asymmetric_feedback=True
        )

    def test_an_invalid_joint_produces_no_torque(self) -> None:
        controller = _controller()
        follower = make_joint_state(JOINTS, valid={"a": False})
        with pytest.raises(ValueError, match="no torque is produced from a partial measurement"):
            controller.compute(make_joint_state(JOINTS), follower, 0.01)

    def test_a_missing_joint_produces_no_torque(self) -> None:
        controller = _controller()
        with pytest.raises(ValueError, match="does not cover coupled joint"):
            controller.compute(make_joint_state(("a",)), make_joint_state(JOINTS), 0.01)

    def test_non_positive_dt_is_refused(self) -> None:
        controller = _controller()
        with pytest.raises(ValueError, match="dt must be positive"):
            controller.compute(make_joint_state(JOINTS), make_joint_state(JOINTS), 0.0)

    def test_negative_gains_are_refused(self) -> None:
        with pytest.raises(ValueError, match="must be non-negative"):
            BilateralController(
                JOINTS, BilateralGains(stiffness_nm_per_rad=-1.0), allow_uncompensated=True
            )

    def test_duplicate_joint_names_are_refused(self) -> None:
        with pytest.raises(ValueError, match="must be unique"):
            BilateralController(("a", "a"), allow_uncompensated=True)


class TestRebaselining:
    def test_rebaselining_is_refused_while_engaged(self) -> None:
        controller = _controller()
        with pytest.raises(RuntimeError, match="Release the force feedback first"):
            controller.rebaseline_offset(make_joint_state(JOINTS), make_joint_state(JOINTS))

    def test_rebaselining_after_a_clutch_zeroes_the_error(self) -> None:
        controller = _controller()
        leader = make_joint_state(JOINTS, {"a": 0.4, "b": -0.1})
        follower = make_joint_state(JOINTS, {"a": 0.9, "b": 0.2})
        assert controller.alignment_error_rad(leader, follower)[0] == pytest.approx(0.5)

        controller.clutch()
        controller.rebaseline_offset(leader, follower)
        np.testing.assert_allclose(
            controller.alignment_error_rad(leader, follower), 0.0, atol=1e-12
        )

        controller.engage()
        output = controller.compute(leader, follower, 0.01)
        np.testing.assert_allclose(output.coupling_torque_nm, 0.0, atol=1e-12)


class TestUnvalidatedProvider:
    def test_it_reports_zero_torque_and_admits_it_is_unvalidated(self) -> None:
        provider = UnvalidatedGravityProvider(side="leader", justification="simulated plant")
        assert provider.validated is False
        np.testing.assert_allclose(provider.gravity_torque_nm(JOINTS, np.zeros(2)), 0.0)
