"""End-to-end runs of the whole stack against the simulated bus."""

from __future__ import annotations

import numpy as np
import pytest

from robopy.config.robot_config.rakuda_config import (
    RakudaBilateralConfig,
    RakudaControlConfig,
    RakudaJointCalibrationSpec,
)
from robopy.control.types import ControlMode, ServoState
from robopy.motor.dynamixel_control_table import OperatingMode
from robopy.motor.sim_dynamixel_bus import SimulatedJoint, SimulatedMotorFault
from robopy.robots.rakuda.rakuda_control import RakudaControlSystem

from .conftest import COUPLED_MOTORS, make_sim_bus


def _calibration(validated: bool = True) -> dict[str, RakudaJointCalibrationSpec]:
    return {
        name: RakudaJointCalibrationSpec(
            direction=1,
            zero_count=2048,
            lower_limit_rad=-2.0,
            upper_limit_rad=2.0,
            max_velocity_rad_s=3.0,
            torque_constant_nm_per_a=1.5,
            current_limit_a=1.0,
            validated=validated,
        )
        for name in COUPLED_MOTORS
    }


def _bilateral_config(**overrides: object) -> RakudaControlConfig:
    config = RakudaControlConfig(
        mode="bilateral_joint",
        control_period_s=0.002,
        # These tests drive `run_once` by hand as fast as the machine allows,
        # so the wall time between a read and its check is whatever the test
        # runner and the garbage collector leave; it is not the 2 ms period the
        # config names. The staleness guard itself is exercised directly in
        # test_servo_loop.py, so it is given room here rather than being
        # loosened in the library.
        max_state_age_s=5.0,
        max_cross_bus_skew_s=5.0,
        leader_joint_calibration=_calibration(),
        follower_joint_calibration=_calibration(),
        bilateral=RakudaBilateralConfig(
            coupled_motors=list(COUPLED_MOTORS),
            stiffness_nm_per_rad=2.0,
            damping_nm_s_per_rad=0.05,
            max_torque_nm=1.0,
            max_torque_rate_nm_s=50.0,
            ramp_time_s=0.05,
            velocity_filter_hz=50.0,
            leader_current_limit_a={name: 0.5 for name in COUPLED_MOTORS},
            follower_current_limit_a={name: 1.0 for name in COUPLED_MOTORS},
            allow_uncompensated=True,
        ),
        allow_hardware_current_output=True,
    )
    for key, value in overrides.items():
        setattr(config, key, value)
    return config


def _build(config: RakudaControlConfig, **buses: object) -> RakudaControlSystem:
    leader = buses.get("leader_bus") or make_sim_bus(model="xc330-t288")
    follower = buses.get("follower_bus") or make_sim_bus(model="xm430-w350")
    return RakudaControlSystem.from_buses(
        config,
        leader,  # type: ignore[arg-type]
        follower,  # type: ignore[arg-type]
        leader_torque_enabled=list(COUPLED_MOTORS),
        follower_torque_enabled=list(COUPLED_MOTORS),
    )


class TestBilateralEndToEnd:
    def test_the_follower_tracks_the_leader_through_the_whole_stack(self) -> None:
        leader_bus = make_sim_bus(model="xc330-t288")
        follower_bus = make_sim_bus(model="xm430-w350")
        system = _build(_bilateral_config(), leader_bus=leader_bus, follower_bus=follower_bus)
        system.configure()
        system.align()
        system.prepare_running()

        dt = 0.002
        for step in range(1500):
            # The operator moves the leader; the plants integrate in lock-step.
            leader_bus.joint("torso_yaw").position_rad = 0.3 * min(1.0, step * dt / 0.5)
            leader_bus.step(dt)
            follower_bus.step(dt)
            system.loop.run_once(dt)

        assert follower_bus.joint("torso_yaw").position_rad == pytest.approx(0.3, abs=0.01)
        assert system.manager.faults == ()

    def test_configuration_zeroes_goal_current_after_the_mode_change(self) -> None:
        follower_bus = make_sim_bus()
        system = _build(_bilateral_config(), follower_bus=follower_bus)
        system.configure()
        for name in COUPLED_MOTORS:
            registers = follower_bus.registers(name)
            assert registers.operating_mode == OperatingMode.CURRENT
            assert registers.goal_current_raw == 0
            assert registers.torque_enable == 1
        assert system.manager.state is ServoState.READY

    def test_current_output_is_gated_until_explicitly_allowed(self) -> None:
        config = _bilateral_config(allow_hardware_current_output=False)
        system = _build(config)
        with pytest.raises(PermissionError, match="allow_hardware_current_output is false"):
            system.configure()

    def test_an_unmeasured_calibration_blocks_configuration(self) -> None:
        config = _bilateral_config()
        config.follower_joint_calibration = _calibration(validated=False)
        system = _build(config)
        with pytest.raises(Exception, match="validation level 'hardware'"):
            system.configure()

    def test_a_large_misalignment_refuses_to_engage(self) -> None:
        leader_bus = make_sim_bus(model="xc330-t288")
        follower_bus = make_sim_bus()
        leader_bus.joint("torso_yaw").position_rad = 0.8
        system = _build(_bilateral_config(), leader_bus=leader_bus, follower_bus=follower_bus)
        system.configure()
        with pytest.raises(RuntimeError, match="out of alignment"):
            system.align()
        # It falls back to READY rather than sitting in a half-started state.
        assert system.manager.state is ServoState.READY

    def test_a_silent_motor_faults_the_cycle_and_stops_both_sides(self) -> None:
        leader_bus = make_sim_bus(model="xc330-t288")
        follower_bus = make_sim_bus()
        system = _build(_bilateral_config(), leader_bus=leader_bus, follower_bus=follower_bus)
        system.configure()
        system.align()
        system.prepare_running()
        system.loop.run_once(0.002)

        follower_bus.set_fault("l_arm_sh_pitch1", SimulatedMotorFault.SILENT)
        with pytest.raises(RuntimeError, match="no valid reading"):
            system.loop.run_once(0.002)

        system.manager.fault("cycle_failed", "silent motor")
        results = system.stop()
        assert len(results) == 2
        assert follower_bus.registers("torso_yaw").goal_current_raw == 0

    def test_contact_on_the_follower_is_felt_on_the_leader(self) -> None:
        leader_bus = make_sim_bus(model="xc330-t288")
        follower_bus = make_sim_bus()
        # The follower's torso meets a stiff obstacle at 0.1 rad.
        follower_bus.joint("torso_yaw").contact_position_rad = 0.1
        follower_bus.joint("torso_yaw").contact_stiffness_nm_per_rad = 50.0

        system = _build(_bilateral_config(), leader_bus=leader_bus, follower_bus=follower_bus)
        system.configure()
        system.align()
        system.prepare_running()

        dt = 0.002
        for step in range(1200):
            leader_bus.joint("torso_yaw").position_rad = 0.3 * min(1.0, step * dt / 0.4)
            leader_bus.step(dt)
            follower_bus.step(dt)
            system.loop.run_once(dt)

        # The follower is stopped by the obstacle, so the leader is commanded a
        # torque opposing the operator's motion.
        assert follower_bus.joint("torso_yaw").position_rad < 0.25
        leader_current = leader_bus.registers("torso_yaw").goal_current_raw
        assert leader_current < 0

    def test_the_report_states_configured_and_measured_timing_separately(self) -> None:
        system = _build(_bilateral_config())
        system.configure()
        report = system.report()
        assert report["timing"]["configured_rate_hz"] == pytest.approx(500.0)
        assert np.isnan(report["timing"]["measured_period_s"]["p50"])
        assert report["mode"] == "bilateral_joint"
        assert report["calibration_gaps"] == {"leader": {}, "follower": {}}


class TestModeAndExclusion:
    def test_a_command_cannot_be_issued_before_running(self) -> None:
        system = _build(_bilateral_config())
        system.configure()
        with pytest.raises(Exception, match="no command lease"):
            system.follower.command_torques_nm({"torso_yaw": 0.1})

    def test_stopping_releases_the_leases(self) -> None:
        system = _build(_bilateral_config())
        system.configure()
        system.align()
        system.prepare_running()
        assert system.manager.is_held("follower_command")
        system.stop()
        assert not system.manager.is_held("follower_command")

    def test_a_cartesian_target_is_refused_in_bilateral_mode(self) -> None:
        system = _build(_bilateral_config())
        with pytest.raises(RuntimeError, match="meaningless in bilateral_joint"):
            system.set_target(None)  # type: ignore[arg-type]

    def test_coupling_a_gripper_is_refused(self) -> None:
        motors = ("torso_yaw", "l_arm_grip")
        config = _bilateral_config()
        config.bilateral.coupled_motors = list(motors)
        config.leader_joint_calibration = {
            name: config.leader_joint_calibration["torso_yaw"] for name in motors
        }
        config.follower_joint_calibration = dict(config.leader_joint_calibration)
        leader = make_sim_bus(motors, model="xc330-t288")
        follower = make_sim_bus(motors)
        with pytest.raises(ValueError, match="separate hold/disable policy"):
            RakudaControlSystem.from_buses(
                config,
                leader,
                follower,
                leader_torque_enabled=list(motors),
                follower_torque_enabled=list(motors),
            )

    def test_a_coupled_joint_that_is_torque_off_is_refused(self) -> None:
        config = _bilateral_config()
        with pytest.raises(Exception, match="not adjusted automatically"):
            RakudaControlSystem.from_buses(
                config,
                make_sim_bus(model="xc330-t288"),
                make_sim_bus(),
                leader_torque_enabled=["torso_yaw"],
                follower_torque_enabled=list(COUPLED_MOTORS),
            )


class TestPositionTeleopMode:
    def test_the_follower_is_commanded_the_leader_positions(self) -> None:
        leader_bus = make_sim_bus()
        follower_bus = make_sim_bus()
        config = RakudaControlConfig(
            mode="position_teleop",
            max_state_age_s=5.0,
            max_cross_bus_skew_s=5.0,
            leader_joint_calibration=_calibration(),
            follower_joint_calibration=_calibration(),
        )
        system = RakudaControlSystem.from_buses(
            config, leader_bus, follower_bus, leader_torque_enabled=list(COUPLED_MOTORS)
        )
        assert system.mode is ControlMode.POSITION_TELEOP
        system.configure()
        system.align()
        system.prepare_running()

        leader_bus.joint("torso_yaw").position_rad = 0.4
        system.loop.run_once(0.005)

        registers = follower_bus.registers("torso_yaw")
        expected = follower_bus._rad_to_count(registers, 0.4)  # noqa: SLF001 - introspection
        assert abs(registers.goal_position_count - expected) <= 1

    def test_position_mode_does_not_require_torque_calibration(self) -> None:
        config = RakudaControlConfig(
            mode="position_teleop",
            leader_joint_calibration={
                name: RakudaJointCalibrationSpec(
                    zero_count=2048, lower_limit_rad=-2.0, upper_limit_rad=2.0
                )
                for name in COUPLED_MOTORS
            },
            follower_joint_calibration={
                name: RakudaJointCalibrationSpec(
                    zero_count=2048, lower_limit_rad=-2.0, upper_limit_rad=2.0
                )
                for name in COUPLED_MOTORS
            },
        )
        system = RakudaControlSystem.from_buses(config, make_sim_bus(), make_sim_bus())
        system.configure()
        assert system.manager.state is ServoState.READY


class TestSimulatedContactStability:
    """Gain ranges checked against the simulated plant.

    This records what was actually verified against *this* plant, with its
    friction, contact and one-cycle command delay.  It is not a stability proof
    for the machine.
    """

    @pytest.mark.parametrize("stiffness", [0.5, 1.0, 2.0, 4.0])
    def test_free_motion_does_not_diverge(self, stiffness: float) -> None:
        leader_bus = make_sim_bus(model="xc330-t288")
        follower_bus = make_sim_bus()
        config = _bilateral_config()
        config.bilateral.stiffness_nm_per_rad = stiffness
        config.bilateral.damping_nm_s_per_rad = 0.05
        system = _build(config, leader_bus=leader_bus, follower_bus=follower_bus)
        system.configure()
        system.align()
        system.prepare_running()

        dt = 0.002
        worst = 0.0
        for _ in range(2000):
            leader_bus.step(dt)
            follower_bus.step(dt)
            system.loop.run_once(dt)
            worst = max(worst, abs(follower_bus.joint("torso_yaw").velocity_rad_s))
        assert worst < 1.0
        assert np.isfinite(follower_bus.joint("torso_yaw").position_rad)

    def test_a_stiff_contact_settles_instead_of_oscillating(self) -> None:
        leader_bus = make_sim_bus(model="xc330-t288")
        follower_bus = make_sim_bus()
        follower_bus.joint("torso_yaw").contact_position_rad = 0.05
        follower_bus.joint("torso_yaw").contact_stiffness_nm_per_rad = 200.0

        system = _build(_bilateral_config(), leader_bus=leader_bus, follower_bus=follower_bus)
        system.configure()
        system.align()
        system.prepare_running()

        dt = 0.002
        leader_bus.joint("torso_yaw").position_rad = 0.2
        velocities = []
        for _ in range(2000):
            leader_bus.step(dt)
            follower_bus.step(dt)
            system.loop.run_once(dt)
            velocities.append(abs(follower_bus.joint("torso_yaw").velocity_rad_s))

        assert max(velocities[-200:]) < max(velocities[:200]) + 1e-9
        assert follower_bus.joint("torso_yaw").position_rad < 0.12

    def test_a_heavier_plant_still_saturates_rather_than_diverging(self) -> None:
        leader_bus = make_sim_bus(model="xc330-t288")
        follower_bus = make_sim_bus()
        follower_bus._registers["torso_yaw"].joint = SimulatedJoint(  # noqa: SLF001
            inertia_kg_m2=2e-2, damping_nm_s=1e-3, coulomb_friction_nm=5e-3
        )
        config = _bilateral_config()
        config.bilateral.stiffness_nm_per_rad = 8.0
        system = _build(config, leader_bus=leader_bus, follower_bus=follower_bus)
        system.configure()
        system.align()
        system.prepare_running()

        dt = 0.002
        for _ in range(2000):
            # The operator holds the leader at a fixed pose, so the reaction
            # torque does not simply push the (unloaded) leader arm away.
            leader_bus.joint("torso_yaw").position_rad = 0.5
            leader_bus.joint("torso_yaw").velocity_rad_s = 0.0
            follower_bus.step(dt)
            system.loop.run_once(dt)
            # The torque ceiling holds regardless of the gain.
            raw = follower_bus.registers("torso_yaw").goal_current_raw
            assert abs(raw * 0.00269) <= 1.0 + 1e-9
        assert np.isfinite(follower_bus.joint("torso_yaw").position_rad)
        # A heavy joint under a high gain lags, but it converges rather than
        # oscillating away.
        assert follower_bus.joint("torso_yaw").position_rad == pytest.approx(0.5, abs=0.05)
