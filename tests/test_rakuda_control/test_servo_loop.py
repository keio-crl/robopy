"""The servo: configuration order, command exclusion, faults and stopping."""

from __future__ import annotations

import math

import numpy as np
import pytest

from robopy.control.joint_mapping import JointMapError
from robopy.control.mode_manager import ModeManager, ModeTransitionError
from robopy.control.servo_loop import ArmServo, ServoLoop, ServoLoopConfig, StopPolicy, TimingStats
from robopy.control.types import ControlMode, ServoState, monotonic_ns
from robopy.motor.dynamixel_control_table import OperatingMode, XControlTable
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedMotorFault

from .conftest import COUPLED_MOTORS, make_calibration, make_sim_bus


def _servo(
    bus: SimulatedDynamixelBus,
    manager: ModeManager,
    *,
    validated: bool = True,
    config: ServoLoopConfig | None = None,
) -> ArmServo:
    return ArmServo(
        "follower",
        bus,
        make_calibration(COUPLED_MOTORS, validated=validated),
        manager,
        config or ServoLoopConfig(),
    )


def _running(manager: ModeManager) -> None:
    manager.transition(ServoState.CONFIGURING)
    manager.transition(ServoState.READY)
    manager.transition(ServoState.ALIGNING)
    manager.transition(ServoState.RUNNING)


class TestStateReading:
    def test_raw_counts_become_si_units(self, sim_bus: SimulatedDynamixelBus) -> None:
        sim_bus.joint("torso_yaw").position_rad = 0.5
        sim_bus.joint("torso_yaw").velocity_rad_s = -0.25
        servo = _servo(sim_bus, ModeManager())
        state = servo.read_state()

        index = state.index("torso_yaw")
        assert state.position_rad[index] == pytest.approx(0.5, abs=2 * math.pi / 4096)
        assert state.velocity_rad_s[index] == pytest.approx(-0.25, abs=0.03)
        assert state.all_valid
        assert state.acquisition_span_s >= 0.0

    def test_a_silent_motor_is_marked_invalid_rather_than_zero_filled(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        sim_bus.joint("r_arm_sh_pitch1").position_rad = 0.4
        sim_bus.set_fault("r_arm_sh_pitch1", SimulatedMotorFault.SILENT)
        servo = _servo(sim_bus, ModeManager())
        state = servo.read_state()

        assert not state.all_valid
        assert state.valid[state.index("torso_yaw")]
        assert not state.valid[state.index("r_arm_sh_pitch1")]
        assert "no valid reading" in "; ".join(servo.check_state(state))

    def test_the_snapshot_is_cached_for_other_readers(self, sim_bus: SimulatedDynamixelBus) -> None:
        servo = _servo(sim_bus, ModeManager())
        assert servo.latest_state() is None
        state = servo.read_state()
        before = sim_bus.transaction_count
        assert servo.latest_state() is state
        assert sim_bus.transaction_count == before

    def test_an_out_of_range_position_is_reported(self, sim_bus: SimulatedDynamixelBus) -> None:
        sim_bus.joint("torso_yaw").position_rad = 3.0  # calibrated limit is 2.0 rad
        servo = _servo(sim_bus, ModeManager())
        problems = servo.check_state(servo.read_state())
        assert any("outside its calibrated range" in p for p in problems)

    def test_a_stale_snapshot_is_reported(self, sim_bus: SimulatedDynamixelBus) -> None:
        servo = _servo(sim_bus, ModeManager(), config=ServoLoopConfig(max_state_age_s=0.0))
        problems = servo.check_state(servo.read_state())
        assert any("old, over the" in p for p in problems)


class TestDiagnostics:
    def test_unread_diagnostics_are_unknown_not_healthy(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        servo = _servo(sim_bus, ModeManager())
        assert servo.check_diagnostics() == ["follower: no diagnostics have been read yet."]

    @pytest.mark.parametrize(
        ("fault", "expected"),
        [
            (SimulatedMotorFault.OVERHEAT, "exceeds"),
            (SimulatedMotorFault.UNDERVOLTAGE, "outside"),
            (SimulatedMotorFault.HARDWARE_ERROR, "HARDWARE_ERROR_STATUS"),
        ],
    )
    def test_each_diagnostic_fault_is_detected(
        self, sim_bus: SimulatedDynamixelBus, fault: SimulatedMotorFault, expected: str
    ) -> None:
        sim_bus.set_fault("torso_yaw", fault)
        servo = _servo(sim_bus, ModeManager())
        servo.poll_diagnostics(force=True)
        assert any(expected in p for p in servo.check_diagnostics())

    def test_a_healthy_bus_reports_nothing(self, sim_bus: SimulatedDynamixelBus) -> None:
        servo = _servo(sim_bus, ModeManager())
        servo.poll_diagnostics(force=True)
        assert servo.check_diagnostics() == []


class TestConfiguration:
    def test_current_mode_rewrites_goal_current_to_zero_before_torque_on(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        servo = _servo(sim_bus, ModeManager())
        servo.configure_for_mode(
            ControlMode.BILATERAL_JOINT,
            current_limits_a={name: 0.5 for name in COUPLED_MOTORS},
        )
        for name in COUPLED_MOTORS:
            registers = sim_bus.registers(name)
            assert registers.operating_mode == OperatingMode.CURRENT
            # The simulated motor resets GOAL_CURRENT to the limit on a mode
            # change, exactly as a real one does; it must have been zeroed again.
            assert registers.goal_current_raw == 0
            assert registers.torque_enable == 1

    def test_position_mode_sets_the_goal_to_the_measured_pose(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        sim_bus.joint("torso_yaw").position_rad = 0.3
        servo = _servo(sim_bus, ModeManager())
        servo.configure_for_mode(ControlMode.POSITION_TELEOP)
        registers = sim_bus.registers("torso_yaw")
        assert registers.operating_mode == OperatingMode.POSITION
        measured = sim_bus._rad_to_count(registers, 0.3)  # noqa: SLF001 - test introspection
        assert abs(registers.goal_position_count - measured) <= 1

    def test_current_mode_needs_a_complete_hardware_calibration(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        servo = _servo(sim_bus, ModeManager(), validated=False)
        with pytest.raises(JointMapError, match="validation level 'hardware'"):
            servo.configure_for_mode(
                ControlMode.BILATERAL_JOINT, current_limits_a={"torso_yaw": 0.1}
            )

    def test_current_mode_needs_an_explicit_current_limit(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        servo = _servo(sim_bus, ModeManager())
        with pytest.raises(ValueError, match="explicit per-motor current limit"):
            servo.configure_for_mode(ControlMode.BILATERAL_JOINT)

    def test_a_current_limit_above_the_calibrated_ceiling_is_refused(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        servo = _servo(sim_bus, ModeManager())
        with pytest.raises(ValueError, match="exceeds the calibrated ceiling"):
            servo.configure_for_mode(
                ControlMode.BILATERAL_JOINT, current_limits_a={"torso_yaw": 99.0}
            )

    def test_a_failed_configuration_leaves_no_motor_torque_enabled(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        servo = _servo(sim_bus, ModeManager())
        with pytest.raises(ValueError):
            servo.configure_for_mode(
                ControlMode.BILATERAL_JOINT, current_limits_a={"torso_yaw": 99.0}
            )
        assert all(sim_bus.registers(n).torque_enable == 0 for n in COUPLED_MOTORS)

    def test_the_watchdog_is_written_when_configured(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        servo = _servo(sim_bus, ModeManager(), config=ServoLoopConfig(bus_watchdog_counts=10))
        servo.configure_for_mode(ControlMode.POSITION_TELEOP)
        assert sim_bus.registers("torso_yaw").bus_watchdog == 10


class TestCommanding:
    def test_commanding_without_a_lease_is_refused(self, sim_bus: SimulatedDynamixelBus) -> None:
        servo = _servo(sim_bus, ModeManager())
        with pytest.raises(ModeTransitionError, match="no command lease"):
            servo.command_positions_rad({"torso_yaw": 0.1})

    def test_a_stale_command_is_dropped(self, sim_bus: SimulatedDynamixelBus) -> None:
        manager = ModeManager()
        _running(manager)
        servo = _servo(sim_bus, manager, config=ServoLoopConfig(max_command_age_s=0.001))
        servo.acquire_command_path()
        with pytest.raises(ModeTransitionError, match="old, over the"):
            servo.command_positions_rad(
                {"torso_yaw": 0.1}, issued_ns=monotonic_ns() - 500_000_000
            )

    def test_a_command_from_an_old_generation_is_dropped(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        manager = ModeManager()
        _running(manager)
        servo = _servo(sim_bus, manager)
        lease = servo.acquire_command_path()
        with pytest.raises(ModeTransitionError, match="computed in generation"):
            servo.command_positions_rad({"torso_yaw": 0.1}, generation=lease.generation - 1)

    def test_a_non_finite_command_is_refused(self, sim_bus: SimulatedDynamixelBus) -> None:
        manager = ModeManager()
        _running(manager)
        servo = _servo(sim_bus, manager)
        servo.acquire_command_path()
        with pytest.raises(ValueError, match="non-finite"):
            servo.command_positions_rad({"torso_yaw": float("nan")})

    def test_torque_commands_use_the_measured_constant_and_the_ceiling(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        manager = ModeManager()
        _running(manager)
        servo = _servo(sim_bus, manager)
        servo.acquire_command_path()
        # torque_constant = 1.5 Nm/A, current_limit = 1.0 A.
        currents = servo.command_torques_nm({"torso_yaw": 0.75, "l_arm_sh_pitch1": -9.0})
        assert currents["torso_yaw"] == pytest.approx(0.5)
        assert currents["l_arm_sh_pitch1"] == pytest.approx(-1.0)

    def test_a_joint_without_a_torque_constant_cannot_be_driven(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        manager = ModeManager()
        _running(manager)
        joint_map = make_calibration(COUPLED_MOTORS).with_updates(
            {"torso_yaw": {"torque_constant_nm_per_a": None}}
        )
        servo = ArmServo("follower", sim_bus, joint_map, manager)
        servo.acquire_command_path()
        with pytest.raises(JointMapError, match="not calibrated"):
            servo.command_torques_nm({"torso_yaw": 0.1})

    def test_two_command_paths_on_one_bus_cannot_coexist(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        manager = ModeManager()
        _running(manager)
        servo = _servo(sim_bus, manager)
        servo.acquire_command_path()
        with pytest.raises(ModeTransitionError, match="already held"):
            manager.acquire("follower_command")


class TestStopping:
    def test_zero_current_leaves_torque_enabled(self, sim_bus: SimulatedDynamixelBus) -> None:
        servo = _servo(sim_bus, ModeManager(), config=ServoLoopConfig(stop_policy=StopPolicy.ZERO_CURRENT))
        servo.configure_for_mode(
            ControlMode.BILATERAL_JOINT, current_limits_a={n: 0.5 for n in COUPLED_MOTORS}
        )
        sim_bus.write_goal_current_raw({"torso_yaw": 100})
        servo.stop()
        assert sim_bus.registers("torso_yaw").goal_current_raw == 0
        assert sim_bus.registers("torso_yaw").torque_enable == 1

    def test_torque_off_disables_every_motor(self, sim_bus: SimulatedDynamixelBus) -> None:
        servo = _servo(sim_bus, ModeManager(), config=ServoLoopConfig(stop_policy=StopPolicy.TORQUE_OFF))
        sim_bus.torque_enabled(list(COUPLED_MOTORS))
        servo.stop()
        assert all(sim_bus.registers(n).torque_enable == 0 for n in COUPLED_MOTORS)

    def test_hold_position_switches_to_position_control_at_the_measured_pose(
        self, sim_bus: SimulatedDynamixelBus
    ) -> None:
        sim_bus.joint("torso_yaw").position_rad = 0.42
        servo = _servo(
            sim_bus, ModeManager(), config=ServoLoopConfig(stop_policy=StopPolicy.HOLD_POSITION)
        )
        servo.stop()
        registers = sim_bus.registers("torso_yaw")
        assert registers.operating_mode == OperatingMode.POSITION
        assert registers.torque_enable == 1
        expected = sim_bus._rad_to_count(registers, 0.42)  # noqa: SLF001 - test introspection
        assert abs(registers.goal_position_count - expected) <= 1

    def test_an_unknown_stop_policy_is_refused(self) -> None:
        with pytest.raises(ValueError, match="stop_policy must be one of"):
            ServoLoopConfig(stop_policy="pray")


class TestServoLoopCycle:
    def test_a_fault_on_either_side_stops_both(self) -> None:
        manager = ModeManager()
        _running(manager)
        leader_bus = make_sim_bus()
        follower_bus = make_sim_bus()
        leader = ArmServo("leader", leader_bus, make_calibration(COUPLED_MOTORS), manager)
        follower = ArmServo("follower", follower_bus, make_calibration(COUPLED_MOTORS), manager)
        for servo in (leader, follower):
            servo.poll_diagnostics(force=True)

        loop = ServoLoop((leader, follower), manager, lambda states, dt: None)
        loop.run_once(0.01)

        leader_bus.set_fault("torso_yaw", SimulatedMotorFault.SILENT)
        with pytest.raises(RuntimeError, match="no valid reading"):
            loop.run_once(0.01)

    def test_excessive_cross_bus_skew_is_detected(self, monkeypatch: pytest.MonkeyPatch) -> None:
        manager = ModeManager()
        _running(manager)
        leader = ArmServo("leader", make_sim_bus(), make_calibration(COUPLED_MOTORS), manager)
        follower = ArmServo("follower", make_sim_bus(), make_calibration(COUPLED_MOTORS), manager)
        for servo in (leader, follower):
            servo.poll_diagnostics(force=True)
        loop = ServoLoop(
            (leader, follower),
            manager,
            lambda states, dt: None,
            ServoLoopConfig(max_cross_bus_skew_s=0.0),
        )
        with pytest.raises(RuntimeError, match="apart, over the"):
            loop.run_once(0.01)

    def test_log_records_are_dropped_rather_than_blocking_the_cycle(self) -> None:
        manager = ModeManager()
        _running(manager)
        servo = ArmServo("follower", make_sim_bus(), make_calibration(COUPLED_MOTORS), manager)
        loop = ServoLoop((servo,), manager, lambda states, dt: None)
        for index in range(2000):
            loop.publish_log({"i": index})
        assert loop.dropped_logs > 0
        drained = loop.drain_logs()
        assert len(drained) == 1024

    def test_duplicate_servo_names_are_refused(self) -> None:
        manager = ModeManager()
        servo = ArmServo("s", make_sim_bus(), make_calibration(COUPLED_MOTORS), manager)
        with pytest.raises(ValueError, match="must be unique"):
            ServoLoop((servo, servo), manager, lambda states, dt: None)


class TestTimingStats:
    def test_percentiles_and_overruns(self) -> None:
        stats = TimingStats("read")
        for value in (0.001, 0.002, 0.003, 0.050):
            stats.record(value, budget_s=0.01)
        summary = stats.summary()
        assert summary["count"] == 4
        assert summary["overruns"] == 1
        assert summary["max"] == pytest.approx(0.050)
        assert summary["p50"] == pytest.approx(0.0025)

    def test_an_empty_sample_set_reports_nan_not_zero(self) -> None:
        summary = TimingStats("read").summary()
        assert summary["count"] == 0
        assert np.isnan(summary["p50"])

    def test_configured_and_measured_rates_are_reported_separately(self) -> None:
        manager = ModeManager()
        servo = ArmServo("follower", make_sim_bus(), make_calibration(COUPLED_MOTORS), manager)
        loop = ServoLoop(
            (servo,), manager, lambda states, dt: None, ServoLoopConfig(control_period_s=0.002)
        )
        report = loop.timing_report()
        assert report["configured_rate_hz"] == pytest.approx(500.0)
        # Nothing has run yet, so the measured period is unknown rather than 500 Hz.
        assert np.isnan(report["measured_period_s"]["p50"])
