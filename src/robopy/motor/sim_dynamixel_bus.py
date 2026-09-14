"""A simulated DYNAMIXEL bus with a simple plant, for running without hardware.

It presents the same surface the control stack uses -- ``read_state_block``,
``write_goal_current_a``, ``sync_read``/``sync_write``, ``write_with_readback``,
``read_diagnostics``, ``torque_enabled``/``torque_disabled``, ``capabilities`` --
so :class:`robopy.control.servo_loop.ArmServo` drives it unchanged.

It deliberately reproduces the behaviours that break naive control code:

* an operating-mode change resets ``GOAL_CURRENT`` to the current limit and
  clears the position gains, so code that forgets to rewrite zero after the
  switch produces a torque step here too;
* a motor can be told to stop answering, producing a partial SyncRead rather
  than a silent zero;
* ``BUS_WATCHDOG`` counts down on *instruction packets*, so a loop that keeps
  reading while it has stopped commanding does not trip it;
* raw values are quantised exactly as the wire format quantises them.

It is a test fixture, not a dynamics model.  A gain range that is stable against
this plant has been shown to be stable against *this plant*.
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np

from .dynamixel_bus import DynamixelMotor, MotorStateReading
from .dynamixel_control_table import (
    ControlItem,
    Dtype,
    OperatingMode,
    XControlTable,
    encode_value,
    get_motor_capabilities,
)

__all__ = ["SimulatedDynamixelBus", "SimulatedJoint", "SimulatedMotorFault"]

_RAW_VELOCITY_UNIT_RAD_S = 0.229 * 2.0 * math.pi / 60.0


class SimulatedMotorFault(Enum):
    """Fault a simulated motor can be told to exhibit.

    Attributes:
        NONE: Healthy.
        SILENT: Stops answering reads, producing a partial SyncRead.
        OVERHEAT: Reports a temperature above any sane threshold.
        HARDWARE_ERROR: Sets a non-zero ``HARDWARE_ERROR_STATUS``.
        UNDERVOLTAGE: Reports an input voltage below any sane threshold.
    """

    NONE = "none"
    SILENT = "silent"
    OVERHEAT = "overheat"
    HARDWARE_ERROR = "hardware_error"
    UNDERVOLTAGE = "undervoltage"


@dataclass
class SimulatedJoint:
    """One simulated joint: a damped inertia with friction and a joint stop.

    Attributes:
        inertia_kg_m2: Rotor-plus-link inertia seen at the joint.
        damping_nm_s: Viscous damping.
        coulomb_friction_nm: Constant friction magnitude opposing motion.
        torque_constant_nm_per_a: Torque produced per ampere.
        position_rad: Current angle.
        velocity_rad_s: Current angular velocity.
        contact_stiffness_nm_per_rad: Stiffness of an optional external contact.
        contact_position_rad: Angle beyond which the contact engages.  ``None``
            means free motion.
        gravity_torque_nm: Constant external torque, standing in for gravity.
    """

    inertia_kg_m2: float = 2e-3
    damping_nm_s: float = 2e-3
    coulomb_friction_nm: float = 1e-3
    torque_constant_nm_per_a: float = 1.5
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    contact_stiffness_nm_per_rad: float = 0.0
    contact_position_rad: float | None = None
    gravity_torque_nm: float = 0.0

    def integrate(self, applied_torque_nm: float, dt: float) -> None:
        """Advance the joint by ``dt`` under ``applied_torque_nm``."""
        torque = applied_torque_nm + self.gravity_torque_nm
        if self.contact_position_rad is not None:
            penetration = self.position_rad - self.contact_position_rad
            if penetration > 0.0:
                torque -= self.contact_stiffness_nm_per_rad * penetration
        torque -= self.damping_nm_s * self.velocity_rad_s
        if abs(self.velocity_rad_s) > 1e-9:
            torque -= math.copysign(self.coulomb_friction_nm, self.velocity_rad_s)
        elif abs(torque) < self.coulomb_friction_nm:
            torque = 0.0
        self.velocity_rad_s += torque / self.inertia_kg_m2 * dt
        self.position_rad += self.velocity_rad_s * dt


@dataclass
class _MotorRegisters:
    """Register file of one simulated motor."""

    motor: DynamixelMotor
    joint: SimulatedJoint
    zero_count: int
    direction: int = 1
    operating_mode: int = OperatingMode.POSITION
    torque_enable: int = 0
    goal_current_raw: int = 0
    goal_position_count: int = 0
    current_limit_raw: int = 1000
    position_p_gain: int = 800
    position_i_gain: int = 0
    position_d_gain: int = 0
    bus_watchdog: int = 0
    hardware_error: int = 0
    temperature_c: int = 35
    voltage_dv: int = 120
    fault: SimulatedMotorFault = SimulatedMotorFault.NONE
    profile_velocity: int = 0
    profile_acceleration: int = 0
    drive_mode: int = 0
    homing_offset: int = 0
    _watchdog_elapsed_s: float = field(default=0.0, repr=False)


class SimulatedDynamixelBus:
    """A stand-in for :class:`robopy.motor.dynamixel_bus.DynamixelBus`."""

    def __init__(
        self,
        motors: Mapping[str, DynamixelMotor],
        *,
        joints: Mapping[str, SimulatedJoint] | None = None,
        zero_counts: Mapping[str, int] | None = None,
        port: str = "sim://dynamixel",
        auto_step: bool = True,
    ) -> None:
        """Create a simulated bus.

        Args:
            motors: ``{name: DynamixelMotor}``, exactly as the real bus takes.
            joints: Per-motor plant.  Defaults to an identical joint each.
            zero_counts: Encoder count corresponding to zero radians.  Defaults
                to 2048 for every motor.
            port: Cosmetic port name.
            auto_step: Advance the plant automatically on each state read, using
                the elapsed wall time.  Set ``False`` to drive
                :meth:`step` by hand from a test.
        """
        self.motors: Dict[str, DynamixelMotor] = dict(motors)
        self.port_handler = _SimPortHandler(port)
        self._lock = threading.RLock()
        self._auto_step = auto_step
        self._last_step_ns = time.monotonic_ns()
        self._registers: Dict[str, _MotorRegisters] = {}
        for name, motor in self.motors.items():
            self._registers[name] = _MotorRegisters(
                motor=motor,
                joint=(joints or {}).get(name, SimulatedJoint()),
                zero_count=(zero_counts or {}).get(name, 2048),
            )
        self.calibration: Dict[str, Tuple[int, bool]] = {}
        self.transaction_count = 0
        self.instruction_count = 0

    # -- test controls ------------------------------------------------------

    def joint(self, motor_name: str) -> SimulatedJoint:
        """The simulated plant behind ``motor_name``."""
        return self._registers[motor_name].joint

    def set_fault(self, motor_name: str, fault: SimulatedMotorFault) -> None:
        """Make ``motor_name`` misbehave in a specific way."""
        self._registers[motor_name].fault = fault

    def registers(self, motor_name: str) -> _MotorRegisters:
        """Raw register file of ``motor_name``, for assertions in tests."""
        return self._registers[motor_name]

    def step(self, dt: float) -> None:
        """Advance every joint by ``dt`` under its current command."""
        with self._lock:
            for registers in self._registers.values():
                torque = 0.0
                if registers.torque_enable:
                    if registers.operating_mode == OperatingMode.CURRENT:
                        unit = get_motor_capabilities(registers.motor.model_name).current_unit_a
                        amps = registers.goal_current_raw * unit
                        torque = (
                            registers.direction * amps * registers.joint.torque_constant_nm_per_a
                        )
                    elif registers.operating_mode in (
                        OperatingMode.POSITION,
                        OperatingMode.EXTENDED_POSITION,
                        OperatingMode.CURRENT_BASED_POSITION,
                    ):
                        goal_rad = self._count_to_rad(registers, registers.goal_position_count)
                        error = goal_rad - registers.joint.position_rad
                        gain = registers.position_p_gain / 128.0
                        torque = gain * error - 0.02 * registers.joint.velocity_rad_s
                registers.joint.integrate(torque, dt)
                if registers.bus_watchdog > 0:
                    registers._watchdog_elapsed_s += dt
                    if registers._watchdog_elapsed_s > registers.bus_watchdog * 0.02:
                        # Watchdog expiry latches -1 and stops the motor.
                        registers.bus_watchdog = -1
                        registers.goal_current_raw = 0

    def _auto_advance(self) -> None:
        if not self._auto_step:
            return
        now = time.monotonic_ns()
        dt = (now - self._last_step_ns) * 1e-9
        self._last_step_ns = now
        if 0.0 < dt < 1.0:
            self.step(dt)

    # -- conversions --------------------------------------------------------

    @staticmethod
    def _count_to_rad(registers: _MotorRegisters, count: int) -> float:
        return registers.direction * (count - registers.zero_count) * 2.0 * math.pi / 4096.0

    @staticmethod
    def _rad_to_count(registers: _MotorRegisters, angle: float) -> int:
        return int(
            round(registers.zero_count + registers.direction * angle * 4096.0 / (2.0 * math.pi))
        )

    # -- bus surface --------------------------------------------------------

    def capabilities(self, motor_name: str) -> Any:
        """Verified physical units and modes of ``motor_name``'s model."""
        if motor_name not in self.motors:
            raise ValueError(f"Unknown motor '{motor_name}' on the simulated bus.")
        return get_motor_capabilities(self.motors[motor_name].model_name)

    def read_state_block(
        self,
        motor_names: Sequence[str],
        *,
        timeout_s: float = 0.02,
    ) -> Tuple[Dict[str, MotorStateReading], int, int]:
        """Bulk-read current, velocity and position, exactly like the real bus."""
        del timeout_s
        unknown = [n for n in motor_names if n not in self.motors]
        if unknown:
            raise ValueError(f"Unknown motor(s) on this bus: {unknown}")
        self._auto_advance()
        start_ns = time.monotonic_ns()
        with self._lock:
            self.transaction_count += 1
            self.instruction_count += 1
            for registers in self._registers.values():
                registers._watchdog_elapsed_s = 0.0
            readings: Dict[str, MotorStateReading] = {}
            for name in motor_names:
                registers = self._registers[name]
                if registers.fault is SimulatedMotorFault.SILENT:
                    readings[name] = MotorStateReading(name, registers.motor.id, 0, 0, 0, False)
                    continue
                unit = get_motor_capabilities(registers.motor.model_name).current_unit_a
                applied_a = registers.goal_current_raw * unit if registers.torque_enable else 0.0
                readings[name] = MotorStateReading(
                    motor_name=name,
                    motor_id=registers.motor.id,
                    present_current=int(round(applied_a / unit)),
                    present_velocity=int(
                        round(
                            registers.direction
                            * registers.joint.velocity_rad_s
                            / _RAW_VELOCITY_UNIT_RAD_S
                        )
                    ),
                    present_position=self._rad_to_count(registers, registers.joint.position_rad),
                    valid=True,
                )
        return readings, start_ns, time.monotonic_ns()

    def write_goal_current_a(
        self,
        currents_a: Dict[str, float],
        *,
        timeout_s: float = 0.02,
    ) -> Dict[str, int]:
        """Write goal currents in amperes, quantised per model."""
        del timeout_s
        raw: Dict[str, int] = {}
        for name, amps in currents_a.items():
            raw[name] = int(round(amps / self.capabilities(name).current_unit_a))
        self.write_goal_current_raw(raw)
        return raw

    def write_goal_current_raw(self, values: Dict[str, int], *, timeout_s: float = 0.02) -> None:
        """Write signed raw ``GOAL_CURRENT`` counts."""
        del timeout_s
        with self._lock:
            self.transaction_count += 1
            self.instruction_count += 1
            for name, value in values.items():
                if name not in self._registers:
                    raise ValueError(f"Unknown motor '{name}' on this bus.")
                registers = self._registers[name]
                encode_value(int(value), Dtype.INT16)
                limit = registers.current_limit_raw
                registers.goal_current_raw = int(np.clip(int(value), -limit, limit))
                registers._watchdog_elapsed_s = 0.0

    def sync_write(self, item: Enum, values: Dict[str, int | float]) -> None:
        """Write one control-table item to several motors."""
        control_item: ControlItem = item.value
        with self._lock:
            self.transaction_count += 1
            self.instruction_count += 1
            for name, value in values.items():
                if name not in self._registers:
                    continue
                self._write_register(self._registers[name], item, int(value))
                self._registers[name]._watchdog_elapsed_s = 0.0
        del control_item

    def sync_read(self, item: Enum, motor_names: List[str]) -> Dict[str, Any]:
        """Read one control-table item from several motors."""
        self._auto_advance()
        with self._lock:
            self.transaction_count += 1
            self.instruction_count += 1
            out: Dict[str, Any] = {}
            for name in motor_names:
                registers = self._registers.get(name)
                if registers is None or registers.fault is SimulatedMotorFault.SILENT:
                    continue
                registers._watchdog_elapsed_s = 0.0
                out[name] = self._read_register(registers, item)
            return out

    def read(self, item: Enum, motor_name: str) -> Any:
        """Read one item from one motor."""
        return self.sync_read(item, [motor_name]).get(motor_name)

    def write(self, item: Enum, motor_name: str, value: int | float) -> None:
        """Write one item to one motor."""
        self.sync_write(item, {motor_name: value})

    def write_with_readback(
        self,
        item: Enum,
        values: Dict[str, int | float],
        *,
        tolerance: int = 0,
    ) -> Dict[str, int]:
        """Write and verify, exactly like the real bus."""
        self.sync_write(item, values)
        readback = self.sync_read(item, list(values))
        mismatched = {}
        for name, wanted in values.items():
            if name not in readback:
                raise ConnectionError(f"Motor '{name}' did not answer the read-back.")
            if abs(int(readback[name]) - int(wanted)) > tolerance:
                mismatched[name] = (int(wanted), int(readback[name]))
        if mismatched:
            raise ValueError(f"Read-back mismatch: {mismatched}")
        return {name: int(readback[name]) for name in values}

    def read_diagnostics(
        self, motor_names: Sequence[str] | None = None
    ) -> Dict[str, Dict[str, int]]:
        """Temperature, voltage and hardware error for each motor that answers."""
        names = list(self.motors) if motor_names is None else list(motor_names)
        with self._lock:
            self.transaction_count += 1
            self.instruction_count += 1
            out: Dict[str, Dict[str, int]] = {}
            for name in names:
                registers = self._registers.get(name)
                if registers is None or registers.fault is SimulatedMotorFault.SILENT:
                    continue
                temperature = (
                    120
                    if registers.fault is SimulatedMotorFault.OVERHEAT
                    else registers.temperature_c
                )
                voltage = (
                    80
                    if registers.fault is SimulatedMotorFault.UNDERVOLTAGE
                    else registers.voltage_dv
                )
                hardware_error = (
                    0x20
                    if registers.fault is SimulatedMotorFault.HARDWARE_ERROR
                    else registers.hardware_error
                )
                out[name] = {
                    "temperature_c": temperature,
                    "voltage_dv": voltage,
                    "hardware_error": hardware_error,
                }
            return out

    def torque_enabled(self, specific_motor_names: List[str] | None = None) -> None:
        """Enable torque on the given motors, or on all of them."""
        names = list(self.motors) if specific_motor_names is None else specific_motor_names
        self.sync_write(XControlTable.TORQUE_ENABLE, {name: 1 for name in names})

    def torque_disabled(self, specific_motor_names: List[str] | None = None) -> None:
        """Disable torque on the given motors, or on all of them."""
        names = list(self.motors) if specific_motor_names is None else specific_motor_names
        self.sync_write(XControlTable.TORQUE_ENABLE, {name: 0 for name in names})

    def open(self, baudrate: int = 1_000_000) -> None:
        """No-op: a simulated port is always open."""
        del baudrate

    def close(self) -> None:
        """No-op."""

    def __len__(self) -> int:
        return len(self.motors)

    def __repr__(self) -> str:
        return f"SimulatedDynamixelBus(motors=[{', '.join(self.motors)}])"

    # -- register access ----------------------------------------------------

    def _write_register(self, registers: _MotorRegisters, item: Enum, value: int) -> None:
        if item is XControlTable.OPERATING_MODE:
            if value == registers.operating_mode:
                return
            registers.operating_mode = value
            # A real X-series motor resets goal values and gains on a mode
            # change: GOAL_CURRENT comes back at the current limit, not at zero.
            registers.goal_current_raw = registers.current_limit_raw
            registers.position_p_gain = 800
            registers.position_i_gain = 0
            registers.position_d_gain = 0
            registers.goal_position_count = self._rad_to_count(
                registers, registers.joint.position_rad
            )
        elif item is XControlTable.TORQUE_ENABLE:
            registers.torque_enable = value
        elif item is XControlTable.GOAL_CURRENT:
            limit = registers.current_limit_raw
            registers.goal_current_raw = int(np.clip(value, -limit, limit))
        elif item is XControlTable.GOAL_POSITION:
            registers.goal_position_count = value
        elif item is XControlTable.CURRENT_LIMIT:
            registers.current_limit_raw = value
        elif item is XControlTable.POSITION_P_GAIN:
            registers.position_p_gain = value
        elif item is XControlTable.POSITION_I_GAIN:
            registers.position_i_gain = value
        elif item is XControlTable.POSITION_D_GAIN:
            registers.position_d_gain = value
        elif item is XControlTable.BUS_WATCHDOG:
            registers.bus_watchdog = value
            registers._watchdog_elapsed_s = 0.0
        elif item is XControlTable.DRIVE_MODE:
            registers.drive_mode = value
        elif item is XControlTable.HOMING_OFFSET:
            registers.homing_offset = value
        elif item is XControlTable.PROFILE_VELOCITY:
            registers.profile_velocity = value
        elif item is XControlTable.PROFILE_ACCELERATION:
            registers.profile_acceleration = value
        elif item is XControlTable.LED:
            pass
        else:
            raise ValueError(f"The simulated bus does not support writing {item.name}.")

    def _read_register(self, registers: _MotorRegisters, item: Enum) -> int:
        mapping = {
            XControlTable.MODEL_NUMBER: registers.motor.model_number,
            XControlTable.ID: registers.motor.id,
            XControlTable.FIRMWARE_VERSION: 52,
            XControlTable.OPERATING_MODE: registers.operating_mode,
            XControlTable.TORQUE_ENABLE: registers.torque_enable,
            XControlTable.GOAL_CURRENT: registers.goal_current_raw,
            XControlTable.GOAL_POSITION: registers.goal_position_count,
            XControlTable.CURRENT_LIMIT: registers.current_limit_raw,
            XControlTable.POSITION_P_GAIN: registers.position_p_gain,
            XControlTable.POSITION_I_GAIN: registers.position_i_gain,
            XControlTable.POSITION_D_GAIN: registers.position_d_gain,
            XControlTable.BUS_WATCHDOG: registers.bus_watchdog,
            XControlTable.DRIVE_MODE: registers.drive_mode,
            XControlTable.HOMING_OFFSET: registers.homing_offset,
            XControlTable.PROFILE_VELOCITY: registers.profile_velocity,
            XControlTable.PROFILE_ACCELERATION: registers.profile_acceleration,
            XControlTable.PRESENT_POSITION: self._rad_to_count(
                registers, registers.joint.position_rad
            ),
            XControlTable.PRESENT_VELOCITY: int(
                round(registers.joint.velocity_rad_s / _RAW_VELOCITY_UNIT_RAD_S)
            ),
            XControlTable.PRESENT_CURRENT: registers.goal_current_raw
            if registers.torque_enable
            else 0,
        }
        if item is XControlTable.PRESENT_TEMPERATURE:
            return (
                120 if registers.fault is SimulatedMotorFault.OVERHEAT else registers.temperature_c
            )
        if item is XControlTable.PRESENT_INPUT_VOLTAGE:
            return (
                80 if registers.fault is SimulatedMotorFault.UNDERVOLTAGE else registers.voltage_dv
            )
        if item is XControlTable.HARDWARE_ERROR_STATUS:
            return 0x20 if registers.fault is SimulatedMotorFault.HARDWARE_ERROR else 0
        if item in mapping:
            return int(mapping[item])
        raise ValueError(f"The simulated bus does not support reading {item.name}.")


@dataclass
class _SimPortHandler:
    """Minimal stand-in for the SDK port handler, for log messages."""

    port_name: str
