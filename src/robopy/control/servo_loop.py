"""The one owner of each serial port, and the periodic control cycle.

Every port has exactly one :class:`ArmServo`.  It is the only object that talks
to that bus: everything else reads the cached snapshot it publishes and submits
commands through it.  No other thread issues a SyncRead of its own, and position
commands go through the same entry point as current commands so that the two
cannot be in flight at once.

Timing is measured with :func:`time.monotonic_ns`.  When a cycle overruns, the
loop skips the missed cycles instead of firing them back-to-back to "catch up":
replaying stale commands at full rate is worse than dropping them.
"""

from __future__ import annotations

import logging
import queue
import threading
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Mapping, Protocol, Sequence, Tuple

import numpy as np

from robopy.motor.dynamixel_control_table import OperatingMode, XControlTable

from .joint_mapping import JointMap, ValidationLevel
from .mode_manager import CommandLease, ModeManager, ModeTransitionError
from .types import ControlMode, JointState, monotonic_ns

logger = logging.getLogger(__name__)

__all__ = [
    "ArmServo",
    "BusLike",
    "ServoLoop",
    "ServoLoopConfig",
    "StopPolicy",
    "TimingStats",
]


class BusLike(Protocol):
    """The bus surface :class:`ArmServo` depends on.

    Both :class:`robopy.motor.dynamixel_bus.DynamixelBus` and
    :class:`robopy.motor.sim_dynamixel_bus.SimulatedDynamixelBus` satisfy it, so
    the whole control stack runs against a simulated bus with no hardware.
    """

    motors: Mapping[str, Any]

    def read_state_block(
        self, motor_names: Sequence[str], *, timeout_s: float = ...
    ) -> Tuple[Dict[str, Any], int, int]: ...

    def write_goal_current_a(
        self, currents_a: Dict[str, float], *, timeout_s: float = ...
    ) -> Dict[str, int]: ...

    def sync_write(self, item: Any, values: Dict[str, int | float]) -> None: ...

    def sync_read(self, item: Any, motor_names: List[str]) -> Dict[str, Any]: ...

    def write_with_readback(
        self, item: Any, values: Dict[str, int | float], *, tolerance: int = ...
    ) -> Dict[str, int]: ...

    def read_diagnostics(
        self, motor_names: Sequence[str] | None = ...
    ) -> Dict[str, Dict[str, int]]: ...

    def torque_enabled(self, specific_motor_names: List[str] | None = ...) -> None: ...

    def torque_disabled(self, specific_motor_names: List[str] | None = ...) -> None: ...

    def capabilities(self, motor_name: str) -> Any: ...


class StopPolicy:
    """How a servo brings its motors to rest.

    There is no universally safe choice: cutting current or disabling torque can
    let a joint fall under its own weight, and snapping into position control is
    equally capable of producing a jolt.  The policy is therefore configured per
    machine, against its actual mechanics and support conditions.

    Attributes:
        HOLD_POSITION: Switch to position control at the *measured* pose.  Needs
            a mechanism that can hold itself there.
        ZERO_CURRENT: Command zero current, keeping torque enabled.  Safe only
            where the joint is backdrivable and supported, or horizontal.
        TORQUE_OFF: Disable torque.  Safe only where nothing can fall.
    """

    HOLD_POSITION = "hold_position"
    ZERO_CURRENT = "zero_current"
    TORQUE_OFF = "torque_off"

    ALL = (HOLD_POSITION, ZERO_CURRENT, TORQUE_OFF)


@dataclass
class ServoLoopConfig:
    """Periods, deadlines and thresholds for one servo.

    Attributes:
        control_period_s: Nominal cycle period.  This is the *configured* rate;
            what was actually achieved is reported separately by
            :class:`TimingStats`.
        read_timeout_s: Budget for one state-read transaction.
        write_timeout_s: Budget for one command-write transaction.
        max_state_age_s: A snapshot older than this is not usable for control.
        max_command_age_s: A command older than this is dropped rather than
            issued late.
        diagnostics_period_s: How often temperature, voltage and hardware-error
            status are polled.
        max_temperature_c: Fault threshold.
        min_voltage_dv: Fault threshold, in units of 0.1 V as the motor reports.
        max_voltage_dv: Fault threshold, in units of 0.1 V.
        max_acquisition_span_s: Fault threshold on how far apart the samples in
            one snapshot may be.
        max_cross_bus_skew_s: Fault threshold on the time difference between two
            buses' snapshots, checked by :class:`ServoLoop`.
        stop_policy: See :class:`StopPolicy`.
        bus_watchdog_counts: Value written to ``BUS_WATCHDOG`` (20 ms per count),
            or ``None`` to leave the register alone.
    """

    control_period_s: float = 0.005
    read_timeout_s: float = 0.02
    write_timeout_s: float = 0.02
    max_state_age_s: float = 0.05
    max_command_age_s: float = 0.05
    diagnostics_period_s: float = 1.0
    max_temperature_c: int = 70
    min_voltage_dv: int = 95
    max_voltage_dv: int = 160
    max_acquisition_span_s: float = 0.01
    max_cross_bus_skew_s: float = 0.01
    stop_policy: str = StopPolicy.ZERO_CURRENT
    bus_watchdog_counts: int | None = None

    def __post_init__(self) -> None:
        if self.stop_policy not in StopPolicy.ALL:
            raise ValueError(
                f"stop_policy must be one of {StopPolicy.ALL}, got '{self.stop_policy}'."
            )
        if self.control_period_s <= 0.0:
            raise ValueError("control_period_s must be positive.")


class TimingStats:
    """Percentile statistics of one measured duration.

    The configured period and the achieved period are reported separately on
    purpose: a loop written for 500 Hz has not been shown to run at 500 Hz until
    these numbers say so.
    """

    def __init__(self, name: str, capacity: int = 4096) -> None:
        """Track up to ``capacity`` most recent samples of ``name``."""
        self._name = name
        self._capacity = capacity
        self._samples: List[float] = []
        self._overruns = 0
        self._count = 0

    @property
    def name(self) -> str:
        """The name of the measured quantity."""
        return self._name

    @property
    def count(self) -> int:
        """Total number of samples recorded, including discarded ones."""
        return self._count

    @property
    def overruns(self) -> int:
        """Number of samples that exceeded their declared budget."""
        return self._overruns

    def record(self, seconds: float, *, budget_s: float | None = None) -> None:
        """Record one sample, counting it as an overrun if it exceeds ``budget_s``."""
        self._count += 1
        if budget_s is not None and seconds > budget_s:
            self._overruns += 1
        self._samples.append(seconds)
        if len(self._samples) > self._capacity:
            del self._samples[: len(self._samples) - self._capacity]

    def summary(self) -> Dict[str, float]:
        """p50/p95/p99/max in seconds, plus the sample and overrun counts."""
        if not self._samples:
            return {
                "count": 0.0,
                "overruns": float(self._overruns),
                "p50": float("nan"),
                "p95": float("nan"),
                "p99": float("nan"),
                "max": float("nan"),
            }
        data = np.asarray(self._samples, dtype=np.float64)
        return {
            "count": float(self._count),
            "overruns": float(self._overruns),
            "p50": float(np.percentile(data, 50)),
            "p95": float(np.percentile(data, 95)),
            "p99": float(np.percentile(data, 99)),
            "max": float(np.max(data)),
        }

    def reset(self) -> None:
        """Discard every sample."""
        self._samples.clear()
        self._overruns = 0
        self._count = 0


@dataclass
class _Snapshot:
    """The latest published state of one bus."""

    state: JointState | None = None
    diagnostics: Dict[str, Dict[str, int]] = field(default_factory=dict)
    diagnostics_ns: int = 0


class ArmServo:
    """Sole owner of one bus: reads its state, and issues its commands.

    Reads are cached.  While the servo is running, observation APIs read the
    cache rather than issuing their own transaction, which is what keeps a
    second writer or reader off the port.
    """

    def __init__(
        self,
        name: str,
        bus: BusLike,
        joint_map: JointMap,
        manager: ModeManager,
        config: ServoLoopConfig | None = None,
        *,
        motor_names: Sequence[str] | None = None,
    ) -> None:
        """Bind a servo to one bus.

        Args:
            name: Identifier used in logs and faults, e.g. ``"follower"``.
            bus: The bus this servo owns.
            joint_map: Calibration for the motors on that bus.
            manager: The shared mode manager.
            config: Periods and thresholds.
            motor_names: Motors to include in the control cycle.  Defaults to
                every motor in ``joint_map``.
        """
        self._name = name
        self._bus = bus
        self._map = joint_map
        self._manager = manager
        self._config = config or ServoLoopConfig()
        self._motor_names: Tuple[str, ...] = tuple(motor_names or joint_map.motor_names)

        unknown = [n for n in self._motor_names if n not in joint_map]
        if unknown:
            raise ValueError(f"{name}: motor(s) {unknown} are not in the joint map.")

        self._lock = threading.RLock()
        self._snapshot = _Snapshot()
        self._sequence = 0
        self._read_stats = TimingStats(f"{name}.read")
        self._write_stats = TimingStats(f"{name}.write")
        self._lease: CommandLease | None = None
        self._operating_modes: Dict[str, int] = {}

    # -- properties ---------------------------------------------------------

    @property
    def name(self) -> str:
        """Identifier of this servo."""
        return self._name

    @property
    def motor_names(self) -> Tuple[str, ...]:
        """Motors this servo reads and commands."""
        return self._motor_names

    @property
    def joint_map(self) -> JointMap:
        """The calibration in use."""
        return self._map

    @property
    def read_stats(self) -> TimingStats:
        """Timing of the state read."""
        return self._read_stats

    @property
    def write_stats(self) -> TimingStats:
        """Timing of the command write."""
        return self._write_stats

    def latest_state(self) -> JointState | None:
        """The most recent cached snapshot, without touching the bus."""
        with self._lock:
            return self._snapshot.state

    def latest_diagnostics(self) -> Dict[str, Dict[str, int]]:
        """The most recent diagnostics, without touching the bus."""
        with self._lock:
            return dict(self._snapshot.diagnostics)

    # -- reading ------------------------------------------------------------

    def read_state(self) -> JointState:
        """Perform one bulk read and publish the resulting snapshot.

        Returns:
            The snapshot, in SI units.

        Raises:
            Exception: Whatever the bus raises on a timeout or communication
                failure.  The caller decides whether that is a fault.
        """
        started = time.perf_counter()
        readings, start_ns, end_ns = self._bus.read_state_block(
            self._motor_names, timeout_s=self._config.read_timeout_s
        )
        self._read_stats.record(time.perf_counter() - started, budget_s=self._config.read_timeout_s)

        n = len(self._motor_names)
        position = np.zeros(n)
        velocity = np.zeros(n)
        current = np.zeros(n)
        valid = np.zeros(n, dtype=bool)
        for i, motor in enumerate(self._motor_names):
            reading = readings.get(motor)
            calibration = self._map[motor]
            if reading is None or not reading.valid:
                continue
            valid[i] = True
            position[i] = calibration.count_to_rad(reading.present_position)
            velocity[i] = calibration.raw_velocity_to_rad_s(reading.present_velocity)
            current[i] = calibration.raw_current_to_a(reading.present_current)

        with self._lock:
            self._sequence += 1
            state = JointState(
                joint_names=self._motor_names,
                position_rad=position,
                velocity_rad_s=velocity,
                current_a=current,
                valid=valid,
                read_start_ns=start_ns,
                read_end_ns=end_ns,
                sequence=self._sequence,
                mode_generation=self._manager.generation,
            )
            self._snapshot.state = state
        return state

    def poll_diagnostics(self, *, force: bool = False) -> Dict[str, Dict[str, int]]:
        """Read temperature, voltage and hardware error if the period has elapsed.

        Args:
            force: Read regardless of the period.

        Returns:
            The diagnostics read, or the cached ones when the period has not
            elapsed.  An empty dictionary means nothing has been read yet --
            which is *not* the same as "everything is fine", and
            :meth:`check_diagnostics` treats it as unknown, not healthy.
        """
        now = monotonic_ns()
        with self._lock:
            due = force or (
                (now - self._snapshot.diagnostics_ns) * 1e-9 >= self._config.diagnostics_period_s
            )
            if not due:
                return dict(self._snapshot.diagnostics)
        values = self._bus.read_diagnostics(self._motor_names)
        with self._lock:
            self._snapshot.diagnostics = values
            self._snapshot.diagnostics_ns = now
        return values

    def check_diagnostics(self) -> List[str]:
        """Problems visible in the cached diagnostics.

        Returns:
            A list of human-readable problems.  A motor with no diagnostics at
            all, or diagnostics older than twice the polling period, is reported
            as unknown rather than assumed healthy.
        """
        cfg = self._config
        with self._lock:
            values = dict(self._snapshot.diagnostics)
            age_ns = monotonic_ns() - self._snapshot.diagnostics_ns
        problems: List[str] = []
        if not values:
            return [f"{self._name}: no diagnostics have been read yet."]
        if age_ns * 1e-9 > 2.0 * cfg.diagnostics_period_s:
            problems.append(
                f"{self._name}: diagnostics are {age_ns * 1e-9:.2f} s old and cannot be treated "
                "as current."
            )
        for motor in self._motor_names:
            entry = values.get(motor)
            if entry is None:
                problems.append(f"{self._name}/{motor}: no diagnostics.")
                continue
            if entry["hardware_error"] != 0:
                problems.append(
                    f"{self._name}/{motor}: HARDWARE_ERROR_STATUS = "
                    f"0x{entry['hardware_error']:02x}."
                )
            if entry["temperature_c"] > cfg.max_temperature_c:
                problems.append(
                    f"{self._name}/{motor}: {entry['temperature_c']} C exceeds "
                    f"{cfg.max_temperature_c} C."
                )
            if not cfg.min_voltage_dv <= entry["voltage_dv"] <= cfg.max_voltage_dv:
                problems.append(
                    f"{self._name}/{motor}: input voltage {entry['voltage_dv'] / 10:.1f} V is "
                    f"outside [{cfg.min_voltage_dv / 10:.1f}, {cfg.max_voltage_dv / 10:.1f}] V."
                )
        return problems

    def check_state(self, state: JointState) -> List[str]:
        """Problems visible in one snapshot: validity, age, spread, range, finiteness."""
        cfg = self._config
        problems: List[str] = []
        invalid = [n for i, n in enumerate(state.joint_names) if not state.valid[i]]
        if invalid:
            problems.append(f"{self._name}: no valid reading for {invalid}.")
        age = state.age_s()
        if age > cfg.max_state_age_s:
            problems.append(
                f"{self._name}: state is {age * 1e3:.1f} ms old, over the "
                f"{cfg.max_state_age_s * 1e3:.1f} ms limit."
            )
        if state.acquisition_span_s > cfg.max_acquisition_span_s:
            problems.append(
                f"{self._name}: the samples in this snapshot span "
                f"{state.acquisition_span_s * 1e3:.1f} ms, over the "
                f"{cfg.max_acquisition_span_s * 1e3:.1f} ms limit; they cannot be treated as "
                "simultaneous."
            )
        for array, label in (
            (state.position_rad, "position"),
            (state.velocity_rad_s, "velocity"),
            (state.current_a, "current"),
        ):
            if not np.all(np.isfinite(array)):
                problems.append(f"{self._name}: non-finite {label} values in the snapshot.")
        for i, motor in enumerate(state.joint_names):
            if not state.valid[i]:
                continue
            calibration = self._map[motor]
            lower, upper = calibration.lower_limit_rad, calibration.upper_limit_rad
            if lower is not None and upper is not None:
                if not lower <= state.position_rad[i] <= upper:
                    problems.append(
                        f"{self._name}/{motor}: {state.position_rad[i]:.4f} rad is outside its "
                        f"calibrated range [{lower:.4f}, {upper:.4f}]."
                    )
        return problems

    # -- configuration ------------------------------------------------------

    def configure_for_mode(
        self,
        mode: ControlMode,
        *,
        current_limits_a: Mapping[str, float] | None = None,
        torque_enabled: Sequence[str] | None = None,
    ) -> None:
        """Bring the bus into the register state ``mode`` requires.

        The order matters and is fixed: torque OFF, operating mode, current
        limit and any gains or profile, initial goal values, read-back, torque
        ON.  In current mode (0) ``GOAL_CURRENT`` is rewritten to zero *after*
        the mode change, because changing the operating mode resets goal values
        and gains.  In a position mode the goal position is set to the measured
        pose, so enabling torque does not command a jump.

        If anything fails part-way, torque is disabled again rather than leaving
        some motors enabled in a half-configured state.

        Args:
            mode: The control mode to configure for.
            current_limits_a: Per-motor current ceiling in amperes.  Required for
                :attr:`ControlMode.BILATERAL_JOINT`.
            torque_enabled: Motors to enable at the end.  ``None`` enables every
                motor this servo owns.

        Raises:
            ValueError: If the calibration is not complete enough for the mode,
                or a motor's model does not support the required mode.
        """
        target_mode = {
            ControlMode.POSITION_TELEOP: OperatingMode.POSITION,
            ControlMode.CARTESIAN_TELEOP: OperatingMode.POSITION,
            ControlMode.BILATERAL_JOINT: OperatingMode.CURRENT,
        }[mode]

        level = (
            ValidationLevel.HARDWARE
            if target_mode == OperatingMode.CURRENT
            else ValidationLevel.GEOMETRY
        )
        self._map.require(level, motor_names=self._motor_names)

        for motor in self._motor_names:
            capabilities = self._bus.capabilities(motor)
            if target_mode not in capabilities.supported_operating_modes:
                raise ValueError(
                    f"{self._name}/{motor}: model {capabilities.model_name} does not support "
                    f"operating mode {target_mode}. Sharing a control table is not evidence that "
                    "a model supports current control."
                )

        if target_mode == OperatingMode.CURRENT and current_limits_a is None:
            raise ValueError(
                f"{self._name}: current control needs an explicit per-motor current limit."
            )

        enable = list(self._motor_names if torque_enabled is None else torque_enabled)
        unknown = sorted(set(enable) - set(self._motor_names))
        if unknown:
            raise ValueError(f"{self._name}: cannot torque-enable unknown motor(s) {unknown}.")

        configured = False
        try:
            self._bus.torque_disabled(list(self._motor_names))
            self._bus.write_with_readback(
                XControlTable.OPERATING_MODE,
                {name: target_mode for name in self._motor_names},
            )
            self._operating_modes = {name: target_mode for name in self._motor_names}

            if current_limits_a is not None:
                raw_limits: Dict[str, int | float] = {}
                for motor in self._motor_names:
                    if motor not in current_limits_a:
                        continue
                    calibration = self._map[motor]
                    ceiling = calibration.current_limit_a
                    wanted = float(current_limits_a[motor])
                    if ceiling is not None and wanted > ceiling:
                        raise ValueError(
                            f"{self._name}/{motor}: requested current limit {wanted} A exceeds "
                            f"the calibrated ceiling {ceiling} A."
                        )
                    unit = self._bus.capabilities(motor).current_unit_a
                    raw_limits[motor] = int(round(wanted / unit))
                if raw_limits:
                    self._bus.write_with_readback(XControlTable.CURRENT_LIMIT, raw_limits)

            if self._config.bus_watchdog_counts is not None:
                self._bus.write_with_readback(
                    XControlTable.BUS_WATCHDOG,
                    {name: self._config.bus_watchdog_counts for name in self._motor_names},
                )

            state = self.read_state()
            if target_mode == OperatingMode.CURRENT:
                # Changing the operating mode resets GOAL_CURRENT, so write zero
                # again here -- before torque is enabled.
                self._bus.write_goal_current_a(
                    {name: 0.0 for name in self._motor_names},
                    timeout_s=self._config.write_timeout_s,
                )
            else:
                goals: Dict[str, int | float] = {}
                for i, motor in enumerate(self._motor_names):
                    if not state.valid[i]:
                        raise ValueError(
                            f"{self._name}/{motor}: no valid position read back, so no safe "
                            "initial goal position can be set."
                        )
                    goals[motor] = self._map[motor].rad_to_count(float(state.position_rad[i]))
                self._bus.sync_write(XControlTable.GOAL_POSITION, goals)

            self._bus.torque_enabled(enable)
            configured = True
        finally:
            if not configured:
                try:
                    self._bus.torque_disabled(list(self._motor_names))
                except Exception:  # noqa: BLE001 - best effort during cleanup
                    logger.exception(
                        "%s: failed to disable torque while unwinding a failed configuration.",
                        self._name,
                    )

    # -- commanding ---------------------------------------------------------

    def acquire_command_path(self, path: str | None = None) -> CommandLease:
        """Take the single command lease for this bus.

        One lease name per bus, deliberately: position commands and current
        commands are the same path, so an IK command and a bilateral command can
        never both be live on one machine.
        """
        lease = self._manager.acquire(path or f"{self._name}_command")
        self._lease = lease
        return lease

    def release_command_path(self) -> None:
        """Release this bus's command lease, if held."""
        if self._lease is not None:
            self._lease.close()
            self._lease = None

    def _require_lease(self) -> CommandLease:
        if self._lease is None:
            raise ModeTransitionError(
                f"{self._name}: no command lease is held. Commands must go through "
                "acquire_command_path() so that only one writer exists per bus."
            )
        self._lease.require_valid()
        return self._lease

    def command_positions_rad(
        self,
        targets_rad: Mapping[str, float],
        *,
        generation: int | None = None,
        issued_ns: int | None = None,
    ) -> None:
        """Write goal positions, in radians.

        Args:
            targets_rad: ``{motor_name: radians}``.
            generation: Command generation the targets were computed from.  A
                command from a superseded generation is dropped.
            issued_ns: Monotonic time the command was produced.  A command older
                than :attr:`ServoLoopConfig.max_command_age_s` is dropped.

        Raises:
            ModeTransitionError: If no valid lease is held, or the command is
                stale or from an old generation.
            ValueError: On an unknown motor or a non-finite target.
        """
        lease = self._require_lease()
        self._check_command_freshness(generation, issued_ns, lease)
        counts: Dict[str, int | float] = {}
        for motor, angle in targets_rad.items():
            if motor not in self._motor_names:
                raise ValueError(f"{self._name}: '{motor}' is not owned by this servo.")
            if not np.isfinite(angle):
                raise ValueError(f"{self._name}/{motor}: non-finite position target.")
            counts[motor] = self._map[motor].rad_to_count(float(angle))
        started = time.perf_counter()
        self._bus.sync_write(XControlTable.GOAL_POSITION, counts)
        self._write_stats.record(
            time.perf_counter() - started, budget_s=self._config.write_timeout_s
        )

    def command_torques_nm(
        self,
        torques_nm: Mapping[str, float],
        *,
        generation: int | None = None,
        issued_ns: int | None = None,
    ) -> Dict[str, float]:
        """Convert joint torques to currents and write them.

        The conversion uses each joint's own validated torque constant; a joint
        without one raises rather than falling back to a datasheet ratio.  The
        resulting current is then clamped to the calibrated per-joint ceiling.

        Returns:
            The currents actually commanded, in amperes.

        Raises:
            ModeTransitionError: If no valid lease is held or the command is stale.
            JointMapError: If a joint has no validated torque constant.
        """
        lease = self._require_lease()
        self._check_command_freshness(generation, issued_ns, lease)

        currents: Dict[str, float] = {}
        for motor, torque in torques_nm.items():
            if motor not in self._motor_names:
                raise ValueError(f"{self._name}: '{motor}' is not owned by this servo.")
            if not np.isfinite(torque):
                raise ValueError(f"{self._name}/{motor}: non-finite torque command.")
            calibration = self._map[motor]
            amps = calibration.torque_to_current_a(float(torque))
            ceiling = calibration.current_limit_a
            if ceiling is not None:
                amps = float(np.clip(amps, -ceiling, ceiling))
            currents[motor] = amps

        started = time.perf_counter()
        self._bus.write_goal_current_a(currents, timeout_s=self._config.write_timeout_s)
        self._write_stats.record(
            time.perf_counter() - started, budget_s=self._config.write_timeout_s
        )
        return currents

    def _check_command_freshness(
        self,
        generation: int | None,
        issued_ns: int | None,
        lease: CommandLease,
    ) -> None:
        if generation is not None and generation != lease.generation:
            raise ModeTransitionError(
                f"{self._name}: refusing a command computed in generation {generation}; the "
                f"current generation is {lease.generation}."
            )
        if issued_ns is not None:
            age = (monotonic_ns() - issued_ns) * 1e-9
            if age > self._config.max_command_age_s:
                raise ModeTransitionError(
                    f"{self._name}: refusing a command that is {age * 1e3:.1f} ms old, over the "
                    f"{self._config.max_command_age_s * 1e3:.1f} ms limit."
                )

    # -- stopping -----------------------------------------------------------

    def stop(self) -> str:
        """Apply the configured stop policy on whatever side is still reachable.

        Returns:
            A description of what was done.

        Notes:
            This can only act on a bus that still answers.  A disconnected port
            cannot be stopped by writing to it; that case is covered by the
            motor's own ``BUS_WATCHDOG``, configured via
            :attr:`ServoLoopConfig.bus_watchdog_counts`.
        """
        policy = self._config.stop_policy
        if policy == StopPolicy.ZERO_CURRENT:
            self._bus.write_goal_current_a(
                {name: 0.0 for name in self._motor_names},
                timeout_s=self._config.write_timeout_s,
            )
            return f"{self._name}: commanded zero current, torque left enabled."
        if policy == StopPolicy.TORQUE_OFF:
            self._bus.torque_disabled(list(self._motor_names))
            return f"{self._name}: torque disabled."

        state = self.read_state()
        self._bus.torque_disabled(list(self._motor_names))
        self._bus.write_with_readback(
            XControlTable.OPERATING_MODE,
            {name: OperatingMode.POSITION for name in self._motor_names},
        )
        goals: Dict[str, int | float] = {
            motor: self._map[motor].rad_to_count(float(state.position_rad[i]))
            for i, motor in enumerate(self._motor_names)
            if state.valid[i]
        }
        self._bus.sync_write(XControlTable.GOAL_POSITION, goals)
        self._bus.torque_enabled(list(goals))
        return f"{self._name}: holding the measured pose in position control."

    def timing_report(self) -> Dict[str, Dict[str, float]]:
        """Read and write timing statistics for this servo."""
        return {"read": self._read_stats.summary(), "write": self._write_stats.summary()}


class ServoLoop:
    """Runs the control cycle over one or two servos on a fixed period.

    The loop owns the cycle; each :class:`ArmServo` owns its port.  The two
    servos' reads are issued from this thread in sequence, and their snapshot
    times are compared so that an excessive skew between the machines is
    detected rather than assumed away.
    """

    def __init__(
        self,
        servos: Sequence[ArmServo],
        manager: ModeManager,
        step: Callable[[Dict[str, JointState], float], None],
        config: ServoLoopConfig | None = None,
    ) -> None:
        """Configure the loop.

        Args:
            servos: The servos to drive.  Each must own a distinct bus.
            manager: The shared mode manager.
            step: ``step(states, dt)`` -- the per-cycle control computation.  It
                receives the freshly read snapshots keyed by servo name.
            config: Periods and thresholds.  The first servo's configuration is
                used when this is ``None``.
        """
        if not servos:
            raise ValueError("A servo loop needs at least one servo.")
        names = [s.name for s in servos]
        if len(set(names)) != len(names):
            raise ValueError("Servo names must be unique.")
        self._servos = tuple(servos)
        self._manager = manager
        self._step = step
        self._config = config or ServoLoopConfig()
        self._cycle_stats = TimingStats("cycle")
        self._period_stats = TimingStats("period")
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._log_queue: queue.Queue[Dict[str, Any]] = queue.Queue(maxsize=1024)
        self._dropped_logs = 0

    @property
    def servos(self) -> Tuple[ArmServo, ...]:
        """The servos this loop drives."""
        return self._servos

    @property
    def is_running(self) -> bool:
        """Whether the loop thread is alive."""
        return self._thread is not None and self._thread.is_alive()

    def publish_log(self, record: Mapping[str, Any]) -> None:
        """Hand a log record to the bounded queue, never blocking the cycle.

        When the queue is full the record is dropped and counted.  Waiting for a
        consumer -- a file writer, an image saver -- inside the control cycle is
        exactly what this avoids.
        """
        try:
            self._log_queue.put_nowait(dict(record))
        except queue.Full:
            self._dropped_logs += 1

    def drain_logs(self, limit: int = 1024) -> List[Dict[str, Any]]:
        """Take up to ``limit`` queued log records."""
        out: List[Dict[str, Any]] = []
        while len(out) < limit:
            try:
                out.append(self._log_queue.get_nowait())
            except queue.Empty:
                break
        return out

    @property
    def dropped_logs(self) -> int:
        """Number of log records dropped because the queue was full."""
        return self._dropped_logs

    def run_once(self, dt: float) -> Dict[str, JointState]:
        """Execute one cycle: read every servo, check it, then call ``step``.

        Returns:
            The snapshots read this cycle.

        Raises:
            Exception: Whatever a bus or the step function raises.  The caller
                -- or :meth:`start` -- converts that into a fault.
        """
        states: Dict[str, JointState] = {}
        for servo in self._servos:
            state = servo.read_state()
            problems = servo.check_state(state)
            servo.poll_diagnostics()
            problems.extend(servo.check_diagnostics())
            if problems:
                raise RuntimeError("; ".join(problems))
            states[servo.name] = state

        if len(states) > 1:
            times = [s.read_end_ns for s in states.values()]
            skew = (max(times) - min(times)) * 1e-9
            if skew > self._config.max_cross_bus_skew_s:
                raise RuntimeError(
                    f"The two buses' snapshots are {skew * 1e3:.1f} ms apart, over the "
                    f"{self._config.max_cross_bus_skew_s * 1e3:.1f} ms limit."
                )
        self._step(states, dt)
        return states

    def start(self) -> None:
        """Start the loop in its own thread."""
        if self.is_running:
            raise RuntimeError("The servo loop is already running.")
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run, name="robopy-servo-loop", daemon=True)
        self._thread.start()

    def stop(self, timeout_s: float = 2.0) -> None:
        """Ask the loop to finish the current cycle and exit."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=timeout_s)
            self._thread = None

    def _run(self) -> None:
        period_ns = int(self._config.control_period_s * 1e9)
        next_deadline = monotonic_ns()
        previous = next_deadline
        while not self._stop_event.is_set():
            now = monotonic_ns()
            if now < next_deadline:
                # Busy-wait only for the last stretch; sleep for the rest.
                remaining = (next_deadline - now) * 1e-9
                if remaining > 1e-3:
                    time.sleep(remaining - 1e-3)
                continue

            dt = (now - previous) * 1e-9
            previous = now
            self._period_stats.record(dt, budget_s=self._config.control_period_s * 1.5)

            started = time.perf_counter()
            try:
                self.run_once(max(dt, 1e-6))
            except Exception as exc:  # noqa: BLE001 - any failure is a fault
                logger.exception("Servo cycle failed; faulting.")
                self._manager.fault("cycle_failed", str(exc), source="servo_loop")
                self._emergency_stop()
                return
            self._cycle_stats.record(
                time.perf_counter() - started, budget_s=self._config.control_period_s
            )

            # Skip missed cycles rather than firing them back-to-back.
            now = monotonic_ns()
            next_deadline += period_ns
            if next_deadline < now:
                missed = (now - next_deadline) // period_ns + 1
                next_deadline += missed * period_ns

    def _emergency_stop(self) -> None:
        """Apply each servo's stop policy, on whichever buses still answer."""
        for servo in self._servos:
            try:
                logger.warning("%s", servo.stop())
            except Exception:  # noqa: BLE001 - a dead bus cannot be stopped by writing
                logger.exception(
                    "%s: could not issue a stop command; the bus may be gone. The motor-side "
                    "BUS_WATCHDOG is what covers that case.",
                    servo.name,
                )

    def timing_report(self) -> Dict[str, Any]:
        """Configured versus achieved timing, per servo and for the whole cycle."""
        return {
            "configured_period_s": self._config.control_period_s,
            "configured_rate_hz": 1.0 / self._config.control_period_s,
            "measured_period_s": self._period_stats.summary(),
            "cycle_s": self._cycle_stats.summary(),
            "dropped_logs": self._dropped_logs,
            "servos": {servo.name: servo.timing_report() for servo in self._servos},
        }
