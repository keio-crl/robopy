"""Drive only the real Rakuda's head from the headset; leave the arms alone.

The full hardware path (``robopy-vr --hardware``) starts the Cartesian control
system: leader and follower buses, the dual-arm solver, calibrated joint maps.
For a demonstration where the headset turns the robot's head and the head
camera fills the operator's view, none of that is needed, and none of the
measured calibration it requires (``zero_count`` and friends) has to exist.

:class:`HeadOnlyFollowerBackend` talks to the follower bus directly, in the
motors' own units (encoder counts, or the bus's calibrated degrees when a
calibration is loaded): it reads where the two head motors are when it
starts -- the only time it reads them -- takes that as the pose the operator
re-centres on, and while teleoperating writes ``start + headset angle`` to
those two motors, clamped to a travel around the start pose.  No other
motor is ever written from the headset.  The
sign of each axis relative to the headset is what :func:`head_motor_mapping`
puts into the head tracker: the URDF's sign for the joint (derived from the
model) times the motor's measured ``direction`` when the configuration has
one, ``+1`` otherwise -- so a head that turns the wrong way is fixed with
``--head-signs``, not by editing code.

With a leader (master) arm on its own bus, every other follower motor
follows the leader exactly as position teleoperation does -- the leader's
present position, in encoder counts, becomes the follower's goal -- while
the two head motors keep following the headset.  The leader's own head
readings are ignored.
"""

from __future__ import annotations

import logging
import math
import threading
import time
from dataclasses import dataclass
from typing import Any, Dict, Iterable, Mapping, Tuple

import numpy as np
from numpy.typing import NDArray

from .backend import BackendReport, TeleopCommand
from .head_tracking import HeadJointMapping

logger = logging.getLogger(__name__)

__all__ = [
    "LEADER_GRIP_HOLD_COUNT",
    "HeadMotor",
    "HeadOnlyFollowerBackend",
    "head_motor_mapping",
]

#: Goal written to the leader's gripper motors so they spring back when
#: released, as position teleoperation does (``RakudaPairSys.teleoperate_step``).
LEADER_GRIP_HOLD_COUNT = 2400


@dataclass(frozen=True)
class HeadMotor:
    """One head motor and how its travel relates to the model.

    Attributes:
        motor: Follower motor name (``head_yaw`` / ``head_pitch``).
        urdf_joint: The model joint it moves, for the twin the page draws, or
            ``None`` to leave the twin's head still.
        urdf_neutral_rad: Model angle at which that joint looks straight ahead;
            the start pose is taken to be it.
        direction: URDF radians per motor radian, ``+1`` or ``-1``.  A measured
            property of the machine; ``+1`` until measured.
        range_rad: Travel allowed on either side of the start pose.
        counts_per_revolution: Encoder resolution.
    """

    motor: str
    urdf_joint: str | None = None
    urdf_neutral_rad: float = 0.0
    direction: int = 1
    range_rad: float = math.radians(60.0)
    counts_per_revolution: int = 4096

    def __post_init__(self) -> None:
        if self.direction not in (1, -1):
            raise ValueError("direction must be +1 or -1.")
        if not 0.0 < self.range_rad <= math.pi:
            raise ValueError("range_rad must be within (0, pi].")
        if self.counts_per_revolution <= 0:
            raise ValueError("counts_per_revolution must be positive.")


def head_motor_mapping(
    yaw: HeadMotor,
    pitch: HeadMotor,
    *,
    yaw_sign_urdf: int,
    pitch_sign_urdf: int,
    sign_overrides: Tuple[int, int] | None = None,
) -> HeadJointMapping:
    """A head mapping in *motor* space: targets are radians from the start pose.

    Args:
        yaw: The yaw motor.
        pitch: The pitch motor.
        yaw_sign_urdf: Sign of the URDF yaw joint per headset radian (from
            :meth:`~robopy.vr.head_tracking.HeadJointMapping.from_model`).
        pitch_sign_urdf: Same for pitch.
        sign_overrides: ``(yaw, pitch)`` motor signs stated by the operator,
            replacing the derived ones.
    """
    if sign_overrides is not None:
        yaw_sign, pitch_sign = sign_overrides
        source = "given on the command line"
    else:
        yaw_sign = yaw_sign_urdf * yaw.direction
        pitch_sign = pitch_sign_urdf * pitch.direction
        source = "URDF axis x motor direction"
    return HeadJointMapping(
        yaw_joint=yaw.motor,
        pitch_joint=pitch.motor,
        yaw_sign=yaw_sign,
        pitch_sign=pitch_sign,
        yaw_neutral_rad=0.0,
        pitch_neutral_rad=0.0,
        yaw_limits_rad=(-yaw.range_rad, yaw.range_rad),
        pitch_limits_rad=(-pitch.range_rad, pitch.range_rad),
        forward_source="the motors' start pose (re-centre while the head looks ahead)",
        notes=(
            f"motor signs {source}: {yaw.motor} {yaw_sign:+d}, {pitch.motor} {pitch_sign:+d}; "
            "a head that turns the wrong way needs --head-signs, not a code change",
        ),
    )


class HeadOnlyFollowerBackend:
    """Write the head motors of a follower bus; read them back; touch nothing else.

    Args:
        bus: The follower's bus (:class:`~robopy.motor.dynamixel_bus.DynamixelBus`
            or the simulated one), already open with torque on for the head.
        model: The kinematic model, for the page's twin.
        yaw: The yaw motor.
        pitch: The pitch motor.
        tcp_frames: ``{"left"|"right": frame}`` for hand poses in the report;
            identity poses without.
        rest_positions_rad: Model angles to report for every joint the head
            does not move (the arms as they stand), zeros otherwise.
        leader_bus: The leader (master) arm's bus, or ``None``.  With it, the
            leader's present positions are copied to the follower's other
            motors on every step, as position teleoperation does.
        leader_to_follower: ``{leader_motor: follower_motor}``; identity over
            the leader's motors by default.
        follower_writable: Follower motors that may be written from the
            leader (the torque-enabled ones); all of them by default.  The
            head motors are never written from the leader.
        control_system: A running bilateral
            :class:`~robopy.robots.rakuda.rakuda_control.RakudaControlSystem`.
            The bus then belongs to its loop: head goals go through its
            ``set_direct_goal_counts()`` and are written by that loop, nothing
            is read back, and the leader is the coupling's business, not this
            backend's.  The start pose is read here, so build the backend
            *before* the system starts.
        target_ttl_s: Validity of the goals handed to the control system.
        threaded: Do the bus traffic on a thread of this backend's own, at
            ``rate_hz``, and make :meth:`apply` only *post* the head goals.
            A USB serial round trip costs some 16 ms on Linux, and with a
            leader each step is three of them: done on the teleop socket's
            thread, that falls behind the headset's 60 Hz and the head lags
            further with every message.  ``False`` writes inline, for tests.
        rate_hz: The bus thread's rate.
        home_units: ``{motor: units}`` the head is moved to by :meth:`home`
            (the reset position, in the bus's units); ``None`` keeps the
            pose the head has when the backend is built.
        home_timeout_s: How long :meth:`home` waits for the head to arrive.
        home_tolerance: Distance from the home position, in units, that counts
            as arrived.
    """

    name = "hardware"

    def __init__(
        self,
        bus: Any,
        *,
        model: Any,
        yaw: HeadMotor,
        pitch: HeadMotor,
        tcp_frames: Mapping[str, str] | None = None,
        rest_positions_rad: Mapping[str, float] | None = None,
        leader_bus: Any | None = None,
        leader_to_follower: Mapping[str, str] | None = None,
        follower_writable: Iterable[str] | None = None,
        control_system: Any | None = None,
        target_ttl_s: float = 0.25,
        threaded: bool = True,
        rate_hz: float = 50.0,
        home_units: Mapping[str, float] | None = None,
        home_timeout_s: float = 3.0,
        home_tolerance: float = 8.0,
    ) -> None:
        from robopy.motor.dynamixel_control_table import XControlTable

        if control_system is not None and leader_bus is not None:
            raise ValueError("With a control system the leader is coupled by it; drop leader_bus.")
        if target_ttl_s <= 0.0:
            raise ValueError("target_ttl_s must be positive.")
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive.")
        self._system = control_system
        self._ttl = target_ttl_s
        self._threaded = threaded and control_system is None
        self._period_s = 1.0 / rate_hz
        self._lock = threading.Lock()
        self._pending: Dict[str, float] = {}
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self._cycles = 0
        self._cycle_ms = 0.0
        self._slow_warned = False
        self._bus = bus
        self._leader = leader_bus
        self._leader_names: list[str] = []
        self._leader_map: Dict[str, str] = {}
        self._leader_goals: Dict[str, float] = {}
        if leader_bus is not None:
            self._leader_names = list(leader_bus.motors)
            mapping = dict(leader_to_follower or {n: n for n in self._leader_names})
            writable = set(follower_writable) if follower_writable is not None else set(bus.motors)
            head = {yaw.motor, pitch.motor}
            self._leader_map = {
                leader: follower
                for leader, follower in mapping.items()
                if leader in self._leader_names
                and follower in bus.motors
                and follower in writable
                and follower not in head
            }
        self._model = model
        self._motors = {"yaw": yaw, "pitch": pitch}
        self._by_name = {yaw.motor: yaw, pitch.motor: pitch}
        self._tcp_frames = dict(tcp_frames or {})
        self._rest = {name: 0.0 for name in model.movable_joint_names}
        self._rest.update({k: float(v) for k, v in (rest_positions_rad or {}).items()})
        self._goal_item = XControlTable.GOAL_POSITION
        self._present_item = XControlTable.PRESENT_POSITION
        # Calibrated buses speak degrees; bare ones speak encoder counts.
        self._degrees = bool(getattr(bus, "calibration", None))
        self._home = None if home_units is None else {k: float(v) for k, v in home_units.items()}
        self._home_timeout_s = home_timeout_s
        self._home_tolerance = home_tolerance
        self._homed = home_units is None
        self._homing: Dict[str, float] | None = None
        self._homing_done = threading.Event()
        self._start = self._read_present()
        self._present = dict(self._start)
        self._goals: Dict[str, float] = {}
        self._writes = 0
        self._last_warnings: list[str] = []
        self._last_write_s: float | None = None
        missing = [m.motor for m in self._by_name.values() if m.motor not in self._start]
        if missing:
            raise RuntimeError(f"head motor(s) {missing} did not answer on the bus")

    # -- units ----------------------------------------------------------------

    def _scale(self, motor: HeadMotor) -> float:
        """Motor units per radian."""
        return 180.0 / math.pi if self._degrees else motor.counts_per_revolution / (2.0 * math.pi)

    def _read_present(self) -> Dict[str, float]:
        names = list(self._by_name)
        values = self._bus.sync_read(self._present_item, names)
        return {name: float(values[name]) for name in names if name in values}

    # -- TeleopBackend --------------------------------------------------------

    @property
    def model(self) -> Any:
        """The kinematic model (for the page)."""
        return self._model

    @property
    def bus(self) -> Any:
        """The follower bus."""
        return self._bus

    @property
    def start_units(self) -> Dict[str, float]:
        """Motor readings at start: the pose the operator re-centres on."""
        return dict(self._start)

    def joint_positions(self) -> Dict[str, float]:
        """Model angles: rest values, with the head joints as last commanded."""
        out = dict(self._rest)
        with self._lock:
            present = dict(self._present)
        for motor in self._by_name.values():
            if motor.urdf_joint is None:
                continue
            delta = (present[motor.motor] - self._start[motor.motor]) / self._scale(motor)
            out[motor.urdf_joint] = motor.urdf_neutral_rad + motor.direction * delta
        return out

    def hand_pose(self, side: str) -> NDArray[np.float64]:
        """TCP pose at the reported configuration (identity without TCP frames)."""
        frame = self._tcp_frames.get(side)
        if frame is None:
            return np.eye(4)
        q = self._model.q_from_positions(self.joint_positions(), require_all=False)
        return np.asarray(self._model.frame_pose(q, frame), dtype=np.float64)

    def apply(self, command: TeleopCommand) -> BackendReport:
        """Write the head targets (radians from the start pose) to the motors."""
        started = time.perf_counter()
        warnings: list[str] = []
        goals: Dict[str, float] = {}
        for name, angle in command.head_targets_rad.items():
            motor = self._by_name.get(name)
            if motor is None:
                warnings.append(f"'{name}' is not a head motor; not commanded")
                continue
            if not math.isfinite(angle):
                warnings.append(f"{name}: non-finite target ignored")
                continue
            clipped = max(-motor.range_rad, min(motor.range_rad, float(angle)))
            units = self._start[name] + clipped * self._scale(motor)
            if not self._degrees:
                units = float(min(motor.counts_per_revolution - 1, max(0, round(units))))
            goals[name] = units
        if command.arm_target is not None or command.gripper_targets_rad:
            warnings.append(
                "arms and grippers follow the leader, not the controllers, in head-only mode"
                if self._leader is not None
                else "arms and grippers are not driven in head-only mode"
            )
        if goals and self._system is not None:
            if self._degrees:
                warnings.append("a calibrated bus cannot be driven through the control system")
            else:
                self._system.set_direct_goal_counts(
                    {k: int(v) for k, v in goals.items()}, ttl_s=self._ttl
                )
                self._goals.update(goals)
                self._writes += 1
                self._last_write_s = command.stamp_s
                # The loop owns the bus: no read-back; the goal stands in for it.
                self._present.update(goals)
        elif not self._homed:
            warnings.append("head not homed yet; targets dropped")
        elif self._threaded:
            if goals:
                with self._lock:
                    self._pending.update(goals)
                    self._goals.update(goals)
                self._last_write_s = command.stamp_s
            self._ensure_thread()
        else:
            if goals:
                self._goals.update(goals)
                self._last_write_s = command.stamp_s
            self.step(head_goals=goals)
        self._last_warnings = warnings
        report = BackendReport(joints=self.joint_positions())
        report.hand_poses = {side: self.hand_pose(side) for side in ("left", "right")}
        report.compute_ms = (time.perf_counter() - started) * 1e3
        report.warnings = warnings
        return report

    def set_start_units(self, units: Mapping[str, float]) -> None:
        """Declare the start pose (after the head was homed by other means)."""
        with self._lock:
            for motor, value in units.items():
                if motor in self._by_name:
                    self._start[motor] = float(value)
                    self._present[motor] = float(value)
            self._homed = True

    @property
    def homed(self) -> bool:
        """Whether the head has been put at (or has timed out reaching) its reset position."""
        return self._homed

    def home(self, timeout_s: float | None = None) -> Dict[str, Any]:
        """Move the head to its reset position and make that the start pose.

        Done when the operator connects, before any headset target is issued.
        Head goals posted meanwhile are dropped.  With no ``home_units`` (or
        once homed, or with a control system owning the bus) nothing moves.

        Args:
            timeout_s: Overrides the configured wait.

        Returns:
            ``{"moved": bool, "arrived": bool, "start": {...}}``.
        """
        if self._home is None or self._homed or self._system is not None:
            return {"moved": False, "arrived": self._homed, "start": self.start_units}
        wait = self._home_timeout_s if timeout_s is None else timeout_s
        targets = {
            motor: float(self._clip_units(self._by_name[motor], units))
            for motor, units in self._home.items()
            if motor in self._by_name
        }
        self._homing_done.clear()
        with self._lock:
            self._homing = targets
            self._pending.clear()
        if self._threaded:
            self._ensure_thread()
            arrived = self._homing_done.wait(wait)
        else:
            deadline = time.monotonic() + wait
            arrived = False
            while not arrived and time.monotonic() < deadline:
                self.step()
                arrived = self._homing_done.is_set()
                if not arrived:
                    time.sleep(self._period_s)
        with self._lock:
            self._homing = None
            self._start = dict(self._present)
            for motor, units in targets.items():
                self._start[motor] = units  # the goal it was sent to, arrived or not
                self._present[motor] = self._start[motor]
            self._homed = True
        if not arrived:
            logger.warning(
                "head did not reach its reset position within %.1f s; carrying on from it", wait
            )
        return {"moved": True, "arrived": arrived, "start": self.start_units}

    def _clip_units(self, motor: HeadMotor, units: float) -> float:
        if self._degrees:
            return units
        return float(min(motor.counts_per_revolution - 1, max(0, round(units))))

    def step(self, head_goals: Mapping[str, float] | None = None) -> None:
        """One bus cycle: read the leader, write the follower once.

        Two bus transactions, no more: the leader's present positions come in
        (one read), and one write carries the fifteen joints copied from it
        together with the two head goals from the headset.  The head's own
        position is never read back after the start pose -- the goal just
        written stands in for it in what the page and the log see.

        The only place the bus is used once the backend is built.  Called by
        the bus thread; or inline from :meth:`apply` when not threaded.

        Args:
            head_goals: Head goals to write this cycle; the posted ones when
                ``None``.
        """
        started = time.perf_counter()
        with self._lock:
            homing = None if self._homing is None else dict(self._homing)
            if homing is not None:
                goals = dict(homing)  # headset goals wait until the head is home
                self._pending.clear()
            else:
                goals = dict(self._pending if head_goals is None else head_goals)
                self._pending.clear()
        if self._leader is not None and self._leader_map:
            leader = self._leader.sync_read(self._present_item, self._leader_names)
            leader_goals = {
                follower: float(leader[name])
                for name, follower in self._leader_map.items()
                if name in leader
            }
            with self._lock:
                self._leader_goals = leader_goals
            goals.update(leader_goals)
        if goals:
            self._bus.sync_write(self._goal_item, goals)
            with self._lock:
                self._writes += 1
                if homing is None:
                    self._present.update({k: v for k, v in goals.items() if k in self._by_name})
        if homing is not None:
            # The one time the head is read back: to know it has arrived.
            present = self._read_present()
            with self._lock:
                self._present.update(present)
            if all(
                abs(present.get(m, float("inf")) - u) <= self._home_tolerance
                for m, u in homing.items()
            ):
                self._homing_done.set()
        self._cycles += 1
        elapsed_ms = (time.perf_counter() - started) * 1e3
        self._cycle_ms = (
            elapsed_ms if self._cycles == 1 else 0.9 * self._cycle_ms + 0.1 * elapsed_ms
        )
        if (
            self._threaded
            and not self._slow_warned
            and self._cycles > 20
            and self._cycle_ms > 2.0 * self._period_s * 1e3
        ):
            self._slow_warned = True
            logger.warning(
                "bus cycle takes %.0f ms, more than twice the %.0f ms period: the head will lag. "
                "On Linux set the USB serial latency timer to 1 ms, e.g. "
                "echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB*/latency_timer",
                self._cycle_ms,
                self._period_s * 1e3,
            )

    def _ensure_thread(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="vr-head-bus", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        next_time = time.monotonic()
        while not self._stop.is_set():
            try:
                self.step()
            except Exception:  # noqa: BLE001 - a bus hiccup must not end the loop
                logger.exception("bus cycle failed")
                self._stop.wait(0.1)
            next_time += self._period_s
            delay = next_time - time.monotonic()
            if delay > 0.0:
                self._stop.wait(delay)
            else:
                next_time = time.monotonic()

    def close(self) -> None:
        """Stop the bus thread (the motors keep their last goals)."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(2.0)
            self._thread = None

    def hold(self) -> None:
        """Nothing to do: position-controlled motors hold their last goal."""

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly status."""
        return {
            "name": self.name,
            "mode": "head_only+bilateral"
            if self._system is not None
            else "head_only+leader"
            if self._leader is not None
            else "head_only",
            "units": "deg" if self._degrees else "counts",
            "motors": {
                m.motor: {
                    "urdf_joint": m.urdf_joint,
                    "direction": m.direction,
                    "range_deg": math.degrees(m.range_rad),
                    "start": self._start.get(m.motor),
                    "present": self._present.get(m.motor),
                    "goal": self._goals.get(m.motor),
                }
                for m in self._by_name.values()
            },
            "writes": self._writes,
            "warnings": list(self._last_warnings),
            "home": None if self._home is None else dict(self._home),
            "homed": self._homed,
            "bus_thread": None
            if not self._threaded
            else {
                "running": self._thread is not None and self._thread.is_alive(),
                "rate_hz": 1.0 / self._period_s,
                "cycles": self._cycles,
                "mean_cycle_ms": round(self._cycle_ms, 2),
            },
            "leader": None
            if self._leader is None
            else {
                "follower_motors": sorted(self._leader_map.values()),
                "last_goals": dict(self._leader_goals),
            },
        }
