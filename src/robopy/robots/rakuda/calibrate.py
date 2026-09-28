"""Measure, on the machine, what bilateral control needs in ``config.yaml``.

``robopy-rakuda-calibrate`` walks the two arms, motor by motor, and fills the
``control:`` section of ``.robopy/rakuda/config.yaml`` with **measured**
values -- the ones :class:`~robopy.control.joint_mapping.JointMap` insists on
before any current is commanded:

* read from the motors themselves: model, ``DRIVE_MODE``, ``HOMING_OFFSET``,
  the configured ``VELOCITY_LIMIT`` (as ``max_velocity_rad_s``) and
  ``CURRENT_LIMIT`` (a fraction of it as ``current_limit_a``);
* measured with the torque off, by moving the joint by hand: ``zero_count``
  at the reference pose, ``direction`` from which way the count runs when the
  joint is moved the way the model calls positive, and the two travel limits;
* measured with a known weight on a known lever arm, joint by joint and only
  where the operator chooses to: ``torque_constant_nm_per_a``, from the
  holding current with and without the load.

Nothing is filled in from a data sheet.  A joint whose torque constant was
not measured keeps ``null`` there, is not marked ``validated`` and is not put
into ``bilateral.coupled_motors``; ``allow_hardware_current_output`` is
written ``true`` only when asked for and only when every coupled joint is
complete on both arms.  The finished file is read back through the normal
loader and checked with ``JointMap.require("hardware")`` before the command
reports success.

``--side follower`` measures the follower alone (only ``--follower-port``):
enough to match the machine to the URDF.  Its entries replace the follower's
in the file, the leader's are kept, and the measured travel is also written
as validated ``control.model.soft_limits_rad`` on the URDF joints -- the range
the solver, the viewer's sliders and the machine adapter all resolve -- after
a table comparing it with the URDF's range.

``--simulate`` runs the whole procedure on simulated buses with an automatic
operator, to see the flow (and for the tests).
"""

from __future__ import annotations

import argparse
import logging
import math
import time
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Protocol, Sequence, Tuple

from robopy.config.robot_config.rakuda_config import (
    RAKUDA_HEAD_MOTOR_NAMES,
    RAKUDA_IK_MOTOR_NAMES,
    RakudaModelConfig,
)
from robopy.motor.dynamixel_control_table import XControlTable

__all__ = [
    "ArmCalibrator",
    "Console",
    "JointResult",
    "MotorReadings",
    "StdConsole",
    "build_control_section",
    "compare_with_urdf",
    "main",
    "model_soft_limits",
    "urdf_joint_ranges",
    "write_config",
]

logger = logging.getLogger(__name__)

#: ``VELOCITY_LIMIT`` / ``PRESENT_VELOCITY`` unit on the X series.
RPM_PER_VELOCITY_COUNT = 0.229
GRAVITY_M_S2 = 9.80665

#: The bundled Rakuda model's joints in chain order (the names the generated
#: config template documents).  A configured ``control.model`` overrides them.
_DEFAULT_MODEL = RakudaModelConfig(
    torso_joint="torso_yaw_dof",
    left_arm_joints=[
        "shoulder_pitch_left_dof",
        "shoulder_roll_left_dof",
        "elbow_yaw_left_dof",
        "elbow_pitch_left_dof",
        "wrist_yaw_left_dof",
        "wrist_pitch_left_dof",
    ],
    right_arm_joints=[
        "shoulder_pitch_right_dof",
        "shoulder_roll_right_dof",
        "elbow_yaw_right_dof",
        "elbow_pitch_right_dof",
        "wrist_yaw_right_dof",
        "wrist_pitch_right_dof",
    ],
)

#: The motors of each arm in chain order (shoulder to wrist).  The proposal
#: ``motor -> URDF joint`` pairs the n-th motor with the n-th model joint of the
#: same arm; the operator confirms or corrects each one, because the names do
#: not correspond by any rule.
_ARM_MOTORS = {
    "left": (
        "l_arm_sh_pitch1",
        "l_arm_sh_roll",
        "l_arm_sh_pitch2",
        "l_arm_el_yaw",
        "l_arm_wr_roll",
        "l_arm_wr_yaw",
    ),
    "right": (
        "r_arm_sh_pitch1",
        "r_arm_sh_roll",
        "r_arm_sh_pitch2",
        "r_arm_el_yaw",
        "r_arm_wr_roll",
        "r_arm_wr_yaw",
    ),
}


def proposed_urdf_joints(model: RakudaModelConfig | None = None) -> Dict[str, str]:
    """The chain-order proposal ``{motor: urdf_joint}`` for the 13 IK motors."""
    m = model if model is not None and model.torso_joint else _DEFAULT_MODEL
    out: Dict[str, str] = {"torso_yaw": str(m.torso_joint)}
    for side, motors in _ARM_MOTORS.items():
        joints = m.left_arm_joints if side == "left" else m.right_arm_joints
        out.update(dict(zip(motors, [str(j) for j in joints])))
    return out


# ---------------------------------------------------------------- console


class Console(Protocol):
    """How the calibrator talks to the operator (replaceable for tests)."""

    def say(self, text: str) -> None:
        """Show a line."""
        ...

    def ask(self, prompt: str, default: str = "") -> str:
        """Ask a question; an empty answer means ``default``."""
        ...

    def sleep(self, seconds: float) -> None:
        """Wait (a test console may move the simulated joints here instead)."""
        ...


class StdConsole:
    """The terminal."""

    def say(self, text: str) -> None:
        print(text, flush=True)

    def ask(self, prompt: str, default: str = "") -> str:
        suffix = f" [{default}]" if default else ""
        answer = input(f"{prompt}{suffix}: ").strip()
        return answer or default

    def sleep(self, seconds: float) -> None:
        time.sleep(seconds)


# ---------------------------------------------------------------- results


@dataclass
class MotorReadings:
    """What a motor says about itself."""

    motor: str
    model: str
    drive_mode: int
    homing_offset: int
    operating_mode: int
    velocity_limit_raw: int
    current_limit_raw: int
    counts_per_revolution: int
    current_unit_a: float

    @property
    def max_velocity_rad_s(self) -> float:
        """The configured velocity limit in rad/s."""
        return self.velocity_limit_raw * RPM_PER_VELOCITY_COUNT * 2.0 * math.pi / 60.0

    @property
    def current_limit_a(self) -> float:
        """The configured current ceiling in amperes."""
        return self.current_limit_raw * self.current_unit_a


@dataclass
class JointResult:
    """One motor's calibration, as it will be written."""

    motor: str
    model: str = ""
    urdf_joint: str | None = None
    direction: int = 1
    zero_count: int | None = None
    drive_mode: int | None = None
    homing_offset: int | None = None
    lower_limit_rad: float | None = None
    upper_limit_rad: float | None = None
    max_velocity_rad_s: float | None = None
    torque_constant_nm_per_a: float | None = None
    current_limit_a: float | None = None
    validated: bool = False
    notes: str = ""
    measured: List[str] = field(default_factory=list)

    @property
    def complete_for_hardware(self) -> bool:
        """Whether every field current control needs is present."""
        return all(
            v is not None
            for v in (
                self.zero_count,
                self.lower_limit_rad,
                self.upper_limit_rad,
                self.max_velocity_rad_s,
                self.torque_constant_nm_per_a,
                self.current_limit_a,
            )
        )

    def to_yaml(self) -> Dict[str, Any]:
        """The ``*_joint_calibration`` entry."""
        d = asdict(self)
        for key in ("motor", "model", "measured"):
            d.pop(key)
        return d


# ---------------------------------------------------------------- one arm


class ArmCalibrator:
    """Calibrate the motors of one bus, with the operator's hands and a weight.

    Args:
        bus: The arm's bus, open.  Torque is switched off on every motor when
            the calibration starts and left off at the end.
        side: ``"leader"`` or ``"follower"``.
        motors: Motors to calibrate.
        console: The operator.
        urdf_joints: ``{motor: urdf_joint}`` proposal to confirm.
        current_fraction: ``current_limit_a`` is this fraction of the motor's
            configured ``CURRENT_LIMIT``.
        move_threshold_counts: Count change that counts as "the joint moved"
            when finding the direction.
        move_timeout_s: How long to wait for that movement.
        urdf_ranges: ``{joint: (type, lower, upper)}`` of the model
            (:func:`urdf_joint_ranges`): joint names typed by the operator are
            checked against it, and the review table compares the travel.
    """

    def __init__(
        self,
        bus: Any,
        side: str,
        motors: Sequence[str],
        console: Console,
        *,
        urdf_joints: Mapping[str, str] | None = None,
        current_fraction: float = 0.5,
        move_threshold_counts: int = 40,
        move_timeout_s: float = 20.0,
        current_reader: Callable[[str], float] | None = None,
        urdf_ranges: Mapping[str, Tuple[str, float | None, float | None]] | None = None,
    ) -> None:
        if side not in ("leader", "follower"):
            raise ValueError("side must be 'leader' or 'follower'.")
        if not 0.0 < current_fraction <= 1.0:
            raise ValueError("current_fraction must be within (0, 1].")
        unknown = [m for m in motors if m not in bus.motors]
        if unknown:
            raise ValueError(f"{side}: motor(s) {unknown} are not on the bus.")
        self.bus = bus
        self.side = side
        self.motors = list(motors)
        self.console = console
        self.urdf_joints = dict(urdf_joints or proposed_urdf_joints())
        self.current_fraction = current_fraction
        self.move_threshold = move_threshold_counts
        self.move_timeout_s = move_timeout_s
        self.current_reader = current_reader
        self.urdf_ranges = dict(urdf_ranges) if urdf_ranges is not None else None
        self.results: Dict[str, JointResult] = {m: JointResult(motor=m) for m in self.motors}
        self.pose_description = ""
        self.torque_constants = True
        # The two ends as raw counts, so a redone zero or direction re-derives
        # the limits without moving the joint to its stops again.
        self._ends: Dict[str, Tuple[int, int]] = {}
        # Notes per step, so redoing a step replaces its note instead of piling up.
        self._notes: Dict[str, Dict[str, str]] = {m: {} for m in self.motors}

    # -- readings -----------------------------------------------------------

    def probe(self, motor: str) -> MotorReadings:
        """Read the motor's identity and configured limits."""
        caps = self.bus.capabilities(motor)
        read = self.bus.read
        return MotorReadings(
            motor=motor,
            model=str(caps.model_name),
            drive_mode=int(read(XControlTable.DRIVE_MODE, motor)),
            homing_offset=int(read(XControlTable.HOMING_OFFSET, motor)),
            operating_mode=int(read(XControlTable.OPERATING_MODE, motor)),
            velocity_limit_raw=int(read(XControlTable.VELOCITY_LIMIT, motor)),
            current_limit_raw=int(read(XControlTable.CURRENT_LIMIT, motor)),
            counts_per_revolution=int(caps.counts_per_revolution),
            current_unit_a=float(caps.current_unit_a),
        )

    def _position(self, motor: str) -> int:
        return int(self.bus.read(XControlTable.PRESENT_POSITION, motor))

    def _positions(self, motors: Sequence[str] | None = None) -> Dict[str, int]:
        values = self.bus.sync_read(XControlTable.PRESENT_POSITION, list(motors or self.motors))
        return {m: int(v) for m, v in values.items()}

    def _mark(self, motor: str, *keys: str) -> None:
        measured = self.results[motor].measured
        measured += [k for k in keys if k not in measured]

    def _forget(self, motor: str, *keys: str) -> None:
        result = self.results[motor]
        result.measured = [k for k in result.measured if k not in keys]

    def _note(self, motor: str, step: str, text: str = "") -> None:
        """Set (or, with no text, clear) the note of one step."""
        notes = self._notes[motor]
        if text:
            notes[step] = text
        else:
            notes.pop(step, None)
        self.results[motor].notes = "".join(notes.values())

    def _ask_positive(self, prompt: str) -> float | None:
        """A positive number; asked again when mistyped, ``None`` on an empty answer."""
        while True:
            answer = self.console.ask(f"{prompt} (Enter alone cancels)").strip()
            if not answer:
                return None
            try:
                value = float(answer)
            except ValueError:
                self.console.say(f"    '{answer}' is not a number; again")
                continue
            if value > 0.0 and math.isfinite(value):
                return value
            self.console.say("    must be positive; again")

    # -- steps --------------------------------------------------------------

    def read_registers(self) -> None:
        """Step 1: what the motors know about themselves."""
        self.console.say(f"\n[{self.side}] reading the motors")
        for motor in self.motors:
            r = self.probe(motor)
            result = self.results[motor]
            result.model = r.model
            result.drive_mode = r.drive_mode
            result.homing_offset = r.homing_offset
            result.max_velocity_rad_s = round(r.max_velocity_rad_s, 4)
            result.current_limit_a = round(r.current_limit_a * self.current_fraction, 4)
            result.measured += ["max_velocity_rad_s", "current_limit_a"]
            self.console.say(
                f"  {motor:16s} {r.model:12s} drive_mode={r.drive_mode} homing={r.homing_offset} "
                f"velocity_limit={r.max_velocity_rad_s:.2f} rad/s "
                f"current_limit={r.current_limit_a:.2f} A -> using {result.current_limit_a:.2f} A"
            )

    def confirm_urdf_joints(self, motors: Sequence[str] | None = None) -> None:
        """Step 2: which model joint each motor drives (proposal by chain order).

        A name the URDF does not have is asked again; the same name typed
        twice in a row is taken as meant.
        """
        self.console.say(
            f"\n[{self.side}] motor -> URDF joint (chain order is a proposal, not a rule)"
        )
        for motor in motors or self.motors:
            result = self.results[motor]
            proposal = result.urdf_joint or self.urdf_joints.get(motor, "")
            answer = self.console.ask(f"  {motor}: URDF joint", proposal)
            while self.urdf_ranges is not None and answer and answer not in self.urdf_ranges:
                self.console.say(
                    f"    '{answer}' is not a movable joint of the URDF; type it again to keep it"
                )
                again = self.console.ask(f"  {motor}: URDF joint", proposal)
                if again == answer:
                    break
                answer = again
            result.urdf_joint = answer or None
            self._mark(motor, "urdf_joint")

    def measure_zero(self, pose_description: str, motors: Sequence[str] | None = None) -> None:
        """Step 3: the encoder count at the model's zero pose.

        Args:
            pose_description: The reference pose, as told to the operator.
            motors: Re-measure only these (the others keep their zero);
                every motor by default.  Travel already measured is
                re-derived from the new zero.
        """
        targets = list(motors or self.motors)
        self.console.say(f"\n[{self.side}] zero pose")
        self.bus.torque_disabled(targets)
        what = f"the {self.side}" if motors is None else ", ".join(targets)
        self.console.ask(
            f"  Put {what} in the reference pose: {pose_description}\n  Then press Enter"
        )
        for motor, count in self._positions(targets).items():
            self.results[motor].zero_count = count
            self._mark(motor, "zero_count")
            self.console.say(f"  {motor:16s} zero_count={count}")
            self._apply_limits(motor)

    def measure_direction(self, motor: str) -> int | None:
        """Step 4: which way the count runs for the model's positive direction."""
        result = self.results[motor]
        result.direction = 1
        self._forget(motor, "direction")
        self._note(motor, "direction")
        joint = result.urdf_joint or "?"
        self.console.say(
            f"\n  {motor} -> {joint}: move the joint by hand in the direction the model calls "
            f"POSITIVE (right-hand rule about its axis), then hold."
        )
        start = self._position(motor)
        deadline = time.monotonic() + self.move_timeout_s
        delta = 0
        polls = 0
        while abs(delta) < self.move_threshold:
            if time.monotonic() > deadline and polls > 0:
                self.console.say("  no movement seen; direction left as +1 (NOT measured)")
                self._note(motor, "direction", "direction not measured; ")
                self._apply_limits(motor)
                return None
            self.console.sleep(0.05)
            polls += 1
            delta = self._position(motor) - start
        result.direction = 1 if delta > 0 else -1
        self._mark(motor, "direction")
        self.console.say(f"  count {start} -> {start + delta}: direction={result.direction:+d}")
        self._apply_limits(motor)
        return result.direction

    def measure_limits(self, motor: str) -> None:
        """Step 5: both ends of the travel, by hand."""
        result = self.results[motor]
        zero = result.zero_count
        if zero is None:
            raise RuntimeError("measure_zero() must run before the limits.")
        rad_per_count = self._rad_per_count(motor)
        counts: List[int] = []
        for which in ("one end", "the other end"):
            self.console.ask(
                f"  {motor}: move the joint to {which} of its travel, then press Enter"
            )
            counts.append(self._position(motor))
            rad = result.direction * (counts[-1] - zero) * rad_per_count
            self.console.say(f"    count={counts[-1]} -> {rad:+.3f} rad")
        self._ends[motor] = (counts[0], counts[1])
        self._apply_limits(motor, announce=False)

    def _rad_per_count(self, motor: str) -> float:
        return 2.0 * math.pi / self.bus.capabilities(motor).counts_per_revolution

    def _apply_limits(self, motor: str, *, announce: bool = True) -> None:
        """The limits from the recorded ends, the zero and the direction."""
        result = self.results[motor]
        ends = self._ends.get(motor)
        if ends is None or result.zero_count is None:
            return
        zero, rad_per_count = result.zero_count, self._rad_per_count(motor)
        lo, hi = sorted(result.direction * (c - zero) * rad_per_count for c in ends)
        result.lower_limit_rad = result.upper_limit_rad = None
        self._forget(motor, "lower_limit_rad", "upper_limit_rad")
        if hi - lo < 1e-6:
            self.console.say("  the two ends coincide; limits NOT recorded")
            self._note(motor, "limits", "limits not measured; ")
            return
        self._note(motor, "limits")
        result.lower_limit_rad = round(lo, 4)
        result.upper_limit_rad = round(hi, 4)
        self._mark(motor, "lower_limit_rad", "upper_limit_rad")
        if announce:
            self.console.say(f"    {motor}: travel now [{lo:+.3f}, {hi:+.3f}] rad")

    def measure_torque_constant(
        self,
        motor: str,
        *,
        samples: int = 20,
        settle_s: float = 1.0,
        current_reader: Callable[[str], float] | None = None,
    ) -> float | None:
        """Step 6 (optional): torque per ampere, from holding a known load.

        The joint is put in current-based position mode and made to hold its
        pose.  Its holding current is averaged without a load, then with a mass
        ``m`` hung at lever arm ``r`` (the joint axis horizontal, the arm
        level), and ``K_t = m g r / |I_load - I_free|``.

        Args:
            motor: The motor.
            samples: Current samples to average per measurement.
            settle_s: Wait after enabling torque before sampling.
            current_reader: Returns the present current in amperes; the bus
                by default.

        Returns:
            The torque constant, or ``None`` when the operator skipped it.
        """
        result = self.results[motor]
        if result.torque_constant_nm_per_a is not None:
            self.console.say(
                f"    recorded now: {result.torque_constant_nm_per_a} N m / A (N keeps it)"
            )
        answer = self.console.ask(
            f"  {motor}: measure the torque constant with a known weight? (y/N)", "n"
        )
        if answer.lower() not in ("y", "yes"):
            return None
        result.torque_constant_nm_per_a = None
        self._forget(motor, "torque_constant_nm_per_a")
        self._note(motor, "kt")
        caps = self.bus.capabilities(motor)
        unit = float(caps.current_unit_a)

        reader = current_reader or self.current_reader

        def read_current() -> float:
            if reader is not None:
                return reader(motor)
            return float(self.bus.read(XControlTable.PRESENT_CURRENT, motor)) * unit

        def average() -> float:
            total = 0.0
            for _ in range(samples):
                total += abs(read_current())
                self.console.sleep(0.02)
            return total / samples

        self.console.ask(
            f"  Pose {motor} so its axis is horizontal and the link is level; support it; "
            "press Enter"
        )
        pose = self._position(motor)
        self.bus.torque_disabled([motor])
        self.bus.write(XControlTable.OPERATING_MODE, motor, 5)  # current-based position
        self.bus.write(XControlTable.GOAL_POSITION, motor, pose)
        self.bus.torque_enabled([motor])
        try:
            self.console.sleep(settle_s)
            free = average()
            self.console.say(f"    holding current without load: {free:.3f} A")
            mass = self._ask_positive("  mass hung on the link, kg")
            lever = (
                None
                if mass is None
                else self._ask_positive("  lever arm from the joint axis to the mass, m")
            )
            if mass is None or lever is None:
                self.console.say("  cancelled; torque constant NOT recorded")
                return None
            self.console.ask("  hang the mass, let it settle, then press Enter")
            self.console.sleep(settle_s)
            loaded = average()
            self.console.say(f"    holding current with load:    {loaded:.3f} A")
        finally:
            self.bus.torque_disabled([motor])
            self.bus.write(XControlTable.OPERATING_MODE, motor, 3)  # back to position mode
        delta = abs(loaded - free)
        if delta < 1e-4:
            self.console.say("  no current difference seen; torque constant NOT recorded")
            self._note(motor, "kt", "torque constant: no current difference; ")
            return None
        kt = mass * GRAVITY_M_S2 * lever / delta
        result.torque_constant_nm_per_a = round(kt, 4)
        self._mark(motor, "torque_constant_nm_per_a")
        self._note(motor, "kt", f"Kt from {mass} kg at {lever} m ({loaded:.3f}-{free:.3f} A); ")
        self.console.say(f"    torque constant = {kt:.3f} N m / A")
        return kt

    # -- redoing -------------------------------------------------------------

    def _redo_letters(self) -> str:
        return "uzdl" + ("k" if self.torque_constants else "")

    def _redo_help(self) -> str:
        kt = ", k Kt" if self.torque_constants else ""
        return f"u URDF joint, z zero, d direction, l travel{kt}, r = d+l{'+k' if kt else ''}"

    def redo(self, motor: str, letters: str) -> bool:
        """Measure one joint's steps again, named by letter (see :meth:`_redo_help`).

        The steps run in the procedure's order whatever order they are typed
        in.  Returns ``False`` (and does nothing) on an unknown letter.
        """
        letters = letters.lower().replace("r", "dl" + ("k" if self.torque_constants else ""))
        unknown = sorted(set(letters) - set(self._redo_letters()))
        if unknown:
            self.console.say(f"    unknown step(s) {''.join(unknown)!r}: {self._redo_help()}")
            return False
        if "u" in letters:
            self.confirm_urdf_joints([motor])
        if "z" in letters:
            self.measure_zero(self.pose_description, [motor])
        if "d" in letters:
            self.measure_direction(motor)
        if "l" in letters:
            self.measure_limits(motor)
        if "k" in letters:
            self.measure_torque_constant(motor)
        return True

    def summary_lines(self) -> List[str]:
        """One numbered line per motor with what is recorded (and the URDF check)."""
        ranges = self.urdf_ranges
        lines = [
            f"  {'#':>2s} {'motor':16s} {'URDF joint':26s} {'dir':>3s} {'zero':>5s} "
            f"{'travel (rad)':>17s}"
            + (f" {'Kt':>7s}" if self.torque_constants else "")
            + ("  check" if ranges is not None else "")
        ]
        for index, (motor, r) in enumerate(self.results.items(), start=1):
            direction = f"{r.direction:+d}" if "direction" in r.measured else "?"
            zero = "?" if r.zero_count is None else str(r.zero_count)
            travel = (
                "not measured"
                if r.lower_limit_rad is None or r.upper_limit_rad is None
                else f"[{r.lower_limit_rad:+.3f}, {r.upper_limit_rad:+.3f}]"
            )
            line = (
                f"  {index:>2d} {motor:16s} {r.urdf_joint or '?':26s} {direction:>3s} {zero:>5s} "
                f"{travel:>17s}"
            )
            if self.torque_constants:
                kt = r.torque_constant_nm_per_a
                line += f" {'-' if kt is None else f'{kt:.3f}':>7s}"
            if ranges is not None:
                line += "  " + ("; ".join(_urdf_check(r, ranges)[1]) or "ok")
            lines.append(line)
        return lines

    def review(self) -> None:
        """Show everything recorded and redo what the operator names, until Enter."""
        while True:
            self.console.say(f"\n[{self.side}] review before writing")
            for line in self.summary_lines():
                self.console.say(line)
            answer = self.console.ask(
                "  redo: '<#|motor> <steps>' (" + self._redo_help() + "), "
                "'zero' for the whole reference pose; Enter = write"
            ).strip()
            if not answer:
                return
            words = answer.split()
            if words[0].lower() == "zero" and len(words) == 1:
                self.measure_zero(self.pose_description)
                continue
            motor = self._motor_named(words[0])
            if motor is None or len(words) > 2:
                self.console.say(f"    not understood: {answer!r} (e.g. '3 dl', 'torso_yaw z')")
                continue
            self.redo(motor, words[1] if len(words) == 2 else "r")

    def _motor_named(self, word: str) -> str | None:
        if word.isdigit() and 1 <= int(word) <= len(self.motors):
            return self.motors[int(word) - 1]
        return word if word in self.results else None

    def finish(self) -> None:
        """Mark complete joints validated and switch torque off."""
        for motor, result in self.results.items():
            result.validated = result.complete_for_hardware
            notes = "".join(self._notes[motor].values())
            result.notes = (
                f"measured by robopy-rakuda-calibrate on {datetime.now():%Y-%m-%d}: "
                + ", ".join(dict.fromkeys(result.measured))
                + ("; " + notes if notes else "")
            )
        self.bus.torque_disabled(self.motors)

    def run(
        self, *, pose_description: str, torque_constants: bool = True, review: bool = True
    ) -> Dict[str, JointResult]:
        """All steps in order.

        After each joint the operator may redo any of its steps; with
        ``review`` every result is shown once more before anything is
        returned, and any joint (or the whole zero pose) can be redone.
        """
        self.pose_description = pose_description
        self.torque_constants = torque_constants
        self.console.say(
            f"\n=== {self.side.upper()} ===  torque OFF on {len(self.motors)} motors; "
            "support the arms"
        )
        self.bus.torque_disabled(self.motors)
        self.read_registers()
        self.confirm_urdf_joints()
        self.measure_zero(pose_description)
        for motor in self.motors:
            self.measure_direction(motor)
            self.measure_limits(motor)
            if torque_constants:
                self.measure_torque_constant(motor)
            while True:
                answer = self.console.ask(
                    f"  {motor}: Enter = next joint, or redo ({self._redo_help()})"
                ).strip()
                if not answer:
                    break
                self.redo(motor, answer)
        if review:
            self.review()
        self.finish()
        return self.results


# ---------------------------------------------------------------- the file


def build_control_section(
    leader: Mapping[str, JointResult] | None,
    follower: Mapping[str, JointResult] | None,
    *,
    existing: Mapping[str, Any] | None = None,
    allow_current: bool = False,
) -> Tuple[Dict[str, Any], List[str]]:
    """The ``control:`` section from the arms' results.

    Args:
        leader: Leader results, or ``None`` when the leader was not measured
            this time (its entries in ``existing`` are kept as they are).
        follower: Follower results, or ``None`` likewise.
        existing: The current ``control:`` section, whose other settings
            (gains, periods, model) are kept.  Motors measured now replace
            their entries; the others are kept.
        allow_current: Write ``allow_hardware_current_output: true``; honoured
            only when every coupled joint is complete on both sides.

    Returns:
        The section and a list of notes for the operator.
    """
    notes: List[str] = []
    control: Dict[str, Any] = dict(existing or {})
    tables: Dict[str, Dict[str, Any]] = {}
    for side, results in (("leader", leader), ("follower", follower)):
        key = f"{side}_joint_calibration"
        table = dict(control.get(key) or {})
        if results is not None:
            table.update({m: r.to_yaml() for m, r in results.items()})
        control[key] = table
        tables[side] = table

    def validated(side: str, motor: str) -> bool:
        entry = tables[side].get(motor)
        return isinstance(entry, Mapping) and bool(entry.get("validated"))

    both = [m for m in RAKUDA_IK_MOTOR_NAMES if m in tables["leader"] and m in tables["follower"]]
    coupled = [m for m in both if validated("leader", m) and validated("follower", m)]
    left_out = [m for m in both if m not in coupled]
    # bilateral_joint without a coupled joint does not load; a follower-only
    # calibration is for position control and the kinematic model.
    control.setdefault("mode", "bilateral_joint" if coupled else "position_teleop")
    if left_out:
        notes.append(
            "not coupled (incomplete on at least one side, usually the torque constant): "
            + ", ".join(left_out)
        )
    bilateral: Dict[str, Any] = dict(control.get("bilateral") or {})
    bilateral["coupled_motors"] = coupled
    for side in ("leader", "follower"):
        bilateral[f"{side}_current_limit_a"] = {
            m: tables[side][m].get("current_limit_a") for m in coupled
        }
    for key, value in (
        ("stiffness_nm_per_rad", 1.0),
        ("damping_nm_s_per_rad", 0.05),
        ("max_torque_nm", 0.3),
        ("max_torque_rate_nm_s", 10.0),
        ("velocity_filter_hz", 20.0),
        ("ramp_time_s", 1.0),
        ("max_alignment_error_rad", 0.1),
        ("allow_uncompensated", False),
    ):
        bilateral.setdefault(key, value)
    control["bilateral"] = bilateral
    complete = bool(coupled)
    control["allow_hardware_current_output"] = bool(allow_current and complete)
    if leader is None or follower is None:
        return control, notes  # one arm only: the coupling notes are not about this run
    if allow_current and not complete:
        notes.append("allow_hardware_current_output left false: no joint is complete on both arms")
    if not allow_current:
        notes.append(
            "allow_hardware_current_output left false (pass --allow-current once the values are "
            "checked)"
        )
    if coupled and not bilateral.get("allow_uncompensated"):
        notes.append(
            "bilateral.allow_uncompensated is false: current control also needs a validated "
            "gravity model per arm, or set it true knowingly for joints gravity does not load"
        )
    return control, notes


def model_soft_limits(
    follower: Mapping[str, JointResult],
    *,
    existing_model: Mapping[str, Any] | None = None,
    margin_rad: float = 0.0,
) -> Tuple[Dict[str, Any], List[str]]:
    """``control.model`` with the follower's measured travel as soft limits.

    The joint map's limits guard only the machine adapter; the solver, the
    viewer's sliders and the home-pose check read the URDF range narrowed by
    ``control.model.soft_limits_rad``.  Each joint whose zero, direction and
    both ends were measured gets its travel, shrunk by ``margin_rad`` on each
    side (the ends were found against the hard stops), as a *validated* soft
    limit on its URDF joint.  A soft limit only narrows: where the machine
    travels further than the URDF allows, the URDF range still stands.

    Args:
        follower: The follower's results.
        existing_model: The current ``control.model`` section; its other
            entries, and soft limits of joints not measured now, are kept.
        margin_rad: Inward margin on each end.

    Returns:
        The model section and notes for the operator.
    """
    notes: List[str] = []
    model: Dict[str, Any] = dict(existing_model or {})
    soft: Dict[str, Any] = dict(model.get("soft_limits_rad") or {})
    for motor, r in follower.items():
        needed = ("zero_count", "direction", "lower_limit_rad", "upper_limit_rad", "urdf_joint")
        missing = [k for k in needed if k not in r.measured]
        if missing or r.urdf_joint is None:
            notes.append(f"{motor}: no soft limit written ({', '.join(missing)} not measured)")
            continue
        assert r.lower_limit_rad is not None and r.upper_limit_rad is not None
        lower, upper = r.lower_limit_rad + margin_rad, r.upper_limit_rad - margin_rad
        if lower >= upper:
            notes.append(f"{motor}: travel narrower than twice the margin; no soft limit written")
            continue
        soft[r.urdf_joint] = {
            "lower": round(lower, 4),
            "upper": round(upper, 4),
            "validated": True,
            "note": f"follower {motor} travel measured by robopy-rakuda-calibrate on "
            f"{datetime.now():%Y-%m-%d}, {math.degrees(margin_rad):.1f} deg margin per end",
        }
    model["soft_limits_rad"] = soft
    return model, notes


def urdf_joint_ranges(urdf_path: Path) -> Dict[str, Tuple[str, float | None, float | None]]:
    """``{joint: (type, lower, upper)}`` of the movable joints in a URDF."""
    import xml.etree.ElementTree as ET

    out: Dict[str, Tuple[str, float | None, float | None]] = {}
    for joint in ET.parse(urdf_path).getroot().iter("joint"):
        kind = joint.get("type", "")
        if kind in ("fixed", "floating", "planar"):
            continue
        limit = joint.find("limit")
        lower = upper = None
        if kind != "continuous" and limit is not None:
            lower = float(limit.get("lower", "0"))
            upper = float(limit.get("upper", "0"))
        out[str(joint.get("name"))] = (kind, lower, upper)
    return out


def compare_with_urdf(
    follower: Mapping[str, JointResult],
    ranges: Mapping[str, Tuple[str, float | None, float | None]],
    *,
    tolerance_rad: float = 0.05,
) -> List[str]:
    """A table of the machine's travel against the URDF's range, joint by joint.

    Flags a zero pose outside the measured travel (the reference pose or the
    direction is probably wrong), a URDF range wider than the machine (the
    soft limit narrows it) and a machine travelling further than the URDF
    allows (the URDF stays binding; an override with its reason is the only
    way to widen it).
    """
    lines = [
        f"  {'motor':16s} {'URDF joint':26s} {'URDF range':>17s}   {'machine range':>17s}  check"
    ]
    for motor, r in follower.items():
        lo_m, hi_m = r.lower_limit_rad, r.upper_limit_rad
        urdf_text, flags = _urdf_check(r, ranges, tolerance_rad=tolerance_rad)
        machine_text = (
            "not measured" if lo_m is None or hi_m is None else f"[{lo_m:+.3f}, {hi_m:+.3f}]"
        )
        lines.append(
            f"  {motor:16s} {r.urdf_joint or '?':26s} {urdf_text:>17s}   {machine_text:>17s}  "
            + ("; ".join(flags) or "ok")
        )
    return lines


def _urdf_check(
    r: JointResult,
    ranges: Mapping[str, Tuple[str, float | None, float | None]],
    *,
    tolerance_rad: float = 0.05,
) -> Tuple[str, List[str]]:
    """The URDF range of ``r``'s joint as text, and what disagrees with the travel."""
    kind, lo_u, hi_u = ranges.get(r.urdf_joint or "?", ("missing", None, None))
    lo_m, hi_m = r.lower_limit_rad, r.upper_limit_rad
    urdf_text = (
        "continuous"
        if kind == "continuous"
        else ("NOT IN URDF" if kind == "missing" else f"[{lo_u:+.3f}, {hi_u:+.3f}]")
    )
    flags: List[str] = []
    if kind == "missing":
        flags.append("unknown joint name")
    if lo_m is not None and hi_m is not None:
        if lo_m > tolerance_rad or hi_m < -tolerance_rad:
            flags.append("zero pose outside the travel: check the pose and the direction")
        if lo_u is not None and hi_u is not None:
            if lo_m < lo_u - tolerance_rad or hi_m > hi_u + tolerance_rad:
                flags.append("machine goes beyond URDF (URDF stays binding)")
            if lo_m > lo_u + tolerance_rad or hi_m < hi_u - tolerance_rad:
                flags.append("URDF wider than machine (soft limit narrows)")
        elif kind == "continuous":
            flags.append("URDF has no range (soft limit supplies it)")
    return urdf_text, flags


def write_config(
    path: Path,
    control: Mapping[str, Any],
    *,
    coupled: Sequence[str],
    keep_backup: bool = True,
) -> Path:
    """Write ``control:`` into the config file, keeping its other sections.

    The torque-enable lists are widened to the coupled motors, which the
    loader requires.  Comments in the existing file are lost (YAML is
    re-serialised); a timestamped backup is kept.
    """
    import yaml

    from robopy.config.dotrobopy import load_yaml

    path = Path(path)
    data: Dict[str, Any] = {}
    if path.is_file():
        data = dict(load_yaml(path) or {})
        if keep_backup:
            backup = path.with_name(f"{path.name}.bak-{datetime.now():%Y%m%d-%H%M%S}")
            backup.write_text(path.read_text(encoding="utf-8"), encoding="utf-8")
    for side in ("leader", "follower"):
        section = dict(data.get(side) or {})
        enabled = section.get("torque_enabled")
        if isinstance(enabled, list):
            section["torque_enabled"] = list(dict.fromkeys([*enabled, *coupled]))
        elif side == "leader" and coupled:
            # The leader's default is grippers only; the coupled joints need torque.
            section["torque_enabled"] = list(dict.fromkeys(["l_arm_grip", "r_arm_grip", *coupled]))
        data[side] = section
    data["control"] = dict(control)
    header = (
        f"# Written by robopy-rakuda-calibrate on {datetime.now():%Y-%m-%d %H:%M}.\n"
        "# Every number in control.*_joint_calibration was measured on this machine;\n"
        "# see the per-joint notes.  Nothing here comes from a data sheet.\n"
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        header + yaml.safe_dump(data, sort_keys=False, allow_unicode=True), encoding="utf-8"
    )
    return path


def self_check(
    path: Path,
    models: Mapping[str, Mapping[str, str]],
    *,
    sides: Sequence[str] = ("leader", "follower"),
) -> List[str]:
    """Load the written file the normal way and list what still blocks its use.

    With both sides, what blocks current control (``JointMap.require("hardware")``
    on the coupled joints).  With one side, what blocks position control and
    the kinematic model: ``require("geometry")`` on that side's motors in
    ``models``.

    Args:
        path: The config file.
        models: ``{"leader"|"follower": {motor: model_name}}`` as probed.
        sides: The sides measured this time.
    """
    from robopy.config.dotrobopy import load_yaml, parse_rakuda_control_yaml
    from robopy.control.joint_mapping import JointCalibration, JointMap, ValidationLevel

    data = load_yaml(path) or {}
    control = parse_rakuda_control_yaml(data.get("control"))
    problems: List[str] = []
    if control is None:
        return ["no control section was written"]
    coupled = list(control.bilateral.coupled_motors)
    single = len(sides) == 1
    for side in sides:
        specs = getattr(control, f"{side}_joint_calibration")
        entries = [
            JointCalibration(
                motor_name=name,
                motor_id=index + 1,
                model=models.get(side, {}).get(name, "xm430-w350"),
                urdf_joint=spec.urdf_joint,
                direction=spec.direction,
                zero_count=spec.zero_count,
                lower_limit_rad=spec.lower_limit_rad,
                upper_limit_rad=spec.upper_limit_rad,
                max_velocity_rad_s=spec.max_velocity_rad_s,
                torque_constant_nm_per_a=spec.torque_constant_nm_per_a,
                current_limit_a=spec.current_limit_a,
                validated=spec.validated,
            )
            for index, (name, spec) in enumerate(specs.items())
        ]
        if not entries:
            problems.append(f"{side}: no calibration written")
            continue
        joint_map = JointMap(entries)
        level, wanted = ValidationLevel.HARDWARE, [m for m in coupled if m in joint_map]
        if single:
            level, wanted = ValidationLevel.GEOMETRY, [m for m in models.get(side, {})]
        try:
            joint_map.require(level, motor_names=wanted)
        except Exception as exc:  # noqa: BLE001 - reported verbatim
            problems.append(f"{side}: {exc}")
    if single:
        return problems
    if not coupled:
        problems.append("bilateral.coupled_motors is empty: no joint is complete on both arms")
    if not control.allow_hardware_current_output:
        problems.append("allow_hardware_current_output is false")
    return problems


# ---------------------------------------------------------------- simulation


class AutoConsole:
    """An operator that answers everything and moves the simulated joints."""

    def __init__(self, buses: Mapping[str, Any], *, say: Callable[[str], None] = print) -> None:
        self._buses = buses
        self._say = say
        self._asked = 0
        self.loaded = False  # whether the pretend weight hangs on the link
        self.transcript: List[str] = []

    def current_a(self, motor: str) -> float:
        """A pretend holding current: 0.10 A free, 0.35 A with the weight."""
        return 0.35 if self.loaded else 0.10

    def say(self, text: str) -> None:
        self.transcript.append(text)
        self._say(text)

    def ask(self, prompt: str, default: str = "") -> str:
        self._asked += 1
        self.transcript.append(prompt)
        self._say(f"{prompt} -> auto")
        if "torque constant" in prompt:
            return "y"
        if "Pose " in prompt:
            self.loaded = False
        if "hang the mass" in prompt:
            self.loaded = True
        if "mass hung" in prompt:
            return "0.5"
        if "lever arm" in prompt:
            return "0.2"
        if "one end" in prompt or "other end" in prompt:
            self._nudge(prompt, +0.8 if "one end" in prompt else -0.8)
        return default

    def sleep(self, seconds: float) -> None:
        # Instead of waiting: "move" the joint the direction step is polling.
        for bus in self._buses.values():
            for name in bus.motors:
                joint = bus.joint(name)
                if getattr(joint, "_auto_nudge", 0.0):
                    joint.position_rad += joint._auto_nudge

    def arm(self, bus: Any, motor: str, per_poll_rad: float) -> None:
        """Make the direction poll see this motor moving."""
        bus.joint(motor)._auto_nudge = per_poll_rad

    def _nudge(self, prompt: str, rad: float) -> None:
        for bus in self._buses.values():
            for name in bus.motors:
                if f"  {name}:" in prompt:
                    bus.joint(name).position_rad = rad


def simulated_buses(motors: Sequence[str]) -> Dict[str, Any]:
    """Leader and follower buses that exist only in memory."""
    from robopy.motor.dynamixel_bus import DynamixelMotor
    from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint

    out = {}
    for side, model in (("leader", "xc330-t288"), ("follower", "xm430-w350")):
        motor_objs = {n: DynamixelMotor(i + 1, n, model) for i, n in enumerate(motors)}
        out[side] = SimulatedDynamixelBus(
            motor_objs, joints={n: SimulatedJoint() for n in motors}, auto_step=False
        )
    return out


# ---------------------------------------------------------------- command


def main(argv: Sequence[str] | None = None) -> int:
    """``robopy-rakuda-calibrate``."""
    parser = argparse.ArgumentParser(
        prog="robopy-rakuda-calibrate",
        description="Measure, on the machine, the per-joint calibration (motor <-> URDF joint, "
        "zero, direction, travel; and for bilateral control the torque constant) and write it "
        "into .robopy/rakuda/config.yaml.",
    )
    parser.add_argument(
        "--side",
        choices=("both", "leader", "follower"),
        default="both",
        help="which arm(s) to measure; 'follower' needs only --follower-port and is enough to "
        "match the machine to the URDF (default: both, for bilateral control)",
    )
    parser.add_argument("--leader-port", default=None)
    parser.add_argument("--follower-port", default=None)
    parser.add_argument(
        "--motors",
        default=",".join(RAKUDA_IK_MOTOR_NAMES),
        help="motors to calibrate (default: the 13 torso + arm motors)",
    )
    parser.add_argument(
        "--output", type=Path, default=None, help="config file (default .robopy/rakuda/config.yaml)"
    )
    parser.add_argument(
        "--urdf",
        type=Path,
        default=None,
        help="URDF to compare the follower's travel with (default: control.model.urdf_path, "
        "else the bundled model)",
    )
    parser.add_argument(
        "--no-soft-limits",
        action="store_true",
        help="do not write the follower's measured travel into control.model.soft_limits_rad",
    )
    parser.add_argument(
        "--limit-margin-deg",
        type=float,
        default=2.0,
        help="inward margin per end of the soft limits, since the ends are the hard stops",
    )
    parser.add_argument(
        "--current-fraction",
        type=float,
        default=0.5,
        help="current_limit_a as a fraction of each motor's configured CURRENT_LIMIT",
    )
    parser.add_argument(
        "--no-torque-constant",
        action="store_true",
        help="skip the weight measurement (the joints then stay uncoupled)",
    )
    parser.add_argument(
        "--allow-current",
        action="store_true",
        help="write allow_hardware_current_output: true when every coupled joint is complete",
    )
    parser.add_argument(
        "--zero-pose",
        default="the model's zero: torso facing forward, both arms hanging straight down along "
        "the body, wrists neutral (see the URDF's zero configuration in the viewer)",
        help="how to describe the reference pose to the operator",
    )
    parser.add_argument(
        "--simulate", action="store_true", help="simulated buses, automatic operator"
    )
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO)

    motors = [m.strip() for m in args.motors.split(",") if m.strip()]
    bad = [m for m in motors if m in RAKUDA_HEAD_MOTOR_NAMES or m.endswith("_grip")]
    if bad:
        parser.error(f"head and gripper motors are never coupled: {bad}")
    if args.limit_margin_deg < 0.0:
        parser.error("--limit-margin-deg must not be negative")
    sides = ("leader", "follower") if args.side == "both" else (args.side,)

    from robopy.config.dotrobopy import get_rakuda_yaml_path, load_yaml

    output = args.output or get_rakuda_yaml_path()
    existing = (load_yaml(output) or {}).get("control") if output.is_file() else None
    console: Any
    buses: Dict[str, Any]
    arms: Dict[str, Any] = {}
    if args.simulate:
        buses = simulated_buses(motors)
        console = AutoConsole(buses)
    else:
        missing = [f"--{side}-port" for side in sides if not getattr(args, f"{side}_port")]
        if missing:
            parser.error(f"{' and '.join(missing)} needed for --side {args.side} (or --simulate)")
        from robopy.config.dotrobopy import apply_rakuda_dotconfig
        from robopy.config.robot_config.rakuda_config import RakudaConfig

        leader_port, follower_port = args.leader_port or "", args.follower_port or ""
        cfg = apply_rakuda_dotconfig(
            RakudaConfig(leader_port=leader_port, follower_port=follower_port)
        )
        cfg.leader_port, cfg.follower_port = leader_port, follower_port
        console = StdConsole()
        console.say(
            f"CALIBRATION: torque will be switched OFF on the selected motors of the "
            f"{' and '.join(sides)}.\nSupport the arms before continuing; they will go limp."
        )
        console.ask("Press Enter to connect")
        if "leader" in sides:
            from robopy.robots.rakuda.rakuda_leader import RakudaLeader

            arms["leader"] = RakudaLeader(cfg)
        if "follower" in sides:
            from robopy.robots.rakuda.rakuda_follower import RakudaFollower

            arms["follower"] = RakudaFollower(cfg)
        for arm in arms.values():
            arm.connect()
        buses = {side: arm.motors for side, arm in arms.items()}
    urdf = args.urdf or _configured_urdf(existing or {})
    ranges = urdf_joint_ranges(urdf) if urdf is not None and urdf.is_file() else None
    try:
        results: Dict[str, Dict[str, JointResult]] = {}
        for side in sides:
            calibrator = ArmCalibrator(
                buses[side],
                side,
                motors,
                console,
                current_fraction=args.current_fraction,
                current_reader=console.current_a if args.simulate else None,
                urdf_ranges=ranges,
            )
            if args.simulate:
                for motor in motors:
                    console.arm(buses[side], motor, 0.02)  # the auto operator "moves" every joint
            results[side] = calibrator.run(
                pose_description=args.zero_pose, torque_constants=not args.no_torque_constant
            )
        control, notes = build_control_section(
            results.get("leader"),
            results.get("follower"),
            existing=existing,
            allow_current=args.allow_current,
        )
        follower = results.get("follower")
        if follower is not None:
            if ranges is not None:
                console.say(f"\nfollower travel against {urdf} (rad):")
                for line in compare_with_urdf(follower, ranges):
                    console.say(line)
            else:
                console.say("\nno URDF found to compare the follower's travel with")
            if not args.no_soft_limits:
                model, soft_notes = model_soft_limits(
                    follower,
                    existing_model=control.get("model"),
                    margin_rad=math.radians(args.limit_margin_deg),
                )
                control["model"] = model
                notes += soft_notes
                notes.append(
                    "control.model.soft_limits_rad now holds the follower's measured travel; "
                    "the viewer (--config), the IK and the machine read it"
                )
        path = write_config(output, control, coupled=control["bilateral"]["coupled_motors"])
        console.say(f"\nwritten: {path}")
        for note in notes:
            console.say(f"  note: {note}")
        problems = self_check(
            path,
            {side: {m: r.model for m, r in res.items()} for side, res in results.items()},
            sides=sides,
        )
        if problems:
            console.say(
                "still blocking:" if len(sides) == 1 else "still blocking bilateral control:"
            )
            for problem in problems:
                console.say(f"  - {problem}")
            return 1
        if len(sides) == 1:
            console.say(
                f"the {sides[0]}'s calibration passes JointMap.require('geometry') "
                "(position control and the kinematic model)."
            )
        else:
            console.say("the file passes JointMap.require('hardware') for every coupled joint.")
        return 0
    finally:
        for arm in arms.values():
            try:
                arm.disconnect()
            except Exception:  # noqa: BLE001 - best effort
                logger.exception("disconnect failed")


def _configured_urdf(control: Mapping[str, Any]) -> Path | None:
    """The URDF the rest of robopy would load: the configured one, else the bundled one."""
    configured = (control.get("model") or {}).get("urdf_path")
    if configured:
        return Path(configured)
    from robopy.models import find_rakuda_model

    found = find_rakuda_model()
    return Path(found.convex_collision_urdf) if found is not None else None


if __name__ == "__main__":
    raise SystemExit(main())
