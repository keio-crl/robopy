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
    "main",
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
        self.results: Dict[str, JointResult] = {m: JointResult(motor=m) for m in self.motors}

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

    def _positions(self) -> Dict[str, int]:
        values = self.bus.sync_read(XControlTable.PRESENT_POSITION, self.motors)
        return {m: int(v) for m, v in values.items()}

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

    def confirm_urdf_joints(self) -> None:
        """Step 2: which model joint each motor drives (proposal by chain order)."""
        self.console.say(
            f"\n[{self.side}] motor -> URDF joint (chain order is a proposal, not a rule)"
        )
        for motor in self.motors:
            proposal = self.urdf_joints.get(motor, "")
            answer = self.console.ask(f"  {motor}: URDF joint", proposal)
            self.results[motor].urdf_joint = answer or None
            self.results[motor].measured.append("urdf_joint")

    def measure_zero(self, pose_description: str) -> None:
        """Step 3: the encoder count at the model's zero pose."""
        self.console.say(f"\n[{self.side}] zero pose")
        self.bus.torque_disabled(self.motors)
        self.console.ask(
            f"  Put the {self.side} in the reference pose: {pose_description}\n  Then press Enter"
        )
        for motor, count in self._positions().items():
            self.results[motor].zero_count = count
            self.results[motor].measured.append("zero_count")
            self.console.say(f"  {motor:16s} zero_count={count}")

    def measure_direction(self, motor: str) -> int | None:
        """Step 4: which way the count runs for the model's positive direction."""
        result = self.results[motor]
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
                result.notes += "direction not measured; "
                return None
            self.console.sleep(0.05)
            polls += 1
            delta = self._position(motor) - start
        result.direction = 1 if delta > 0 else -1
        result.measured.append("direction")
        self.console.say(f"  count {start} -> {start + delta}: direction={result.direction:+d}")
        return result.direction

    def measure_limits(self, motor: str) -> None:
        """Step 5: both ends of the travel, by hand."""
        result = self.results[motor]
        zero = result.zero_count
        if zero is None:
            raise RuntimeError("measure_zero() must run before the limits.")
        caps = self.bus.capabilities(motor)
        rad_per_count = 2.0 * math.pi / caps.counts_per_revolution
        ends: List[float] = []
        for which in ("one end", "the other end"):
            self.console.ask(
                f"  {motor}: move the joint to {which} of its travel, then press Enter"
            )
            count = self._position(motor)
            ends.append(result.direction * (count - zero) * rad_per_count)
            self.console.say(f"    count={count} -> {ends[-1]:+.3f} rad")
        lo, hi = sorted(ends)
        if hi - lo < 1e-6:
            self.console.say("  the two ends coincide; limits NOT recorded")
            result.notes += "limits not measured; "
            return
        result.lower_limit_rad = round(lo, 4)
        result.upper_limit_rad = round(hi, 4)
        result.measured += ["lower_limit_rad", "upper_limit_rad"]

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
        answer = self.console.ask(
            f"  {motor}: measure the torque constant with a known weight? (y/N)", "n"
        )
        if answer.lower() not in ("y", "yes"):
            return None
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
            mass = float(self.console.ask("  mass hung on the link, kg"))
            lever = float(self.console.ask("  lever arm from the joint axis to the mass, m"))
            if mass <= 0.0 or lever <= 0.0:
                raise ValueError("mass and lever arm must be positive.")
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
            result.notes += "torque constant: no current difference; "
            return None
        kt = mass * GRAVITY_M_S2 * lever / delta
        result.torque_constant_nm_per_a = round(kt, 4)
        result.measured.append("torque_constant_nm_per_a")
        result.notes += f"Kt from {mass} kg at {lever} m ({loaded:.3f}-{free:.3f} A); "
        self.console.say(f"    torque constant = {kt:.3f} N m / A")
        return kt

    def finish(self) -> None:
        """Mark complete joints validated and switch torque off."""
        for result in self.results.values():
            result.validated = result.complete_for_hardware
            result.notes = (
                f"measured by robopy-rakuda-calibrate on {datetime.now():%Y-%m-%d}: "
                + ", ".join(dict.fromkeys(result.measured))
                + ("; " + result.notes if result.notes else "")
            )
        self.bus.torque_disabled(self.motors)

    def run(
        self, *, pose_description: str, torque_constants: bool = True
    ) -> Dict[str, JointResult]:
        """All steps in order."""
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
        self.finish()
        return self.results


# ---------------------------------------------------------------- the file


def build_control_section(
    leader: Mapping[str, JointResult],
    follower: Mapping[str, JointResult],
    *,
    existing: Mapping[str, Any] | None = None,
    allow_current: bool = False,
) -> Tuple[Dict[str, Any], List[str]]:
    """The ``control:`` section from both arms' results.

    Args:
        leader: Leader results.
        follower: Follower results.
        existing: The current ``control:`` section, whose other settings
            (gains, periods, model) are kept.
        allow_current: Write ``allow_hardware_current_output: true``; honoured
            only when every coupled joint is complete on both sides.

    Returns:
        The section and a list of notes for the operator.
    """
    notes: List[str] = []
    control: Dict[str, Any] = dict(existing or {})
    control.setdefault("mode", "bilateral_joint")
    control["leader_joint_calibration"] = {m: r.to_yaml() for m, r in leader.items()}
    control["follower_joint_calibration"] = {m: r.to_yaml() for m, r in follower.items()}
    coupled = [
        m
        for m in RAKUDA_IK_MOTOR_NAMES
        if m in leader and m in follower and leader[m].validated and follower[m].validated
    ]
    left_out = [
        m for m in RAKUDA_IK_MOTOR_NAMES if m in leader and m in follower and m not in coupled
    ]
    if left_out:
        notes.append(
            "not coupled (incomplete on at least one side, usually the torque constant): "
            + ", ".join(left_out)
        )
    bilateral: Dict[str, Any] = dict(control.get("bilateral") or {})
    bilateral["coupled_motors"] = coupled
    bilateral["leader_current_limit_a"] = {m: leader[m].current_limit_a for m in coupled}
    bilateral["follower_current_limit_a"] = {m: follower[m].current_limit_a for m in coupled}
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


def self_check(path: Path, models: Mapping[str, Mapping[str, str]]) -> List[str]:
    """Load the written file the normal way and list what still blocks current control.

    Args:
        path: The config file.
        models: ``{"leader"|"follower": {motor: model_name}}`` as probed.
    """
    from robopy.config.dotrobopy import load_yaml, parse_rakuda_control_yaml
    from robopy.control.joint_mapping import JointCalibration, JointMap, ValidationLevel

    data = load_yaml(path) or {}
    control = parse_rakuda_control_yaml(data.get("control"))
    problems: List[str] = []
    if control is None:
        return ["no control section was written"]
    coupled = list(control.bilateral.coupled_motors)
    for side in ("leader", "follower"):
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
        try:
            joint_map.require(
                ValidationLevel.HARDWARE, motor_names=[m for m in coupled if m in joint_map]
            )
        except Exception as exc:  # noqa: BLE001 - reported verbatim
            problems.append(f"{side}: {exc}")
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
        description="Measure, on the machine, the per-joint calibration bilateral control needs, "
        "and write it into .robopy/rakuda/config.yaml.",
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

    from robopy.config.dotrobopy import get_rakuda_yaml_path

    output = args.output or get_rakuda_yaml_path()
    console: Any
    buses: Dict[str, Any]
    arms: Dict[str, Any] = {}
    if args.simulate:
        buses = simulated_buses(motors)
        console = AutoConsole(buses)
    else:
        if not args.leader_port or not args.follower_port:
            parser.error("--leader-port and --follower-port are needed (or --simulate)")
        from robopy.config.dotrobopy import apply_rakuda_dotconfig
        from robopy.config.robot_config.rakuda_config import RakudaConfig
        from robopy.robots.rakuda.rakuda_follower import RakudaFollower
        from robopy.robots.rakuda.rakuda_leader import RakudaLeader

        cfg = apply_rakuda_dotconfig(
            RakudaConfig(leader_port=args.leader_port, follower_port=args.follower_port)
        )
        cfg.leader_port, cfg.follower_port = args.leader_port, args.follower_port
        console = StdConsole()
        console.say(
            "CALIBRATION: torque will be switched OFF on the selected motors of both arms.\n"
            "Support the arms before continuing; they will go limp."
        )
        console.ask("Press Enter to connect")
        arms = {"leader": RakudaLeader(cfg), "follower": RakudaFollower(cfg)}
        for arm in arms.values():
            arm.connect()
        buses = {side: arm.motors for side, arm in arms.items()}
    try:
        results: Dict[str, Dict[str, JointResult]] = {}
        for side in ("leader", "follower"):
            calibrator = ArmCalibrator(
                buses[side],
                side,
                motors,
                console,
                current_fraction=args.current_fraction,
                current_reader=console.current_a if args.simulate else None,
            )
            if args.simulate:
                for motor in motors:
                    console.arm(buses[side], motor, 0.02)  # the auto operator "moves" every joint
            results[side] = calibrator.run(
                pose_description=args.zero_pose, torque_constants=not args.no_torque_constant
            )
        from robopy.config.dotrobopy import load_yaml

        existing = (load_yaml(output) or {}).get("control") if output.is_file() else None
        control, notes = build_control_section(
            results["leader"],
            results["follower"],
            existing=existing,
            allow_current=args.allow_current,
        )
        path = write_config(output, control, coupled=control["bilateral"]["coupled_motors"])
        console.say(f"\nwritten: {path}")
        for note in notes:
            console.say(f"  note: {note}")
        problems = self_check(
            path, {side: {m: r.model for m, r in res.items()} for side, res in results.items()}
        )
        if problems:
            console.say("still blocking bilateral control:")
            for problem in problems:
                console.say(f"  - {problem}")
            return 1
        console.say("the file passes JointMap.require('hardware') for every coupled joint.")
        return 0
    finally:
        for arm in arms.values():
            try:
                arm.disconnect()
            except Exception:  # noqa: BLE001 - best effort
                logger.exception("disconnect failed")


if __name__ == "__main__":
    raise SystemExit(main())
