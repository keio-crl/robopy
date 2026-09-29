"""The real follower's joint angles, read-only, for the viewer to draw.

The viewer is a simulator: nothing on its page talks to a motor.  With
``--mirror-follower PORT`` it also *reads* the follower: the port is opened,
``PRESENT_POSITION`` is read from every calibrated motor at a fixed rate and
turned into URDF joint angles with the configured
``control.follower_joint_calibration`` (``zero_count`` and ``direction``), and
the page can draw the model in that pose.  Nothing is ever written to the bus
-- not the torque, not the operating mode, not a goal -- so the machine keeps
whatever state it is in (limp, or holding) and can be moved by hand to check
that the model follows the right way.

A motor without a URDF joint or a zero is not mirrored (its joint keeps the
page's value) and is listed as such.

The one thing this module writes is the *configuration file*, never the bus:
:func:`set_zero_from_pose` takes "the machine is in this model pose right
now" (the operator posed the model to the machine on the page) and turns the
present counts into new ``zero_count`` values, shifting the recorded travel
and soft limits with them (:func:`robopy.robots.rakuda.calibrate.rezero`).
"""

from __future__ import annotations

import logging
import math
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Callable, Dict, List, Mapping, Sequence, Tuple

from robopy.control.joint_mapping import JointMap
from robopy.motor.dynamixel_control_table import XControlTable

__all__ = ["MachineMirror", "open_follower_mirror", "set_zero_from_pose", "write_travel"]

logger = logging.getLogger(__name__)


class MachineMirror:
    """Poll a bus for positions and keep the latest joint angles.

    Args:
        bus: An open bus (:class:`~robopy.motor.dynamixel_bus.DynamixelBus` or
            the simulated one).  Only ``sync_read`` is ever called on it.
        joint_map: The motors' calibration; motors with a URDF joint and a
            ``zero_count`` are mirrored.
        rate_hz: Polling rate.
        close: Called by :meth:`stop` to release the bus (e.g. close the port).
        clock: Monotonic time source.
        config_path: The ``config.yaml`` the calibration came from; where
            :func:`set_zero_from_pose` writes.  ``None`` disables that.
        side: Which arm's calibration this is.
    """

    def __init__(
        self,
        bus: Any,
        joint_map: JointMap,
        *,
        rate_hz: float = 20.0,
        close: Callable[[], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
        config_path: Path | None = None,
        side: str = "follower",
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive.")
        self.bus = bus
        self.joint_map = joint_map
        self.config_path = None if config_path is None else Path(config_path)
        self.side = side
        self.period_s = 1.0 / rate_hz
        self._close = close
        self._clock = clock
        self.motors: List[str] = []
        self.unmapped: Dict[str, str] = {}
        for name in joint_map.motor_names:
            entry = joint_map[name]
            if entry.urdf_joint is None:
                self.unmapped[name] = "no urdf_joint"
            elif entry.zero_count is None:
                self.unmapped[name] = "no zero_count"
            else:
                self.motors.append(name)
        self._lock = threading.Lock()
        self._joints: Dict[str, float] = {}
        self._counts: Dict[str, int] = {}
        self._stamp: float | None = None
        self._error: str | None = None
        self._missing: List[str] = []
        self._reads = 0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        # Travel recording: the least and greatest angle each motor's joint
        # showed while the operator moved the machine by hand.
        self.recording_travel = False
        self._travel: Dict[str, Tuple[float, float]] = {}

    def read_once(self) -> None:
        """Read every mirrored motor once and keep the result."""
        try:
            counts = self.bus.sync_read(XControlTable.PRESENT_POSITION, self.motors)
        except Exception as exc:  # noqa: BLE001 - reported on the page, polling goes on
            with self._lock:
                self._error = f"{type(exc).__name__}: {exc}"
            return
        joints: Dict[str, float] = {}
        for name in self.motors:
            if name in counts:
                entry = self.joint_map[name]
                assert entry.urdf_joint is not None
                joints[entry.urdf_joint] = entry.count_to_rad(float(counts[name]))
        with self._lock:
            if self.recording_travel:
                for name in self.motors:
                    joint = self.joint_map[name].urdf_joint
                    if joint in joints:
                        lo, hi = self._travel.get(name, (math.inf, -math.inf))
                        self._travel[name] = (min(lo, joints[joint]), max(hi, joints[joint]))
            self._joints = joints
            self._counts = {m: int(counts[m]) for m in self.motors if m in counts}
            self._missing = [m for m in self.motors if m not in counts]
            self._stamp = self._clock()
            self._error = None
            self._reads += 1

    def snapshot(self) -> Dict[str, Any]:
        """The latest reading, JSON-friendly."""
        with self._lock:
            age = None if self._stamp is None else self._clock() - self._stamp
            return {
                "available": True,
                "read_only": True,
                "can_set_zero": self.config_path is not None,
                "joints": dict(self._joints),
                "counts": dict(self._counts),
                "motors": {m: self.joint_map[m].urdf_joint for m in self.motors},
                "unmapped": dict(self.unmapped),
                "missing": list(self._missing),
                "age_s": age,
                "reads": self._reads,
                "error": self._error,
                "travel": {
                    "recording": self.recording_travel,
                    "motors": {
                        m: {"joint": self.joint_map[m].urdf_joint, "min": lo, "max": hi}
                        for m, (lo, hi) in self._travel.items()
                    },
                },
            }

    def travel(self, action: str) -> None:
        """``start`` / ``stop`` / ``reset`` the travel recording."""
        with self._lock:
            if action == "start":
                self.recording_travel = True
            elif action == "stop":
                self.recording_travel = False
            elif action == "reset":
                self._travel = {}
            else:
                raise ValueError("action must be start, stop, reset or write")

    def start(self) -> None:
        """Poll in a background thread until :meth:`stop`."""
        if self._thread is not None:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="machine-mirror", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Stop polling and release the bus."""
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if self._close is not None:
            self._close()
            self._close = None

    def _run(self) -> None:
        while not self._stop.is_set():
            started = self._clock()
            self.read_once()
            self._stop.wait(max(0.0, self.period_s - (self._clock() - started)))


def set_zero_from_pose(
    mirror: MachineMirror,
    joints_rad: Mapping[str, float],
    *,
    bundle: Any = None,
    max_age_s: float = 1.0,
) -> Dict[str, Any]:
    """The machine is in ``joints_rad`` right now: make that its calibration.

    For every mirrored motor whose URDF joint is in ``joints_rad`` the count
    the motor would show at the model's zero is ``count - direction * angle /
    rad_per_count``; that becomes its ``zero_count`` and the recorded travel
    and soft limits move with it (:func:`~robopy.robots.rakuda.calibrate.rezero`).
    The file is rewritten (with a backup), the mirror converts with the new
    zeros at once and, when ``bundle`` is given, its soft limits are updated
    so the page's ranges follow without a restart.  A shifted soft limit that
    no longer overlaps the URDF range is left out of the live model (and
    reported); the file still holds it.

    Args:
        mirror: The follower mirror, with a ``config_path``.
        joints_rad: ``{urdf_joint: rad}`` the model is posed in.
        bundle: The viewer's :class:`~robopy.viewer.model_bundle.ModelBundle`.
        max_age_s: Refuse when the last reading is older than this.

    Raises:
        RuntimeError: Without a config path, or without a fresh reading.
        ValueError: On a joint value that is not a number.
    """
    from robopy.config.dotrobopy import load_yaml
    from robopy.robots.rakuda.calibrate import rezero, write_config

    if mirror.config_path is None:
        raise RuntimeError("the mirror was opened without a configuration file to write")
    snap = mirror.snapshot()
    if snap["error"]:
        raise RuntimeError(f"the machine cannot be read right now: {snap['error']}")
    if snap["age_s"] is None or snap["age_s"] > max_age_s:
        raise RuntimeError("no fresh reading of the machine")
    for name, value in joints_rad.items():
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"joint '{name}' has a non-finite value")
    new_zeros: Dict[str, int] = {}
    rad_per_count: Dict[str, float] = {}
    for motor in mirror.motors:
        entry = mirror.joint_map[motor]
        joint = entry.urdf_joint
        if joint is None or joint not in joints_rad or motor not in snap["counts"]:
            continue
        k = entry.rad_per_count
        rad_per_count[motor] = k
        new_zeros[motor] = int(
            round(snap["counts"][motor] - entry.direction * float(joints_rad[joint]) / k)
        )
    if not new_zeros:
        raise ValueError("none of the given joints is a mirrored motor's")
    path = mirror.config_path
    data = load_yaml(path) or {}
    control = dict(data.get("control") or {})
    control, lines, _ = rezero(control, mirror.side, new_zeros, rad_per_count)
    coupled = list(((control.get("bilateral") or {}).get("coupled_motors")) or [])
    write_config(path, control, coupled=coupled)

    table = control[f"{mirror.side}_joint_calibration"]
    mirror.joint_map = mirror.joint_map.with_updates(
        {
            m: {
                "zero_count": table[m]["zero_count"],
                "lower_limit_rad": table[m].get("lower_limit_rad"),
                "upper_limit_rad": table[m].get("upper_limit_rad"),
            }
            for m in new_zeros
        }
    )
    mirror.read_once()

    ignored: List[str] = []
    if bundle is not None and mirror.side == "follower":
        from .model_bundle import _without_disjoint_soft_limits

        soft_all = (control.get("model") or {}).get("soft_limits_rad") or {}
        touched = {mirror.joint_map[m].urdf_joint for m in new_zeros}
        soft = {j: e for j, e in soft_all.items() if j in touched and isinstance(e, Mapping)}
        usable, ignored = _without_disjoint_soft_limits(bundle.model, soft)
        if usable:
            bundle.model.set_soft_limits(usable)
            for j, e in usable.items():
                bundle.soft_limits[j] = (float(e["lower"]), float(e["upper"]))
    return {
        "written": str(path),
        "zero_counts": dict(new_zeros),
        "lines": lines,
        "ignored_soft_limits": ignored,
    }


def write_travel(
    mirror: MachineMirror,
    *,
    motors: Sequence[str] | None = None,
    margin_rad: float = math.radians(2.0),
    bundle: Any = None,
) -> Dict[str, Any]:
    """Write the recorded travel as the motors' limits and the joints' soft limits.

    For each selected motor the recorded least and greatest angle become its
    ``lower_limit_rad`` / ``upper_limit_rad`` (the ends themselves, as the
    calibration records them), and, through
    :func:`~robopy.robots.rakuda.calibrate.model_soft_limits`, a validated
    ``control.model.soft_limits_rad`` entry on its URDF joint, ``margin_rad``
    inside each end (the ends were found against the stops).  A travel that
    would not overlap the URDF range is not written as a soft limit.

    Args:
        mirror: The follower mirror, with a ``config_path``.
        motors: Motors to write; every recorded one by default.
        margin_rad: Inward margin per end of the soft limits.
        bundle: The viewer's model, to update its ranges live.
    """
    from robopy.config.dotrobopy import load_yaml
    from robopy.robots.rakuda.calibrate import (
        JointResult,
        model_soft_limits,
        urdf_joint_ranges,
        write_config,
    )

    if mirror.config_path is None:
        raise RuntimeError("the mirror was opened without a configuration file to write")
    if margin_rad < 0.0:
        raise ValueError("margin must not be negative")
    snap = mirror.snapshot()
    recorded = snap["travel"]["motors"]
    chosen = list(motors) if motors is not None else list(recorded)
    unknown = [m for m in chosen if m not in recorded]
    if unknown:
        raise ValueError(f"no travel recorded for {unknown}")
    if not chosen:
        raise ValueError("no motor selected")
    today = f"{datetime.now():%Y-%m-%d}"
    path = mirror.config_path
    data = load_yaml(path) or {}
    control = dict(data.get("control") or {})
    key = f"{mirror.side}_joint_calibration"
    table = {m: dict(e) for m, e in (control.get(key) or {}).items()}
    results: Dict[str, JointResult] = {}
    lines: List[str] = []
    for m in chosen:
        entry = table.get(m)
        if entry is None:
            raise ValueError(f"{m} has no calibration entry")
        lo, hi = float(recorded[m]["min"]), float(recorded[m]["max"])
        entry["lower_limit_rad"], entry["upper_limit_rad"] = round(lo, 4), round(hi, 4)
        entry["notes"] = (
            f"{entry.get('notes', '')}; {today}: travel recorded in the viewer while the "
            f"machine was moved by hand [{lo:+.3f}, {hi:+.3f}]"
        )
        results[m] = JointResult(
            motor=m,
            urdf_joint=entry.get("urdf_joint"),
            direction=int(entry.get("direction", 1)),
            zero_count=entry.get("zero_count"),
            lower_limit_rad=round(lo, 4),
            upper_limit_rad=round(hi, 4),
            measured=[
                "urdf_joint",
                "zero_count",
                "direction",
                "lower_limit_rad",
                "upper_limit_rad",
            ],
        )
        lines.append(
            f"{m}: [{math.degrees(lo):+.0f}, {math.degrees(hi):+.0f}] deg "
            f"({math.degrees(hi - lo):.0f} deg of travel)"
        )
    control[key] = table
    notes: List[str] = []
    if mirror.side == "follower":
        ranges = None
        if bundle is not None:
            ranges = urdf_joint_ranges(Path(bundle.urdf_path))
        model, notes = model_soft_limits(
            results,
            existing_model=control.get("model"),
            margin_rad=margin_rad,
            urdf_ranges=ranges,
        )
        control["model"] = model
    coupled = list(((control.get("bilateral") or {}).get("coupled_motors")) or [])
    write_config(path, control, coupled=coupled)

    mirror.joint_map = mirror.joint_map.with_updates(
        {
            m: {
                "lower_limit_rad": table[m]["lower_limit_rad"],
                "upper_limit_rad": table[m]["upper_limit_rad"],
            }
            for m in chosen
        }
    )
    ignored: List[str] = []
    if bundle is not None and mirror.side == "follower":
        from .model_bundle import _without_disjoint_soft_limits

        soft_all = (control.get("model") or {}).get("soft_limits_rad") or {}
        touched = {table[m].get("urdf_joint") for m in chosen}
        soft = {j: e for j, e in soft_all.items() if j in touched and isinstance(e, Mapping)}
        usable, ignored = _without_disjoint_soft_limits(bundle.model, soft)
        if usable:
            bundle.model.set_soft_limits(usable)
            for j, e in usable.items():
                bundle.soft_limits[j] = (float(e["lower"]), float(e["upper"]))
    return {
        "written": str(path),
        "motors": chosen,
        "lines": lines,
        "notes": notes,
        "ignored_soft_limits": ignored,
    }


def open_follower_mirror(
    port: str,
    calibration: Mapping[str, Any],
    *,
    known_urdf_joints: Sequence[str] | None = None,
    rate_hz: float = 20.0,
    config_path: Path | None = None,
) -> MachineMirror:
    """Open the follower's port read-only and mirror its calibrated motors.

    The follower's motor table is used, but the arm is *not* connected
    (connecting initialises the grippers, i.e. writes): only the port is opened.

    Args:
        port: The follower's serial port.
        calibration: ``control.follower_joint_calibration``.
        known_urdf_joints: The model's movable joints, to reject a bad name.
        rate_hz: Polling rate.
        config_path: Where :func:`set_zero_from_pose` may write the new zeros.
    """
    from robopy.config.robot_config.rakuda_config import RakudaConfig
    from robopy.robots.rakuda.rakuda_control import build_joint_map
    from robopy.robots.rakuda.rakuda_follower import RakudaFollower

    arm = RakudaFollower(RakudaConfig(leader_port="", follower_port=port))
    bus = arm.motors
    joint_map = build_joint_map(
        bus.motors, calibration, known_urdf_joints=known_urdf_joints, side="follower"
    )
    bus.open()
    mirror = MachineMirror(
        bus, joint_map, rate_hz=rate_hz, close=bus.close, config_path=config_path
    )
    mirror.read_once()
    return mirror
