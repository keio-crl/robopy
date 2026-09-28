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
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Callable, Dict, List, Mapping, Sequence

from robopy.control.joint_mapping import JointMap
from robopy.motor.dynamixel_control_table import XControlTable

__all__ = ["MachineMirror", "open_follower_mirror"]

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
    """

    def __init__(
        self,
        bus: Any,
        joint_map: JointMap,
        *,
        rate_hz: float = 20.0,
        close: Callable[[], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        if rate_hz <= 0.0:
            raise ValueError("rate_hz must be positive.")
        self.bus = bus
        self.joint_map = joint_map
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
                "joints": dict(self._joints),
                "counts": dict(self._counts),
                "motors": {m: self.joint_map[m].urdf_joint for m in self.motors},
                "unmapped": dict(self.unmapped),
                "missing": list(self._missing),
                "age_s": age,
                "reads": self._reads,
                "error": self._error,
            }

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


def open_follower_mirror(
    port: str,
    calibration: Mapping[str, Any],
    *,
    known_urdf_joints: Sequence[str] | None = None,
    rate_hz: float = 20.0,
) -> MachineMirror:
    """Open the follower's port read-only and mirror its calibrated motors.

    The follower's motor table is used, but the arm is *not* connected
    (connecting initialises the grippers, i.e. writes): only the port is opened.

    Args:
        port: The follower's serial port.
        calibration: ``control.follower_joint_calibration``.
        known_urdf_joints: The model's movable joints, to reject a bad name.
        rate_hz: Polling rate.
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
    mirror = MachineMirror(bus, joint_map, rate_hz=rate_hz, close=bus.close)
    mirror.read_once()
    return mirror
