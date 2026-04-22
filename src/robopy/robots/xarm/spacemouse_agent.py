"""SpaceMouse-based Cartesian teleop driver for xArm7.

Reads 6-DOF deltas from a 3Dconnexion SpaceMouse via ``pyspacemouse`` and
streams them to :meth:`XArmFollower.command_cartesian_relative`. Button 0
toggles the gripper open/closed.

This module is standalone — it does NOT require GELLO hardware. Import it
directly and call :func:`run_spacemouse_teleop` or use the example at
``examples/robot/xarm_spacemouse_teleop.py``.
"""

from __future__ import annotations

import logging
import time
from dataclasses import dataclass

import numpy as np

from robopy.robots.xarm.xarm_follower import XArmFollower

logger = logging.getLogger(__name__)


@dataclass
class SpaceMouseConfig:
    """Tuning knobs for the SpaceMouse teleop loop."""

    pos_scale: float = 1.0
    rot_scale: float = 0.5
    deadzone: float = 0.05
    hz: float = 50.0
    gripper_toggle_button: int = 0


def run_spacemouse_teleop(
    follower: XArmFollower,
    cfg: SpaceMouseConfig | None = None,
    max_seconds: float | None = None,
) -> None:
    """Run a SpaceMouse -> xArm Cartesian teleop loop.

    Args:
        follower: An already-connected :class:`XArmFollower`.
        cfg: Optional tuning parameters.
        max_seconds: Stop after this many seconds (``None`` = until Ctrl-C).
    """
    try:
        import pyspacemouse  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ImportError(
            "pyspacemouse is required for SpaceMouse teleop. Install it via `uv add pyspacemouse`."
        ) from exc

    if cfg is None:
        cfg = SpaceMouseConfig()

    success = pyspacemouse.open()
    if not success:
        raise RuntimeError("Failed to open SpaceMouse device.")
    logger.info("SpaceMouse opened. Press Ctrl-C to stop.")

    interval = 1.0 / cfg.hz
    gripper_open = True
    prev_button = False
    start = time.time()

    try:
        while True:
            state = pyspacemouse.read()

            raw = np.array(
                [state.x, state.y, state.z, state.roll, state.pitch, state.yaw],
                dtype=np.float32,
            )
            mask = np.abs(raw) > cfg.deadzone
            raw *= mask

            delta = np.zeros(6, dtype=np.float32)
            delta[:3] = raw[:3] * cfg.pos_scale
            delta[3:] = raw[3:] * cfg.rot_scale

            button_pressed = bool(getattr(state, "buttons", [0])[cfg.gripper_toggle_button])
            if button_pressed and not prev_button:
                gripper_open = not gripper_open
                logger.info("Gripper %s", "OPEN" if gripper_open else "CLOSED")
            prev_button = button_pressed

            gripper_val = 0.0 if gripper_open else 1.0
            follower.command_cartesian_relative(delta, gripper=gripper_val)

            if max_seconds is not None and (time.time() - start) >= max_seconds:
                break
            time.sleep(interval)
    except KeyboardInterrupt:
        logger.info("SpaceMouse teleop stopped.")
    finally:
        pyspacemouse.close()
