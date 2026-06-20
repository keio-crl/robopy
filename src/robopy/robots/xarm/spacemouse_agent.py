"""SpaceMouse-based Cartesian teleop driver for xArm7.

Reads 6-DOF deltas from a 3Dconnexion SpaceMouse via ``pyspacemouse`` and
streams them to :meth:`XArmFollower.command_cartesian_relative`. A configurable
button toggles the gripper open/closed.

The teleop loop uses the shared :class:`robopy.config.input_config.SpaceMouseConfig`
so xArm and so101 share the same sensitivity model:

* ``linear_speed`` (m/s) is the **EE translation speed at full stick deflection**
  (raw axis = ±1.0). Lowering it makes the robot move slowly even when the
  SpaceMouse is pushed all the way -- this is the "sensitivity" knob.
* ``angular_speed`` (rad/s) is the analogous rotational sensitivity.
* ``input_smoothing`` applies an exponential moving average on the raw axes to
  damp jitter before the delta is computed.
* ``deadzone`` zeroes out small inputs so the arm stays still when the user is
  not actively pushing the stick.

Per control step (``1 / control_hz`` seconds), the commanded delta is::

    dx_mm = raw_axis * linear_speed * dt * 1000
    drx   = raw_axis * angular_speed * dt

This module is standalone -- it does NOT require GELLO hardware. Import it
directly and call :func:`run_spacemouse_teleop` or use the example at
``examples/robot/xarm_spacemouse_teleop.py``.
"""

from __future__ import annotations

import logging
import time
from typing import Any, cast

import numpy as np

from robopy.config.input_config.spacemouse_config import SpaceMouseConfig
from robopy.robots.xarm.xarm_follower import XArmFollower

logger = logging.getLogger(__name__)

__all__ = ["SpaceMouseConfig", "run_spacemouse_teleop"]


def run_spacemouse_teleop(
    follower: XArmFollower,
    cfg: SpaceMouseConfig | None = None,
    max_seconds: float | None = None,
    *,
    gripper_toggle_button: int = 0,
) -> None:
    """Run a SpaceMouse -> xArm Cartesian teleop loop.

    Args:
        follower: An already-connected :class:`XArmFollower`.
        cfg: Sensitivity / smoothing parameters. ``linear_speed`` and
            ``angular_speed`` act as the sensitivity knobs (max EE velocity
            at full stick deflection). Defaults to :class:`SpaceMouseConfig`
            defaults (0.10 m/s, 0.5 rad/s).
        max_seconds: Stop after this many seconds (``None`` = until Ctrl-C).
        gripper_toggle_button: Index of the SpaceMouse button that toggles the
            gripper open/closed on a rising edge. Defaults to 0.
    """
    try:
        import pyspacemouse  # type: ignore[import-not-found]
    except ImportError as exc:
        raise ImportError(
            "pyspacemouse is required for SpaceMouse teleop. Install it via `uv add pyspacemouse`."
        ) from exc
    pyspacemouse = cast(Any, pyspacemouse)

    if cfg is None:
        cfg = SpaceMouseConfig()

    if cfg.control_hz <= 0:
        raise ValueError(f"control_hz must be > 0, got {cfg.control_hz}")
    if not (0.0 <= cfg.input_smoothing < 1.0):
        raise ValueError(f"input_smoothing must be in [0, 1), got {cfg.input_smoothing}")

    success = pyspacemouse.open()
    if not success:
        raise RuntimeError("Failed to open SpaceMouse device.")
    logger.info(
        "SpaceMouse opened (linear_speed=%.3f m/s, angular_speed=%.3f rad/s, "
        "smoothing=%.2f, hz=%d). Press Ctrl-C to stop.",
        cfg.linear_speed,
        cfg.angular_speed,
        cfg.input_smoothing,
        cfg.control_hz,
    )

    dt = 1.0 / cfg.control_hz
    gripper_open = True
    prev_button = False
    filtered = np.zeros(6, dtype=np.float32)
    filter_initialized = False
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

            if not filter_initialized:
                filtered = raw.copy()
                filter_initialized = True
            elif cfg.input_smoothing > 0.0:
                alpha = 1.0 - cfg.input_smoothing
                filtered = alpha * raw + cfg.input_smoothing * filtered
            else:
                filtered = raw

            delta = np.zeros(6, dtype=np.float32)
            # m/s -> mm per control step.
            delta[:3] = filtered[:3] * (cfg.linear_speed * dt * 1000.0)
            # rad/s -> rad per control step.
            delta[3:] = filtered[3:] * (cfg.angular_speed * dt)

            buttons = getattr(state, "buttons", [0])
            button_pressed = (
                bool(buttons[gripper_toggle_button])
                if gripper_toggle_button < len(buttons)
                else False
            )
            if button_pressed and not prev_button:
                gripper_open = not gripper_open
                logger.info("Gripper %s", "OPEN" if gripper_open else "CLOSED")
            prev_button = button_pressed

            gripper_val = 0.0 if gripper_open else 1.0
            follower.command_cartesian_relative(delta, gripper=gripper_val)

            if max_seconds is not None and (time.time() - start) >= max_seconds:
                break
            time.sleep(dt)
    except KeyboardInterrupt:
        logger.info("SpaceMouse teleop stopped.")
    finally:
        pyspacemouse.close()
