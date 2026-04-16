"""GELLO (Dynamixel) teleoperation controller that acts as the xArm leader.

The original xarm_modules system drives GELLO through the ``gello.dynamixel``
submodule. Here we re-implement the same behaviour using robopy's own
:class:`robopy.motor.dynamixel_bus.DynamixelBus` so no external submodule is
required.
"""

from __future__ import annotations

import glob
import logging
import time
from typing import List

import numpy as np
from numpy.typing import NDArray

from robopy.config.robot_config.xarm_config import (
    XARM_LEADER_MOTOR_NAMES,
    GelloArmConfig,
    XArmConfig,
)
from robopy.motor.control_table import XControlTable
from robopy.motor.dynamixel_bus import DynamixelBus, DynamixelMotor
from robopy.robots.xarm.xarm_arm import XArmArm

logger = logging.getLogger(__name__)


class XArmLeader(XArmArm):
    """GELLO Dynamixel controller wrapped as an :class:`XArmArm` leader."""

    def __init__(self, cfg: XArmConfig, gello_cfg: GelloArmConfig | None = None):
        self.config = cfg
        self._gello_cfg = gello_cfg if gello_cfg is not None else cfg.gello
        # Port auto-detection is deferred to connect() so the class can be
        # instantiated in hardware-less environments (tests, CI).
        self._port: str | None = cfg.leader_port

        self._joint_offsets = np.array(
            tuple(self._gello_cfg.joint_offsets) + (0.0,),
            dtype=np.float32,
        )
        self._joint_signs = np.array(
            tuple(self._gello_cfg.joint_signs) + (1,),
            dtype=np.int8,
        )
        self._gripper_open_rad = float(np.deg2rad(self._gello_cfg.gripper_open_deg))
        self._gripper_close_rad = float(np.deg2rad(self._gello_cfg.gripper_close_deg))
        self._alpha = float(self._gello_cfg.ema_alpha)

        self._motor_map = self._create_motors()
        self._motors: DynamixelBus | None = None
        if self._port is not None:
            self._motors = DynamixelBus(port=self._port, motors=self._motor_map)
        self._last_pos: NDArray[np.float32] | None = None
        self._is_connected = False

    # ------------------------------------------------------------------ setup
    def _create_motors(self) -> dict[str, DynamixelMotor]:
        """Build the Dynamixel motor map for GELLO (7 joints + gripper)."""
        motor_ids = tuple(self._gello_cfg.joint_ids) + (self._gello_cfg.gripper_id,)
        if len(motor_ids) != len(XARM_LEADER_MOTOR_NAMES):
            raise ValueError(
                f"GELLO config produced {len(motor_ids)} motors, expected "
                f"{len(XARM_LEADER_MOTOR_NAMES)}"
            )
        return {
            name: DynamixelMotor(motor_id, name, self._gello_cfg.motor_model)
            for name, motor_id in zip(XARM_LEADER_MOTOR_NAMES, motor_ids)
        }

    @staticmethod
    def _auto_detect_port() -> str:
        """Pick the first ``/dev/serial/by-id/*`` entry (GELLO convention)."""
        candidates = sorted(glob.glob("/dev/serial/by-id/*"))
        if not candidates:
            raise OSError(
                "No /dev/serial/by-id/* device found for GELLO. "
                "Provide XArmConfig.leader_port explicitly."
            )
        chosen = candidates[0]
        logger.info("Auto-detected GELLO port: %s", chosen)
        return chosen

    # --------------------------------------------------------------- runtime
    def connect(self) -> None:
        if self._is_connected:
            logger.info("XArmLeader already connected.")
            return
        if self._port is None:
            self._port = self._auto_detect_port()
        if self._motors is None:
            self._motors = DynamixelBus(port=self._port, motors=self._motor_map)
        self._motors.open(baudrate=self._gello_cfg.baudrate)
        # GELLO is manipulated by hand: keep torque off by default.
        try:
            self._motors.torque_disabled()
        except Exception as exc:  # pragma: no cover - hardware dependent
            logger.warning("Failed to disable torque on GELLO: %s", exc)
        self._is_connected = True
        logger.info("Connected to XArmLeader (GELLO) on %s", self._port)
        if self.config.start_joints is not None:
            self._align_offsets_to_start_joints(self.config.start_joints)

    def disconnect(self) -> None:
        if not self._is_connected or self._motors is None:
            return
        try:
            self._motors.torque_disabled()
        except Exception as exc:  # pragma: no cover - best effort
            logger.warning("Failed to disable GELLO torque on disconnect: %s", exc)
        self._motors.close()
        self._is_connected = False
        logger.info("Disconnected XArmLeader.")

    def set_torque_mode(self, on: bool) -> None:
        """Enable or disable Dynamixel torque (rarely used for GELLO)."""
        if self._motors is None:
            raise ConnectionError("XArmLeader is not connected; cannot set torque mode.")
        if on:
            self._motors.torque_enabled()
        else:
            self._motors.torque_disabled()

    # ----------------------------------------------------------- observation
    def _read_raw_radians(self) -> NDArray[np.float32]:
        """Read the Dynamixel present positions and convert to radians.

        ``DynamixelBus.sync_read(PRESENT_POSITION, ...)`` returns calibrated
        values in degrees (centred around zero). We simply convert to radians
        — joint offsets and signs are applied later in
        :meth:`get_joint_state`.
        """
        if self._motors is None:
            raise ConnectionError("XArmLeader is not connected.")
        motor_names = list(XARM_LEADER_MOTOR_NAMES)
        deg_values = self._motors.sync_read(XControlTable.PRESENT_POSITION, motor_names)
        deg_array = np.array(
            [float(deg_values[name]) for name in motor_names],
            dtype=np.float32,
        )
        return np.deg2rad(deg_array).astype(np.float32)

    def get_joint_state(self) -> NDArray[np.float32]:
        """Return the 8-DOF state: 7 joints [rad] + gripper [0, 1]."""
        raw_rad = self._read_raw_radians()
        pos = (raw_rad - self._joint_offsets) * self._joint_signs

        # Gripper normalization: 0.0 = fully open, 1.0 = fully closed.
        span = self._gripper_close_rad - self._gripper_open_rad
        if abs(span) < 1e-9:
            g = 0.0
        else:
            g = (float(pos[-1]) - self._gripper_open_rad) / span
        pos[-1] = float(np.clip(g, 0.0, 1.0))

        # Exponential moving average matches the original GELLO behaviour.
        if self._last_pos is None:
            self._last_pos = pos.astype(np.float32)
        else:
            pos = self._last_pos * (1 - self._alpha) + pos * self._alpha
            self._last_pos = pos.astype(np.float32)
        return pos.astype(np.float32)

    # ------------------------------------------------------------ alignment
    def _align_offsets_to_start_joints(
        self,
        start_joints: NDArray[np.float32],
        retries: int = 3,
    ) -> None:
        """Adjust joint offsets by multiples of 2π so the current pose is closest
        to ``start_joints`` (GELLO's ``start_joints`` trick).

        ``start_joints`` is expected to be an array of 7 joint targets in
        radians (no gripper value required).
        """
        start_joints = np.asarray(start_joints, dtype=np.float32)
        if start_joints.shape[0] != 7:
            logger.warning(
                "start_joints must have 7 values for xArm7; got %s — skipping.",
                start_joints.shape,
            )
            return

        last_err: Exception | None = None
        for attempt in range(retries):
            try:
                current = self.get_joint_state()[:7]
                break
            except Exception as exc:  # pragma: no cover - hardware dependent
                last_err = exc
                time.sleep(0.05)
        else:
            logger.warning("Skipping GELLO start_joints alignment: %s", last_err)
            return

        two_pi = 2.0 * np.pi
        new_offsets = self._joint_offsets.copy()
        for i in range(7):
            new_offsets[i] = (
                two_pi
                * float(np.round((current[i] - start_joints[i]) / two_pi))
                * float(self._joint_signs[i])
                + self._joint_offsets[i]
            )
        self._joint_offsets = new_offsets.astype(np.float32)
        # Reset EMA buffer so next read reflects the updated offsets.
        self._last_pos = None

    # -------------------------------------------------------------- getters
    @property
    def motors(self) -> DynamixelBus | None:
        return self._motors

    @property
    def motor_names(self) -> List[str]:
        if self._motors is None:
            return list(self._motor_map.keys())
        return list(self._motors.motors.keys())

    @property
    def motor_models(self) -> List[str]:
        if self._motors is None:
            return [m.model_name for m in self._motor_map.values()]
        return [motor.model_name for motor in self._motors.motors.values()]

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def port(self) -> str | None:
        return self._port
