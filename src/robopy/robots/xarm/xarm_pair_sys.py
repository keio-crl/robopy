"""Leader/follower coordinator for an xArm7 system driven by GELLO.

Implements :class:`robopy.robots.common.robot.Robot` like
:class:`robopy.robots.rakuda.rakuda_pair_sys.RakudaPairSys` and
:class:`robopy.robots.koch.koch_pair_sys.KochPairSys`, but with GELLO as the
leader and an xArm7 controller as the follower.
"""

from __future__ import annotations

import logging
import time
from typing import Dict

import numpy as np
from numpy.typing import NDArray

from robopy.config.robot_config.xarm_config import XArmArmObs, XArmConfig
from robopy.robots.common.robot import Robot
from robopy.robots.xarm.sim_xarm_follower import SimXArmFollower
from robopy.robots.xarm.xarm_follower import XArmFollower
from robopy.robots.xarm.xarm_leader import XArmLeader

logger = logging.getLogger(__name__)

_DEFAULT_ALIGN_STEPS = 25
_DEFAULT_ALIGN_DELTA = 0.05
_ALIGNMENT_TOLERANCE_RAD = 0.8


class XArmPairSys(Robot):
    """GELLO leader + xArm7 follower pair."""

    def __init__(self, cfg: XArmConfig) -> None:
        self.config = cfg
        self._leader = XArmLeader(cfg)
        if cfg.sim_mode:
            self._follower: XArmFollower | SimXArmFollower = SimXArmFollower(
                host=cfg.sim_host, port=cfg.sim_port
            )
        else:
            self._follower = XArmFollower(cfg)
        self._is_connected = False

    # ------------------------------------------------------------- lifecycle
    def connect(self) -> None:
        if self._is_connected:
            logger.info("XArmPairSys already connected.")
            return
        try:
            self._follower.connect()
            self._leader.connect()
            self._align_leader_follower()
            self._is_connected = True
            logger.info("XArmPairSys connected.")
        except Exception:
            self.disconnect()
            raise

    def disconnect(self) -> None:
        try:
            self._leader.disconnect()
        except Exception as exc:  # pragma: no cover - best effort
            logger.warning("Failed to disconnect leader: %s", exc)
        try:
            self._follower.disconnect()
        except Exception as exc:  # pragma: no cover - best effort
            logger.warning("Failed to disconnect follower: %s", exc)
        self._is_connected = False

    # ---------------------------------------------------------- safety aligner
    def _align_leader_follower(self) -> None:
        """Smoothly move the follower to match the leader's pose.

        On hardware, fail fast if leader/follower differ by more than 0.8 rad
        (a port of ``xarm_modules/src/run_env.py:77-113``). In sim_mode the
        follower always boots at zero, so we skip the pre-check and instead
        run enough ``max_delta``-limited smooth steps to close the gap, then
        verify the residual mismatch.
        """
        leader_state = self._leader.get_joint_state()
        follower_state = self._follower.get_joint_state()
        deltas = np.abs(leader_state - follower_state)
        max_gap = float(deltas.max())

        if not self.config.sim_mode and max_gap > _ALIGNMENT_TOLERANCE_RAD:
            ids = np.where(deltas > _ALIGNMENT_TOLERANCE_RAD)[0]
            details = ", ".join(
                f"joint[{i}]: leader={leader_state[i]:.3f}, "
                f"follower={follower_state[i]:.3f}, "
                f"delta={deltas[i]:.3f}"
                for i in ids
            )
            raise RuntimeError(
                f"Leader/Follower initial mismatch > {_ALIGNMENT_TOLERANCE_RAD} rad: {details}"
            )

        required_steps = max(
            _DEFAULT_ALIGN_STEPS,
            int(np.ceil(max_gap / _DEFAULT_ALIGN_DELTA)) + 5,
        )
        for _ in range(required_steps):
            cmd = self._leader.get_joint_state()
            curr = self._follower.get_joint_state()
            diff = cmd - curr
            max_abs = float(np.abs(diff).max())
            if max_abs > _DEFAULT_ALIGN_DELTA and max_abs > 0.0:
                diff = diff / max_abs * _DEFAULT_ALIGN_DELTA
            self._follower.command_joint_state(curr + diff)
            time.sleep(1.0 / 100.0)

        if self.config.sim_mode:
            return

        leader_state = self._leader.get_joint_state()
        follower_state = self._follower.get_joint_state()
        residuals = np.abs(leader_state - follower_state)
        if residuals.max() > _ALIGNMENT_TOLERANCE_RAD:
            ids = np.where(residuals > _ALIGNMENT_TOLERANCE_RAD)[0]
            details = ", ".join(
                f"joint[{i}]: leader={leader_state[i]:.3f}, "
                f"follower={follower_state[i]:.3f}, "
                f"delta={residuals[i]:.3f}"
                for i in ids
            )
            raise RuntimeError(
                f"Leader/Follower residual mismatch > {_ALIGNMENT_TOLERANCE_RAD} rad "
                f"after smooth align: {details}"
            )

    # ------------------------------------------------------------ observation
    def get_observation(self) -> XArmArmObs:
        if not self._is_connected:
            raise ConnectionError("XArmPairSys is not connected. Call connect() first.")
        leader_obs = self._leader.get_joint_state()
        follower_obs = self._follower.get_joint_state()
        ee_pos_quat = self._follower.get_ee_pos_quat()
        return XArmArmObs(
            leader=leader_obs.astype(np.float32),
            follower=follower_obs.astype(np.float32),
            ee_pos_quat=ee_pos_quat.astype(np.float32),
        )

    # ------------------------------------------------------------ teleoperation
    def teleoperate(self, max_seconds: float | None = None) -> None:
        """Run the GELLO -> xArm teleoperation loop (100 Hz)."""
        if not self._is_connected:
            raise ConnectionError("XArmPairSys is not connected. Call connect() first.")
        logger.info("Starting xArm teleoperation.")
        start = time.time()
        try:
            while True:
                action = self._leader.get_joint_state()
                self._follower.command_joint_state(action)
                if max_seconds is not None and (time.time() - start) >= max_seconds:
                    logger.info("Reached max_seconds=%.1f; exiting teleop.", max_seconds)
                    break
                time.sleep(0.01)
        except KeyboardInterrupt:
            logger.info("xArm teleoperation stopped by user.")

    def teleoperate_step(self) -> XArmArmObs:
        """One iteration of teleop used by the higher-level record loop."""
        if not self._is_connected:
            raise ConnectionError("XArmPairSys is not connected. Call connect() first.")
        action = self._leader.get_joint_state()
        self._follower.command_joint_state(action)
        return self.get_observation()

    def send_follower_action(self, action: NDArray[np.float32]) -> None:
        """Forward a desired joint vector directly to the follower."""
        if not self._is_connected:
            raise ConnectionError("XArmPairSys is not connected. Call connect() first.")
        self._follower.command_joint_state(np.asarray(action, dtype=np.float32))

    def get_leader_action(self) -> Dict[str, float]:
        """Return the current leader joint vector as ``{name: value}``."""
        if not self._is_connected:
            raise ConnectionError("XArmPairSys is not connected. Call connect() first.")
        values = self._leader.get_joint_state()
        return {name: float(v) for name, v in zip(self._leader.motor_names, values)}

    # -------------------------------------------------------------- properties
    @property
    def leader(self) -> XArmLeader:
        return self._leader

    @property
    def follower(self) -> XArmFollower | SimXArmFollower:
        return self._follower

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def port(self) -> str:
        return f"leader={self._leader.port}, follower={self._follower.port}"
