"""Hardware-light FK/IK helpers for the xArm7.

Unlike SO-101 and Koch where FK/IK are pure-numpy computations, the xArm
controller (UFactory) exposes the kinematic model only through its TCP
protocol. ``XArmKinematics`` opens a lightweight connection to the controller
(real Box or UFactory Studio simulator) and queries
:meth:`~xarm.wrapper.XArmAPI.get_forward_kinematics` /
:meth:`~xarm.wrapper.XArmAPI.get_inverse_kinematics`. These calls are
**read-only** -- the controller computes FK/IK without moving the arm.

For repeated queries prefer the context-manager form so the TCP socket stays
open::

    from robopy.robots.xarm import XArmKinematics

    with XArmKinematics("192.168.1.240") as k:
        pose = k.forward_kinematics(joint_angles_rad)
        joints = k.inverse_kinematics(target_pose)

If you already have a connected :class:`XArmFollower`, call its
``forward_kinematics`` / ``inverse_kinematics`` methods directly to reuse
that connection.
"""

from __future__ import annotations

import logging
from types import TracebackType
from typing import Any

import numpy as np
from numpy.typing import ArrayLike, NDArray

logger = logging.getLogger(__name__)


class XArmKinematics:
    """Stateless wrapper around ``XArmAPI`` for FK / IK only.

    The TCP connection is required to reach the kinematic engine baked into
    the xArm controller, but no joint command is ever sent -- the robot does
    not move. In particular, this class does **not** call
    ``clean_error`` / ``set_state`` / ``set_collision_*`` / gripper setup,
    and it does not spawn a control thread. It only opens a socket so the
    controller can answer FK / IK queries.

    Args:
        ip: IP address of the xArm controller or UFactory Studio simulator
            (e.g. ``"192.168.1.240"`` for a real Box, ``"127.0.0.1"`` for the
            simulator running locally).
    """

    def __init__(self, ip: str) -> None:
        self._ip = ip
        self._robot: Any | None = None

    @property
    def ip(self) -> str:
        return self._ip

    @property
    def is_connected(self) -> bool:
        return self._robot is not None

    def connect(self) -> None:
        """Open a TCP session to the xArm controller (no motion is sent)."""
        if self._robot is not None:
            return
        try:
            from xarm.wrapper import XArmAPI  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportError(
                "xarm-python-sdk is required for XArmKinematics. "
                "Install it via `uv add xarm-python-sdk`."
            ) from exc
        self._robot = XArmAPI(self._ip, is_radian=True)
        logger.info(
            "XArmKinematics connected to %s (FK/IK only -- the arm will not move).",
            self._ip,
        )

    def disconnect(self) -> None:
        """Close the TCP session, if any."""
        if self._robot is None:
            return
        try:
            self._robot.disconnect()
        except Exception as exc:  # pragma: no cover - defensive
            logger.warning("XArmKinematics disconnect failed: %s", exc)
        finally:
            self._robot = None

    def __enter__(self) -> "XArmKinematics":
        self.connect()
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.disconnect()

    # ----------------------------------------------------------------- FK
    def forward_kinematics(self, joint_angles_rad: ArrayLike) -> NDArray[np.float64]:
        """Compute end-effector pose for a given joint configuration.

        Args:
            joint_angles_rad: (7,) joint angles in radians.

        Returns:
            (6,) array ``[x, y, z, roll, pitch, yaw]`` -- position in mm and
            orientation as Euler angles in radians (xArm convention).

        Raises:
            ConnectionError: if :meth:`connect` was not called.
            ValueError: if the input does not have 7 elements.
            RuntimeError: if the controller reports a non-zero status code.
        """
        if self._robot is None:
            raise ConnectionError(
                "XArmKinematics is not connected. Call connect() or use as a context manager."
            )
        angles = np.asarray(joint_angles_rad, dtype=np.float64).flatten()
        if angles.shape[0] != 7:
            raise ValueError(f"Expected 7 joint angles, got {angles.shape[0]}.")
        code, pose = self._robot.get_forward_kinematics(
            angles.tolist(), input_is_radian=True, return_is_radian=True
        )
        if code != 0 or pose is None:
            raise RuntimeError(
                f"xArm forward_kinematics failed (code={code}). "
                "See xArm SDK API codes for details."
            )
        return np.asarray(pose, dtype=np.float64)

    # ----------------------------------------------------------------- IK
    def inverse_kinematics(self, target_pose: ArrayLike) -> NDArray[np.float64]:
        """Compute joint angles for a target end-effector pose.

        Args:
            target_pose: (6,) array ``[x, y, z, roll, pitch, yaw]`` -- position
                in mm, orientation in radians.

        Returns:
            (7,) joint angles in radians.

        Raises:
            ConnectionError: if :meth:`connect` was not called.
            ValueError: if the input does not have 6 elements.
            RuntimeError: if the controller cannot find an IK solution.
        """
        if self._robot is None:
            raise ConnectionError(
                "XArmKinematics is not connected. Call connect() or use as a context manager."
            )
        pose = np.asarray(target_pose, dtype=np.float64).flatten()
        if pose.shape[0] != 6:
            raise ValueError(f"Expected 6 pose elements [x,y,z,r,p,y], got {pose.shape[0]}.")
        code, joints = self._robot.get_inverse_kinematics(
            pose.tolist(), input_is_radian=True, return_is_radian=True
        )
        if code != 0 or joints is None:
            raise RuntimeError(
                f"xArm inverse_kinematics failed (code={code}). "
                "Common causes: target is outside reachable workspace, "
                "or the orientation is in a singular configuration."
            )
        return np.asarray(joints, dtype=np.float64)
