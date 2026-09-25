"""xArm7 follower driven via ``xarm-python-sdk`` over TCP/IP.

This class is a faithful port of the original
``xarm_modules/src/xarm_robot.py:XArmRobot`` adapted to robopy's
leader/follower conventions. It keeps the 50 Hz background control thread and
velocity-limited Cartesian motion used by the legacy implementation.
"""

from __future__ import annotations

import dataclasses
import logging
import threading
import time
from typing import Any, Optional

import numpy as np
from numpy.typing import NDArray

from robopy.config.robot_config.xarm_config import (
    XArmConfig,
    XArmWorkspaceBounds,
    resolve_workspace_bounds,
)
from robopy.robots.xarm.xarm_arm import XArmArm

logger = logging.getLogger(__name__)


# --------------------------------------------------------------------------- helpers


def _aa_from_quat(quat: np.ndarray) -> np.ndarray:
    """Convert quaternion ``[x, y, z, w]`` to axis-angle ``[rx, ry, rz]``."""
    from pyquaternion import Quaternion

    assert quat.shape == (4,), "quaternion must be a 4-vector"
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        raise ValueError("Input quaternion is zero.")
    q = quat / norm
    quaternion = Quaternion(w=q[3], x=q[0], y=q[1], z=q[2])
    return quaternion.axis * quaternion.angle


def _quat_from_aa(aa: np.ndarray) -> np.ndarray:
    """Convert axis-angle ``[rx, ry, rz]`` to quaternion ``[x, y, z, w]``."""
    from pyquaternion import Quaternion

    assert aa.shape == (3,), "axis-angle must be a 3-vector"
    norm = float(np.linalg.norm(aa))
    if norm == 0.0:
        return np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float32)
    axis = aa / norm
    quaternion = Quaternion(axis=axis, angle=norm)
    return np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=np.float32)


@dataclasses.dataclass(frozen=True)
class RobotState:
    """Snapshot of the xArm follower state (matches legacy ``RobotState``)."""

    x: float
    y: float
    z: float
    gripper: float
    j1: float
    j2: float
    j3: float
    j4: float
    j5: float
    j6: float
    j7: float
    aa: np.ndarray

    @staticmethod
    def from_robot(
        cartesian: np.ndarray,
        joints: np.ndarray,
        gripper: float,
        aa: np.ndarray,
    ) -> "RobotState":
        return RobotState(
            float(cartesian[0]),
            float(cartesian[1]),
            float(cartesian[2]),
            float(gripper),
            float(joints[0]),
            float(joints[1]),
            float(joints[2]),
            float(joints[3]),
            float(joints[4]),
            float(joints[5]),
            float(joints[6]),
            np.asarray(aa, dtype=np.float32),
        )

    def cartesian_pos(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z], dtype=np.float32)

    def quat(self) -> np.ndarray:
        return _quat_from_aa(np.asarray(self.aa, dtype=np.float32))

    def joints(self) -> np.ndarray:
        return np.array(
            [self.j1, self.j2, self.j3, self.j4, self.j5, self.j6, self.j7],
            dtype=np.float32,
        )

    def gripper_pos(self) -> float:
        return self.gripper


class _Rate:
    """Simple busy-wait rate limiter (matches legacy ``Rate``)."""

    def __init__(self, duration: float) -> None:
        self.duration = duration
        self.last = time.time()

    def sleep(self) -> None:
        now = time.time()
        remaining = self.duration - (now - self.last)
        if remaining > 1e-4:
            time.sleep(remaining)
        self.last = time.time()


# --------------------------------------------------------------------------- class


class XArmFollower(XArmArm):
    """xArm7 wrapped as a robopy-style follower.

    The connection to the xArm controller box is established via
    ``XArmAPI(ip, is_radian=True)``. A dedicated background thread runs at
    ``cfg.control_frequency`` Hz, smoothing commands with ``cfg.max_delta``
    and polling state via ``get_servo_angle`` / ``get_position_aa``.
    """

    def __init__(self, cfg: XArmConfig) -> None:
        self.config = cfg
        self._ip = cfg.follower_ip
        self._workspace: XArmWorkspaceBounds | None = resolve_workspace_bounds(
            cfg.workspace_bounds, cfg.restriction
        )
        self._control_frequency = float(cfg.control_frequency)
        self._max_delta = float(cfg.max_delta)
        self._cartesian_speed = int(cfg.cartesian_speed)
        self._cartesian_mvacc = int(cfg.cartesian_mvacc)
        self._collision_sensitivity = int(cfg.collision_sensitivity)
        self._gripper_open = float(cfg.gripper_open)
        self._gripper_close = float(cfg.gripper_close)
        self._gripper_speed = int(cfg.gripper_speed)
        self._gripper_force = int(cfg.gripper_force)



        self._robot: Any | None = None  # XArmAPI (lazy import)

        # Admittance control
        self._admittance_enabled = False
        self._motion_paused = False
        self._control_lock = threading.RLock()


        self._last_state_lock = threading.Lock()
        self._target_command_lock = threading.Lock()
        self._last_state: RobotState = RobotState.from_robot(
            np.zeros(3, dtype=np.float32),
            np.zeros(7, dtype=np.float32),
            0.0,
            np.zeros(3, dtype=np.float32),
        )
        self._target_command: dict[str, Any] = {
            "joints": np.zeros(7, dtype=np.float32),
            "gripper": None,
        }
        self._running = False
        self._thread: threading.Thread | None = None
        self._is_connected = False

    # ------------------------------------------------------------- lifecycle
    def connect(self) -> None:
        if self._is_connected:
            logger.info("XArmFollower already connected.")
            return
        try:
            from xarm.wrapper import XArmAPI  # type: ignore[import-not-found]
        except ImportError as exc:
            raise ImportError(
                "xarm-python-sdk is required for XArmFollower. "
                "Install it via `uv add xarm-python-sdk`."
            ) from exc

        self._robot = XArmAPI(self._ip, is_radian=True)
        self._clear_error_states()
        self._robot.set_collision_rebound(True)
        self._set_gripper_position(self._gripper_open)

        self._last_state = self._update_last_state()
        with self._target_command_lock:
            self._target_command = {
                "joints": self._last_state.joints(),
                "gripper": 0.0,
            }
        self._running = True
        self._thread = threading.Thread(target=self._robot_thread, daemon=True)
        self._thread.start()
        self._is_connected = True
        logger.info("Connected to XArmFollower at %s.", self._ip)


    def disconnect(self) -> None:
        if not self._is_connected:
            return

        # Stop the background control thread
        self._running = False

        if self._thread is not None:
            self._thread.join(timeout=5.0)

            if self._thread.is_alive():
                raise RuntimeError(
                    "Failed to stop XArmFollower control thread."
                )

            self._thread = None

        with self._control_lock:
            self._motion_paused = True

            if self._robot is not None:

                # Disable admittance control
                code = self._robot.set_ft_sensor_mode(0)

                if code != 0:
                    raise RuntimeError(
                        "Failed to disable admittance control: "
                        f"code={code}"
                    )

                self._admittance_enabled = False

                logger.info("Admittance control disabled.")

                # Disconnect SDK
                self._robot.disconnect()
                self._robot = None

            self._is_connected = False

        logger.info("Disconnected XArmFollower.")



    # ----------------------------------------------------------- public API
    def enable_admittance_control(self) -> None:
        """Enable xArm's built-in admittance control."""

        if self._robot is None or not self._is_connected:
            raise RuntimeError("XArmFollower is not connected.")

        def check(code: int, name: str) -> None:
            if code != 0:
                raise RuntimeError(
                    f"{name} failed: code={code}"
                )

        # Parameters verified in the standalone SDK test
        M = 0.06
        J = M * 0.01

        K_pos = 1200 #300
        K_ori = 4

        mass = [M, M, M, J, J, J]

        stiffness = [
            K_pos, K_pos, K_pos,
            K_ori, K_ori, K_ori
        ]

        damping = [0] * 6

        ref_frame = 0
        c_axis = [0, 0, 1, 0, 0, 0]

        with self._control_lock:
            self._motion_paused = True
            arm = self._robot

            try:
                check(
                    arm.set_ft_sensor_admittance_parameters(
                        mass, stiffness, damping
                    ),
                    "set admittance parameters"
                )

                check(
                    arm.set_ft_sensor_admittance_parameters(
                        ref_frame, c_axis
                    ),
                    "set admittance axes"
                )

                check(
                    arm.set_ft_sensor_enable(1),
                    "enable F/T sensor"
                )

                time.sleep(0.2)

                check(
                    arm.set_ft_sensor_mode(1),
                    "enable admittance mode"
                )

                check(
                    arm.set_state(0),
                    "start admittance control"
                )

            except Exception:
                self._admittance_enabled = False
                arm.set_ft_sensor_mode(0)
                raise

            self._admittance_enabled = True

            logger.info("Admittance control enabled.")


    def disable_admittance_control(self) -> None:
        """Disable admittance control."""

        if self._robot is None or not self._is_connected:
            raise RuntimeError("XArmFollower is not connected.")

        with self._control_lock:
            self._motion_paused = True

            code = self._robot.set_ft_sensor_mode(0)

            if code != 0:
                raise RuntimeError(
                    f"Failed to disable admittance control: "
                    f"code={code}"
                )

            self._admittance_enabled = False

            logger.info("Admittance control disabled.")


    def get_joint_state(self) -> NDArray[np.float32]:
        """Return ``[j1..j7, gripper]`` (8-DOF) matching the leader format."""
        with self._last_state_lock:
            state = self._last_state
        return np.concatenate(
            [state.joints(), np.array([state.gripper_pos()], dtype=np.float32)]
        ).astype(np.float32)

    def get_ee_pos_quat(self) -> NDArray[np.float32]:
        """Return ``[x, y, z, qx, qy, qz, qw]`` (metres + quaternion)."""
        with self._last_state_lock:
            state = self._last_state
        return np.concatenate([state.cartesian_pos(), state.quat()]).astype(np.float32)

    def command_joint_state(self, joint_state: NDArray[np.float32]) -> None:
        """Submit a joint (+gripper) target to the background thread."""
        joint_state = np.asarray(joint_state, dtype=np.float32)
        with self._target_command_lock:
            if joint_state.shape[0] == 8:
                self._target_command = {
                    "mode": "joint",
                    "joints": joint_state[:7].copy(),
                    "gripper": float(joint_state[7]),
                }
            elif joint_state.shape[0] == 7:
                self._target_command = {
                    "mode": "joint",
                    "joints": joint_state.copy(),
                    "gripper": None,
                }
            else:
                raise ValueError(f"Invalid joint_state length: {joint_state.shape[0]}")

    def command_cartesian_absolute(
        self,
        pos_aa: NDArray[np.float32],
        gripper: float | None = None,
    ) -> None:
        """Command an absolute EE pose: ``[x, y, z, rx, ry, rz]``.

        Coordinates are in **millimetres** for position and **radians** for
        axis-angle orientation -- matching the xArm SDK convention used by
        ``set_position``. Workspace bounds are applied before sending.
        """
        pos_aa = np.asarray(pos_aa, dtype=np.float32)
        if pos_aa.shape[0] != 6:
            raise ValueError("pos_aa must be a 6-vector [x, y, z, rx, ry, rz].")
        with self._target_command_lock:
            self._target_command = {
                "mode": "cartesian_abs",
                "pose": pos_aa.copy(),
                "gripper": gripper,
            }

    def command_cartesian_relative(
        self,
        delta: NDArray[np.float32],
        gripper: float | None = None,
    ) -> None:
        """Command a relative EE movement: ``[dx, dy, dz, drx, dry, drz]``.

        Position deltas are in **millimetres**, orientation deltas in
        **radians**. The delta is applied to the current EE pose each control
        cycle. Workspace bounds are applied after adding the delta.
        """
        delta = np.asarray(delta, dtype=np.float32)
        if delta.shape[0] != 6:
            raise ValueError("delta must be a 6-vector [dx, dy, dz, drx, dry, drz].")
        with self._target_command_lock:
            self._target_command = {
                "mode": "cartesian_rel",
                "delta": delta.copy(),
                "gripper": gripper,
            }


    def resume_motion_commands(self) -> None:
        """Resume motion commands after mode switching."""

        if self._robot is None or not self._is_connected:
            raise RuntimeError("XArmFollower is not connected.")

        with self._control_lock:

            state = self._update_last_state()

            # Reset target to the current robot state
            with self._target_command_lock:
                self._target_command = {
                    "mode": "joint",
                    "joints": state.joints().copy(),
                    "gripper": None,
                }

            self._motion_paused = False

            logger.info("Motion commands resumed.")

    # ---------------------------------------------------------- xArm helpers
    def _clear_error_states(self) -> None:
        if self._robot is None:
            return
        self._robot.clean_error()
        self._robot.clean_warn()
        self._robot.motion_enable(True)
        time.sleep(0.2)
        self._robot.set_mode(0)
        time.sleep(0.2)
        self._robot.set_collision_sensitivity(self._collision_sensitivity)
        time.sleep(0.2)
        self._robot.set_state(state=0)
        time.sleep(0.2)
        self._robot.set_gripper_enable(True)
        time.sleep(0.2)


    def _set_gripper_position(self, pos: int) -> None:
        if self._robot is None:
            return
        self._robot.set_gripper_g2_position(
            pos,
            speed=self._gripper_speed,
            force=self._gripper_force,
            wait=False,
        )


    def _get_gripper_pos(self) -> float:
        if self._robot is None:
            raise RuntimeError("XArm is not connected.")

        code, gripper_pos = self._robot.get_gripper_g2_position()

        retries = 0

        while (code != 0 or gripper_pos is None) and retries < 10:
            logger.warning(
                "get_gripper_g2_position error: code=%s, value=%s "
                "(retry=%d/10)",
                code,
                gripper_pos,
                retries + 1,
            )

            # Retry only. Do not change the robot control state.
            time.sleep(0.01)

            code, gripper_pos = self._robot.get_gripper_g2_position()

            retries += 1

        if code != 0 or gripper_pos is None:
            raise RuntimeError(
                "Failed to get gripper position after retries: "
                f"code={code}, value={gripper_pos}"
            )

        span = self._gripper_close - self._gripper_open

        if span == 0:
            return 0.0

        return (
            float(gripper_pos) - self._gripper_open
        ) / span


    def _update_last_state(self) -> RobotState:
        with self._last_state_lock:
            if self._robot is None:
                raise RuntimeError("XArm is not connected.")

            # Gripper position
            gripper_pos = self._get_gripper_pos()

            # Joint angles
            code, servo_angle = self._robot.get_servo_angle(
                is_radian=True
            )

            retries = 0

            while (code != 0 or servo_angle is None) and retries < 10:
                logger.warning(
                    "get_servo_angle error: code=%s "
                    "(retry=%d/10)",
                    code,
                    retries + 1,
                )

                time.sleep(0.01)

                code, servo_angle = self._robot.get_servo_angle(
                    is_radian=True
                )

                retries += 1

            if code != 0 or servo_angle is None:
                raise RuntimeError(
                    "Failed to get servo angles after retries: "
                    f"code={code}"
                )

            # Cartesian pose
            code, cart_pos = self._robot.get_position_aa(
                is_radian=True
            )

            retries = 0

            while (code != 0 or cart_pos is None) and retries < 10:
                logger.warning(
                    "get_position_aa error: code=%s "
                    "(retry=%d/10)",
                    code,
                    retries + 1,
                )

                time.sleep(0.01)

                code, cart_pos = self._robot.get_position_aa(
                    is_radian=True
                )

                retries += 1

            if code != 0 or cart_pos is None:
                raise RuntimeError(
                    "Failed to get Cartesian pose after retries: "
                    f"code={code}"
                )

            cart_arr = np.asarray(
                cart_pos,
                dtype=np.float32,
            )

            aa = cart_arr[3:].copy()
            cartesian = cart_arr[:3].copy() / 1000.0

            joints = np.asarray(
                servo_angle,
                dtype=np.float32,
            )[:7]

            return RobotState.from_robot(
                cartesian,
                joints,
                gripper_pos,
                aa,
            )

    def _set_position(self, joints: np.ndarray) -> None:
        if self._robot is None:
            return
        _, pose = self._robot.get_forward_kinematics(joints.tolist(), input_is_radian=True)
        if self._workspace is not None:
            pose[0] = float(np.clip(pose[0], self._workspace.min_x, self._workspace.max_x))
            pose[1] = float(np.clip(pose[1], self._workspace.min_y, self._workspace.max_y))
            min_z = self._workspace.effective_min_z(pose[0], pose[1])
            pose[2] = float(np.clip(pose[2], min_z, self._workspace.max_z))

        ret = self._robot.set_position(
            x=pose[0],
            y=pose[1],
            z=pose[2],
            roll=pose[3],
            pitch=pose[4],
            yaw=pose[5],
            wait_motion=False,
            is_radian=True,
            speed=self._cartesian_speed,
            mvacc=self._cartesian_mvacc,
            radius=0,
        )
        if ret != 0:
            self._motion_paused = True
            raise RuntimeError(
                f"xArm set_position failed: code={ret}"
            )

    def _send_cartesian(self, pose: np.ndarray) -> None:
        """Send a 6-DOF Cartesian pose ``[x, y, z, rx, ry, rz]`` (mm, rad)."""
        if self._robot is None:
            return
        if self._workspace is not None:
            pose[0] = float(np.clip(pose[0], self._workspace.min_x, self._workspace.max_x))
            pose[1] = float(np.clip(pose[1], self._workspace.min_y, self._workspace.max_y))
            min_z = self._workspace.effective_min_z(pose[0], pose[1])
            pose[2] = float(np.clip(pose[2], min_z, self._workspace.max_z))
        ret = self._robot.set_position(
            x=pose[0],
            y=pose[1],
            z=pose[2],
            roll=pose[3],
            pitch=pose[4],
            yaw=pose[5],
            wait_motion=False,
            is_radian=True,
            speed=self._cartesian_speed,
            mvacc=self._cartesian_mvacc,
            radius=0,
        )
        if ret != 0:
            self._motion_paused = True
            raise RuntimeError(
                f"xArm set_position failed: code={ret}"
            )


    def _robot_thread(self) -> None:
        rate = _Rate(duration=1.0 / self._control_frequency)
        step_times: list[float] = []
        count = 0
        
        last_gripper_command = None


        while self._running:
            s_t = time.time()

            try:
                with self._control_lock:
                    self._last_state = self._update_last_state()

            except Exception as exc:
                logger.error(
                    "Failed to update robot state: %s",
                    exc,
                )

                # Stop sending new motion commands
                with self._control_lock:
                    self._motion_paused = True

                # Keep the thread alive for subsequent state polling
                rate.sleep()
                continue


            try:
                with self._control_lock:

                    if not self._motion_paused:
                        with self._target_command_lock:
                            cmd = dict(self._target_command)

                        mode = cmd.get("mode", "joint")

                        gripper_command: Optional[float] = cmd.get(
                            "gripper"
                        )

                        if mode == "cartesian_abs":

                            self._send_cartesian(
                                np.asarray(cmd["pose"], dtype=np.float32)
                            )

                        elif mode == "cartesian_rel":

                            if self._robot is None:
                                raise RuntimeError("XArm is not connected.")

                            code, cur_cart = self._robot.get_position_aa(
                                is_radian=True
                            )

                            if code != 0 or cur_cart is None:
                                raise RuntimeError(
                                    f"Failed to get current Cartesian pose: code={code}"
                                )

                            cur_arr = np.asarray(
                                cur_cart,
                                dtype=np.float32
                            )

                            target = cur_arr + np.asarray(
                                cmd["delta"],
                                dtype=np.float32
                            )

                            self._send_cartesian(target)

                        else:

                            target_joints = np.asarray(
                                cmd.get("joints", np.zeros(7)),
                                dtype=np.float32
                            )

                            joint_delta = (
                                target_joints
                                - self._last_state.joints()
                            )

                            norm = float(np.linalg.norm(joint_delta))

                            if norm > self._max_delta and norm > 0.0:
                                delta = (
                                    joint_delta / norm * self._max_delta
                                )
                            else:
                                delta = joint_delta

                            self._set_position(
                                self._last_state.joints() + delta
                            )

                        # Gripper control
                        if gripper_command is not None:

                            if (
                                last_gripper_command is None
                                or abs(
                                    gripper_command - last_gripper_command
                                ) > 1e-3
                            ):

                                gripper_pos = (
                                    self._gripper_open
                                    + float(gripper_command)
                                    * (
                                        self._gripper_close
                                        - self._gripper_open
                                    )
                                )

                                self._set_gripper_position(gripper_pos)

                                last_gripper_command = gripper_command
            except Exception:
                logger.exception(
                    "Failed to send xArm motion command."
                )

                with self._control_lock:
                    self._motion_paused = True



            try:
                with self._control_lock:
                    self._last_state = self._update_last_state()

            except Exception as exc:
                logger.error(
                    "Failed to update robot state: %s",
                    exc,
                )

                with self._control_lock:
                    self._motion_paused = True

            rate.sleep()


            step_times.append(time.time() - s_t)
            count += 1
            if count % 1000 == 0:
                frequency = 1.0 / float(np.mean(step_times))
                logger.info(
                    "XArmFollower loop frequency: mean=%.2fHz (n=%d)",
                    frequency,
                    len(step_times),
                )
                step_times = []

    # -------------------------------------------------------------- getters
    @property
    def is_connected(self) -> bool:
        return self._is_connected

    @property
    def port(self) -> str:
        """For xArm, the IP address is used in place of the serial port."""
        return self._ip
