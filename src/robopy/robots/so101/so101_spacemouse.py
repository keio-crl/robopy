"""SpaceMouse-based EE-space controller for SO-101."""

from __future__ import annotations

import logging
import queue
import threading
import time
from collections import defaultdict
from typing import DefaultDict, Dict, List

import numpy as np
from numpy.typing import NDArray
from rich.progress import Progress

from robopy.config.input_config.spacemouse_config import SpaceMouseConfig
from robopy.input.spacemouse import SpaceMouseReader
from robopy.utils.worker.so101_save_worker import So101ArmObs, So101Obs

from .so101_robot import So101Robot

logger = logging.getLogger(__name__)


def _apply_deadzone(value: float, deadzone: float) -> float:
    """Zero out values whose absolute magnitude is below *deadzone*."""
    if abs(value) < deadzone:
        return 0.0
    return value


class So101SpaceMouseController:
    """Control an SO-101 follower arm via a 3Dconnexion SpaceMouse.

    The SpaceMouse 6-DOF input (x, y, z, roll, pitch, yaw — all normalised
    to ``[-1, 1]``) is mapped to incremental end-effector velocity commands.
    At each control step the current EE pose is updated by the velocity
    delta, then Inverse Kinematics converts the target pose to joint angles
    that are sent to the follower arm.

    The controller operates **without a physical leader arm** — the SpaceMouse
    replaces it.  Gripper open/close is driven by the left/right buttons.

    Args:
        robot: A configured :class:`So101Robot` instance.  Only the follower
            arm is used; a leader arm is *not* required.
        spacemouse_config: Optional tuning parameters.  Defaults are
            reasonable for desktop-scale manipulation.
    """

    def __init__(
        self,
        robot: So101Robot,
        spacemouse_config: SpaceMouseConfig | None = None,
    ) -> None:
        self._robot = robot
        self._sm_config = spacemouse_config or SpaceMouseConfig()
        self._reader = SpaceMouseReader()

    @property
    def robot(self) -> So101Robot:
        return self._robot

    @property
    def is_connected(self) -> bool:
        return self._robot.is_connected and self._reader.is_running

    def connect(self) -> None:
        """Connect the robot and start the SpaceMouse reader."""
        if not self._robot.is_connected:
            self._robot.connect()
        self._reader.start()
        logger.info("So101SpaceMouseController connected.")

    def disconnect(self) -> None:
        """Stop the SpaceMouse reader and disconnect the robot."""
        self._reader.stop()
        self._robot.disconnect()
        logger.info("So101SpaceMouseController disconnected.")

    # ------------------------------------------------------------------
    # Teleoperation
    # ------------------------------------------------------------------

    def teleoperation(self, max_seconds: float | None = None) -> None:
        """Run real-time SpaceMouse → EE-space teleoperation.

        The control loop reads the SpaceMouse at *control_hz*, computes the
        incremental EE delta, runs IK and sends joint angles to the follower.

        Args:
            max_seconds: If given, stop after this many seconds.  Otherwise
                the loop runs until interrupted (``KeyboardInterrupt``).
        """
        if not self.is_connected:
            raise ConnectionError(
                "Controller is not connected. Call connect() first."
            )

        cfg = self._sm_config
        dt = 1.0 / cfg.control_hz

        # Initialise current EE pose from the follower's actual position
        current_joints_deg = self._read_follower_joints()
        ee = So101Robot.forward_kinematics(current_joints_deg)
        target_ee = np.array([ee.x, ee.y, ee.z, ee.pitch, ee.roll], dtype=np.float64)
        gripper_deg = float(current_joints_deg[5]) if len(current_joints_deg) > 5 else 0.0
        current_joints_for_ik = current_joints_deg.copy()

        end_time = (time.perf_counter() + max_seconds) if max_seconds is not None else None

        logger.info("SpaceMouse teleoperation started. Press Ctrl+C to stop.")
        try:
            while True:
                loop_start = time.perf_counter()
                if end_time is not None and loop_start >= end_time:
                    break

                target_ee, gripper_deg, current_joints_for_ik = self._control_step(
                    target_ee, gripper_deg, current_joints_for_ik, dt
                )

                elapsed = time.perf_counter() - loop_start
                time.sleep(max(0.0, dt - elapsed))

        except KeyboardInterrupt:
            logger.info("SpaceMouse teleoperation stopped by user.")

    # ------------------------------------------------------------------
    # Recording
    # ------------------------------------------------------------------

    def record_parallel(
        self,
        max_frame: int,
        fps: int = 20,
        control_hz: int | None = None,
        is_async: bool = True,
    ) -> So101Obs:
        """Record a SpaceMouse-driven trajectory with camera observations.

        Architecture mirrors :meth:`So101Robot.record_parallel` — a fast
        control thread drives the arm while the main thread captures camera
        frames at the requested *fps*.

        The returned ``So101Obs.arms.leader`` contains the *commanded* joint
        angles (IK output), so the trajectory can be replayed verbatim with
        ``So101Robot.send()``.  ``So101Obs.arms.follower`` contains the
        actual motor readings.

        Args:
            max_frame: Number of sensor frames to record.
            fps: Camera / observation capture rate.
            control_hz: SpaceMouse control loop rate (defaults to config).
            is_async: Use asynchronous camera reads when available.

        Returns:
            ``So101Obs`` with arm and camera observations.
        """
        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")
        if fps <= 0:
            raise ValueError("fps must be greater than 0.")

        if not self.is_connected:
            raise ConnectionError(
                "Controller is not connected. Call connect() first."
            )

        hz = control_hz or self._sm_config.control_hz
        dt = 1.0 / hz

        max_processing_time_ms = 1000.0 / fps * 0.9

        arm_obs_queue: queue.Queue[So101ArmObs] = queue.Queue(maxsize=hz * 2)
        stop_event = threading.Event()

        # Initialise EE state
        current_joints_deg = self._read_follower_joints()
        ee = So101Robot.forward_kinematics(current_joints_deg)
        init_target_ee = np.array([ee.x, ee.y, ee.z, ee.pitch, ee.roll], dtype=np.float64)
        init_gripper_deg = float(current_joints_deg[5]) if len(current_joints_deg) > 5 else 0.0
        init_joints_for_ik = current_joints_deg.copy()

        def control_worker() -> None:
            target_ee = init_target_ee.copy()
            gripper_deg = init_gripper_deg
            joints_for_ik = init_joints_for_ik.copy()

            while not stop_event.is_set():
                loop_start = time.perf_counter()

                target_ee, gripper_deg, joints_for_ik = self._control_step(
                    target_ee, gripper_deg, joints_for_ik, dt
                )

                # Build observation: commanded joints as leader, actual as follower
                commanded = joints_for_ik.copy()
                try:
                    actual = self._read_follower_joints()
                except Exception:
                    actual = commanded.copy()

                arm_obs = So101ArmObs(
                    leader=commanded.astype(np.float32),
                    follower=actual.astype(np.float32),
                )

                try:
                    arm_obs_queue.put(arm_obs, timeout=dt)
                except queue.Full:
                    pass

                elapsed = time.perf_counter() - loop_start
                time.sleep(max(0.0, dt - elapsed))

        # Start control thread
        control_thread = threading.Thread(target=control_worker, daemon=True)
        control_thread.start()

        # Main thread: capture sensor data at *fps*
        leader_obs: List[NDArray[np.float32]] = []
        follower_obs: List[NDArray[np.float32]] = []
        camera_obs: DefaultDict[str, List[NDArray[np.float32] | NDArray[np.uint8] | None]] = (
            defaultdict(list)
        )

        get_obs_interval = 1.0 / fps
        max_processing_time = max_processing_time_ms / 1000.0
        frame_count = 0
        skipped_frames = 0
        total_processing_time = 0.0

        logger.info(
            "SpaceMouse parallel recording: frames=%s, fps=%s, control_hz=%s",
            max_frame, fps, hz,
        )

        try:
            with Progress() as progress:
                task = progress.add_task(
                    "[green]Recording SO-101 (SpaceMouse)...", total=max_frame
                )
                while frame_count < max_frame:
                    frame_start = time.perf_counter()

                    # Get latest arm observation from control thread
                    try:
                        arm_obs = arm_obs_queue.get(timeout=get_obs_interval)
                        while not arm_obs_queue.empty():
                            arm_obs = arm_obs_queue.get_nowait()
                    except queue.Empty:
                        logger.warning("No arm observation available in time.")
                        continue

                    # Capture cameras
                    sensor_data = self._robot.sensors_observation(
                        timeout_ms=max_processing_time_ms / 2.0, async_mode=is_async
                    )

                    leader_obs.append(arm_obs.leader)
                    follower_obs.append(arm_obs.follower)

                    for cam_name, frame in sensor_data.items():
                        camera_obs[cam_name].append(frame)

                    frame_count += 1
                    progress.update(task, advance=1)
                    processing_time = time.perf_counter() - frame_start
                    total_processing_time += processing_time

                    if processing_time > max_processing_time:
                        skipped_frames += 1
                        logger.warning(
                            "Frame %s took %.1f ms (> %.1f ms), skipping sleep",
                            frame_count,
                            processing_time * 1000.0,
                            max_processing_time_ms,
                        )
                        continue

                    sleep_time = max(0.0, get_obs_interval - processing_time)
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                progress.remove_task(task)

        except KeyboardInterrupt:
            logger.info("SpaceMouse recording interrupted by user.")
            raise
        except Exception as exc:
            logger.error("Error during SpaceMouse parallel recording: %s", exc)
            raise
        finally:
            stop_event.set()
            control_thread.join(timeout=1.0)

        if frame_count == 0:
            raise RuntimeError("No frames captured during SpaceMouse recording.")

        avg_proc = total_processing_time / frame_count * 1000.0
        logger.info(
            "SpaceMouse recording completed: frames=%s, skipped=%s, avg_proc=%.1f ms",
            frame_count, skipped_frames, avg_proc,
        )

        leader_arr = np.asarray(leader_obs, dtype=np.float32)
        follower_arr = np.asarray(follower_obs, dtype=np.float32)
        arms = So101ArmObs(leader=leader_arr, follower=follower_arr)

        camera_obs_np: Dict[str, NDArray[np.uint8] | NDArray[np.float32] | None] = {}
        for cam_name, frames in camera_obs.items():
            if frames and all(f is not None for f in frames):
                camera_obs_np[cam_name] = np.asarray(frames)
            else:
                camera_obs_np[cam_name] = None

        return So101Obs(arms=arms, cameras=camera_obs_np)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _control_step(
        self,
        target_ee: NDArray[np.float64],
        gripper_deg: float,
        current_joints_for_ik: NDArray[np.float32],
        dt: float,
    ) -> tuple[NDArray[np.float64], float, NDArray[np.float32]]:
        """Execute one SpaceMouse → IK → send cycle.

        Returns the updated ``(target_ee, gripper_deg, joint_angles_deg)``.
        """
        cfg = self._sm_config
        sm = self._reader.get_state()

        # Apply deadzone
        sx = _apply_deadzone(sm.x, cfg.deadzone)
        sy = _apply_deadzone(sm.y, cfg.deadzone)
        sz = _apply_deadzone(sm.z, cfg.deadzone)
        sp = _apply_deadzone(sm.pitch, cfg.deadzone)
        sr = _apply_deadzone(sm.roll, cfg.deadzone)

        # Compute EE delta
        target_ee = target_ee.copy()
        target_ee[0] += sx * cfg.linear_speed * dt
        target_ee[1] += sy * cfg.linear_speed * dt
        target_ee[2] += sz * cfg.linear_speed * dt
        target_ee[3] += sp * cfg.angular_speed * dt
        target_ee[4] += sr * cfg.angular_speed * dt

        # Gripper: left button = close, right button = open
        if sm.buttons[0]:
            gripper_deg = max(0.0, gripper_deg - cfg.gripper_speed * dt)
        if sm.buttons[1]:
            gripper_deg = min(100.0, gripper_deg + cfg.gripper_speed * dt)

        # IK → joint angles (degrees)
        result = self._robot.inverse_kinematics(target_ee, current_joints_for_ik)

        if not result.success:
            logger.debug(
                "IK best-effort: pos_err=%.4f m, ori_err=%.4f rad",
                result.position_error,
                result.orientation_error,
            )

        joint_deg = np.rad2deg(result.joint_angles_rad).astype(np.float32)
        full_action = np.append(joint_deg, np.float32(gripper_deg))

        # Send to follower
        self._robot.send_frame_action(full_action)

        return target_ee, gripper_deg, full_action

    def _read_follower_joints(self) -> NDArray[np.float32]:
        """Read current follower joint positions (degrees, 6-element)."""
        obs = self._robot.get_arm_observation()
        return obs.follower
