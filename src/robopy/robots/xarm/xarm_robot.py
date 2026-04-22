"""High-level :class:`ComposedRobot` implementation for xArm7 + GELLO.

This module mirrors :class:`robopy.robots.rakuda.rakuda_robot.RakudaRobot` but
targets the xArm/GELLO pair. Sensor integration, parallel recording and fixed
leader playback follow the same design as the Rakuda implementation.
"""

from __future__ import annotations

import queue
import threading
import time
from collections import defaultdict
from concurrent.futures import Future, ThreadPoolExecutor
from logging import getLogger
from typing import Dict, List

import numpy as np
from numpy.typing import NDArray
from rich.console import Console
from rich.table import Table

from robopy.config.robot_config.xarm_config import (
    XArmArmObs,
    XArmConfig,
    XArmObs,
    XArmSensorConfigs,
    XArmSensorObs,
)
from robopy.config.sensor_config.params_config import AudioParams, TactileParams
from robopy.config.sensor_config.sensors import Sensors
from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig
from robopy.robots.common.composed import ComposedRobot
from robopy.robots.xarm.xarm_pair_sys import XArmPairSys
from robopy.sensors.audio.audio_sensor import AudioSensor
from robopy.sensors.tactile.digit_sensor import DigitSensor
from robopy.sensors.visual.realsense_camera import RealsenseCamera

logger = getLogger(__name__)


class XArmRobot(ComposedRobot[XArmPairSys, Sensors, XArmObs]):
    """Sensor-aware facade combining :class:`XArmPairSys` with robopy sensors."""

    def __init__(self, cfg: XArmConfig) -> None:
        self.config = cfg
        self._pair_sys = XArmPairSys(cfg)
        self._sensor_configs: XArmSensorConfigs = self._init_config()
        self._sensors: Sensors = Sensors(cameras=[], tactile=[], audio=[])

    # ------------------------------------------------------------- lifecycle
    def connect(self) -> None:
        try:
            self._pair_sys.connect()
        except Exception:
            self._pair_sys.disconnect()
            raise
        # Defer sensor initialisation until connect() so tests / hardware-less
        # usage can instantiate XArmRobot without triggering camera discovery.
        self._sensors = self._init_sensors()

    def disconnect(self) -> None:
        self._pair_sys.disconnect()
        for cam in self._sensors.cameras or []:
            try:
                cam.disconnect()
            except Exception as exc:  # pragma: no cover - best effort
                logger.warning("Camera disconnect failed: %s", exc)
        for tac in self._sensors.tactile or []:
            try:
                tac.disconnect()
            except Exception as exc:  # pragma: no cover - best effort
                logger.warning("Tactile disconnect failed: %s", exc)
        for audio in self._sensors.audio or []:
            try:
                audio.disconnect()
            except Exception as exc:  # pragma: no cover - best effort
                logger.warning("Audio disconnect failed: %s", exc)

    # --------------------------------------------------------- teleoperation
    def teleoperation(self, max_seconds: float | None = None) -> None:
        if not self.is_connected:
            raise ConnectionError("XArmRobot is not connected. Call connect() first.")
        if max_seconds is not None and max_seconds > 0:
            self._pair_sys.teleoperate(max_seconds=max_seconds)
        else:
            self._pair_sys.teleoperate()

    # -------------------------------------------------------------- record
    def record(self, max_frame: int, fps: int = 5) -> XArmObs:
        """Sequential record: iterate ``teleoperate_step`` + sensor read per frame."""
        if not self.is_connected:
            self.connect()
        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")

        leader_obs: List[NDArray[np.float32]] = []
        follower_obs: List[NDArray[np.float32]] = []
        ee_obs: List[NDArray[np.float32]] = []
        camera_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)
        tactile_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)
        audio_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)

        interval = 1.0 / fps
        frame_start = time.time()

        try:
            for _ in range(max_frame):
                arm_obs = self._pair_sys.teleoperate_step()
                leader_obs.append(arm_obs.leader)
                follower_obs.append(arm_obs.follower)
                ee_obs.append(arm_obs.ee_pos_quat)

                sensor_data = self.sensors_observation()
                for name, frame in sensor_data.cameras.items():
                    camera_obs[name].append(frame)
                for name, frame in sensor_data.tactile.items():
                    tactile_obs[name].append(frame)
                for name, frame in sensor_data.audio.items():
                    audio_obs[name].append(frame)

                elapsed = time.time() - frame_start
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                frame_start = time.time()
        except KeyboardInterrupt:
            logger.info("Recording interrupted by user.")

        return self._build_obs(leader_obs, follower_obs, ee_obs, camera_obs, tactile_obs, audio_obs)

    # ---------------------------------------------------------- record_parallel
    def record_parallel(
        self,
        max_frame: int,
        fps: int = 20,
        teleop_hz: int = 25,
        max_processing_time_ms: float = 40.0,
    ) -> XArmObs:
        """High-frequency record: teleop thread + main sensor capture loop."""
        if not self.is_connected:
            self.connect()
        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")

        arm_obs_queue: queue.Queue[XArmArmObs] = queue.Queue(maxsize=teleop_hz * 2)
        stop_event = threading.Event()

        def teleop_worker() -> None:
            interval = 1.0 / teleop_hz
            while not stop_event.is_set():
                start_time = time.perf_counter()
                try:
                    obs = self._pair_sys.teleoperate_step()
                    try:
                        arm_obs_queue.put(obs, timeout=interval)
                    except queue.Full:
                        pass
                except Exception as exc:  # pragma: no cover - hardware dependent
                    logger.warning("teleop_step failed: %s", exc)
                elapsed = time.perf_counter() - start_time
                time.sleep(max(0.0, interval - elapsed))

        teleop_thread = threading.Thread(target=teleop_worker, daemon=True)
        teleop_thread.start()

        leader_obs: List[NDArray[np.float32]] = []
        follower_obs: List[NDArray[np.float32]] = []
        ee_obs: List[NDArray[np.float32]] = []
        camera_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)
        tactile_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)
        audio_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)

        get_obs_interval = 1.0 / fps
        max_processing_time = max_processing_time_ms / 1000.0
        frame_count = 0
        skipped_frames = 0
        total_processing_time = 0.0

        try:
            while frame_count < max_frame:
                frame_start = time.perf_counter()
                try:
                    arm_obs = arm_obs_queue.get(timeout=get_obs_interval)
                    while not arm_obs_queue.empty():
                        arm_obs = arm_obs_queue.get_nowait()
                except queue.Empty:
                    logger.warning("No arm observation available in time.")
                    continue

                camera_data, tactile_data, audio_data = self._capture_sensors_parallel(
                    max_processing_time_ms
                )

                leader_obs.append(arm_obs.leader)
                follower_obs.append(arm_obs.follower)
                ee_obs.append(arm_obs.ee_pos_quat)
                for name, frame in camera_data.items():
                    camera_obs[name].append(frame)
                for name, frame in tactile_data.items():
                    tactile_obs[name].append(frame)
                for name, frame in audio_data.items():
                    audio_obs[name].append(frame)

                frame_count += 1
                processing_time = time.perf_counter() - frame_start
                total_processing_time += processing_time
                if processing_time > max_processing_time:
                    skipped_frames += 1
                    continue
                sleep_time = max(0.0, get_obs_interval - processing_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            logger.info("Recording interrupted by user.")
        finally:
            stop_event.set()
            teleop_thread.join(timeout=1.0)

        avg = total_processing_time / max(1, frame_count) * 1000.0
        logger.info(
            "record_parallel completed: frames=%d, skipped=%d, avg_proc=%.1fms",
            frame_count,
            skipped_frames,
            avg,
        )
        return self._build_obs(leader_obs, follower_obs, ee_obs, camera_obs, tactile_obs, audio_obs)

    def record_with_fixed_leader(
        self,
        max_frame: int,
        leader_action: NDArray[np.float32],
        fps: int = 20,
        teleop_hz: int = 100,
        max_processing_time_ms: float = 40.0,
    ) -> XArmObs:
        """Replay a pre-recorded leader trajectory and record follower + sensors."""
        if not self.is_connected:
            self.connect()
        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")
        if len(leader_action) != max_frame:
            raise ValueError("Length of leader_action must match max_frame.")
        if leader_action.ndim != 2 or leader_action.shape[1] != 8:
            raise ValueError(
                f"leader_action must be of shape ({max_frame}, 8) (7 joints + gripper)."
            )

        follower_obs_queue: queue.Queue[XArmArmObs] = queue.Queue(maxsize=teleop_hz * 2)
        stop_event = threading.Event()

        def control_worker() -> None:
            interval = 1.0 / teleop_hz
            start_time = time.perf_counter()
            while not stop_event.is_set():
                loop_start = time.perf_counter()
                t = loop_start - start_time
                idx_float = t * fps
                if idx_float >= max_frame - 1:
                    action = leader_action[-1]
                else:
                    idx0 = int(np.floor(idx_float))
                    idx1 = min(idx0 + 1, max_frame - 1)
                    alpha = idx_float - idx0
                    action = (1 - alpha) * leader_action[idx0] + alpha * leader_action[idx1]
                try:
                    self.send_frame_action(action)
                    obs = self._pair_sys.get_observation()
                    follower_obs_queue.put(obs, timeout=interval)
                except queue.Full:
                    pass
                except Exception as exc:  # pragma: no cover
                    logger.warning("fixed_leader control_worker: %s", exc)
                elapsed = time.perf_counter() - loop_start
                time.sleep(max(0.0, interval - elapsed))

        control_thread = threading.Thread(target=control_worker, daemon=True)
        control_thread.start()

        leader_log: List[NDArray[np.float32]] = []
        follower_log: List[NDArray[np.float32]] = []
        ee_log: List[NDArray[np.float32]] = []
        camera_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)
        tactile_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)
        audio_obs: Dict[str, List[NDArray[np.float32] | None]] = defaultdict(list)

        get_obs_interval = 1.0 / fps
        max_processing_time = max_processing_time_ms / 1000.0
        frame_count = 0
        skipped_frames = 0
        total_processing_time = 0.0

        try:
            while frame_count < max_frame:
                frame_start = time.perf_counter()
                try:
                    cur = follower_obs_queue.get(timeout=get_obs_interval)
                    while not follower_obs_queue.empty():
                        cur = follower_obs_queue.get_nowait()
                except queue.Empty:
                    logger.warning("No follower_obs available in time.")
                    continue
                camera_data, tactile_data, audio_data = self._capture_sensors_parallel(
                    max_processing_time_ms
                )

                leader_log.append(leader_action[frame_count])
                follower_log.append(cur.follower)
                ee_log.append(cur.ee_pos_quat)
                for name, frame in camera_data.items():
                    camera_obs[name].append(frame)
                for name, frame in tactile_data.items():
                    tactile_obs[name].append(frame)
                for name, frame in audio_data.items():
                    audio_obs[name].append(frame)

                frame_count += 1
                processing_time = time.perf_counter() - frame_start
                total_processing_time += processing_time
                if processing_time > max_processing_time:
                    skipped_frames += 1
                    continue
                sleep_time = max(0.0, get_obs_interval - processing_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except KeyboardInterrupt:
            logger.info("Fixed-leader recording interrupted by user.")
        finally:
            stop_event.set()
            control_thread.join(timeout=1.0)

        return self._build_obs(leader_log, follower_log, ee_log, camera_obs, tactile_obs, audio_obs)

    # ---------------------------------------------------------------- API
    def get_observation(self) -> XArmObs:
        if not self.is_connected:
            raise ConnectionError("XArmRobot is not connected. Call connect() first.")
        arms = self.get_arm_observation()
        sensors = self.sensors_observation()
        return XArmObs(arms=arms, sensors=sensors)

    def get_arm_observation(self) -> XArmArmObs:
        if not self.is_connected:
            raise ConnectionError("XArmRobot is not connected. Call connect() first.")
        return self._pair_sys.get_observation()

    def sensors_observation(self) -> XArmSensorObs:
        if not self.is_connected:
            raise ConnectionError("XArmRobot is not connected. Call connect() first.")
        camera_data: Dict[str, NDArray[np.float32] | None] = {}
        if self._sensors.cameras:
            for cam in self._sensors.cameras:
                if cam.is_connected:
                    camera_data[cam.name] = cam.async_read(timeout_ms=16)
                else:
                    camera_data[cam.name] = None
        tactile_data: Dict[str, NDArray[np.float32] | None] = {}
        if self._sensors.tactile:
            for tac in self._sensors.tactile:
                if tac.is_connected:
                    frame = tac.async_read(timeout_ms=50)
                    if frame is not None and frame.ndim == 3 and frame.shape[2] == 3:
                        frame = frame.transpose(2, 0, 1)
                    tactile_data[tac.name] = frame
                else:
                    tactile_data[tac.name] = None
        audio_data: Dict[str, NDArray[np.float32] | None] = {}
        if self._sensors.audio:
            for audio in self._sensors.audio:
                if audio.is_connected:
                    frame = audio.async_read(timeout_ms=50)
                    if frame is not None and frame.ndim == 2:
                        frame = frame.transpose(1, 0)
                    audio_data[audio.name] = frame
                else:
                    audio_data[audio.name] = None
        return XArmSensorObs(cameras=camera_data, tactile=tactile_data, audio=audio_data)

    def send_frame_action(self, leader_action: NDArray[np.float32]) -> None:
        """Send a single 8-DOF action (7 joints + gripper) to the follower."""
        self._pair_sys.send_follower_action(leader_action)

    def send(
        self,
        max_frame: int,
        fps: int,
        leader_action: NDArray[np.float32],
        teleop_hz: int = 100,
    ) -> None:
        """Replay a leader trajectory without recording."""
        if not self.is_connected:
            raise ConnectionError("XArmRobot is not connected. Call connect() first.")
        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")
        if leader_action.shape[0] != max_frame:
            raise ValueError("Length of leader_action must match max_frame.")
        if leader_action.ndim != 2 or leader_action.shape[1] != 8:
            raise ValueError("leader_action must be of shape (max_frame, 8).")

        interval = 1.0 / teleop_hz
        total_time = (max_frame - 1) / fps
        start_time = time.perf_counter()
        sent_count = 0
        try:
            while True:
                t = time.perf_counter() - start_time
                if t > total_time:
                    break
                idx_float = t * fps
                idx0 = int(np.floor(idx_float))
                idx1 = min(idx0 + 1, max_frame - 1)
                alpha = idx_float - idx0
                action = (1 - alpha) * leader_action[idx0] + alpha * leader_action[idx1]
                self.send_frame_action(action)
                sent_count += 1
                next_time = start_time + sent_count * interval
                while time.perf_counter() < next_time:
                    pass
            self.send_frame_action(leader_action[-1])

            elapsed = time.perf_counter() - start_time
            table = Table(title="XArm Send Summary")
            table.add_column("Metric", style="cyan")
            table.add_column("Value", style="magenta")
            table.add_row("Original frames", f"{max_frame} @ {fps}Hz")
            table.add_row("Sent frames", f"{sent_count} @ {sent_count / elapsed:.2f}Hz")
            table.add_row("Elapsed", f"{elapsed:.2f}s")
            Console().print(table)
        except KeyboardInterrupt:
            logger.info("Send interrupted by user.")

    # ----------------------------------------------------------- initialisers
    def _init_config(self) -> XArmSensorConfigs:
        from robopy.config.sensor_config.visual_config.camera_config import WebCameraConfig

        if self.config.sensors is None:
            return XArmSensorConfigs(cameras=[], tactile=[], audio=[])
        camera_configs: List[RealsenseCameraConfig | WebCameraConfig] = []
        for cam_param in self.config.sensors.cameras:
            cam_cfg = RealsenseCameraConfig()
            cam_cfg.name = cam_param.name
            cam_cfg.width = cam_param.width
            cam_cfg.height = cam_param.height
            cam_cfg.fps = cam_param.fps
            cam_cfg.index = cam_param.index
            camera_configs.append(cam_cfg)
        tactile_configs: List[TactileParams] = list(self.config.sensors.tactile)
        audio_configs: List[AudioParams] = list(self.config.sensors.audio)
        return XArmSensorConfigs(
            cameras=camera_configs, tactile=tactile_configs, audio=audio_configs
        )

    def _init_sensors(self) -> Sensors:
        cameras: List[RealsenseCamera] = []
        for cam_cfg in self._sensor_configs.cameras:
            if not isinstance(cam_cfg, RealsenseCameraConfig):
                logger.warning("Skipping unsupported camera config: %s", type(cam_cfg))
                continue
            cam = RealsenseCamera(cam_cfg)
            cam.connect()
            cameras.append(cam)

        tactiles: List[DigitSensor] = []
        if self._sensor_configs.tactile:
            for tac_cfg in self._sensor_configs.tactile:
                tac = DigitSensor(tac_cfg)
                tac.connect()
                tactiles.append(tac)

        audios: List[AudioSensor] = []
        if self._sensor_configs.audio:
            for audio_cfg in self._sensor_configs.audio:
                audio = AudioSensor(audio_cfg)
                audio.connect()
                audios.append(audio)

        sensors = Sensors(cameras=cameras, tactile=tactiles, audio=audios)

        table = Table(title="Initialized XArm Sensors")
        table.add_column("Type", style="cyan")
        table.add_column("Name", style="magenta")
        for cam in cameras:
            table.add_row("Camera", cam.name)
        for tac in tactiles:
            table.add_row("Tactile", tac.name)
        for audio in audios:
            table.add_row("Audio", audio.name)
        Console().print(table)
        return sensors

    # -------------------------------------------------------------- helpers
    def _capture_sensors_parallel(
        self, max_processing_time_ms: float
    ) -> tuple[
        Dict[str, NDArray[np.float32] | None],
        Dict[str, NDArray[np.float32] | None],
        Dict[str, NDArray[np.float32] | None],
    ]:
        timeout = (max_processing_time_ms / 1000.0) * 0.5
        with ThreadPoolExecutor(max_workers=4) as executor:
            camera_futures: Dict[str, Future] = {}
            if self._sensors.cameras:
                for cam in self._sensors.cameras:
                    if cam.is_connected:
                        camera_futures[cam.name] = executor.submit(cam.async_read, timeout_ms=5)
            tactile_futures: Dict[str, Future] = {}
            if self._sensors.tactile:
                for tac in self._sensors.tactile:
                    if tac.is_connected:
                        tactile_futures[tac.name] = executor.submit(tac.async_read, timeout_ms=5)
            audio_futures: Dict[str, Future] = {}
            if self._sensors.audio:
                for audio in self._sensors.audio:
                    if audio.is_connected:
                        audio_futures[audio.name] = executor.submit(audio.async_read, timeout_ms=5)

            camera_data: Dict[str, NDArray[np.float32] | None] = {}
            for name, fut in camera_futures.items():
                try:
                    camera_data[name] = fut.result(timeout=timeout)
                except Exception as exc:
                    logger.warning("Camera %s capture failed: %s", name, exc)
                    camera_data[name] = None
            tactile_data: Dict[str, NDArray[np.float32] | None] = {}
            for name, fut in tactile_futures.items():
                try:
                    tactile_data[name] = fut.result(timeout=timeout)
                except Exception as exc:
                    logger.warning("Tactile %s capture failed: %s", name, exc)
                    tactile_data[name] = None
            audio_data: Dict[str, NDArray[np.float32] | None] = {}
            for name, fut in audio_futures.items():
                try:
                    audio_data[name] = fut.result(timeout=timeout)
                except Exception as exc:
                    logger.warning("Audio %s capture failed: %s", name, exc)
                    audio_data[name] = None
        return camera_data, tactile_data, audio_data

    def _build_obs(
        self,
        leader_list: List[NDArray[np.float32]],
        follower_list: List[NDArray[np.float32]],
        ee_list: List[NDArray[np.float32]],
        camera_obs: Dict[str, List[NDArray[np.float32] | None]],
        tactile_obs: Dict[str, List[NDArray[np.float32] | None]],
        audio_obs: Dict[str, List[NDArray[np.float32] | None]],
    ) -> XArmObs:
        leader_np = (
            np.asarray(leader_list, dtype=np.float32)
            if leader_list
            else np.zeros((0, 8), np.float32)
        )
        follower_np = (
            np.asarray(follower_list, dtype=np.float32)
            if follower_list
            else np.zeros((0, 8), np.float32)
        )
        ee_np = np.asarray(ee_list, dtype=np.float32) if ee_list else np.zeros((0, 7), np.float32)
        arms = XArmArmObs(leader=leader_np, follower=follower_np, ee_pos_quat=ee_np)

        def to_array(d: Dict[str, List[NDArray[np.float32] | None]]) -> Dict[str, NDArray | None]:
            out: Dict[str, NDArray | None] = {}
            for name, frames in d.items():
                if frames and all(f is not None for f in frames):
                    out[name] = np.asarray(frames)
                else:
                    out[name] = None
            return out

        sensors = XArmSensorObs(
            cameras=to_array(camera_obs),
            tactile=to_array(tactile_obs),
            audio=to_array(audio_obs),
        )
        return XArmObs(arms=arms, sensors=sensors)

    # ------------------------------------------------------------- properties
    @property
    def is_connected(self) -> bool:
        return self._pair_sys.is_connected

    @property
    def sensors(self) -> Sensors:
        return self._sensors

    @property
    def robot_system(self) -> XArmPairSys:
        return self._pair_sys

    @property
    def sensor_configs(self) -> XArmSensorConfigs:
        return self._sensor_configs

    def __del__(self) -> None:  # pragma: no cover - best effort cleanup
        try:
            self.disconnect()
        except Exception:
            pass
