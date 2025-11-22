import queue
import threading
import time
from collections import defaultdict
from logging import getLogger
from typing import DefaultDict, Dict, List, override

import numpy as np
from numpy.typing import NDArray
from rich.progress import Progress

from robopy.config.robot_config.koch_config import (
    KOCH_MOTOR_MAPPING,
    KochConfig,
    KochSensorConfig,
)
from robopy.config.sensor_config.sensors import Sensors
from robopy.config.sensor_config.visual_config.camera_config import (
    RealsenseCameraConfig,
    WebCameraConfig,
)
from robopy.robots.common.composed import ComposedRobot
from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.sensors.visual.web_camera import WebCamera
from robopy.utils.worker.koch_save_worker import KochArmObs, KochObs

from .koch_pair_sys import KochPairSys

logger = getLogger(__name__)


class KochRobot(ComposedRobot[KochPairSys, Sensors, KochObs]):
    def __init__(self, cfg: KochConfig) -> None:
        super().__init__()
        self.config = self._init_config(cfg)
        self._sensor_configs = self.config.sensors
        self._robot_system = KochPairSys(self.config)
        self._sensors = self._init_sensors()
        self._cameras: List[WebCamera | RealsenseCamera] = list(self._sensors.cameras or [])

    @override
    def connect(self) -> None:
        try:
            self._robot_system.connect()
        except Exception as exc:  # pragma: no cover - hardware interaction
            logger.error("Failed to connect Koch robot system: %s", exc)
            self._robot_system.disconnect()
            raise

        for cam in self._cameras:
            if cam.is_connected:
                continue
            try:
                cam.connect()
            except Exception as exc:  # pragma: no cover - hardware interaction
                logger.error("Failed to connect camera %s: %s", cam.name, exc)
                cam.disconnect()
                raise

    @override
    def disconnect(self) -> None:
        self._robot_system.disconnect()
        for cam in self._cameras:
            if cam.is_connected:
                cam.disconnect()

    @override
    def teleoperation(self, max_seconds: float | None = None) -> None:
        """Start teleoperation mode where leader controls follower."""
        if not self.is_connected:
            raise ConnectionError("KochRobot is not connected. Call connect() first.")

        if self._robot_system.leader is None:
            raise ConnectionError("Leader arm is not available. Cannot start teleoperation.")

        if max_seconds is not None and max_seconds > 0:
            end_time = time.perf_counter() + max_seconds
            while time.perf_counter() < end_time:
                self._teleoperate_step(record=False)
                time.sleep(0.01)
            return

        self._robot_system.teleoperate()

    def record(self, max_frame: int, fps: int = 5) -> KochObs:
        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")
        if fps <= 0:
            raise ValueError("fps must be greater than 0.")

        if not self.is_connected:
            self.connect()

        leader_obs: List[NDArray[np.float32]] = []
        follower_obs: List[NDArray[np.float32]] = []
        camera_obs: DefaultDict[str, List[NDArray[np.float32] | NDArray[np.uint8] | None]] = (
            defaultdict(list)
        )

        interval = 1.0 / fps
        frame_start = time.perf_counter()

        try:
            for _ in range(max_frame):
                arm_obs = self._teleoperate_step(record=True)
                if arm_obs is None:
                    continue

                leader_obs.append(arm_obs.leader)
                follower_obs.append(arm_obs.follower)

                sensor_data = self.sensors_observation()
                for cam_name, frame in sensor_data.items():
                    camera_obs[cam_name].append(frame)

                elapsed = time.perf_counter() - frame_start
                sleep_time = interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)
                frame_start = time.perf_counter()

        except KeyboardInterrupt:  # pragma: no cover - manual interruption
            logger.info("Recording interrupted by user.")
            raise
        except Exception as exc:
            logger.error("An error occurred during recording: %s", exc)
            raise

        leader_arr = np.asarray(leader_obs, dtype=np.float32)
        follower_arr = np.asarray(follower_obs, dtype=np.float32)
        arms = KochArmObs(leader=leader_arr, follower=follower_arr)

        camera_obs_np: Dict[str, NDArray[np.uint8] | NDArray[np.float32] | None] = {}
        for cam_name, frames in camera_obs.items():
            if frames and all(frame is not None for frame in frames):
                camera_obs_np[cam_name] = np.asarray(frames)
            else:
                camera_obs_np[cam_name] = None

        return KochObs(arms=arms, cameras=camera_obs_np)

    def record_parallel(
        self,
        max_frame: int,
        fps: int = 20,
        teleop_hz: int = 25,
        max_processing_time_ms: float | None = None,
        is_async: bool = True,
    ) -> KochObs:
        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")
        if fps <= 0:
            raise ValueError("fps must be greater than 0.")

        if not self.is_connected:
            self.connect()

        if max_processing_time_ms is None:
            max_processing_time_ms = 1000.0 / fps * 0.9

        arm_obs_queue: queue.Queue[KochArmObs] = queue.Queue(maxsize=teleop_hz * 2)
        stop_event = threading.Event()

        def teleop_worker() -> None:
            interval = 1.0 / teleop_hz
            while not stop_event.is_set():
                start_time = time.perf_counter()
                arm_obs = self._teleoperate_step(record=True)
                if arm_obs is not None:
                    try:
                        arm_obs_queue.put(arm_obs, timeout=interval)
                    except queue.Full:
                        pass

                elapsed = time.perf_counter() - start_time
                time.sleep(max(0.0, interval - elapsed))

        teleop_thread = threading.Thread(target=teleop_worker, daemon=True)
        teleop_thread.start()

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
            "Starting Koch parallel recording: frames=%s, fps=%s, teleop_hz=%s",
            max_frame,
            fps,
            teleop_hz,
        )

        try:
            with Progress() as progress:
                task = progress.add_task("[green]Recording Koch Robot...", total=max_frame)
                while frame_count < max_frame:
                    frame_start = time.perf_counter()

                    try:
                        arm_obs = arm_obs_queue.get(timeout=get_obs_interval)
                        while not arm_obs_queue.empty():
                            arm_obs = arm_obs_queue.get_nowait()
                    except queue.Empty:
                        logger.warning("No arm observation available in time.")
                        continue

                    sensor_data = self.sensors_observation(
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

        except KeyboardInterrupt:  # pragma: no cover - manual interruption
            logger.info("Recording interrupted by user.")
            raise
        except Exception as exc:
            logger.error("An error occurred during parallel recording: %s", exc)
            raise
        finally:
            stop_event.set()
            teleop_thread.join(timeout=1.0)

        if frame_count == 0:
            raise RuntimeError("No frames captured during parallel recording.")

        avg_processing_time = total_processing_time / frame_count * 1000.0
        logger.info(
            "Koch parallel recording completed: frames=%s, skipped=%s, avg_proc=%.1f ms",
            frame_count,
            skipped_frames,
            avg_processing_time,
        )

        leader_arr = np.asarray(leader_obs, dtype=np.float32)
        follower_arr = np.asarray(follower_obs, dtype=np.float32)
        arms = KochArmObs(leader=leader_arr, follower=follower_arr)

        camera_obs_np: Dict[str, NDArray[np.uint8] | NDArray[np.float32] | None] = {}
        for cam_name, frames in camera_obs.items():
            if frames and all(frame is not None for frame in frames):
                camera_obs_np[cam_name] = np.asarray(frames)
            else:
                camera_obs_np[cam_name] = None

        return KochObs(arms=arms, cameras=camera_obs_np)

    @override
    def get_observation(self) -> KochObs:
        if not self.is_connected:
            raise ConnectionError("KochRobot is not connected. Call connect() first.")

        arm_obs_dict = self._robot_system.get_observation()
        arms = self._to_arm_obs(arm_obs_dict)
        camera_data = self.sensors_observation(async_mode=False)

        return KochObs(arms=arms, cameras=camera_data)

    def get_arm_observation(self) -> KochArmObs:
        if not self.is_connected:
            raise ConnectionError("KochRobot is not connected. Call connect() first.")

        arm_obs_dict = self._robot_system.get_observation()
        return self._to_arm_obs(arm_obs_dict)

    def sensors_observation(
        self, *, async_mode: bool = True, timeout_ms: float = 16.0
    ) -> Dict[str, NDArray[np.float32] | NDArray[np.uint8] | None]:
        if not self.is_connected:
            raise ConnectionError("KochRobot is not connected. Call connect() first.")

        return self._capture_camera_data(async_mode=async_mode, timeout_ms=timeout_ms)

    def send(
        self,
        max_frame: int,
        fps: int,
        leader_action: NDArray[np.float32],
        teleop_hz: int = 100,
    ) -> None:
        if not self.is_connected:
            raise ConnectionError("KochRobot is not connected. Call connect() first.")

        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")
        if leader_action.shape[0] != max_frame:
            raise ValueError("Length of leader_action must match max_frame.")
        if leader_action.ndim != 2:
            raise ValueError("leader_action must be a 2D array.")

        # Get leader motor names from mapping if leader is not connected
        if self._robot_system.leader is not None and self._robot_system.leader.motors is not None:
            leader_motor_names = list(self._robot_system.leader.motors.motors.keys())
        else:
            # Use motor mapping keys as leader motor names when leader is not connected
            leader_motor_names = list(KOCH_MOTOR_MAPPING.keys())

        if leader_action.shape[1] != len(leader_motor_names):
            raise ValueError(
                f"leader_action must be of shape ({max_frame}, {len(leader_motor_names)})."
            )

        interval = 1.0 / teleop_hz
        total_time = (max_frame - 1) / fps
        start_time = time.perf_counter()
        sent_count = 0

        try:
            while True:
                now = time.perf_counter()
                t = now - start_time
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

        except KeyboardInterrupt:  # pragma: no cover - manual interruption
            logger.info("Send interrupted by user.")
            self._robot_system.disconnect()
        except Exception as exc:
            logger.error("An error occurred during send: %s", exc)
            raise

    def send_frame_action(self, leader_action: NDArray[np.float32]) -> None:
        follower_goals: Dict[str, float] = {}

        # Get leader motor names from mapping if leader is not connected
        if self._robot_system.leader is not None and self._robot_system.leader.motors is not None:
            leader_motor_names = list(self._robot_system.leader.motors.motors.keys())
        else:
            # Use motor mapping keys as leader motor names when leader is not connected
            leader_motor_names = list(KOCH_MOTOR_MAPPING.keys())

        if len(leader_action) != len(leader_motor_names):
            raise ValueError(
                f"Leader action length {len(leader_action)} does not match "
                f"number of leader motors {len(leader_motor_names)}"
            )

        for i, motor_name in enumerate(leader_motor_names):
            follower_name = KOCH_MOTOR_MAPPING.get(motor_name)
            if follower_name is not None:
                follower_goals[follower_name] = float(leader_action[i])

        self._robot_system.send_follower_action(follower_goals)

    def _init_config(self, cfg: KochConfig) -> KochConfig:
        if cfg.sensors is None:
            cfg.sensors = KochSensorConfig()
        return cfg

    def _init_sensors(self) -> Sensors:
        cameras: List[WebCamera | RealsenseCamera] = []
        index = 0
        for name, cam_cfg in self.config.sensors.cameras.items():
            cam: WebCamera | RealsenseCamera
            if isinstance(cam_cfg, WebCameraConfig):
                cam = WebCamera(index, name, cam_cfg)
            elif isinstance(cam_cfg, RealsenseCameraConfig):
                cam = RealsenseCamera(cam_cfg)
            else:
                raise ValueError(f"Unsupported camera config type: {type(cam_cfg)}")
            cameras.append(cam)
            index += 1
        return Sensors(cameras=cameras, tactile=None)

    @property
    def sensor_configs(self) -> KochSensorConfig:
        return self._sensor_configs

    @property
    def is_connected(self) -> bool:
        return getattr(self._robot_system, "_is_connected", False)

    @property
    def sensors(self) -> Sensors:
        return self._sensors

    @property
    def robot_system(self) -> KochPairSys:
        return self._robot_system

    def __del__(self) -> None:  # pragma: no cover - best effort cleanup
        try:
            self.disconnect()
        except Exception:
            pass

    def _teleoperate_step(self, *, record: bool) -> KochArmObs | None:
        arm_obs = self._robot_system.teleope_step(if_record=record)
        if arm_obs is None or not record:
            return None
        return self._to_arm_obs(arm_obs)

        if record:
            arm_obs_dict = self._robot_system.teleope_step(if_record=True)
            if arm_obs_dict is None:
                return None
            return self._to_arm_obs(arm_obs_dict)

        self._robot_system.teleope_step(if_record=False)
        return None

    def _capture_camera_data(
        self, *, async_mode: bool, timeout_ms: float
    ) -> Dict[str, NDArray[np.float32] | NDArray[np.uint8] | None]:
        camera_data: Dict[str, NDArray[np.float32] | NDArray[np.uint8] | None] = {}

        for cam in self._cameras:
            color_key = f"{cam.name}.rgb"
            depth_key = f"{cam.name}.depth"

            if not cam.is_connected:
                camera_data[color_key] = None
                if isinstance(cam, RealsenseCamera) and cam.config.is_depth_camera:
                    camera_data[depth_key] = None
                continue

            try:
                if async_mode and hasattr(cam, "async_read"):
                    color_frame = cam.async_read(timeout_ms=timeout_ms)
                else:
                    color_frame = cam.read()
                camera_data[color_key] = color_frame
            except Exception as exc:
                logger.warning("Camera %s failed to read color frame: %s", cam.name, exc)
                camera_data[color_key] = None

            if isinstance(cam, RealsenseCamera) and cam.config.is_depth_camera:
                try:
                    if async_mode and hasattr(cam, "async_read_depth"):
                        depth_frame = cam.async_read_depth(timeout_ms=timeout_ms)
                    else:
                        depth_frame = cam.read_depth()
                    camera_data[depth_key] = depth_frame
                except Exception as exc:
                    logger.warning("Camera %s failed to read depth frame: %s", cam.name, exc)
                    camera_data[depth_key] = None

        return camera_data

    def _to_arm_obs(self, arm_data: object) -> KochArmObs:
        if isinstance(arm_data, KochArmObs):
            return arm_data
        if isinstance(arm_data, dict):
            leader = np.asarray(arm_data["leader"], dtype=np.float32)
            follower = np.asarray(arm_data["follower"], dtype=np.float32)
            return KochArmObs(leader=leader, follower=follower)

        leader = np.asarray(getattr(arm_data, "leader"), dtype=np.float32)
        follower = np.asarray(getattr(arm_data, "follower"), dtype=np.float32)
        return KochArmObs(leader=leader, follower=follower)
