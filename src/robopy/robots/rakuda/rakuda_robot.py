import queue
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from logging import getLogger
from typing import DefaultDict, Dict, List, override

import numpy as np
from numpy.typing import NDArray
from rich.console import Console
from rich.table import Table

from robopy.config.robot_config.rakuda_config import (
    RakudaArmObs,
    RakudaConfig,
    RakudaObs,
    RakudaSensorConfigs,
    RakudaSensorObs,
)
from robopy.config.sensor_config.sensors import Sensors
from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig
from robopy.sensors.tactile.digit_sensor import DigitSensor
from robopy.sensors.visual.realsense_camera import RealsenseCamera

from ..common.composed import ComposedRobot
from .rakuda_pair_sys import RakudaPairSys

logger = getLogger(__name__)


class RakudaRobot(ComposedRobot):
    def __init__(self, cfg: RakudaConfig):
        self.config = cfg
        self._pair_sys = RakudaPairSys(cfg)
        self._sensor_configs: RakudaSensorConfigs = self._init_config()
        self._sensors: Sensors = self._init_sensors()

    @override
    def connect(self) -> None:
        try:
            self._pair_sys.connect()
        except Exception as e:
            self._pair_sys.disconnect()
            raise e

    @override
    def disconnect(self) -> None:
        self._pair_sys.disconnect()

        for cam in self._sensors.cameras or []:
            cam.disconnect()

        for tac in self._sensors.tactile or []:
            tac.disconnect()

    @override
    def teleoperation(self, max_seconds: float | None = None) -> None:
        """Start teleoperation for Rakuda robot."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        if max_seconds is not None and max_seconds > 0:
            self._pair_sys.teleoperate(max_seconds=max_seconds)
        else:
            self._pair_sys.teleoperate()

    def record(self, max_frame: int, fps: int = 5) -> RakudaObs:
        if not self.is_connected:
            self.connect()

        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")

        leader_obs = []
        follower_obs = []
        camera_obs: Dict[str, List] = DefaultDict(list)
        tactile_obs: Dict[str, List] = DefaultDict(list)

        get_obs_interval = 1.0 / fps
        frame_count = 0
        interval_start = time.time()

        try:
            while frame_count < max_frame:
                temp_arm_obs = self.robot_system.teleoperate_step()

                if time.time() - interval_start < get_obs_interval:
                    continue

                arm_obs = temp_arm_obs
                leader_obs.append(arm_obs["leader"])
                follower_obs.append(arm_obs["follower"])

                sensor_data = self.sensors_observation()
                camera_data = sensor_data["cameras"]
                tactile_data = sensor_data["tactile"]

                for cam_name, cam_frame in camera_data.items():
                    camera_obs[cam_name].append(cam_frame)

                for tac_name, tac_frame in tactile_data.items():
                    tactile_obs[tac_name].append(tac_frame)

                frame_count += 1
                interval_start = time.time()

        except KeyboardInterrupt:
            logger.info("Recording interrupted by user.")
        except Exception as e:
            logger.error(f"An error occurred during recording: {e}")
            raise e

        # proccess observations to numpy arrays
        leader_obs_np = np.array(leader_obs)
        follower_obs_np = np.array(follower_obs)
        arms: RakudaArmObs = {"leader": leader_obs_np, "follower": follower_obs_np}

        # process camera observations
        camera_obs_np: Dict[str, NDArray[np.float32] | None] = {}
        for cam_name, frames in camera_obs.items():
            if frames:
                camera_obs_np[cam_name] = np.array(frames)
            else:
                camera_obs_np[cam_name] = None

        # process tactile observations
        tactile_obs_np: Dict[str, NDArray[np.float32] | None] = {}
        for tac_name, frames in tactile_obs.items():
            if frames:
                tactile_obs_np[tac_name] = np.array(frames)
            else:
                tactile_obs_np[tac_name] = None

        sensors_obs = RakudaSensorObs(cameras=camera_obs_np, tactile=tactile_obs_np)
        return RakudaObs(arms=arms, sensors=sensors_obs)

    def record_parallel(
        self,
        max_frame: int,
        fps: int = 30,
        teleop_hz: int = 100,
        max_processing_time_ms: float = 25.0,
    ) -> RakudaObs:
        """
        teleoperate_stepをteleop_hzで回しつつ、fpsごとに最新のarm_obsを記録し、
        センサデータは並列取得する高速記録。
        """
        if not self.is_connected:
            self.connect()

        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")

        # arm_obsを高頻度で取得するためのキュー
        arm_obs_queue: queue.Queue[RakudaArmObs] = queue.Queue(maxsize=teleop_hz * 2)
        stop_event = threading.Event()

        def teleop_worker():
            interval = 1.0 / teleop_hz
            while not stop_event.is_set():
                obs = self.robot_system.teleoperate_step()
                try:
                    arm_obs_queue.put(obs, timeout=interval)
                except queue.Full:
                    pass
                time.sleep(interval)

        teleop_thread = threading.Thread(target=teleop_worker, daemon=True)
        teleop_thread.start()

        leader_obs = []
        follower_obs = []
        camera_obs: Dict[str, List] = DefaultDict(list)
        tactile_obs: Dict[str, List] = DefaultDict(list)

        get_obs_interval = 1.0 / fps
        max_processing_time = max_processing_time_ms / 1000.0
        frame_count = 0
        skipped_frames = 0
        total_processing_time = 0.0

        logger.info(f"Starting parallel recording: {max_frame} frames at {fps}Hz")
        logger.info(
            f"""Target interval: {get_obs_interval * 1000:.1f}ms, 
            Max processing time: {max_processing_time_ms}ms"""
        )

        try:
            while frame_count < max_frame:
                frame_start_time = time.perf_counter()

                # 最新のarm_obsを取得（バッファが空なら待つ）
                try:
                    while True:
                        arm_obs = arm_obs_queue.get(timeout=get_obs_interval)
                        while not arm_obs_queue.empty():
                            arm_obs = arm_obs_queue.get_nowait()
                        break
                except queue.Empty:
                    logger.warning("No arm_obs available in time.")
                    continue

                # センサデータは並列取得
                try:
                    with ThreadPoolExecutor(max_workers=4) as executor:
                        # Camera futures
                        camera_futures = {}
                        if self._sensors.cameras:
                            for cam in self._sensors.cameras:
                                if cam.is_connected:
                                    camera_futures[cam.name] = executor.submit(
                                        cam.async_read, timeout_ms=5
                                    )

                        # Tactile futures
                        tactile_futures = {}
                        if self._sensors.tactile:
                            for tac in self._sensors.tactile:
                                if tac.is_connected:
                                    if hasattr(tac, "async_read"):
                                        tactile_futures[tac.name] = executor.submit(
                                            tac.async_read, timeout_ms=5
                                        )
                                    else:
                                        tactile_futures[tac.name] = executor.submit(tac.read)

                        timeout = max_processing_time * 0.8

                        camera_data: Dict[str, NDArray | None] = {}
                        for cam_name, future in camera_futures.items():
                            try:
                                camera_data[cam_name] = future.result(timeout=timeout / 2)
                            except Exception as e:
                                logger.warning(
                                    f"Camera {cam_name} failed in frame {frame_count}: {e}"
                                )
                                camera_data[cam_name] = None

                        tactile_data: Dict[str, NDArray | None] = {}
                        for tac_name, future in tactile_futures.items():
                            try:
                                tactile_data[tac_name] = future.result(timeout=timeout / 2)
                            except Exception as e:
                                logger.warning(
                                    f"Tactile {tac_name} failed in frame {frame_count}: {e}"
                                )
                                tactile_data[tac_name] = None

                    # 記録
                    leader_obs.append(arm_obs["leader"])
                    follower_obs.append(arm_obs["follower"])

                    for cam_name, cam_frame in camera_data.items():
                        camera_obs[cam_name].append(cam_frame)

                    for tac_name, tac_frame in tactile_data.items():
                        tactile_obs[tac_name].append(tac_frame)

                    frame_count += 1

                    # タイミング調整
                    processing_time = time.perf_counter() - frame_start_time
                    total_processing_time += processing_time
                    if processing_time > max_processing_time:
                        skipped_frames += 1
                        logger.warning(
                            f"""Frame {frame_count} took {processing_time * 1000:.1f}ms 
                            (>{max_processing_time_ms}ms), skipping"""
                        )
                        continue

                    elapsed = time.perf_counter() - frame_start_time
                    sleep_time = max(0, get_obs_interval - elapsed)
                    if sleep_time > 0:
                        time.sleep(sleep_time)

                except Exception as e:
                    logger.error(f"Unexpected error in frame {frame_count}: {e}")
                    continue

        except KeyboardInterrupt:
            logger.info("Recording interrupted by user.")
        except Exception as e:
            logger.error(f"An error occurred during parallel recording: {e}")
            raise e
        finally:
            stop_event.set()
            teleop_thread.join(timeout=1.0)

        avg_processing_time = total_processing_time / max(1, frame_count) * 1000
        logger.info(f"Recording completed: {frame_count} frames, {skipped_frames} skipped")
        logger.info(f"Average processing time: {avg_processing_time:.1f}ms")

        leader_obs_np = np.array(leader_obs)
        follower_obs_np = np.array(follower_obs)
        arms: RakudaArmObs = {"leader": leader_obs_np, "follower": follower_obs_np}

        camera_obs_np: Dict[str, NDArray[np.float32] | None] = {}
        for cam_name, frames in camera_obs.items():
            if frames and all(frame is not None for frame in frames):
                camera_obs_np[cam_name] = np.array(frames)
            else:
                camera_obs_np[cam_name] = None

        tactile_obs_np: Dict[str, NDArray[np.float32] | None] = {}
        for tac_name, frames in tactile_obs.items():
            if frames and all(frame is not None for frame in frames):
                tactile_obs_np[tac_name] = np.array(frames).transpose(0, 3, 1, 2)
            else:
                tactile_obs_np[tac_name] = None
        sensors_obs = RakudaSensorObs(cameras=camera_obs_np, tactile=tactile_obs_np)
        return RakudaObs(arms=arms, sensors=sensors_obs)

    @override
    def get_observation(self) -> RakudaObs:
        """get_observation get the current observation from the robot system and sensors."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        arm_obs = self.get_arm_observation()
        sensor_obs = self.sensors_observation()
        return RakudaObs(arms=arm_obs, sensors=sensor_obs)

    def get_arm_observation(self) -> RakudaArmObs:
        """Get the current observation from the robot system (leader and follower positions)."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        return self._pair_sys.get_observation()

    def sensors_observation(self) -> RakudaSensorObs:
        """Get the current observation from the sensors."""
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        if self._sensors is None:
            raise RuntimeError("Sensors are not initialized.")

        # Get camera data with reduced timeout for better performance
        camera_data: Dict[str, NDArray[np.float32] | None] = {}
        if self._sensors.cameras is not None:
            for cam in self._sensors.cameras:
                if cam.is_connected:
                    # Reduced timeout from 10ms to 5ms for 30Hz performance
                    camera_data[cam.name] = cam.async_read(timeout_ms=16)
                else:
                    logger.warning(f"Camera {cam.name} is not connected.")
                    camera_data[cam.name] = None
        else:
            logger.warning("No cameras are initialized in sensors.")
            camera_data = {}

        # Get tactile data using async_read when available
        tactile_data: Dict[str, NDArray[np.float32] | None] = {}
        if self._sensors.tactile is not None:
            for tac in self._sensors.tactile:
                if tac.is_connected:
                    # Use async_read if available for better performance
                    if hasattr(tac, "async_read"):
                        tactile_data[tac.name] = tac.async_read(timeout_ms=50)  # Increased timeout
                    else:
                        tactile_data[tac.name] = tac.read()
                else:
                    tactile_data[tac.name] = None
        else:
            logger.warning("No tactile sensors are initialized in sensors.")
            tactile_data = {}

        return RakudaSensorObs(cameras=camera_data, tactile=tactile_data)

    def send(
        self,
        max_frame: int,
        fps: int,
        leader_action: NDArray[np.float32],
        teleop_hz: int = 100,
    ) -> None:
        """send

        Args:
            max_frame (int): max frame to send
            fps (int): frame per second of leader_action
            leader_action (NDArray[np.float32]): action array of shape (max_frame, 17)
            teleop_hz (int, optional): The frequency to teleoperate
            leader-follower. Defaults to 100.

        Raises:
            ConnectionError: RakudaRobot is not connected. Call connect() first.
            ValueError: max_frame must be greater than 0.
            ValueError: Length of leader_action must match max_frame.
            ValueError: leader_action must be of shape (max_frame, 17).
            ValueError: Leader action length does not match number of leader motors.
        """
        if not self.is_connected:
            raise ConnectionError("RakudaRobot is not connected. Call connect() first.")

        if max_frame <= 0:
            raise ValueError("max_frame must be greater than 0.")

        if len(leader_action) != max_frame:
            raise ValueError("Length of leader_action must match max_frame.")

        if leader_action.ndim != 2 or leader_action.shape[1] != 17:
            raise ValueError("leader_action must be of shape (max_frame, 17).")

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

                # 現在時刻に対応するfpsインデックス
                idx_float = t * fps
                idx0 = int(np.floor(idx_float))
                idx1 = min(idx0 + 1, max_frame - 1)
                alpha = idx_float - idx0

                # 線形補間
                action = (1 - alpha) * leader_action[idx0] + alpha * leader_action[idx1]
                self.send_frame_action(action)
                sent_count += 1

                # busy wait
                next_time = start_time + sent_count * interval
                while time.perf_counter() < next_time:
                    pass

            # 最後のフレームを念のため送信
            self.send_frame_action(leader_action[-1])

            elapsed = time.perf_counter() - start_time
            table = Table(title="Send Action Summary")
            table.add_column("Metric", style="cyan", no_wrap=True)
            table.add_column("Value", style="magenta")
            table.add_row("Original Frames (fps)", f"{max_frame} ({fps}Hz)")
            table.add_row(
                "Sent Frames (after interpolation)", f"{sent_count} ({sent_count / elapsed:.2f}Hz)"
            )
            table.add_row("Total Time (s)", f"{elapsed:.2f}")
            console = Console()
            console.print(table)
        except KeyboardInterrupt:
            logger.info("Send interrupted by user.")
            self._pair_sys.disconnect()
        except Exception as e:
            logger.error(f"An error occurred during send: {e}")
            raise e

    def send_frame_action(self, leader_action: NDArray[np.float32]) -> None:
        leader_action_dict: Dict[str, float] = {}

        leader_motor_names = list(self._pair_sys.leader.motors.motors.keys())
        if len(leader_action) != len(leader_motor_names):
            raise ValueError(
                f"Leader action length {len(leader_action)} does not match "
                f"number of leader motors {len(leader_motor_names)}"
            )

        for i, motor_name in enumerate(leader_motor_names):
            leader_action_dict[motor_name] = int(leader_action[i])

        self._pair_sys.send_follower_action(leader_action_dict)

    def _init_config(self) -> RakudaSensorConfigs:
        """Initialize sensor configurations based on the provided robot configuration."""

        # if sensors config is provided in RakudaConfig, use it
        if self.config.sensors is not None:
            camera_params = self.config.sensors.cameras
            camera_configs = []

            if camera_params is None:
                camera_configs.append(RealsenseCameraConfig())
            else:
                for cam_param in camera_params:
                    came_cfg = RealsenseCameraConfig()
                    came_cfg.name = cam_param.name
                    came_cfg.width = cam_param.width
                    came_cfg.height = cam_param.height
                    came_cfg.fps = cam_param.fps

                    camera_configs.append(came_cfg)

            tactile_configs = []
            if self.config.sensors.tactile is not None:
                tactile_params = self.config.sensors.tactile
                for tac_param in tactile_params:
                    tactile_configs.append(tac_param)
        # if no sensors config is provided, use default configs
        else:
            camera_configs = [RealsenseCameraConfig()]
            tactile_configs = []

        sensor_configs = RakudaSensorConfigs(cameras=camera_configs, tactile=tactile_configs)
        return sensor_configs

    def _init_sensors(self) -> Sensors:
        if self._sensor_configs is None:
            raise RuntimeError("Failed to initialize sensor configurations.")

        cameras = []
        for cam_cfg in self._sensor_configs.cameras:
            cam = RealsenseCamera(cam_cfg)
            cam.connect()
            cameras.append(cam)
        tactiles = []

        for tac_cfg in self._sensor_configs.tactile:
            digit = DigitSensor(tac_cfg)
            digit.connect()
            tactiles.append(digit)

        sensors = Sensors(cameras=cameras, tactile=tactiles)
        self._sensors = sensors
        return sensors

    @property
    def sensor_configs(self) -> RakudaSensorConfigs:
        if self._sensor_configs is None:
            raise RuntimeError("Failed to initialize sensor configurations.")
        return self._sensor_configs

    @property
    def is_connected(self) -> bool:
        return self._pair_sys.is_connected

    @property
    def sensors(self) -> Sensors:
        if self._sensors is None:
            raise RuntimeError("Failed to initialize sensors.")
        return self._sensors

    @property
    def robot_system(self) -> RakudaPairSys:
        return self._pair_sys

    def __del__(self):
        try:
            self.disconnect()
        except Exception:
            pass
