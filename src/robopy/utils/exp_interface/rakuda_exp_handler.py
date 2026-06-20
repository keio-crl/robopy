import time
from time import sleep
from typing import Any, Dict, List
from venv import logger

import numpy as np
from numpy import float32
from numpy.typing import NDArray

from robopy.config import RakudaConfig, RakudaObs
from robopy.config.robot_config.rakuda_config import RakudaSensorParams
from robopy.config.sensor_config.params_config import CameraParams
from robopy.robots import RakudaRobot
from robopy.utils.worker.rakuda_save_worker import RakudaSaveWorker

from .exp_handler import ExpHandler
from .meta_data_config import MetaDataConfig


class RakudaExpHandler(ExpHandler[RakudaObs, RakudaRobot, RakudaConfig, RakudaSaveWorker]):
    """This class handles the experimental interface for the Rakuda robot.

    Sensors:
    1x Realsense, 2x Digit,

    Example:
    ```python
    from robopy.utils.exp_interface import RakudaExpHandler
    handler = RakudaExpHandler(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
        left_digit_serial="D20542",
        right_digit_serial="D20537",
        fps=20,
    )
    handler.record_save(max_frames=150, save_path="test_01", if_async=True)
    ```
    """

    def __init__(
        self,
        rakuda_config: RakudaConfig,
        metadata_config: MetaDataConfig,
        fps: int = 10,
        control_hz: int = 100,
    ) -> None:
        """__init__ initialize Rakuda experimental handler

        Args:
            leader_port (str): leader serial port
            follower_port (str): follower serial port
            left_digit_serial (str): left digit serial number
            right_digit_serial (str): right digit serial number
            fps (int, optional): The frequency to capture obs. Defaults to 10

        Raises:
            ValueError: fps must be between 1 and 20
            RuntimeError: failed to connect to Rakuda robot
        """
        if 20 < fps or fps < 1:
            raise ValueError("FPS must be between 1 and 20")
        if control_hz < fps:
            raise ValueError("control_hz must be greater than or equal to fps")

        super().__init__(metadata_config, fps)
        config = self._init_config(rakuda_config)
        self.control_hz = control_hz
        self._last_follower_action: NDArray[float32] | None = None

        self._robot = RakudaRobot(config)
        self._save_worker = RakudaSaveWorker(config, worker_num=6, fps=self.fps)

        try:
            self._robot.connect()
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Rakuda robot: {e}")

    def record(self, max_frames: int, if_async: bool = True) -> RakudaObs:
        """record data from Rakuda robot

        Args:
            max_frames (int): maximum number of frames to record
            if_async (bool, optional): If use parallel. Defaults to True.

        Raises:
            RuntimeError: failed to record from Rakuda robot

        Returns:
            RakudaObs: recorded observation
        """
        try:
            obs = self.robot.record_parallel(max_frame=max_frames, fps=self.fps)
        except Exception as e:
            raise RuntimeError(f"Failed to record from Rakuda robot: {e}")
        except KeyboardInterrupt:
            logger.info("Recording stopped by user...")
            sleep(0.5)
            self.close()
            raise RuntimeError("Recording stopped by user")
        return obs

    def send(self, max_frame: int, fps: int, leader_action: NDArray[float32]) -> None:
        """send leader action to Rakuda robot"""
        self.robot.send(max_frame, fps, leader_action)

    def send_action(self, follower_action: NDArray[float32]) -> RakudaObs:
        """Send one follower action with interpolation and return the resulting observation.

        The action is interpreted in follower motor order. One call occupies one
        policy frame (``1 / fps`` seconds); intermediate commands are emitted at
        approximately ``control_hz``.
        """
        target = np.asarray(follower_action, dtype=np.float32)
        _, expected_dim = self._get_follower_pose_spec()
        if target.ndim != 1 or target.shape[0] != expected_dim:
            raise ValueError(f"follower_action must be a 1D array with {expected_dim} elements.")
        if not np.all(np.isfinite(target)):
            raise ValueError("follower_action must contain only finite values.")

        if not self.robot.is_connected:
            self.robot.connect()
            self._last_follower_action = None

        start_action = self._last_follower_action
        if start_action is None:
            start_action = self.robot.get_follower_frame_action()

        steps = max(1, int(round(self.control_hz / self.fps)))
        step_interval = 1.0 / self.fps / steps
        start_time = time.perf_counter()

        try:
            for step in range(1, steps + 1):
                alpha = step / steps
                action = (1.0 - alpha) * start_action + alpha * target
                self.robot.send_follower_frame_action(action.astype(np.float32))

                deadline = start_time + step * step_interval
                sleep_time = deadline - time.perf_counter()
                if sleep_time > 0:
                    time.sleep(sleep_time)
        except Exception:
            self._last_follower_action = None
            raise

        self._last_follower_action = target.copy()
        return self.robot.get_observation()

    @property
    def robot(self) -> RakudaRobot:
        return self._robot

    @property
    def save_worker(self) -> RakudaSaveWorker:
        return self._save_worker

    @property
    def config(self) -> RakudaConfig:
        return self._robot.config

    def _init_config(self, rakuda_config: RakudaConfig) -> RakudaConfig:
        leader_port_num = rakuda_config.leader_port
        follower_port_num = rakuda_config.follower_port
        config: RakudaConfig = rakuda_config
        cameras: List[CameraParams] = [
            CameraParams(
                name="main",
                width=640,
                height=480,
                fps=30,
            )
        ]
        if config.sensors is None:
            config = RakudaConfig(
                leader_port=leader_port_num,
                follower_port=follower_port_num,
                sensors=RakudaSensorParams(cameras=cameras, tactile=[], audio=[]),
            )
        else:
            if len(config.sensors.cameras) == 0:
                config.sensors.cameras = cameras  # default camera
            if len(config.sensors.tactile) == 0:
                config.sensors.tactile = []  # default no tactile
            if len(config.sensors.audio) == 0:
                config.sensors.audio = []  # default no audio

        return config

    def _extract_data_shapes(self, obs: RakudaObs) -> dict[str, Any]:
        """Extract data shapes from observation for metadata.

        Collects shapes from:
        - Arm positions (leader, follower)
        - Camera images
        - Tactile sensor readings
        - Audio sensor readings

        Args:
            obs: Recorded observation containing arms and sensors data.

        Returns:
            Dictionary with nested structure:
            {
                "arms": {"leader": shape, "follower": shape},
                "sensors": {
                    "cameras": {"camera_name": shape, ...},
                    "tactile": {"sensor_name": shape, ...},
                    "audio": {"sensor_name": shape, ...}
                }
            }
        """
        data_shape: Dict[str, Dict[str, Any]] = {}

        # Extract arm data shapes
        if obs.arms is not None:
            data_shape["arms"] = {}
            for arm_name, arm_data in obs.arms.__dict__.items():
                if arm_data is not None:
                    data_shape["arms"][arm_name] = list(arm_data.shape)

        # Extract sensor data shapes
        if obs.sensors is not None:
            data_shape["sensors"] = {}

            # Extract camera shapes
            if obs.sensors.cameras is not None:
                data_shape["sensors"]["cameras"] = {}
                for camera_name, camera_data in obs.sensors.cameras.items():
                    if camera_data is not None:
                        data_shape["sensors"]["cameras"][camera_name] = list(camera_data.shape)

            # Extract tactile sensor shapes
            if obs.sensors.tactile is not None:
                data_shape["sensors"]["tactile"] = {}
                for tactile_name, tactile_data in obs.sensors.tactile.items():
                    if tactile_data is not None:
                        data_shape["sensors"]["tactile"][tactile_name] = list(tactile_data.shape)

            # Extract audio sensor shapes
            if obs.sensors.audio is not None:
                data_shape["sensors"]["audio"] = {}
                for audio_name, audio_data in obs.sensors.audio.items():
                    if audio_data is not None:
                        data_shape["sensors"]["audio"][audio_name] = list(audio_data.shape)

        return data_shape
