import json
import os
from time import sleep
from typing import Any, Dict, List, override
from venv import logger

from numpy import float32
from numpy.typing import NDArray

from robopy.config import RakudaConfig, RakudaObs
from robopy.config.robot_config.rakuda_config import RakudaSensorParams
from robopy.config.sensor_config.params_config import CameraParams
from robopy.robots import RakudaRobot
from robopy.utils.worker.rakuda_save_woker import RakudaSaveWorker

from .exp_handler import ExpHandler
from .meta_data_config import MetaDataConfig


class RakudaExpHandler(ExpHandler):
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

        config = self._init_config(rakuda_config)
        if 20 < fps or fps < 1:
            raise ValueError("FPS must be between 1 and 20")
        else:
            self.fps = fps
        self.robot = RakudaRobot(config)
        self.save_worker = RakudaSaveWorker(config, worker_num=6, fps=self.fps)
        self.metadata_config = metadata_config
        self.metadata_config.record_fps = self.fps
        try:
            self.robot.connect()
        except Exception as e:
            raise RuntimeError(f"Failed to connect to Rakuda robot: {e}")

    @override
    def save_metadata(self, save_path: str, data_shape: Dict | None = None) -> None:
        """Save metadata to JSON file with custom serialization for non-JSON objects.

        Handles serialization of numpy arrays and dataclass objects by converting
        them to JSON-serializable formats (lists for arrays, dicts for dataclasses).
        """
        metadata: dict[str, Any] = {}
        metadata["task_details"] = self.metadata_config.__dict__
        if data_shape:
            metadata["data_shape"] = data_shape
        metadata["robot_config"] = self._serialize_config(self.robot.config)

        with open(os.path.join(save_path, "metadata.json"), "w") as f:
            json.dump(
                metadata,
                f,
                indent=2,
                default=self._json_serializer,
            )

    @staticmethod
    def _serialize_config(config_obj: object) -> dict | str:
        """Convert config object to JSON-serializable dictionary.

        Args:
            config_obj: Configuration object to serialize.

        Returns:
            dict | str: Serialized configuration as dictionary or string.
        """
        if not hasattr(config_obj, "__dict__"):
            return str(config_obj)

        result: Dict[str, object] = {}
        for key, value in config_obj.__dict__.items():
            if value is None:
                result[key] = None
            elif hasattr(value, "__dict__"):
                # Recursively serialize nested dataclass objects
                result[key] = RakudaExpHandler._serialize_config(value)
            elif isinstance(value, (list, tuple)):
                # Handle lists and tuples that may contain dataclass objects
                result[key] = [
                    RakudaExpHandler._serialize_config(item) if hasattr(item, "__dict__") else item
                    for item in value
                ]
            elif isinstance(value, dict):
                # Handle dictionaries that may contain dataclass objects
                result[key] = {
                    k: RakudaExpHandler._serialize_config(v) if hasattr(v, "__dict__") else v
                    for k, v in value.items()
                }
            else:
                result[key] = value
        return result

    @staticmethod
    def _json_serializer(obj: object) -> object:
        """JSON serializer for objects not serializable by default json code.

        Handles:
        - numpy arrays -> convert to list
        - numpy scalars -> convert to Python native types
        - Unknown objects -> convert to string representation

        Args:
            obj: Object to serialize.

        Returns:
            Serializable representation of the object.
        """
        import numpy as np

        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.integer, np.floating)):
            return obj.item()
        elif hasattr(obj, "__dict__"):
            return obj.__dict__
        else:
            return str(obj)

    def _extract_data_shapes(self, obs: RakudaObs) -> dict:
        """Extract data shapes from observation for metadata.

        Collects shapes from:
        - Arm positions (leader, follower)
        - Camera images
        - Tactile sensor readings

        Args:
            obs: Recorded observation containing arms and sensors data.

        Returns:
            Dictionary with nested structure:
            {
                "arms": {"leader": shape, "follower": shape},
                "sensors": {
                    "cameras": {"camera_name": shape, ...},
                    "tactile": {"sensor_name": shape, ...}
                }
            }
        """
        data_shape: Dict[str, Dict] = {}

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

        return data_shape

    @override
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
            self.robot.disconnect()
            raise RuntimeError("Recording stopped by user")
        return obs

    @override
    def record_save(
        self,
        max_frames: int,
        save_path: str,
        if_async: bool = True,
        save_gif: bool = True,
        warmup_time: int = 5,
    ) -> None:
        """record and save data from Rakuda robot

        Args:
            max_frames (int): maximum number of frames to record
            save_path (str): path to save the recorded data:
            data will be saved to `data/{save_path}`
            if_async (bool, optional): If use parallel. Defaults to True.
            save_gif (bool, optional): if save gif. Defaults to True.
            warmup_time (int, optional): warm up time before recording. Defaults to 5.

        Raises:
            RuntimeError: failed to record from Rakuda robot
            RuntimeError: failed to save data
        """
        try:
            print("Starting recording...")
            if not self.robot.is_connected:
                self.robot.connect()

            while True:
                print("Press 'Enter' to warm up , or 'q' to quit")
                input_str = input()
                if input_str.lower() == "q":
                    print("Exiting...")
                    self.robot.disconnect()
                    sleep(0.5)
                    return
                print(f"Warming up for default {warmup_time} seconds...")
                self.robot.teleoperation(warmup_time)
                print("Press 'Enter' to start recording...")
                input_str = input()
                print("Recpording...")

                print("Using asynchronous recording...")
                obs = self.robot.record_parallel(max_frame=max_frames, fps=self.fps)

                print(
                    "Recording finished. print 1~9 to save data,",
                    " or 'e' to record again",
                )
                input_str = input()

                # handle user input
                if input_str.lower() == "e":
                    print("Recording again...")
                    continue
                # save data
                elif input_str in [str(i) for i in range(1, 10)]:
                    save_dir = os.path.join("data", f"{save_path}", f"{input_str}")
                    count = 1
                    unique_save_dir = save_dir + f"_{count}"
                    while os.path.exists(unique_save_dir):
                        unique_save_dir = f"{save_dir}_{count}"
                        count += 1
                    os.makedirs(unique_save_dir)
                    self.save_worker.save_all_obs(obs, unique_save_dir, save_gif)

                    # Collect data shapes for metadata
                    data_shape = self._extract_data_shapes(obs)

                    self.save_metadata(unique_save_dir, data_shape)
                # disconnect and exit
                else:
                    print("Invalid input. Exiting...")
                    self.robot.disconnect()
                    return
        except Exception as e:
            self.robot.disconnect()
            raise RuntimeError(f"Failed to record from Rakuda robot: {e}")
        except KeyboardInterrupt:
            logger.info("Recording stopped by user...")
            sleep(0.5)
            self.robot.disconnect()

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
                sensors=RakudaSensorParams(cameras=cameras, tactile=[]),
            )
        else:
            if len(config.sensors.cameras) == 0:
                config.sensors.cameras = cameras  # default camera
            if len(config.sensors.tactile) == 0:
                config.sensors.tactile = []  # default no tactile

        return config

    @override
    def send(self, max_frame: int, fps: int, leader_action: NDArray[float32]) -> None:
        """send leader action to Rakuda robot"""
        self.robot.send(max_frame, fps, leader_action)
