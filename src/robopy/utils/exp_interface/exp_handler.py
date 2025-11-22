import json
import os
from abc import ABC, abstractmethod
from time import sleep
from typing import Any, Dict, Generic, TypeVar

import numpy as np
from numpy import float32
from numpy.typing import NDArray

from robopy.robots.common.composed import ComposedRobot
from robopy.utils.worker.save_worker import SaveWorker

from .meta_data_config import MetaDataConfig

ObsType = TypeVar("ObsType")
RobotType = TypeVar("RobotType", bound=ComposedRobot)
ConfigType = TypeVar("ConfigType")
WorkerType = TypeVar("WorkerType", bound=SaveWorker)


class ExpHandler(ABC, Generic[ObsType, RobotType, ConfigType, WorkerType]):
    def __init__(self, metadata_config: MetaDataConfig, fps: int) -> None:
        self.metadata_config = metadata_config
        self.fps = fps
        self.metadata_config.record_fps = self.fps

    @property
    @abstractmethod
    def robot(self) -> RobotType:
        """Robot instance."""
        pass

    @property
    @abstractmethod
    def save_worker(self) -> WorkerType:
        """Save worker instance."""
        pass

    @property
    @abstractmethod
    def config(self) -> ConfigType:
        """Robot configuration."""
        pass

    @abstractmethod
    def record(self, max_frames: int, if_async: bool = True) -> ObsType:
        pass

    @abstractmethod
    def send(self, max_frame: int, fps: int, leader_action: NDArray[np.float32]) -> None:
        pass

    @abstractmethod
    def _extract_data_shapes(self, obs: ObsType) -> dict:
        pass

    def close(self) -> None:
        """Close the handler and clean up resources."""
        if hasattr(self, "robot") and self.robot:
            try:
                self.robot.disconnect()
            except Exception:
                pass
        if hasattr(self, "save_worker") and self.save_worker:
            try:
                self.save_worker.shutdown()
            except Exception:
                pass

    def save_metadata(self, save_path: str, data_shape: Dict | None = None) -> None:
        """Save metadata to JSON file."""
        metadata: dict[str, Any] = {}
        metadata["task_details"] = self.metadata_config.__dict__
        if data_shape:
            metadata["data_shape"] = data_shape
        metadata["robot_config"] = self._serialize_config(self.config)

        with open(os.path.join(save_path, "metadata.json"), "w") as f:
            json.dump(
                metadata,
                f,
                indent=2,
                default=self._json_serializer,
            )

    def record_save(
        self,
        max_frames: int,
        save_path: str,
        if_async: bool = True,
        save_gif: bool = True,
        warmup_time: int = 5,
    ) -> None:
        """Record and save data loop."""
        try:
            print("Starting recording...")
            if not self.robot.is_connected:
                self.robot.connect()

            while True:
                print("Press 'Enter' to warm up , or 'q' to quit")
                input_str = input()
                if input_str.lower() == "q":
                    print("Exiting...")
                    self.close()
                    sleep(0.5)
                    return

                print(f"Warming up for default {warmup_time} seconds...")
                self.robot.teleoperation(warmup_time)

                print("Press 'Enter' to start recording...")
                input_str = input()
                print("Recording...")

                if if_async:
                    print("Using asynchronous recording...")

                obs = self.record(max_frames=max_frames, if_async=if_async)

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
                    self.close()
                    return
        except Exception as e:
            self.close()
            raise RuntimeError(f"Failed to record from robot: {e}")
        except KeyboardInterrupt:
            print("Recording stopped by user...")
            sleep(0.5)
            self.close()

    def record_save_with_fixed_leader(
        self,
        max_frame: int,
        leader_action: NDArray[float32],
        save_path: str,
        save_gif: bool = True,
    ) -> None:
        """Record and save data with fixed leader action."""
        try:
            print("Starting recording with fixed leader action...")
            if not self.robot.is_connected:
                self.robot.connect()

            while True:
                print("Press 'Enter' to start recording with fixed leader action, or 'q' to quit")
                input_str = input()
                if input_str.lower() == "q":
                    print("Exiting...")
                    self.close()
                    sleep(0.5)
                    return

                print("Recording with fixed leader action...")

                obs = self.robot.record_with_fixed_leader(
                    max_frame=max_frame,
                    leader_action=leader_action,
                    fps=self.fps,
                    teleop_hz=100,
                    max_processing_time_ms=40,
                )

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
                    self.close()
                    return

        except Exception as e:
            self.close()
            raise RuntimeError(f"Failed to record from robot: {e}")
        except KeyboardInterrupt:
            print("Recording stopped by user...")
            sleep(0.5)
            self.close()

    @staticmethod
    def _serialize_config(config_obj: object) -> dict | str:
        """Convert config object to JSON-serializable dictionary."""
        if not hasattr(config_obj, "__dict__"):
            return str(config_obj)

        result: Dict[str, object] = {}
        for key, value in config_obj.__dict__.items():
            if value is None:
                result[key] = None
            elif hasattr(value, "__dict__"):
                result[key] = ExpHandler._serialize_config(value)
            elif isinstance(value, (list, tuple)):
                result[key] = [
                    ExpHandler._serialize_config(item) if hasattr(item, "__dict__") else item
                    for item in value
                ]
            elif isinstance(value, dict):
                result[key] = {
                    k: ExpHandler._serialize_config(v) if hasattr(v, "__dict__") else v
                    for k, v in value.items()
                }
            else:
                result[key] = value
        return result

    @staticmethod
    def _json_serializer(obj: object) -> object:
        """JSON serializer for objects not serializable by default json code."""
        import numpy as np

        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.integer, np.floating)):
            return obj.item()
        elif hasattr(obj, "__dict__"):
            return obj.__dict__
        else:
            return str(obj)
