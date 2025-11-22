import json
import os
from time import sleep
from typing import Dict, override

import numpy as np
from numpy.typing import NDArray

from robopy.config import KochConfig
from robopy.robots import KochRobot
from robopy.utils.worker.koch_save_worker import KochObs, KochSaveWorker

from .exp_handler import ExpHandler
from .meta_data_config import MetaDataConfig


class KochExpHandler(ExpHandler[KochObs]):
    """Kochロボットの実験用ハンドラ."""

    def __init__(
        self,
        koch_config: KochConfig,
        metadata_config: MetaDataConfig,
        fps: int = 10,
    ) -> None:
        if fps < 1 or fps > 60:
            raise ValueError("FPS must be between 1 and 60")

        self.fps = fps
        self.config = koch_config
        self.metadata_config = metadata_config
        self.metadata_config.record_fps = fps

        self.robot = KochRobot(cfg=self.config)
        self.save_worker = KochSaveWorker(fps=self.fps)

        try:
            self.robot.connect()
        except Exception as exc:  # pragma: no cover - hardware interaction
            raise RuntimeError(f"Failed to connect to Koch robot: {exc}") from exc

    @override
    def save_metadata(self, save_path: str, data_shape: Dict | None = None) -> None:
        metadata = {
            "task_details": self.metadata_config.__dict__,
            "data_shape": data_shape,
            "robot_config": self._serialize_config(self.config),
        }

        with open(os.path.join(save_path, "metadata.json"), "w", encoding="utf-8") as fp:
            json.dump(metadata, fp, indent=2, default=self._json_serializer)

    @override
    def record(self, max_frames: int, if_async: bool = True) -> KochObs:
        if max_frames <= 0:
            raise ValueError("max_frames must be greater than 0")

        try:
            obs = self.robot.record_parallel(max_frame=max_frames, fps=self.fps, is_async=if_async)
        except KeyboardInterrupt as exc:  # pragma: no cover - manual interruption
            sleep(0.5)
            self.robot.disconnect()
            raise RuntimeError("Recording stopped by user") from exc
        except Exception as exc:
            raise RuntimeError(f"Failed to record from Koch robot: {exc}") from exc

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
        try:
            print("Starting recording...")
            if hasattr(self.robot, "is_connected") and not getattr(self.robot, "is_connected"):
                self.robot.connect()

            while True:
                print("Press 'Enter' to warm up, or 'q' to quit")
                input_str = input()
                if input_str.lower() == "q":
                    print("Exiting...")
                    self.robot.disconnect()
                    sleep(0.5)
                    return

                print(f"Warming up for default {warmup_time} seconds...")
                self._warmup(warmup_time)
                print("Press 'Enter' to start recording...")
                input()
                print("Recording...")

                obs = self.record(max_frames=max_frames, if_async=if_async)

                print("Recording finished. Type 1~9 to save data, or 'e' to record again")
                input_str = input()

                if input_str.lower() == "e":
                    print("Recording again...")
                    continue
                elif input_str in [str(i) for i in range(1, 10)]:
                    save_dir = os.path.join("data", f"{save_path}", f"{input_str}")
                    count = 1
                    unique_save_dir = f"{save_dir}_{count}"
                    while os.path.exists(unique_save_dir):
                        count += 1
                        unique_save_dir = f"{save_dir}_{count}"
                    os.makedirs(unique_save_dir, exist_ok=True)

                    self.save_worker.save_all_obs(obs, unique_save_dir, save_gif)
                    data_shape = self._extract_data_shapes(obs)
                    self.save_metadata(unique_save_dir, data_shape)
                else:
                    print("Invalid input. Exiting...")
                    self.robot.disconnect()
                    return

        except Exception as exc:
            self.robot.disconnect()
            raise RuntimeError(f"Failed to record from Koch robot: {exc}") from exc
        except KeyboardInterrupt:
            sleep(0.5)
            self.robot.disconnect()

    @override
    def send(self, max_frame: int, fps: int, leader_action: NDArray[np.float32]) -> None:
        self.robot.send(max_frame=max_frame, fps=fps, leader_action=leader_action)

    @staticmethod
    def _serialize_config(config_obj: object) -> dict | str:
        if not hasattr(config_obj, "__dict__"):
            return str(config_obj)

        result: Dict[str, object] = {}
        for key, value in config_obj.__dict__.items():
            if value is None:
                result[key] = None
            elif hasattr(value, "__dict__"):
                result[key] = KochExpHandler._serialize_config(value)
            elif isinstance(value, (list, tuple)):
                result[key] = [
                    KochExpHandler._serialize_config(item) if hasattr(item, "__dict__") else item
                    for item in value
                ]
            elif isinstance(value, dict):
                result[key] = {
                    k: KochExpHandler._serialize_config(v) if hasattr(v, "__dict__") else v
                    for k, v in value.items()
                }
            else:
                result[key] = value
        return result

    @staticmethod
    def _json_serializer(obj: object) -> object:
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, (np.integer, np.floating)):
            return obj.item()
        elif hasattr(obj, "__dict__"):
            return obj.__dict__
        else:
            return str(obj)

    @staticmethod
    def _extract_data_shapes(obs: KochObs) -> dict:
        data_shape: Dict[str, Dict] = {}
        data_shape["arms"] = {
            "leader": list(obs.arms.leader.shape),
            "follower": list(obs.arms.follower.shape),
        }

        if obs.cameras:
            data_shape["sensors"] = {"cameras": {}}
            for name, frames in obs.cameras.items():
                if frames is None:
                    data_shape["sensors"]["cameras"][name] = None
                else:
                    data_shape["sensors"]["cameras"][name] = list(frames.shape)

        return data_shape

    def _warmup(self, warmup_time: int) -> None:
        if warmup_time <= 0:
            return

        self.robot.teleoperation(max_seconds=warmup_time)
