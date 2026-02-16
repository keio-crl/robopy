"""Experiment handler for SO-101 with SpaceMouse teleoperation."""

from __future__ import annotations

from time import sleep
from typing import Any, Dict

import numpy as np
from numpy.typing import NDArray

from robopy.config.input_config.spacemouse_config import SpaceMouseConfig
from robopy.config.robot_config.so101_config import So101Config
from robopy.robots.so101.so101_robot import So101Robot
from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController
from robopy.utils.worker.so101_save_worker import So101Obs, So101SaveWorker

from .exp_handler import ExpHandler
from .meta_data_config import MetaDataConfig


class So101SpaceMouseExpHandler(
    ExpHandler[So101Obs, So101Robot, So101Config, So101SaveWorker]
):
    """Experiment handler using SpaceMouse for SO-101 data collection.

    This handler replaces the leader arm with a SpaceMouse device for
    teleoperation and data recording.  It follows the standard
    :class:`ExpHandler` interface so that the interactive ``record_save()``
    workflow (warmup → record → save) works identically.
    """

    def __init__(
        self,
        so101_config: So101Config,
        metadata_config: MetaDataConfig,
        spacemouse_config: SpaceMouseConfig | None = None,
        fps: int = 20,
    ) -> None:
        if fps < 1 or fps > 60:
            raise ValueError("FPS must be between 1 and 60")

        super().__init__(metadata_config, fps)
        self._config = so101_config
        self._robot = So101Robot(cfg=self._config)
        self._controller = So101SpaceMouseController(
            self._robot, spacemouse_config
        )
        self._save_worker = So101SaveWorker(fps=self.fps)

        try:
            self._controller.connect()
        except Exception as exc:
            raise RuntimeError(
                f"Failed to connect SpaceMouse controller: {exc}"
            ) from exc

    @property
    def robot(self) -> So101Robot:
        return self._robot

    @property
    def save_worker(self) -> So101SaveWorker:
        return self._save_worker

    @property
    def config(self) -> So101Config:
        return self._config

    @property
    def controller(self) -> So101SpaceMouseController:
        return self._controller

    def record(self, max_frames: int, if_async: bool = True) -> So101Obs:
        if max_frames <= 0:
            raise ValueError("max_frames must be greater than 0")

        try:
            obs = self._controller.record_parallel(
                max_frame=max_frames,
                fps=self.fps,
                is_async=if_async,
            )
        except KeyboardInterrupt as exc:
            sleep(0.5)
            self.close()
            raise RuntimeError("Recording stopped by user") from exc
        except Exception as exc:
            raise RuntimeError(
                f"Failed to record with SpaceMouse: {exc}"
            ) from exc

        return obs

    def send(
        self,
        max_frame: int,
        fps: int,
        leader_action: NDArray[np.float32],
    ) -> None:
        self._robot.send(
            max_frame=max_frame, fps=fps, leader_action=leader_action
        )

    def _extract_data_shapes(self, obs: So101Obs) -> dict[str, Any]:
        data_shape: Dict[str, Dict[str, Any]] = {}
        data_shape["arms"] = {
            "leader": list(obs.arms.leader.shape),
            "follower": list(obs.arms.follower.shape),
        }
        if obs.arms.leader_ee is not None:
            data_shape["arms"]["leader_ee"] = list(obs.arms.leader_ee.shape)
        if obs.arms.follower_ee is not None:
            data_shape["arms"]["follower_ee"] = list(obs.arms.follower_ee.shape)

        if obs.cameras:
            data_shape["sensors"] = {"cameras": {}}
            for name, frames in obs.cameras.items():
                if frames is None:
                    data_shape["sensors"]["cameras"][name] = None
                else:
                    data_shape["sensors"]["cameras"][name] = list(frames.shape)

        return data_shape

    def close(self) -> None:
        """Disconnect SpaceMouse + robot and shut down the save worker."""
        try:
            self._controller.disconnect()
        except Exception:
            pass
        if self._save_worker:
            try:
                self._save_worker.shutdown()
            except Exception:
                pass

    def record_save(
        self,
        max_frames: int,
        save_path: str,
        if_async: bool = True,
        save_gif: bool = True,
        warmup_time: int = 5,
    ) -> None:
        """Interactive record-and-save loop using SpaceMouse.

        Overrides the base class to use SpaceMouse teleoperation for the
        warmup phase (instead of the leader-arm-based teleoperation).
        """
        import os

        try:
            print("Starting SpaceMouse recording session...")
            if not self._controller.is_connected:
                self._controller.connect()

            while True:
                print("Press 'Enter' to warm up, or 'q' to quit")
                input_str = input()
                if input_str.lower() == "q":
                    print("Exiting...")
                    self.close()
                    sleep(0.5)
                    return

                print(f"Warming up for {warmup_time} seconds (SpaceMouse active)...")
                self._controller.teleoperation(max_seconds=warmup_time)

                print("Press 'Enter' to start recording...")
                input_str = input()
                print("Recording with SpaceMouse...")

                if if_async:
                    print("Using asynchronous recording...")

                obs = self.record(max_frames=max_frames, if_async=if_async)

                print(
                    "Recording finished. Print 1~9 to save data,"
                    " or 'e' to record again"
                )
                input_str = input()

                if input_str.lower() == "e":
                    print("Recording again...")
                    continue
                elif input_str in [str(i) for i in range(1, 10)]:
                    save_dir = os.path.join("data", f"{save_path}", f"{input_str}")
                    count = 1
                    unique_save_dir = save_dir + f"_{count}"
                    while os.path.exists(unique_save_dir):
                        unique_save_dir = f"{save_dir}_{count}"
                        count += 1
                    os.makedirs(unique_save_dir)

                    self.save_worker.save_all_obs(obs, unique_save_dir, save_gif)
                    data_shape = self._extract_data_shapes(obs)
                    self.save_metadata(unique_save_dir, data_shape)
                else:
                    print("Invalid input. Exiting...")
                    self.close()
                    return
        except Exception as e:
            self.close()
            raise RuntimeError(f"Failed to record from SpaceMouse: {e}")
        except KeyboardInterrupt:
            print("Recording stopped by user...")
            sleep(0.5)
            self.close()
