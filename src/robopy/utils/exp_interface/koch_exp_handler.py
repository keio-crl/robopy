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

        super().__init__(metadata_config, fps)
        self._config = koch_config

        self._robot = KochRobot(cfg=self._config)
        self._save_worker = KochSaveWorker(fps=self.fps)

        try:
            self._robot.connect()
        except Exception as exc:  # pragma: no cover - hardware interaction
            raise RuntimeError(f"Failed to connect to Koch robot: {exc}") from exc

    @property
    @override
    def robot(self) -> KochRobot:
        return self._robot

    @property
    @override
    def save_worker(self) -> KochSaveWorker:
        return self._save_worker

    @property
    @override
    def config(self) -> KochConfig:
        return self._config

    @override
    def record(self, max_frames: int, if_async: bool = True) -> KochObs:
        if max_frames <= 0:
            raise ValueError("max_frames must be greater than 0")

        try:
            obs = self.robot.record_parallel(max_frame=max_frames, fps=self.fps, is_async=if_async)
        except KeyboardInterrupt as exc:  # pragma: no cover - manual interruption
            sleep(0.5)
            self.close()
            raise RuntimeError("Recording stopped by user") from exc
        except Exception as exc:
            raise RuntimeError(f"Failed to record from Koch robot: {exc}") from exc

        return obs

    @override
    def send(self, max_frame: int, fps: int, leader_action: NDArray[np.float32]) -> None:
        self.robot.send(max_frame=max_frame, fps=fps, leader_action=leader_action)

    @override
    def _extract_data_shapes(self, obs: KochObs) -> dict:
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
