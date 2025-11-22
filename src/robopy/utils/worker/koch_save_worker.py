import os
from concurrent.futures import Future
from dataclasses import dataclass
from logging import getLogger
from typing import Dict, cast

import numpy as np
from numpy.typing import NDArray

from ..h5_handler import H5Handler
from .save_worker import SaveTask, SaveWorker

logger = getLogger(__name__)


@dataclass
class KochArmObs:
    leader: NDArray[np.float32]
    follower: NDArray[np.float32]


@dataclass
class KochObs:
    arms: KochArmObs
    cameras: Dict[str, NDArray[np.uint8] | NDArray[np.float32] | None]


class KochSaveWorker(SaveWorker[KochObs]):
    """Kochロボット用の非同期保存ワーカー."""

    def __init__(self, fps: int, worker_num: int = 2) -> None:
        super().__init__(worker_num=worker_num)
        self.fps = fps

    def _process_task(self, task: SaveTask) -> Future | None:
        match task.task_type:
            case "hierarchical":
                data = cast(Dict[str, dict], task.data)
                return self._executor.submit(
                    self._save_hierarchical_h5,
                    data,
                    task.save_path,
                )
            case "arm":
                leader, follower = cast(tuple[NDArray[np.float32], NDArray[np.float32]], task.data)
                return self._executor.submit(
                    self.save_arm_datas,
                    leader,
                    follower,
                    task.save_path,
                )
            case "gif":
                frames = cast(NDArray[np.float32], task.data)
                return self._executor.submit(self._save_camera_gif, frames, task.save_path)
            case _:
                logger.warning("未知のタスクタイプ: %s", task.task_type)
                return None

    def save_arm_datas(
        self, leader_obs: NDArray[np.float32], follower_obs: NDArray[np.float32], path: str
    ) -> None:
        """アームデータをHDF5形式で保存する."""
        os.makedirs(path, exist_ok=True)
        hierarchical_data = {
            "arm": {
                "leader": leader_obs,
                "follower": follower_obs,
            }
        }
        arm_h5_path = os.path.join(path, "koch_arm_observations.h5")
        H5Handler.save_hierarchical(hierarchical_data, arm_h5_path, compress=True)
        logger.info("Leaderアーム・FollowerアームのデータをHDF5形式で保存しました: %s", arm_h5_path)

    def save_all_obs(self, obs: KochObs, save_path: str, save_gif: bool) -> None:
        """観測データをバックグラウンドで保存する."""
        os.makedirs(save_path, exist_ok=True)
        camera_data, leader, follower = self._prepare_koch_obs(obs, save_path)

        hierarchical_data = self._build_hierarchical_data(camera_data, leader, follower)
        h5_path = os.path.join(save_path, "koch_observations.h5")
        self.enqueue_save_task(
            SaveTask(
                task_type="hierarchical",
                data=hierarchical_data,
                save_path=h5_path,
            )
        )

        self.enqueue_save_task(
            SaveTask(
                task_type="arm",
                data=(leader, follower),
                save_path=os.path.join(save_path, "arm"),
            )
        )

        if save_gif:
            gif_root = os.path.join(save_path, "camera_gif")
            for name, frames in camera_data.items():
                gif_path = os.path.join(gif_root, name, f"{name}.gif")
                self.enqueue_save_task(
                    SaveTask(
                        task_type="gif",
                        data=frames,
                        save_path=gif_path,
                    )
                )

        logger.info("観測データの保存処理をバックグラウンドで開始しました: %s", save_path)

    def shutdown(self) -> None:
        super().shutdown()

    def _prepare_koch_obs(
        self, obs: KochObs, save_dir: str
    ) -> tuple[
        Dict[str, NDArray[np.float32] | NDArray[np.uint8]], NDArray[np.float32], NDArray[np.float32]
    ]:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        camera_data = {name: frames for name, frames in obs.cameras.items() if frames is not None}
        if not camera_data:
            logger.warning("カメラデータが存在しません。")

        leader = obs.arms.leader
        follower = obs.arms.follower
        return camera_data, leader, follower

    def _save_camera_gif(self, frames: NDArray[np.float32] | NDArray[np.uint8], path: str) -> None:
        """シンプルなGIFを書き出す."""
        try:
            import imageio.v2 as imageio
        except ImportError:
            logger.warning("imageio がインストールされていないため GIF 保存をスキップします。")
            return

        os.makedirs(os.path.dirname(path), exist_ok=True)
        frame_data = frames
        if frame_data.shape[-3] == 3 or frame_data.shape[-3] == 1:
            frame_data = frame_data.transpose(0, 2, 3, 1)  # (N, C, H, W) -> (N, H, W, C)
        if frame_data.shape[-1] == 1:
            frame_data = frame_data[..., 0]  # (N, H, W, 1) -> (N, H, W)
            frame_data = frame_data.astype(np.float32) / max(float(np.max(frame_data)), 1e-6) * 255
        if frame_data.dtype != np.uint8:
            frame_data = np.clip(frame_data, 0, 255).astype(np.uint8)
        imageio.mimsave(path, list(frame_data), fps=self.fps)
        logger.info("GIFを %s に保存しました。", path)

    def _save_hierarchical_h5(self, data_dict: Dict[str, dict], file_path: str) -> None:
        H5Handler.save_hierarchical(data_dict, file_path, compress=True)
        logger.info("観測データをHDF5に保存しました: %s", file_path)

    def _build_hierarchical_data(
        self,
        camera_data: Dict[str, NDArray[np.float32] | NDArray[np.uint8]],
        leader: NDArray[np.float32],
        follower: NDArray[np.float32],
    ) -> Dict[str, dict]:
        hierarchical_data: Dict[str, dict] = {
            "arm": {
                "leader": leader,
                "follower": follower,
            },
            "camera": {},
        }
        for name, frames in camera_data.items():
            hierarchical_data["camera"][name] = frames
        return hierarchical_data
