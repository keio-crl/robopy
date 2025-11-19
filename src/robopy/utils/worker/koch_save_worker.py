import os
from dataclasses import dataclass
from logging import getLogger
from typing import Dict

import numpy as np
from numpy.typing import NDArray

from .save_worker import SaveWorker

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
    """シンプルなKochロボット用保存ワーカー."""

    def __init__(self, fps: int, worker_num: int = 2) -> None:
        self.fps = fps
        self.worker_num = worker_num

    def save_arm_datas(
        self, leader_obs: NDArray[np.float32], follower_obs: NDArray[np.float32], path: str
    ) -> None:
        """アームデータをnp.save形式で保存する."""
        leader_dir = os.path.join(path, "arm", "leader")
        follower_dir = os.path.join(path, "arm", "follower")
        os.makedirs(leader_dir, exist_ok=True)
        os.makedirs(follower_dir, exist_ok=True)

        leader_path = os.path.join(leader_dir, "leader_arm.npy")
        follower_path = os.path.join(follower_dir, "follower_arm.npy")

        np.save(leader_path, leader_obs)
        np.save(follower_path, follower_obs)
        logger.info("Leaderアーム・Followerアームのデータを保存しました。")

    def save_all_obs(self, obs: KochObs, save_path: str, save_gif: bool) -> None:
        """観測データ全体を保存する."""
        os.makedirs(save_path, exist_ok=True)

        # アームデータ保存
        self.save_arm_datas(obs.arms.leader, obs.arms.follower, save_path)

        # カメラデータ保存
        cameras_root = os.path.join(save_path, "camera")
        for name, frames in obs.cameras.items():
            if frames is None:
                logger.warning("カメラ %s のフレームが取得できなかったため保存をスキップします。", name)
                continue

            camera_dir = os.path.join(cameras_root, name)
            os.makedirs(camera_dir, exist_ok=True)

            camera_path = os.path.join(camera_dir, f"{name}_frames.npy")
            np.save(camera_path, frames)
            logger.info("カメラ %s のフレームを %s に保存しました。", name, camera_path)

            if save_gif:
                gif_path = os.path.join(camera_dir, f"{name}.gif")
                self._save_camera_gif(frames, gif_path)

    def _save_camera_gif(self, frames: NDArray[np.float32] | NDArray[np.uint8], path: str) -> None:
        """シンプルなGIFを書き出す."""
        try:
            import imageio.v2 as imageio
        except ImportError:
            logger.warning("imageio がインストールされていないため GIF 保存をスキップします。")
        else:
            frame_data = frames
            if frame_data.dtype != np.uint8:
                frame_data = np.clip(frame_data, 0, 255).astype(np.uint8)
            if frame_data.shape[-3] == 3:
                frame_data = frame_data.transpose(0, 2, 3, 1)  # (N, C, H, W) -> (N, H, W, C)
            if frame_data.ndim == 3:
                frame_data = frame_data[..., np.newaxis]  # (N, H, W) -> (N, H, W, 1)
            imageio.mimsave(path, list(frame_data), fps=self.fps)
            logger.info("GIFを %s に保存しました。", path)


