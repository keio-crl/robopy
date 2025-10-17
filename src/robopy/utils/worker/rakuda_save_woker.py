import concurrent
import concurrent.futures
import os
from concurrent.futures import Future
from logging import getLogger
from typing import Dict

import matplotlib
from rich.console import Console
from rich.table import Table
from tqdm import tqdm

matplotlib.use("Agg")  # thread safe backend
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import ArtistAnimation
from numpy.typing import NDArray

from robopy.config import RakudaConfig
from robopy.config.robot_config.rakuda_config import RakudaObs
from robopy.utils.blosc_handler import BLOSCHandler

from .save_worker import SaveWorker

logger = getLogger(__name__)

consle = Console()


class RakudaSaveWorker(SaveWorker):
    def __init__(self, cfg: RakudaConfig, worker_num: int, fps: int) -> None:
        super().__init__()
        self.worker_num = worker_num
        self.cfg = cfg
        self.fps = fps

    def prepare_rakuda_obs(
        self, obs: RakudaObs, save_dir: str
    ) -> tuple[
        Dict[str, NDArray[np.float32]],
        Dict[str, NDArray[np.float32]],
        NDArray[np.float32],
        NDArray[np.float32],
    ]:
        """Make animation from Rakuda sensor observation data and save to file.

        Args:
            obs (RakudaObs): Observation data from Rakuda robot.
            save_dir (str): Directory to save the animation file.
            fps (int): Frames per second for the animation.
        """
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        try:
            sensors_data = obs.sensors

            if sensors_data is None:
                raise ValueError("No sensor data available in the observation.")

            camera_data = {
                name: data for name, data in sensors_data.cameras.items() if data is not None
            }

            tactile_data = {
                name: data for name, data in sensors_data.tactile.items() if data is not None
            }
            if not camera_data:
                raise ValueError("Camera data is missing.")

            leader = obs.arms.leader
            follower = obs.arms.follower

            return camera_data, tactile_data, leader, follower

        except Exception as e:
            raise RuntimeError(f"Failed to process Rakuda observation data: {e}")

    @staticmethod
    def make_rakuda_obs_animation(
        camera_data: Dict[str, NDArray[np.float32]],
        tactile_data: Dict[str, NDArray[np.float32]],
        save_dir: str,
        fps: int,
    ) -> None:
        """Make animation from Rakuda sensor observation data and save to file.

        Args:
            camera_data (np.ndarray): Camera data from Rakuda robot. Shape: (frames, C, H, W)
            left_tactile_data (np.ndarray): Left tactile sensor data. Shape: (frames, C, H, W)
            right_tactile_data (np.ndarray): Right tactile sensor data. Shape: (frames, C, H, W)
            save_dir (str): Directory to save the animation file.
            fps (int): Frames per second for the animation.
        """

        logger.info("Starting batch preprocessing of animation data...")

        processed_camera = {}
        for name, cam_data in camera_data.items():
            cam_data = cam_data.transpose(0, 2, 3, 1)  # (N, C, H, W) -> (N, H, W, C)
            processed_camera[name] = np.clip(cam_data / 255.0, 0, 1)

        for name, data in tactile_data.items():
            data = data.transpose(0, 2, 3, 1)  # (N, C, H, W) -> (N, H, W, C)
            tactile_data[name] = np.clip(data / 255.0, 0, 1)

        num_frames = list(processed_camera.values())[0].shape[0]

        all_fig_num = len(processed_camera) + len(tactile_data)
        rows = all_fig_num // 3 + int(all_fig_num % 3 != 0)
        cols = min(all_fig_num, 3)

        # layout: 1 row, N columns (cameras + 2 tactile sensors)
        fig = plt.figure(figsize=(5 * cols, 5 * rows))
        axes = fig.subplots(rows, cols)
        if rows == 1 and cols == 1:
            axes = [axes]
        ims = []

        logger.info(f"Generating animation frames for {num_frames} frames...")

        for i in range(num_frames):
            frame_artists = []
            for index, (name, cam_data) in enumerate(processed_camera.items()):
                im1 = axes[index].imshow(cam_data[i], animated=True)
                axes[index].set_title(f"Camera: {name}")
                axes[index].axis("off")
                frame_artists.append(im1)

            for t_index, (name, tactile) in enumerate(tactile_data.items()):
                idx = len(processed_camera) + t_index
                im = axes[idx].imshow(tactile[i], animated=True)
                axes[idx].set_title(f"Tactile Sensor: {name}")
                axes[idx].axis("off")
                frame_artists.append(im)

            # FPS情報を左上に表示
            fps_text = axes[0].text(
                0.02,
                0.75,
                f"FPS: {i}",
                color="white",
                bbox=dict(boxstyle="round,pad=0.3", facecolor="black", alpha=0.7),
                verticalalignment="top",
                transform=fig.transFigure,
            )

            frame_artists.extend([fps_text])

            ims.append(frame_artists)
        fig.tight_layout()
        plt.subplots_adjust(wspace=0.1, top=0.85)
        logger.info("Saving animation...")
        ani = ArtistAnimation(fig, ims, interval=1000 / fps, blit=True)
        ani.save(save_dir, writer="pillow", fps=fps)
        plt.close(fig)
        logger.info(f"Animation saved to {save_dir}")

    @staticmethod
    def make_rakuda_arm_obs(leader: np.ndarray, follower: np.ndarray, save_path: str) -> None:
        if leader.shape[0] != follower.shape[0]:
            raise ValueError("Mismatch in number of frames between leader and follower arm data.")

        reshaped_leader = leader.T
        reshaped_follower = follower.T
        n: int = reshaped_leader.shape[0]

        if reshaped_leader.ndim != 2 or reshaped_follower.ndim != 2:
            raise ValueError("Arm data must be 2-dimensional (joints x frames).")

        with plt.ioff():
            fig = plt.figure(figsize=(8, 24))
            axes = fig.subplots(n, 1)

            # 単一軸の場合の処理
            if n == 1:
                axes = [axes]

            for i in range(n):
                axes[i].clear()
                axes[i].plot(reshaped_leader[i], alpha=0.8, linestyle="-")
                axes[i].plot(reshaped_follower[i], alpha=0.8, linestyle="-.")
                axes[i].set_title(f"Joint {i + 1}")
                axes[i].grid(True, alpha=0.3)

            fig.legend(["Leader", "Follower"], loc="upper right")
            fig.tight_layout()
            fig.subplots_adjust(hspace=0.5, top=0.95)
            fig.savefig(save_path, dpi=150)
            plt.close(fig)

        logger.info(f"Arm observation plot saved to {save_path}")

    def _save_camera_data(
        self, camera_data: Dict[str, NDArray[np.float32]], save_path: str
    ) -> None:
        for name, data in camera_data.items():
            if data is None:
                continue
            # Save each camera's data using BLOSC compression
            file_path = os.path.join(save_path, f"{name}_camera_data.blosc")
            # Ensure array is C-contiguous before saving
            if not data.flags.c_contiguous:
                data = np.ascontiguousarray(data)
            BLOSCHandler.save(data, file_path)
            logger.info(f"Camera data for {name} saved to {file_path}")

    def _save_tactile_data(
        self,
        tactile_data: Dict[str, NDArray[np.float32]],
        save_path: str,
    ) -> None:
        for name, data in tactile_data.items():
            if data is None:
                continue
            # Save each tactile sensor's data using BLOSC compression
            file_path = os.path.join(save_path, f"{name}_tactile_data.blosc")
            logger.info(f"tactile data shape: {data.shape}")
            # Ensure array is C-contiguous before saving
            if not data.flags.c_contiguous:
                data = np.ascontiguousarray(data)
            BLOSCHandler.save(data, file_path)
            logger.info(f"Tactile data for {name} saved to {file_path}")

    def _save_array_safe(self, data: NDArray[np.float32], file_path: str) -> None:
        """Safely save array with BLOSC, ensuring C-contiguous memory layout."""
        if not data.flags.c_contiguous:
            data = np.ascontiguousarray(data)
        BLOSCHandler.save(data, file_path)

    def save_all_obs(self, obs: RakudaObs, save_path: str, save_gif: bool) -> None:
        camera_data, tactile_data, leader, follower = self.prepare_rakuda_obs(obs, save_path)

        table = Table(title="Rakuda Observation Save Summary")
        table.add_column("Data name", style="cyan", no_wrap=True)
        table.add_column("Shape", style="magenta")

        table.add_row("Leader Arm Data", str(leader.shape))
        table.add_row("Follower Arm Data", str(follower.shape))
        for name, data in camera_data.items():
            table.add_row(f"Camera: {name}", str(data.shape))
        for name, data in tactile_data.items():
            table.add_row(f"Tactile Sensor: {name}", str(data.shape))
        consle.print(table)

        if not os.path.exists(save_path):
            os.makedirs(save_path)

        with concurrent.futures.ThreadPoolExecutor(max_workers=self.worker_num) as executor:
            futures: list[Future] = []
            futures.extend(
                [
                    executor.submit(
                        self.make_rakuda_arm_obs,
                        leader,
                        follower,
                        os.path.join(save_path, "arm_obs.png"),
                    ),
                    executor.submit(
                        self._save_camera_data,
                        camera_data,
                        os.path.join(save_path),
                    ),
                    executor.submit(
                        self._save_tactile_data,
                        tactile_data,
                        os.path.join(save_path),
                    ),
                    executor.submit(
                        self._save_array_safe,
                        leader,
                        os.path.join(save_path, "leader_obs.blosc"),
                    ),
                    executor.submit(
                        self._save_array_safe,
                        follower,
                        os.path.join(save_path, "follower_obs.blosc"),
                    ),
                ]
            )
            if save_gif:
                futures.append(
                    executor.submit(
                        self.make_rakuda_obs_animation,
                        camera_data,
                        tactile_data,
                        os.path.join(save_path, "rakuda_obs_animation.gif"),
                        self.fps,
                    ),
                )

            for future in tqdm(concurrent.futures.as_completed(futures)):
                try:
                    future.result()  # 例外があれば再発生
                except Exception as e:
                    raise RuntimeError(f"Error in saving observation data: {e}")

    def save_datas(
        self, leader_obs: NDArray[np.float32], follower_obs: NDArray[np.float32], path: str
    ) -> None:
        if not os.path.exists(path):
            os.makedirs(path)

        # Ensure arrays are C-contiguous before saving
        if not leader_obs.flags.c_contiguous:
            leader_obs = np.ascontiguousarray(leader_obs)
        if not follower_obs.flags.c_contiguous:
            follower_obs = np.ascontiguousarray(follower_obs)

        BLOSCHandler.save(leader_obs, os.path.join(path, "leader_obs.blosc"))
        BLOSCHandler.save(follower_obs, os.path.join(path, "follower_obs.blosc"))
