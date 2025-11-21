import concurrent.futures
import os
import queue
import threading
from concurrent.futures import Future
from logging import getLogger
from typing import Dict, Literal, NamedTuple, cast

import matplotlib
from rich.console import Console
from rich.table import Table

matplotlib.use("Agg")  # thread safe backend

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import ArtistAnimation
from numpy.typing import NDArray

from robopy.config import RakudaConfig
from robopy.config.robot_config.rakuda_config import RakudaObs
from robopy.utils.blosc_handler import BLOSCHandler
from robopy.utils.h5_handler import H5Handler

from .save_worker import SaveWorker

logger = getLogger(__name__)

consle = Console()


class SaveTask(NamedTuple):
    """Data class representing a save task"""

    task_type: Literal[
        "camera", "tactile", "arm_obs", "arm", "animation", "metadata", "hierarchical"
    ]
    data: (
        tuple[Dict[str, NDArray[np.float32]], Dict[str, NDArray[np.float32]]]
        | tuple[NDArray[np.float32], NDArray[np.float32]]
        | Dict[str, NDArray[np.float32]]
        | Dict[str, dict]
    )  # type: ignore
    save_path: str = ""
    fps: int | None = None


class RakudaSaveWorker(SaveWorker):
    def __init__(self, cfg: RakudaConfig, worker_num: int, fps: int) -> None:
        super().__init__()
        self.worker_num = worker_num
        self.cfg = cfg
        self.fps = fps

        # Background save queue and executor setup
        self._save_queue: queue.Queue[SaveTask | None] = queue.Queue()
        self._executor = concurrent.futures.ThreadPoolExecutor(max_workers=worker_num)
        # Use non-daemon thread to ensure tasks complete before shutdown
        self._background_thread = threading.Thread(target=self._background_saver, daemon=False)
        self._background_thread.start()
        self._stop_event = threading.Event()
        self._futures: list[Future] = []

    def _background_saver(self) -> None:
        """Background worker thread that processes save tasks from the queue"""
        logger.info("Background saver thread started")

        while True:
            try:
                task = self._save_queue.get(timeout=1.0)

                if task is None:  # Finish signal
                    break

                match task.task_type:
                    case "camera":
                        camera_data = cast(Dict[str, NDArray[np.float32]], task.data)
                        logger.debug(f"Processing camera save task to {task.save_path}")
                        future = self._executor.submit(
                            self._save_camera_data, camera_data, task.save_path
                        )
                    case "tactile":
                        tactile_data = cast(Dict[str, NDArray[np.float32]], task.data)
                        logger.debug(f"Processing tactile save task to {task.save_path}")
                        future = self._executor.submit(
                            self._save_tactile_data, tactile_data, task.save_path
                        )
                    case "arm_obs":
                        arm_data = cast(tuple[NDArray[np.float32], NDArray[np.float32]], task.data)
                        logger.debug(f"Processing arm_obs save task to {task.save_path}")
                        future = self._executor.submit(
                            self.make_rakuda_arm_obs,
                            arm_data[0],  # leader
                            arm_data[1],  # follower
                            task.save_path,
                        )
                    case "animation":
                        data = cast(
                            tuple[Dict[str, NDArray[np.float32]], Dict[str, NDArray[np.float32]]],
                            task.data,
                        )
                        camera_data, tactile_data = data
                        logger.debug(f"Processing animation save task to {task.save_path}")
                        if (
                            camera_data is not None
                            and tactile_data is not None
                            and task.fps is not None
                        ):
                            future = self._executor.submit(
                                self.make_rakuda_obs_animation,
                                camera_data,
                                tactile_data,
                                task.save_path,
                                task.fps,
                            )
                        else:
                            logger.warning("Animation task missing required data")
                            self._save_queue.task_done()
                            continue

                    case "arm":
                        arm_data = cast(tuple[NDArray[np.float32], NDArray[np.float32]], task.data)
                        logger.debug(f"Processing arm save task to {task.save_path}")
                        future = self._executor.submit(
                            self.save_arm_datas,
                            arm_data[0],  # leader
                            arm_data[1],  # follower
                            task.save_path,
                        )

                    case "hierarchical":
                        hierarchical_data = cast(Dict[str, dict], task.data)
                        logger.debug(f"Processing hierarchical save task to {task.save_path}")
                        future = self._executor.submit(
                            self._save_hierarchical_h5,
                            hierarchical_data,
                            task.save_path,
                        )

                    case _:
                        logger.warning(f"Unknown task type: {task.task_type}")
                        self._save_queue.task_done()
                        continue

                self._futures.append(future)
                self._save_queue.task_done()
                logger.debug(f"Task queued, remaining: {self._save_queue.qsize()}")

            except queue.Empty:
                continue
            except Exception as e:
                logger.error(f"Error in background saver: {e}", exc_info=True)

        # Wait for all tasks to complete
        logger.info(f"Waiting for {len(self._futures)} tasks to complete...")
        for i, future in enumerate(concurrent.futures.as_completed(self._futures)):
            try:
                future.result()
                logger.debug(f"Task {i + 1}/{len(self._futures)} completed")
            except Exception as e:
                logger.error(f"Error in background save task {i + 1}: {e}", exc_info=True)

        logger.info("Background saver thread finished successfully")

    def enqueue_save_task(self, task: SaveTask) -> None:
        """Add a save task to the queue (non-blocking)"""
        self._save_queue.put(task)

    def wait_all_saved(self) -> None:
        """Wait for all save tasks to complete"""
        logger.info("Waiting for all queued tasks to complete...")
        self._save_queue.join()
        logger.info("All queued tasks marked as done")

    def shutdown(self) -> None:
        """Shutdown the save worker"""
        logger.info("Shutting down save worker...")

        # Wait for all tasks in the queue to complete
        self.wait_all_saved()

        # Send shutdown signal to background thread
        logger.info("Sending shutdown signal to background thread...")
        self._stop_event.set()
        self._save_queue.put(None)  # Shutdown signal

        # Wait for background thread to finish (timeout: 60 seconds)
        logger.info("Waiting for background thread to finish...")
        self._background_thread.join(timeout=60)

        if self._background_thread.is_alive():
            logger.warning("Background thread did not finish in time")
        else:
            logger.info("Background thread finished")

        # Shutdown ThreadPoolExecutor
        logger.info("Shutting down thread pool executor...")
        self._executor.shutdown(wait=True)
        logger.info("Save worker shutdown complete")

    def prepare_rakuda_obs(
        self, obs: RakudaObs, save_dir: str
    ) -> tuple[
        Dict[str, NDArray[np.float32]],
        Dict[str, NDArray[np.float32]],
        NDArray[np.float32],
        NDArray[np.float32],
    ]:
        """Extract and prepare Rakuda sensor observation data for saving.

        Args:
            obs (RakudaObs): Observation data from Rakuda robot.
            save_dir (str): Directory to save the data.

        Returns:
            tuple: (camera_data, tactile_data, leader, follower)

        Raises:
            RuntimeError: If failed to process observation data.
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
        """Generate and save animation from camera and tactile sensor data.

        Args:
            camera_data (Dict[str, NDArray[np.float32]]): Camera data.
                Shape: (frames, C, H, W)
            tactile_data (Dict[str, NDArray[np.float32]]): Tactile sensor data.
                Shape: (frames, C, H, W)
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

        # Layout: multiple columns for cameras and tactile sensors
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

            # Display FPS information in upper left
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
        """Save arm observation data as a plot image.

        Args:
            leader (np.ndarray): Leader arm positions. Shape: (frames, joints)
            follower (np.ndarray): Follower arm positions. Shape: (frames, joints)
            save_path (str): Path to save the plot image.

        Raises:
            ValueError: If data dimensions are invalid.
        """
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

            # Handle single axis case
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
        """Save camera data using BLOSC compression.

        Args:
            camera_data (Dict[str, NDArray[np.float32]]): Camera data by name.
            save_path (str): Base path to save the camera data files.
        """
        for name, data in camera_data.items():
            if data is None:
                continue
            # Save each camera's data using BLOSC compression
            file_path = os.path.join(save_path, "camera", name, f"{name}_camera_data.blosc")
            if not os.path.exists(os.path.dirname(file_path)):
                os.makedirs(os.path.dirname(file_path))
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
        """Save tactile sensor data using BLOSC compression.

        Args:
            tactile_data (Dict[str, NDArray[np.float32]]): Tactile sensor data by name.
            save_path (str): Base path to save the tactile data files.
        """
        for name, data in tactile_data.items():
            if data is None:
                continue
            # Save each tactile sensor's data using BLOSC compression
            file_path = os.path.join(save_path, "tactile", name, f"{name}_tactile_data.blosc")
            if not os.path.exists(os.path.dirname(file_path)):
                os.makedirs(os.path.dirname(file_path))
            logger.info(f"tactile data shape: {data.shape}")
            # Ensure array is C-contiguous before saving
            if not data.flags.c_contiguous:
                data = np.ascontiguousarray(data)
            BLOSCHandler.save(data, file_path)
            logger.info(f"Tactile data for {name} saved to {file_path}")

    def _save_array_safe(self, data: NDArray[np.float32], file_path: str) -> None:
        """Safely save array with BLOSC, ensuring C-contiguous memory layout.

        Args:
            data (NDArray[np.float32]): Array to save.
            file_path (str): Path to save the array file.
        """
        if not data.flags.c_contiguous:
            data = np.ascontiguousarray(data)
        BLOSCHandler.save(data, file_path)

    def _save_hierarchical_h5(self, data_dict: Dict[str, dict], file_path: str) -> None:
        """Save hierarchical data structure to HDF5 file.

        Args:
            data_dict (Dict[str, dict]): Hierarchical dictionary of data.
            file_path (str): Path to save the HDF5 file.
        """
        H5Handler.save_hierarchical(data_dict, file_path, compress=True)
        logger.info(f"Hierarchical data saved to {file_path}")

    def _build_hierarchical_data(
        self,
        camera_data: Dict[str, NDArray[np.float32]],
        tactile_data: Dict[str, NDArray[np.float32]],
        leader: NDArray[np.float32],
        follower: NDArray[np.float32],
    ) -> Dict[str, dict]:
        """Build hierarchical data structure for HDF5 storage.

        Args:
            camera_data (Dict[str, NDArray[np.float32]]): Camera data by name.
            tactile_data (Dict[str, NDArray[np.float32]]): Tactile sensor data by name.
            leader (NDArray[np.float32]): Leader arm positions.
            follower (NDArray[np.float32]): Follower arm positions.

        Returns:
            Dict[str, dict]: Hierarchical data structure.
        """
        hierarchical_data: Dict[str, dict] = {
            "camera": {},
            "tactile": {},
            "arm": {
                "leader": leader,
                "follower": follower,
            },
        }

        # Add camera data to hierarchy
        for name, data in camera_data.items():
            if data is not None:
                hierarchical_data["camera"][name] = data

        # Add tactile data to hierarchy
        for name, data in tactile_data.items():
            if data is not None:
                hierarchical_data["tactile"][name] = data

        return hierarchical_data

    def save_all_obs(self, obs: RakudaObs, save_path: str, save_gif: bool) -> None:
        """Queue observation data for background saving.

        Supports both BLOSC (hierarchical file structure) and HDF5 (single file) formats.

        Args:
            obs (RakudaObs): Observation data to save.
            save_path (str): Directory path to save the data.
            save_gif (bool): Whether to generate GIF animation.
        """
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

        # Build hierarchical data structure for HDF5 format
        hierarchical_data = self._build_hierarchical_data(
            camera_data, tactile_data, leader, follower
        )

        # Save as single HDF5 file (unified format)
        h5_file_path = os.path.join(save_path, "rakuda_observations.h5")
        self.enqueue_save_task(
            SaveTask(
                task_type="hierarchical",
                data=hierarchical_data,
                save_path=h5_file_path,
            )
        )

        # Queue save tasks (non-blocking)
        # Save arm observations visualization
        self.enqueue_save_task(
            SaveTask(
                task_type="arm_obs",
                data=(leader, follower),
                save_path=os.path.join(save_path, "arm_obs.png"),
            )
        )

        # Generate GIF (optional)
        if save_gif:
            self.enqueue_save_task(
                SaveTask(
                    task_type="animation",
                    data=(camera_data, tactile_data),
                    save_path=os.path.join(save_path, "rakuda_obs_animation.gif"),
                    fps=self.fps,
                )
            )

        logger.info(f"Queued all save tasks for {save_path}. Processing in background...")

    def save_arm_datas(
        self, leader_obs: NDArray[np.float32], follower_obs: NDArray[np.float32], path: str
    ) -> None:
        """Save arm observation data as BLOSC compressed files.

        Args:
            leader_obs (NDArray[np.float32]): Leader arm positions. Shape: (frames, joints)
            follower_obs (NDArray[np.float32]): Follower arm positions. Shape: (frames, joints)
            path (str): Directory path to save the data.
        """
        if not os.path.exists(path):
            os.makedirs(path)

        # Ensure arrays are C-contiguous before saving
        if not leader_obs.flags.c_contiguous:
            leader_obs = np.ascontiguousarray(leader_obs)
        if not follower_obs.flags.c_contiguous:
            follower_obs = np.ascontiguousarray(follower_obs)
        leader_save_path = os.path.join(path, "arm", "leader", "leader_arm_data.blosc")
        follower_save_path = os.path.join(path, "arm", "follower", "follower_arm_data.blosc")
        if not os.path.exists(os.path.dirname(leader_save_path)):
            os.makedirs(os.path.dirname(leader_save_path))
        if not os.path.exists(os.path.dirname(follower_save_path)):
            os.makedirs(os.path.dirname(follower_save_path))

        BLOSCHandler.save(leader_obs, leader_save_path)
        BLOSCHandler.save(follower_obs, follower_save_path)
