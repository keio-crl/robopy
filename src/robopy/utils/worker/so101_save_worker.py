import os
from concurrent.futures import Future
from dataclasses import dataclass
from logging import getLogger
from typing import Dict, cast

import numpy as np
from numpy.typing import NDArray

from ..h5_handler import H5Handler
from .save_worker import HierarchicalTaskData, SaveTask, SaveWorker

logger = getLogger(__name__)


@dataclass
class So101ArmObs:
    leader: NDArray[np.float32]
    follower: NDArray[np.float32]
    leader_ee: NDArray[np.float32] | None = None
    follower_ee: NDArray[np.float32] | None = None


@dataclass
class So101Obs:
    arms: So101ArmObs
    cameras: Dict[str, NDArray[np.uint8] | NDArray[np.float32] | None]


class So101SaveWorker(SaveWorker[So101Obs]):
    """SO-101 robot save worker for async data persistence."""

    def __init__(self, fps: int, worker_num: int = 2) -> None:
        super().__init__(worker_num=worker_num)
        self.fps = fps

    def _process_task(self, task: SaveTask) -> Future[None] | None:
        match task.task_type:
            case "hierarchical":
                data = cast(HierarchicalTaskData, task.data)
                return self._executor.submit(
                    self._save_hierarchical_h5,
                    data,
                    task.save_path,
                )
            case "arm":
                arm_obs = cast(So101ArmObs, task.data)
                return self._executor.submit(
                    self._save_arm_obs,
                    arm_obs,
                    task.save_path,
                )
            case "gif":
                frames = cast(NDArray[np.float32], task.data)
                return self._executor.submit(self._save_camera_gif, frames, task.save_path)
            case _:
                logger.warning("Unknown task type: %s", task.task_type)
                return None

    def save_arm_datas(
        self, leader_obs: NDArray[np.float32], follower_obs: NDArray[np.float32], path: str
    ) -> None:
        """Save arm data in HDF5 format (base class interface)."""
        self._save_arm_obs(So101ArmObs(leader=leader_obs, follower=follower_obs), path)

    def _save_arm_obs(self, arm_obs: So101ArmObs, path: str) -> None:
        """Save arm data (joint angles and EE coordinates) in HDF5 format."""
        os.makedirs(path, exist_ok=True)
        hierarchical_data: dict[str, dict[str, NDArray[np.float32]]] = {
            "arm": {
                "leader": arm_obs.leader,
                "follower": arm_obs.follower,
            },
        }
        if arm_obs.leader_ee is not None:
            hierarchical_data["arm"]["leader_ee"] = arm_obs.leader_ee
        if arm_obs.follower_ee is not None:
            hierarchical_data["arm"]["follower_ee"] = arm_obs.follower_ee

        arm_h5_path = os.path.join(path, "so101_arm_observations.h5")
        H5Handler.save_hierarchical(hierarchical_data, arm_h5_path, compress=True)
        logger.info("Saved SO-101 arm observations to HDF5: %s", arm_h5_path)

    def save_all_obs(self, obs: So101Obs, save_path: str, save_gif: bool) -> None:
        """Save observation data asynchronously."""
        os.makedirs(save_path, exist_ok=True)
        camera_data, arm_obs = self._prepare_obs(obs, save_path)

        hierarchical_data = self._build_hierarchical_data(camera_data, arm_obs)
        h5_path = os.path.join(save_path, "so101_observations.h5")
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
                data=arm_obs,
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

        logger.info("Enqueued SO-101 observation save tasks: %s", save_path)

    def shutdown(self) -> None:
        super().shutdown()

    def _prepare_obs(
        self, obs: So101Obs, save_dir: str
    ) -> tuple[
        Dict[str, NDArray[np.float32] | NDArray[np.uint8]], So101ArmObs
    ]:
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)

        camera_data = {name: frames for name, frames in obs.cameras.items() if frames is not None}
        if not camera_data:
            logger.warning("No camera data available.")

        return camera_data, obs.arms

    def _save_camera_gif(self, frames: NDArray[np.float32] | NDArray[np.uint8], path: str) -> None:
        """Save camera frames as GIF."""
        try:
            import imageio.v2 as imageio
        except ImportError:
            logger.warning("imageio is not installed, skipping GIF save.")
            return

        os.makedirs(os.path.dirname(path), exist_ok=True)
        frame_data = frames
        if frame_data.shape[-3] == 3 or frame_data.shape[-3] == 1:
            frame_data = frame_data.transpose(0, 2, 3, 1)
        if frame_data.shape[-1] == 1:
            frame_data = frame_data[..., 0]
            frame_data = frame_data.astype(np.float32) / max(float(np.max(frame_data)), 1e-6) * 255
        if frame_data.dtype != np.uint8:
            frame_data = np.clip(frame_data, 0, 255).astype(np.uint8)
        imageio.mimsave(path, list(frame_data), fps=self.fps)
        logger.info("Saved GIF to %s.", path)

    def _save_hierarchical_h5(self, data_dict: HierarchicalTaskData, file_path: str) -> None:
        H5Handler.save_hierarchical(data_dict, file_path, compress=True)
        logger.info("Saved SO-101 observations to HDF5: %s", file_path)

    def _build_hierarchical_data(
        self,
        camera_data: Dict[str, NDArray[np.float32] | NDArray[np.uint8]],
        arm_obs: So101ArmObs,
    ) -> HierarchicalTaskData:
        hierarchical_data: HierarchicalTaskData = {
            "arm": {
                "leader": arm_obs.leader,
                "follower": arm_obs.follower,
            },
            "camera": {},
        }
        if arm_obs.leader_ee is not None:
            hierarchical_data["arm"]["leader_ee"] = arm_obs.leader_ee
        if arm_obs.follower_ee is not None:
            hierarchical_data["arm"]["follower_ee"] = arm_obs.follower_ee
        for name, frames in camera_data.items():
            hierarchical_data["camera"][name] = frames
        return hierarchical_data
