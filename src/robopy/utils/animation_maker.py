import os
from logging import getLogger

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import ArtistAnimation

from robopy.config import RakudaObs
from robopy.config.robot_config.rakuda_config import RakudaArmObs, RakudaSensorObs
from robopy.utils.blocs_handler import BLOSCHandler

logger = getLogger(__name__)


def visualize_rakuda_obs(obs: RakudaObs, save_dir: str, fps: int) -> None:
    """Make animation from Rakuda sensor observation data and save to file.

    Args:
        obs (RakudaObs): Observation data from Rakuda robot.
        save_dir (str): Directory to save the animation file.
        fps (int): Frames per second for the animation.
    """
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    sensors_data = obs["sensors"]

    if sensors_data is None:
        raise ValueError("No sensor data available in the observation.")

    camera_data = sensors_data["cameras"]["main"]  # shape: (frames, H, W, C)
    left_tactile_data = sensors_data["tactile"]["left_digit"]  # shape: (frames, H, W, C)
    right_tactile_data = sensors_data["tactile"]["right_digit"]  # shape: (frames, H, W, C)
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    if left_tactile_data is None or right_tactile_data is None:
        raise ValueError("Tactile data is missing.")
    if camera_data is None:
        raise ValueError("Camera data is missing.")

    if not (camera_data.shape[0] == left_tactile_data.shape[0] == right_tactile_data.shape[0]):
        raise ValueError("Mismatch in number of frames between camera and tactile data.")

    leader = obs["arms"]["leader"]
    follower = obs["arms"]["follower"]

    make_rakuda_obs_animation(
        camera_data=camera_data,
        left_tactile_data=left_tactile_data,
        right_tactile_data=right_tactile_data,
        save_dir=save_dir,
        fps=fps,
    )

    make_rakuda_arm_obs(leader, follower, os.path.join(save_dir, "rakuda_arm_obs.png"))


def make_rakuda_obs_animation(
    camera_data: np.ndarray,
    left_tactile_data: np.ndarray,
    right_tactile_data: np.ndarray,
    save_dir: str,
    fps: int,
) -> None:
    """Make animation from Rakuda sensor observation data and save to file.

    Args:
        camera_data (np.ndarray): Camera data from Rakuda robot. Shape: (frames, H, W, C)
        left_tactile_data (np.ndarray): Left tactile sensor data. Shape: (frames, H, W, C)
        right_tactile_data (np.ndarray): Right tactile sensor data. Shape: (frames, H, W, C)
        save_dir (str): Directory to save the animation file.
        fps (int): Frames per second for the animation.
    """

    num_frames = camera_data.shape[0]

    # layout: 1 row, 3 columns (left tactile sensor | main camera | right tactile sensor)
    fig, axes = plt.subplots(1, 3, figsize=(15, 5))
    ims = []

    for i in range(num_frames):
        frame_artists = []

        lt_frame = left_tactile_data[i]
        lt_frame = np.clip(lt_frame / 255.0, 0, 1)
        im0 = axes[0].imshow(lt_frame, animated=True)
        axes[0].set_title("Left Digit Tactile Sensor")
        axes[0].axis("off")

        # camera - 値を正規化して0-1の範囲にクリップ
        cam_frame = camera_data[i]
        cam_frame = np.clip(cam_frame / 255.0, 0, 1)
        im1 = axes[1].imshow(cam_frame, animated=True)
        axes[1].set_title("Camera")
        axes[1].axis("off")

        # right tactile - 値を正規化して0-1の範囲にクリップ
        rt_frame = right_tactile_data[i]
        rt_frame = np.clip(rt_frame / 255.0, 0, 1)
        im2 = axes[2].imshow(rt_frame, animated=True)
        axes[2].set_title("Right Digit Tactile Sensor")
        axes[2].axis("off")

        frame_artists.extend([im0, im1, im2])

        ims.append(frame_artists)

    ani = ArtistAnimation(fig, ims, interval=1000 / fps, blit=True)
    save_path = os.path.join(save_dir, "rakuda_obs_animation.gif")
    ani.save(save_path, writer="pillow", fps=fps)
    plt.close(fig)
    logger.info(f"Animation saved to {save_path}")


def make_rakuda_arm_obs(leader: np.ndarray, follower: np.ndarray, save_path: str) -> None:
    if leader.shape[0] != follower.shape[0]:
        raise ValueError("Mismatch in number of frames between leader and follower arm data.")
    reshaped_leader = leader.T
    reshaped_follower = follower.T
    n: int = reshaped_leader.shape[0]

    fig, axes = plt.subplots(n, figsize=(6, 18))

    for i in range(n):
        axes[i].plot(reshaped_leader[i], label="leader")
        axes[i].plot(reshaped_follower[i], label="follower")
        axes[i].set_title(f"Joint {i + 1}")

    plt.tight_layout()
    plt.savefig(save_path)


if __name__ == "__main__":
    main_camera_raw = BLOSCHandler.load("./tests/test_rakuda/camera_data.blosc2").astype(np.float32)
    left_digit_raw = BLOSCHandler.load("./tests/test_rakuda/left_tactile.blosc2").astype(np.float32)
    right_digit_raw = BLOSCHandler.load("./tests/test_rakuda/right_tactile.blosc2").astype(
        np.float32
    )

    # main camera: (F,C,H,W) -> (F,H,W,C)
    main_camera_obs = main_camera_raw.transpose(0, 2, 3, 1)
    left_digit_obs = left_digit_raw
    right_digit_obs = right_digit_raw

    save_dir = "./tests/test_rakuda/"
    fps = 30
    visualize_rakuda_obs(
        RakudaObs(
            arms=RakudaArmObs(
                leader=np.random.random((150, 17)).astype(np.float32),
                follower=np.random.random((150, 17)).astype(np.float32),
            ),
            sensors=RakudaSensorObs(
                cameras={"main": main_camera_obs},
                tactile={"left_digit": left_digit_obs, "right_digit": right_digit_obs},
            ),
        ),
        save_dir=save_dir,
        fps=fps,
    )
