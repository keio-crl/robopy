"""Test script for saving and visualizing robot sensor data."""

import json
import logging
import os
import time
import traceback
from datetime import datetime
from typing import Optional

import cv2
import librosa
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import AudioParams, CameraParams, TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot
from robopy.utils.worker.rakuda_save_worker import RakudaSaveWorker

# Constants
TARGET_SIZE = (48, 64)  # Target size for downsampling (H, W)
RECORDING_FPS = 20
RECORDING_MAX_FRAMES = 2000
VISUALIZATION_FPS = 20
SAVE_WORKER_FPS = 20
LOGGER = logging.getLogger(__name__)


def create_test_config() -> RakudaConfig:
    """Create configuration for real robot test.

    Returns:
        RakudaConfig: Configuration object for the robot test.
    """
    # Camera configuration (RealSense) - 30fps
    camera_params = CameraParams(name="main", width=640, height=480, fps=30, index=0)

    # Audio sensor configuration - 20fps (audio processing rate)
    audio_params = AudioParams(name="main", fps=20)

    # Tactile sensor configuration (Digit) - 30fps
    tactile_params_left = TactileParams(serial_num="D20542", name="left", fps=30)
    tactile_params_right = TactileParams(serial_num="D20537", name="right", fps=30)

    # Sensor parameters
    sensor_params = RakudaSensorParams(
        cameras=[camera_params],
        tactile=[tactile_params_left, tactile_params_right],
        audio=[audio_params],
    )

    # Robot configuration with real USB ports
    config = RakudaConfig(
        # leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1", sensors=sensor_params
        leader_port="/dev/ttyUSB1",
        follower_port="/dev/ttyUSB0",
        sensors=sensor_params,
    )
    return config


def _downsample_4d_array(
    data: np.ndarray, target_size: tuple[int, int] = TARGET_SIZE
) -> np.ndarray:
    """Downsample 4D array (T, C, H, W) to target size.

    Args:
        data: Input array with shape (T, C, H, W).
        target_size: Target size (H, W) for downsampling.

    Returns:
        Resized array with shape (T, C, target_h, target_w).
    """
    target_h, target_w = target_size
    T, C, _H, _W = data.shape
    resized = np.empty((T, C, target_h, target_w), dtype=data.dtype)

    for t in range(T):
        for c in range(C):
            resized[t, c] = cv2.resize(
                data[t, c],
                (target_w, target_h),
                interpolation=cv2.INTER_AREA,
            )

    return resized


def _downsample_tactile_obs(obs, target_size: tuple[int, int] = TARGET_SIZE) -> None:
    """Downsample tactile images in observation to target_size (H, W).

    Args:
        obs: Observation object containing sensor data.
        target_size: Target size (H, W) for downsampling.
    """
    if obs.sensors is None or obs.sensors.tactile is None:
        return

    for name, data in obs.sensors.tactile.items():
        if data is None or data.ndim != 4:
            continue
        obs.sensors.tactile[name] = _downsample_4d_array(data, target_size)


def _downsample_camera_obs(obs, target_size: tuple[int, int] = TARGET_SIZE) -> None:
    """Downsample camera images in observation to target_size (H, W).

    Args:
        obs: Observation object containing sensor data.
        target_size: Target size (H, W) for downsampling.
    """
    if obs.sensors is None or obs.sensors.cameras is None:
        return

    for name, data in obs.sensors.cameras.items():
        if data is None or data.ndim != 4:
            continue
        obs.sensors.cameras[name] = _downsample_4d_array(data, target_size)


def _downsample_audio_obs(obs, target_size: tuple[int, int] = TARGET_SIZE) -> None:
    """Downsample audio mel spectrograms in observation to target_size (H, W).

    Args:
        obs: Observation object containing sensor data.
        target_size: Target size (H, W) for downsampling.
    """
    if obs.sensors is None or obs.sensors.audio is None:
        return

    target_h, target_w = target_size

    for name, data in obs.sensors.audio.items():
        if data is None:
            continue

        if data.ndim == 2:
            # (T, H) format: reshape to (H, 1) then resize
            T, H = data.shape
            resized = np.empty((T, target_h, target_w), dtype=data.dtype)

            for t in range(T):
                frame_2d = data[t].reshape(H, 1)
                resized[t] = cv2.resize(
                    frame_2d,
                    (target_w, target_h),
                    interpolation=cv2.INTER_AREA,
                )

            obs.sensors.audio[name] = resized
        elif data.ndim == 3:
            # (T, H, W) format: resize each frame
            T, H, _W = data.shape
            resized = np.empty((T, target_h, target_w), dtype=data.dtype)

            for t in range(T):
                resized[t] = cv2.resize(
                    data[t],
                    (target_w, target_h),
                    interpolation=cv2.INTER_AREA,
                )

            obs.sensors.audio[name] = resized


def _normalize_image(frame: np.ndarray) -> np.ndarray:
    """Normalize image frame to 0-1 range if needed.

    Args:
        frame: Image frame (can be in 0-255 or 0-1 range).

    Returns:
        Normalized frame in 0-1 range.
    """
    if frame.max() > 1.0:
        return frame / 255.0
    return frame


def _prepare_camera_frame(camera_data: np.ndarray, frame_idx: int) -> np.ndarray:
    """Prepare camera frame for display.

    Args:
        camera_data: Camera data array with shape (T, C, H, W).
        frame_idx: Frame index.

    Returns:
        Display-ready frame with shape (H, W, C).
    """
    frame = np.transpose(camera_data[frame_idx], (1, 2, 0))
    return _normalize_image(frame)


def _prepare_tactile_frame(tactile_data: np.ndarray, frame_idx: int) -> np.ndarray:
    """Prepare tactile frame for display.

    Args:
        tactile_data: Tactile data array with shape (T, C, H, W).
        frame_idx: Frame index.

    Returns:
        Display-ready frame with shape (H, W, C).
    """
    frame = np.transpose(tactile_data[frame_idx], (1, 2, 0))
    return _normalize_image(frame)


def _get_audio_visualization_params(
    audio_data: np.ndarray,
) -> tuple[float, float, float, float, int]:
    """Calculate audio visualization parameters.

    Args:
        audio_data: Audio data array.

    Returns:
        Tuple of (freq_min, freq_max, vmin, vmax, audio_w) for visualization.
    """
    audio_params = AudioParams()
    audio_shape = audio_data[0].shape

    if len(audio_shape) == 2:
        audio_w = audio_shape[1]
    else:
        audio_w = audio_shape[1] if len(audio_shape) > 1 else audio_shape[0]

    # Calculate mel frequencies
    mel_freqs = librosa.mel_frequencies(
        n_mels=audio_params.n_mels, fmin=0, fmax=audio_params.fmax, htk=False
    )
    freq_min = mel_freqs[0]
    freq_max = mel_freqs[-1]

    # Calculate global statistics for consistent color scaling
    all_audio_data = np.concatenate([audio_data[i] for i in range(len(audio_data))], axis=0)
    vmin = np.percentile(all_audio_data, 1.0)
    vmax = np.percentile(all_audio_data, 99.0)

    return freq_min, freq_max, vmin, vmax, audio_w


def _extract_sensor_data(obs):
    """Extract sensor data from observation.

    Returns:
        Tuple of (camera_data, audio_data, tactile_data_left, tactile_data_right).
    """
    camera_data = obs.sensors.cameras.get("main") if obs.sensors.cameras else None
    audio_data = obs.sensors.audio.get("main") if obs.sensors.audio else None
    tactile_data_left = obs.sensors.tactile.get("left") if obs.sensors.tactile else None
    tactile_data_right = obs.sensors.tactile.get("right") if obs.sensors.tactile else None

    # Fallback to "main" if left/right don't exist
    if tactile_data_left is None and obs.sensors.tactile:
        tactile_data_left = obs.sensors.tactile.get("main")
    if tactile_data_right is None and obs.sensors.tactile and tactile_data_left is None:
        tactile_data_right = obs.sensors.tactile.get("main")

    return camera_data, audio_data, tactile_data_left, tactile_data_right


def _get_num_frames(
    camera_data: Optional[np.ndarray],
    audio_data: Optional[np.ndarray],
    tactile_data_left: Optional[np.ndarray],
    tactile_data_right: Optional[np.ndarray],
) -> int:
    """Get number of frames from available sensor data.

    Returns:
        Number of frames, or 0 if no data available.
    """
    if camera_data is not None:
        return len(camera_data)
    if audio_data is not None:
        return len(audio_data)
    if tactile_data_left is not None:
        return len(tactile_data_left)
    if tactile_data_right is not None:
        return len(tactile_data_right)
    return 0


def _print_frame_progress(current_frame: int, total_frames: int, label: str) -> None:
    """Log frame progress."""
    if total_frames <= 0:
        return
    current_frame = min(max(current_frame, 0), total_frames)
    LOGGER.info("%s: %s/%s frames", label, current_frame, total_frames)


def _setup_camera_plot(ax, camera_data: Optional[np.ndarray]):
    """Set up camera plot.

    Returns:
        Image object for animation, or None if no data.
    """
    if camera_data is not None:
        camera_frame = _prepare_camera_frame(camera_data, 0)
        im = ax.imshow(camera_frame, aspect="auto", vmin=0, vmax=1)
        ax.set_title("Camera")
        ax.axis("off")
        return im
    else:
        ax.text(0.5, 0.5, "No Camera Data", ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Camera")
        return None


def _setup_audio_plot(ax, audio_data: Optional[np.ndarray]):
    """Set up audio/mel spectrogram plot.

    Returns:
        Tuple of (image object, vmin, vmax) for animation, or (None, None, None) if no data.
    """
    if audio_data is not None:
        freq_min, freq_max, vmin, vmax, audio_w = _get_audio_visualization_params(audio_data)

        im = ax.imshow(
            audio_data[0],
            cmap="magma",
            aspect="auto",
            extent=[0, audio_w, freq_min, freq_max],
            origin="upper",
            vmin=vmin,
            vmax=vmax,
        )
        ax.set_title("Mel Spectrogram")
        ax.set_xlabel("Time (frames)")
        ax.set_ylabel("Frequency (Hz)")
        return im, vmin, vmax
    else:
        ax.text(0.5, 0.5, "No Audio Data", ha="center", va="center", transform=ax.transAxes)
        ax.set_title("Audio Spectrogram")
        return None, None, None


def _setup_tactile_plot(ax, tactile_data: Optional[np.ndarray], title: str):
    """Set up tactile sensor plot.

    Returns:
        Image object for animation, or None if no data.
    """
    if tactile_data is not None:
        tactile_frame = _prepare_tactile_frame(tactile_data, 0)
        im = ax.imshow(tactile_frame, aspect="auto", vmin=0, vmax=1)
        ax.set_title(title)
        ax.axis("off")
        return im
    else:
        ax.text(0.5, 0.5, "No Tactile Data", ha="center", va="center", transform=ax.transAxes)
        ax.set_title(title)
        return None


def create_sensor_gif_visualization(obs, save_path: str, fps: int = VISUALIZATION_FPS) -> bool:
    """Create GIF visualization with 4 sensor animations.

    Args:
        obs: Observation object containing sensor data.
        save_path: Path to save the GIF file.
        fps: Frames per second for the animation.

    Returns:
        True if successful, False otherwise.
    """
    print("\n🎬 Creating Sensor GIF visualization...")

    os.makedirs(save_path, exist_ok=True)

    # Extract sensor data
    camera_data, audio_data, tactile_data_left, tactile_data_right = _extract_sensor_data(obs)

    # Get number of frames
    num_frames = _get_num_frames(camera_data, audio_data, tactile_data_left, tactile_data_right)
    if num_frames == 0:
        print("❌ No sensor data available for visualization")
        return False

    print("📊 Data for visualization:")
    print(f"   - Frames: {num_frames}")
    if camera_data is not None:
        print(f"   - Camera: {camera_data.shape}")
    if audio_data is not None:
        print(f"   - Audio: {audio_data.shape}")
    if tactile_data_left is not None:
        print(f"   - Tactile Left: {tactile_data_left.shape}")
    if tactile_data_right is not None:
        print(f"   - Tactile Right: {tactile_data_right.shape}")

    # Create figure with 2x2 subplots
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle("Sensor Data Visualization", fontsize=16)

    # Set up plots
    camera_im = _setup_camera_plot(axes[0, 0], camera_data)
    audio_im, audio_vmin, audio_vmax = _setup_audio_plot(axes[0, 1], audio_data)
    tactile_im = _setup_tactile_plot(axes[1, 0], tactile_data_left, "Tactile Sensor (Left)")
    tactile_im2 = _setup_tactile_plot(axes[1, 1], tactile_data_right, "Tactile Sensor (Right)")

    # Animation function
    def animate(frame):
        nonlocal audio_vmin, audio_vmax

        _print_frame_progress(frame + 1, num_frames, "🎞️ GIF encoding")

        # Update title with time information
        current_time = frame / fps
        total_time = num_frames / fps
        time_str = f"Time: {current_time:.2f}s / {total_time:.2f}s ({frame}/{num_frames})"
        fig.suptitle(f"Sensor Data Visualization - {time_str}", fontsize=16)

        # Update camera
        if camera_data is not None and camera_im is not None and frame < len(camera_data):
            camera_frame = _prepare_camera_frame(camera_data, frame)
            camera_im.set_array(camera_frame)

        # Update audio
        if audio_data is not None and audio_im is not None and frame < len(audio_data):
            audio_im.set_array(audio_data[frame])
            if audio_vmin is not None and audio_vmax is not None:
                audio_im.set_clim(vmin=audio_vmin, vmax=audio_vmax)

        # Update tactile left
        if (
            tactile_data_left is not None
            and tactile_im is not None
            and frame < len(tactile_data_left)
        ):
            tactile_frame = _prepare_tactile_frame(tactile_data_left, frame)
            tactile_im.set_array(tactile_frame)

        # Update tactile right
        if (
            tactile_data_right is not None
            and tactile_im2 is not None
            and frame < len(tactile_data_right)
        ):
            tactile_frame = _prepare_tactile_frame(tactile_data_right, frame)
            tactile_im2.set_array(tactile_frame)

        return [camera_im, audio_im, tactile_im, tactile_im2]

    # Create and save animation
    try:
        anim = FuncAnimation(fig, animate, frames=num_frames, interval=500 // fps, blit=False)
        gif_path = os.path.join(save_path, "sensor_images.gif")
        print(f"💾 Saving GIF to: {gif_path}")
        anim.save(gif_path, writer="pillow", fps=fps)
        print(f"✅ GIF saved successfully: {gif_path}")
        return True
    except Exception as e:
        print(f"❌ Error saving visualization: {e}")
        return False
    finally:
        plt.close(fig)


def verify_saved_data(save_path: str) -> bool:
    """Verify that the saved data is complete and valid.

    Args:
        save_path: Path to the saved data directory.

    Returns:
        True if all files are valid, False otherwise.
    """
    print(f"\n🔍 Verifying saved data in: {save_path}")

    if not os.path.exists(save_path):
        print(f"❌ Save directory does not exist: {save_path}")
        return False

    # Check HDF5 file
    h5_file = os.path.join(save_path, "rakuda_observations.h5")
    if not os.path.exists(h5_file):
        print(f"❌ HDF5 file not found: {h5_file}")
        return False

    file_size = os.path.getsize(h5_file)
    if file_size == 0:
        print(f"❌ HDF5 file is empty: {h5_file}")
        return False

    print(f"✅ HDF5 file found: {h5_file} ({file_size:,} bytes)")

    # Check metadata file
    metadata_file = os.path.join(save_path, "metadata.json")
    if not os.path.exists(metadata_file):
        print(f"❌ Metadata file not found: {metadata_file}")
        return False

    metadata_size = os.path.getsize(metadata_file)
    if metadata_size == 0:
        print(f"❌ Metadata file is empty: {metadata_file}")
        return False

    print(f"✅ Metadata file found: {metadata_file} ({metadata_size:,} bytes)")

    # Check visualization file
    sensor_gif = os.path.join(save_path, "sensor_images.gif")
    if os.path.exists(sensor_gif):
        gif_size = os.path.getsize(sensor_gif)
        print(f"✅ Sensor GIF found: {sensor_gif} ({gif_size:,} bytes)")
    else:
        print(f"❌ Sensor GIF not found: {sensor_gif}")

    return True


def _print_data_shapes(obs) -> None:
    """Print shapes of recorded data.

    Args:
        obs: Observation object containing recorded data.
    """
    print("📈 Recorded data shapes:")
    print(f"   - Leader arm: {obs.arms.leader.shape}")
    print(f"   - Follower arm: {obs.arms.follower.shape}")

    if obs.sensors.cameras:
        for name, data in obs.sensors.cameras.items():
            if data is not None:
                print(f"   - Camera {name}: {data.shape}")
            else:
                print(f"   - Camera {name}: No data (None)")

    if obs.sensors.tactile:
        for name, data in obs.sensors.tactile.items():
            if data is not None:
                print(f"   - Tactile {name}: {data.shape}")
            else:
                print(f"   - Tactile {name}: No data (None)")

    if obs.sensors.audio:
        for name, data in obs.sensors.audio.items():
            if data is not None:
                print(f"   - Audio {name}: {data.shape}")
            else:
                print(f"   - Audio {name}: No data (None)")


def _create_metadata(obs, config: RakudaConfig, save_path: str) -> None:
    """Create and save metadata file.

    Args:
        obs: Observation object containing recorded data.
        config: Robot configuration.
        save_path: Path to save the metadata file.
    """
    assert config.sensors is not None
    print("📝 Creating metadata file...")
    metadata = {
        "task_details": {
            "task_name": "saver_test",
            "description": "Saver test with follower arm PNG and 4-sensor GIF",
            "timestamp": datetime.now().isoformat(),
        },
        "data_shape": {
            "leader_arm": obs.arms.leader.shape,
            "follower_arm": obs.arms.follower.shape,
            "cameras": (
                {name: data.shape for name, data in obs.sensors.cameras.items()}
                if obs.sensors.cameras
                else {}
            ),
            "tactile": (
                {name: data.shape for name, data in obs.sensors.tactile.items()}
                if obs.sensors.tactile
                else {}
            ),
            "audio": (
                {name: data.shape for name, data in obs.sensors.audio.items() if data is not None}
                if obs.sensors.audio
                else {}
            ),
        },
        "robot_config": {
            "leader_port": config.leader_port,
            "follower_port": config.follower_port,
            "sensors": {
                "cameras": len(config.sensors.cameras),
                "tactile": len(config.sensors.tactile),
                "audio": len(config.sensors.audio),
            },
        },
    }

    metadata_file = os.path.join(save_path, "metadata.json")
    with open(metadata_file, "w") as f:
        json.dump(metadata, f, indent=2)
    print(f"✅ Metadata file created: {metadata_file}")


def main() -> bool:
    """Main test function.

    Returns:
        True if test completed successfully, False otherwise.
    """
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    print("🤖 Saver Test")
    print("=" * 50)
    print("📋 Features:")
    print("   - Follower arm joint positions → PNG")
    print("   - 4 sensor data (Camera, Audio, Left Tactile, Right Tactile) → GIF")
    print("   - Complete data saving with metadata")
    print("=" * 50)

    # Create configuration
    config = create_test_config()
    assert config.sensors is not None
    print("✅ Configuration created")
    print(f"   - Leader port: {config.leader_port}")
    print(f"   - Follower port: {config.follower_port}")
    print(
        f"   - Sensors: {len(config.sensors.cameras)} camera(s), "
        f"{len(config.sensors.tactile)} tactile(s), {len(config.sensors.audio)} audio(s)"
    )

    # Create robot and save worker
    robot = RakudaRobot(config)
    save_worker = RakudaSaveWorker(config, worker_num=1, fps=SAVE_WORKER_FPS)

    # Create save path
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    save_path = f"data/data_{timestamp}"
    print(f"\n💾 Save path: {save_path}")

    try:
        # Step 1: Robot connection
        print("\n🚀 Step 1: Robot Connection")
        print("=" * 30)
        robot.connect()
        print("✅ Robot connected successfully")

        # Step 2: Data collection
        print("\n📊 Step 2: Data Collection")
        print("=" * 30)
        print("📊 Recording parameters:")
        print(f"   - Max frames: {RECORDING_MAX_FRAMES}")
        print(f"   - FPS: {RECORDING_FPS}")
        print(f"   - Duration: ~{RECORDING_MAX_FRAMES / RECORDING_FPS:.0f} seconds")

        print("\n🎥 Recording data...")
        obs = robot.record_parallel(max_frame=RECORDING_MAX_FRAMES, fps=RECORDING_FPS)

        # Downsample sensor data
        _downsample_tactile_obs(obs, target_size=TARGET_SIZE)
        _downsample_camera_obs(obs, target_size=TARGET_SIZE)
        _downsample_audio_obs(obs, target_size=TARGET_SIZE)

        print("✅ Data recording completed")
        _print_data_shapes(obs)

        # Step 3: Data saving
        print("\n💾 Step 3: Data Saving")
        print("=" * 30)
        print(f"💾 Saving data to: {save_path}")
        save_worker.save_all_obs(obs, save_path, save_gif=False)

        _create_metadata(obs, config, save_path)

        # Wait for background saving to complete
        print("⏳ Waiting for background saving to complete...")
        time.sleep(3)
        print("✅ Data saving completed")

        # Step 4: Sensor visualization (GIF)
        print("\n🎬 Step 4: Sensor Visualization (GIF)")
        print("=" * 30)

        try:
            success = create_sensor_gif_visualization(obs, save_path, fps=VISUALIZATION_FPS)
            if not success:
                print("❌ Sensor visualization failed")
                return False
            print("✅ Sensor visualization completed")
        except Exception as e:
            print(f"❌ Error during sensor visualization: {e}")
            traceback.print_exc()
            return False

        # Step 5: Verification
        print("\n🔍 Step 5: Data Verification")
        print("=" * 30)

        try:
            if not verify_saved_data(save_path):
                print("❌ FAILED: Data verification failed")
                return False

            print("\n🎉 SUCCESS: Saver test completed successfully!")
            print(f"📁 All data saved to: {os.path.abspath(save_path)}")
            print("📊 Visualization files:")
            print(f"   - Sensor GIF: {os.path.abspath(save_path)}/sensor_images.gif")
            return True
        except Exception as e:
            print(f"❌ Error during verification: {e}")
            traceback.print_exc()
            return False

    except Exception as e:
        print(f"❌ Error during workflow: {e}")
        traceback.print_exc()
        return False

    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        robot.disconnect()
        print("✅ Robot disconnected")


if __name__ == "__main__":
    main()
