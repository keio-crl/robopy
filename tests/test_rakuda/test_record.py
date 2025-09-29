import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import ArtistAnimation

from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot
from robopy.utils.blosc_handler import BLOSCHandler

config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(
        cameras=None,
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ],
    ),
    slow_mode=False,
)


def test_rakuda_record():
    rakuda = RakudaRobot(config)
    rakuda.connect()
    start_time = time.time()
    obs = rakuda.record_parallel(max_frame=150, fps=30)
    print(f"Recording took {time.time() - start_time:.2f} seconds")
    arm_obs = obs["arms"]
    sensor_obs = obs["sensors"]

    assert arm_obs["leader"].shape[-1] == 17
    assert arm_obs["follower"].shape[-1] == 17

    assert sensor_obs is not None

    # Create animation with tactile data
    create_tactile_animation(sensor_obs)

    # Disconnect
    rakuda.disconnect()


def create_tactile_animation(sensor_obs):
    """Create animation from sensor observations and save as GIF."""

    # Get tactile data
    left_tactile_data = sensor_obs["tactile"]["left"]
    right_tactile_data = sensor_obs["tactile"]["right"]

    # Check if camera data exists
    camera_data = None
    if sensor_obs["cameras"] and "main" in sensor_obs["cameras"]:
        camera_data = sensor_obs["cameras"]["main"]

    print(f"Camera data shape: {camera_data.shape if camera_data is not None else 'None'}")

    BLOSCHandler.save(left_tactile_data, "left_tactile.blosc2")
    BLOSCHandler.save(right_tactile_data, "right_tactile.blosc2")
    if camera_data is not None:
        BLOSCHandler.save(camera_data, "camera_data.blosc2")

    # Determine number of subplots
    num_plots = 0
    plot_titles = []
    data_sources = []

    if camera_data is not None:
        num_plots += 1
        plot_titles.append("Camera")
        data_sources.append(("camera", camera_data))

    if left_tactile_data is not None:
        num_plots += 1
        plot_titles.append("Left Tactile")
        data_sources.append(("tactile", left_tactile_data))

    if right_tactile_data is not None:
        num_plots += 1
        plot_titles.append("Right Tactile")
        data_sources.append(("tactile", right_tactile_data))

    if num_plots == 0:
        print("No valid sensor data found for animation.")
        return

    # Create figure with subplots in a row
    fig, axes = plt.subplots(1, num_plots, figsize=(6 * num_plots, 5))

    # Ensure axes is always a list
    if num_plots == 1:
        axes = [axes]

    # Set titles
    for i, title in enumerate(plot_titles):
        axes[i].set_title(title)
        axes[i].axis("off")

    # Get the minimum frame count
    frame_counts = []
    for data_type, data in data_sources:
        if data is not None:
            frame_counts.append(len(data))

    if not frame_counts:
        print("No frames found in sensor data.")
        return

    num_frames = min(frame_counts)
    print(f"Creating animation with {num_frames} frames")

    # Create animation frames
    artists = []

    for frame_idx in range(num_frames):
        frame_artists = []

        for plot_idx, (data_type, data) in enumerate(data_sources):
            if data is None:
                continue

            if data_type == "camera":
                if len(data.shape) == 4:
                    if data.shape[1] == 3:  # CHW format
                        frame = data[frame_idx].transpose(1, 2, 0)  # Convert to HWC
                    else:  # HWC format
                        frame = data[frame_idx]
                else:
                    print(f"Unexpected camera data shape: {data.shape}")
                    continue

                # Normalize to 0-1 if needed
                if frame.max() > 1.0:
                    frame = frame / 255.0

                im = axes[plot_idx].imshow(frame, animated=True)
                frame_artists.append(im)

            elif data_type == "tactile":
                # Tactile data: assume shape is (frames, height, width, channels)
                if len(data.shape) == 4:
                    frame = data[frame_idx]
                elif len(data.shape) == 3:
                    frame = data[frame_idx]
                else:
                    print(f"Unexpected tactile data shape: {data.shape}")
                    continue

                # Normalize to 0-1 if needed
                if frame.max() > 1.0:
                    frame = frame / 255.0

                # Convert to RGB if it's grayscale
                if len(frame.shape) == 2:
                    frame = np.stack([frame] * 3, axis=-1)
                elif frame.shape[-1] == 1:
                    frame = np.repeat(frame, 3, axis=-1)

                im = axes[plot_idx].imshow(frame, animated=True)
                frame_artists.append(im)

        if frame_artists:
            artists.append(frame_artists)

    if not artists:
        print("No animation frames created.")
        return

    # Create animation
    print("Creating animation...")
    ani = ArtistAnimation(fig, artists, interval=50, blit=True, repeat=True)

    # Save as GIF
    output_path = "/home/tetsugo/Documents/robopy/tests/test_rakuda/sensor_animation.gif"
    print(f"Saving animation to {output_path}...")

    try:
        ani.save(output_path, writer="pillow", fps=20, dpi=100)
        print(f"Animation saved successfully to {output_path}")
    except Exception as e:
        print(f"Error saving animation: {e}")
        print("Trying alternative save method...")
        try:
            ani.save(output_path.replace(".gif", ".mp4"), writer="ffmpeg", fps=20)
            print(f"Video saved to {output_path.replace('.gif', '.mp4')}")
        except Exception as e2:
            print(f"Failed to save video as well: {e2}")

    # Show the plot
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    test_rakuda_record()
