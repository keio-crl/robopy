import json
import os
import time
from datetime import datetime

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import AudioParams, CameraParams, TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot
from robopy.utils.worker.rakuda_save_worker import RakudaSaveWorker


def create_test_config():
    """Create configuration for real robot test."""

    # Camera configuration (RealSense) - use available 30fps instead of 20fps
    camera_params = CameraParams(name="main", width=640, height=480, fps=30, index=0)

    # Audio sensor configuration
    audio_params = AudioParams(name="main", fps=30)

    # Tactile sensor configuration (Digit) - try D20537 first
    tactile_params = TactileParams(serial_num="D20537", name="main", fps=30)

    # Sensor parameters (camera, tactile, and audio)
    sensor_params = RakudaSensorParams(
        cameras=[camera_params], tactile=[tactile_params], audio=[audio_params]
    )

    # Robot configuration with real USB ports
    config = RakudaConfig(
        leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1", sensors=sensor_params
    )
    return config


def create_sensor_gif_visualization(obs, save_path: str, fps: int = 20):
    """Create GIF visualization with 4 sensor animations: Camera, Audio, Left Tactile, Right Tactile."""
    print(f"\n🎬 Creating Sensor GIF visualization...")

    # Create visualization directory
    viz_dir = os.path.join(save_path, "visualization")
    os.makedirs(viz_dir, exist_ok=True)

    # Extract sensor data
    camera_data = obs.sensors.cameras.get("main") if obs.sensors.cameras else None
    audio_data = obs.sensors.audio.get("main") if obs.sensors.audio else None
    tactile_data = obs.sensors.tactile.get("main") if obs.sensors.tactile else None

    # Get number of frames from any available data
    num_frames = 0
    if camera_data is not None:
        num_frames = len(camera_data)
    elif audio_data is not None:
        num_frames = len(audio_data)
    elif tactile_data is not None:
        num_frames = len(tactile_data)
    else:
        print("❌ No sensor data available for visualization")
        return False

    print(f"📊 Data for visualization:")
    print(f"   - Frames: {num_frames}")
    if camera_data is not None:
        print(f"   - Camera: {camera_data.shape}")
    if audio_data is not None:
        print(f"   - Audio: {audio_data.shape}")
    if tactile_data is not None:
        print(f"   - Tactile: {tactile_data.shape}")

    # Create figure with 2x2 subplots for 4 sensors
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle("Real Robot Sensor Data Visualization", fontsize=16)

    # Initialize image displays
    camera_im = None
    audio_im = None
    tactile_im = None
    tactile_im2 = None

    # Set up camera plot (top-left)
    if camera_data is not None:
        # Convert from (frames, channels, height, width) to (height, width, channels) for display
        camera_frame = np.transpose(camera_data[0], (1, 2, 0))
        # Normalize to 0-1 range if the data is in 0-255 range
        if camera_frame.max() > 1.0:
            camera_frame = camera_frame / 255.0
        camera_im = axes[0, 0].imshow(camera_frame, aspect="auto", vmin=0, vmax=1)
        axes[0, 0].set_title("Camera Image")
        axes[0, 0].axis("off")
    else:
        axes[0, 0].text(
            0.5, 0.5, "No Camera Data", ha="center", va="center", transform=axes[0, 0].transAxes
        )
        axes[0, 0].set_title("Camera Image")

    # Set up audio plot (top-right)
    if audio_data is not None:
        audio_im = axes[0, 1].imshow(audio_data[0], cmap="magma", aspect="auto")
        axes[0, 1].set_title("Audio Spectrogram")
        axes[0, 1].set_xlabel("Time")
        axes[0, 1].set_ylabel("Frequency")
        plt.colorbar(audio_im, ax=axes[0, 1])
    else:
        axes[0, 1].text(
            0.5, 0.5, "No Audio Data", ha="center", va="center", transform=axes[0, 1].transAxes
        )
        axes[0, 1].set_title("Audio Spectrogram")

    # Set up tactile plots (bottom row)
    if tactile_data is not None:
        # Convert from (frames, channels, height, width) to (height, width, channels) for display
        tactile_frame = np.transpose(tactile_data[0], (1, 2, 0))
        # Normalize to 0-1 range if the data is in 0-255 range
        if tactile_frame.max() > 1.0:
            tactile_frame = tactile_frame / 255.0
        tactile_im = axes[1, 0].imshow(tactile_frame, aspect="auto", vmin=0, vmax=1)
        axes[1, 0].set_title("Tactile Sensor (Left)")
        axes[1, 0].axis("off")

        # For now, show the same tactile data in both bottom plots
        tactile_im2 = axes[1, 1].imshow(tactile_frame, aspect="auto", vmin=0, vmax=1)
        axes[1, 1].set_title("Tactile Sensor (Right)")
        axes[1, 1].axis("off")
    else:
        axes[1, 0].text(
            0.5, 0.5, "No Tactile Data", ha="center", va="center", transform=axes[1, 0].transAxes
        )
        axes[1, 0].set_title("Tactile Sensor (Left)")
        axes[1, 1].text(
            0.5, 0.5, "No Tactile Data", ha="center", va="center", transform=axes[1, 1].transAxes
        )
        axes[1, 1].set_title("Tactile Sensor (Right)")

    # Animation function
    def animate(frame):
        # Update camera image
        if camera_data is not None and camera_im is not None and frame < len(camera_data):
            camera_frame = np.transpose(camera_data[frame], (1, 2, 0))
            # Normalize to 0-1 range if the data is in 0-255 range
            if camera_frame.max() > 1.0:
                camera_frame = camera_frame / 255.0
            camera_im.set_array(camera_frame)

        # Update audio spectrogram
        if audio_data is not None and audio_im is not None and frame < len(audio_data):
            audio_im.set_array(audio_data[frame])

        # Update tactile images
        if tactile_data is not None and tactile_im is not None and frame < len(tactile_data):
            tactile_frame = np.transpose(tactile_data[frame], (1, 2, 0))
            # Normalize to 0-1 range if the data is in 0-255 range
            if tactile_frame.max() > 1.0:
                tactile_frame = tactile_frame / 255.0
            tactile_im.set_array(tactile_frame)
        if tactile_data is not None and tactile_im2 is not None and frame < len(tactile_data):
            tactile_frame = np.transpose(tactile_data[frame], (1, 2, 0))
            # Normalize to 0-1 range if the data is in 0-255 range
            if tactile_frame.max() > 1.0:
                tactile_frame = tactile_frame / 255.0
            tactile_im2.set_array(tactile_frame)

        return [camera_im, audio_im, tactile_im, tactile_im2]

    # Create animation
    anim = FuncAnimation(fig, animate, frames=num_frames, interval=1000 // fps, blit=False)

    # Save animation
    try:
        # Save as GIF (more compatible than MP4)
        gif_path = os.path.join(viz_dir, "sensor_data_visualization.gif")
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
    """Verify that the saved data is complete and valid."""
    print(f"\n🔍 Verifying saved data in: {save_path}")

    # Check if directory exists
    if not os.path.exists(save_path):
        print(f"❌ Save directory does not exist: {save_path}")
        return False

    # Check for HDF5 file
    h5_file = os.path.join(save_path, "rakuda_observations.h5")
    if not os.path.exists(h5_file):
        print(f"❌ HDF5 file not found: {h5_file}")
        return False

    # Check file size (should be > 0)
    file_size = os.path.getsize(h5_file)
    if file_size == 0:
        print(f"❌ HDF5 file is empty: {h5_file}")
        return False

    print(f"✅ HDF5 file found: {h5_file} ({file_size:,} bytes)")

    # Check for metadata file
    metadata_file = os.path.join(save_path, "metadata.json")
    if not os.path.exists(metadata_file):
        print(f"❌ Metadata file not found: {metadata_file}")
        return False

    metadata_size = os.path.getsize(metadata_file)
    if metadata_size == 0:
        print(f"❌ Metadata file is empty: {metadata_file}")
        return False

    print(f"✅ Metadata file found: {metadata_file} ({metadata_size:,} bytes)")

    # Check for visualization files
    viz_dir = os.path.join(save_path, "visualization")
    if os.path.exists(viz_dir):
        # Check for sensor GIF
        sensor_gif = os.path.join(viz_dir, "sensor_data_visualization.gif")
        if os.path.exists(sensor_gif):
            gif_size = os.path.getsize(sensor_gif)
            print(f"✅ Sensor GIF found: {sensor_gif} ({gif_size:,} bytes)")
        else:
            print(f"❌ Sensor GIF not found: {sensor_gif}")

    return True


def main():
    """Main test function."""
    print("🤖 Saver Test")
    print("=" * 50)
    print("📋 Features:")
    print("   - Follower arm joint positions → PNG")
    print("   - 4 sensor data (Camera, Audio, Left Tactile, Right Tactile) → GIF")
    print("   - Complete data saving with metadata")
    print("=" * 50)

    # Create configuration
    config = create_test_config()
    print("✅ Configuration created")
    print(f"   - Leader port: {config.leader_port}")
    print(f"   - Follower port: {config.follower_port}")
    print(
        f"   - Sensors: {len(config.sensors.cameras)} camera(s), "
        f"{len(config.sensors.tactile)} tactile(s), {len(config.sensors.audio)} audio(s)"
    )

    # Create robot
    robot = RakudaRobot(config)

    # Create save worker
    save_worker = RakudaSaveWorker(config, worker_num=1, fps=20)

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
        print(f"📊 Recording parameters:")
        print(f"   - Max frames: 60")
        print(f"   - FPS: 20")
        print(f"   - Duration: ~3 seconds")

        print("\n🎥 Recording data...")
        obs = robot.record_parallel(max_frame=60, fps=20)

        print("✅ Data recording completed")
        print(f"📈 Recorded data shapes:")
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

        # Step 3: Data saving
        print("\n💾 Step 3: Data Saving")
        print("=" * 30)

        print(f"💾 Saving data to: {save_path}")
        save_worker.save_all_obs(obs, save_path, save_gif=False)

        # Create metadata file manually
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
                "cameras": {name: data.shape for name, data in obs.sensors.cameras.items()}
                if obs.sensors.cameras
                else {},
                "tactile": {name: data.shape for name, data in obs.sensors.tactile.items()}
                if obs.sensors.tactile
                else {},
                "audio": {
                    name: data.shape for name, data in obs.sensors.audio.items() if data is not None
                }
                if obs.sensors.audio
                else {},
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

        # Wait for background saving to complete
        print("⏳ Waiting for background saving to complete...")
        time.sleep(3)  # Give time for background threads to finish

        print("✅ Data saving completed")

        # Step 4: Sensor visualization (GIF)
        print("\n🎬 Step 4: Sensor Visualization (GIF)")
        print("=" * 30)

        try:
            success = create_sensor_gif_visualization(obs, save_path, fps=20)
            if success:
                print("✅ Sensor visualization completed")
            else:
                print("❌ Sensor visualization failed")
                return False

        except Exception as e:
            print(f"❌ Error during sensor visualization: {e}")
            import traceback

            traceback.print_exc()
            return False

        # Step 5: Verification
        print("\n🔍 Step 5: Data Verification")
        print("=" * 30)

        try:
            if verify_saved_data(save_path):
                print("\n🎉 SUCCESS: Saver test completed successfully!")
                print(f"📁 All data saved to: {os.path.abspath(save_path)}")
                print(f"📊 Visualization files:")
                print(
                    f"   - Sensor GIF: {os.path.abspath(save_path)}/visualization/sensor_data_visualization.gif"
                )
                return True
            else:
                print("❌ FAILED: Data verification failed")
                return False

        except Exception as e:
            print(f"❌ Error during verification: {e}")
            import traceback

            traceback.print_exc()
            return False

    except Exception as e:
        print(f"❌ Error during workflow: {e}")
        import traceback

        traceback.print_exc()
        return False

    finally:
        # Cleanup
        print("\n🧹 Cleaning up...")
        robot.disconnect()
        print("✅ Robot disconnected")


if __name__ == "__main__":
    main()
