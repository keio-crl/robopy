"""
Simple performance demonstration for RakudaRobot parallel recording.

This script demonstrates the new parallel recording functionality
without requiring actual hardware connections.
"""

import time
from unittest.mock import MagicMock, patch

import numpy as np


def simulate_processing_times():
    """Simulate realistic processing times for different components."""

    # Typical processing times (in seconds)
    arm_processing_time = 0.002  # 2ms for Dynamixel sync_read
    camera_processing_time = 0.008  # 8ms for RealSense frame capture
    tactile_processing_time = 0.003  # 3ms for Digit sensor

    return {
        "arm": arm_processing_time,
        "camera": camera_processing_time,
        "tactile": tactile_processing_time,
    }


def create_mock_sensor_functions(processing_times):
    """Create mock sensor functions with realistic timing."""

    def mock_arm_step():
        time.sleep(processing_times["arm"])
        return {
            "leader": np.random.random(6).astype(np.float32),
            "follower": np.random.random(6).astype(np.float32),
        }

    def mock_camera_read(timeout_ms=10):
        time.sleep(processing_times["camera"])
        return np.random.random((3, 480, 640)).astype(np.float32)

    def mock_tactile_read():
        time.sleep(processing_times["tactile"])
        return np.random.random((240, 320, 3)).astype(np.float32)

    def mock_tactile_async_read(timeout_ms=10):
        time.sleep(processing_times["tactile"] * 0.8)  # Slightly faster
        return np.random.random((240, 320, 3)).astype(np.float32)

    return mock_arm_step, mock_camera_read, mock_tactile_read, mock_tactile_async_read


def simulate_sequential_processing(num_frames=30, target_fps=30):
    """Simulate current sequential processing approach."""
    processing_times = simulate_processing_times()
    mock_arm, mock_camera, mock_tactile, _ = create_mock_sensor_functions(processing_times)

    target_interval = 1.0 / target_fps
    total_time = 0
    frame_times = []

    print(f"Simulating sequential processing for {num_frames} frames at {target_fps}Hz")
    print(f"Target interval: {target_interval * 1000:.1f}ms")

    for frame in range(num_frames):
        frame_start = time.perf_counter()

        # Sequential processing (current approach)
        arm_data = mock_arm()  # 2ms
        camera_data = mock_camera()  # 8ms
        tactile_data = mock_tactile()  # 3ms

        frame_time = time.perf_counter() - frame_start
        frame_times.append(frame_time)
        total_time += frame_time

        # Wait for target interval
        sleep_time = max(0, target_interval - frame_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    avg_frame_time = np.mean(frame_times) * 1000
    max_frame_time = np.max(frame_times) * 1000

    print("Sequential results:")
    print(f"  Average frame time: {avg_frame_time:.1f}ms")
    print(f"  Maximum frame time: {max_frame_time:.1f}ms")
    print(f"  Processing overhead: {(avg_frame_time - 0):.1f}ms")
    print(f"  Can achieve 30Hz: {'Yes' if avg_frame_time <= 33.3 else 'No'}")

    return frame_times


def simulate_parallel_processing(num_frames=30, target_fps=30):
    """Simulate parallel processing approach."""
    from concurrent.futures import ThreadPoolExecutor

    processing_times = simulate_processing_times()
    mock_arm, mock_camera, _, mock_tactile_async = create_mock_sensor_functions(processing_times)

    target_interval = 1.0 / target_fps
    frame_times = []

    print(f"\nSimulating parallel processing for {num_frames} frames at {target_fps}Hz")

    for frame in range(num_frames):
        frame_start = time.perf_counter()

        # Parallel processing (new approach)
        with ThreadPoolExecutor(max_workers=4) as executor:
            arm_future = executor.submit(mock_arm)
            camera_future = executor.submit(mock_camera)
            tactile_future = executor.submit(mock_tactile_async)

            # Collect results
            arm_data = arm_future.result()
            camera_data = camera_future.result()
            tactile_data = tactile_future.result()

        frame_time = time.perf_counter() - frame_start
        frame_times.append(frame_time)

        # Wait for target interval
        sleep_time = max(0, target_interval - frame_time)
        if sleep_time > 0:
            time.sleep(sleep_time)

    avg_frame_time = np.mean(frame_times) * 1000
    max_frame_time = np.max(frame_times) * 1000

    print("Parallel results:")
    print(f"  Average frame time: {avg_frame_time:.1f}ms")
    print(f"  Maximum frame time: {max_frame_time:.1f}ms")
    print(f"  Processing overhead: {(avg_frame_time - 0):.1f}ms")
    print(f"  Can achieve 30Hz: {'Yes' if avg_frame_time <= 33.3 else 'No'}")

    return frame_times


def demonstrate_30hz_feasibility():
    """Demonstrate 30Hz feasibility with different configurations."""
    print("\n" + "=" * 60)
    print("30Hz FEASIBILITY ANALYSIS")
    print("=" * 60)

    # Test different sensor configurations
    configurations = [
        {"cameras": 1, "tactile": 0},
        {"cameras": 1, "tactile": 1},
        {"cameras": 2, "tactile": 1},
        {"cameras": 2, "tactile": 2},
    ]

    for config in configurations:
        print(
            f"\nConfiguration: {config['cameras']} camera(s), {config['tactile']} tactile sensor(s)"
        )

        # Calculate theoretical processing time
        base_times = simulate_processing_times()
        sequential_time = (
            base_times["arm"]
            + config["cameras"] * base_times["camera"]
            + config["tactile"] * base_times["tactile"]
        ) * 1000

        parallel_time = (
            max(
                base_times["arm"],
                base_times["camera"],  # Cameras run in parallel
                base_times["tactile"],
            )
            * 1000
        )  # Tactile runs in parallel

        print(f"  Theoretical sequential time: {sequential_time:.1f}ms")
        print(f"  Theoretical parallel time: {parallel_time:.1f}ms")
        print(f"  30Hz achievable (sequential): {'Yes' if sequential_time <= 33.3 else 'No'}")
        print(f"  30Hz achievable (parallel): {'Yes' if parallel_time <= 33.3 else 'No'}")


@patch("robopy.robots.rakuda.rakuda_robot.RakudaPairSys")
@patch("robopy.robots.rakuda.rakuda_robot.RealsenseCamera")
@patch("robopy.robots.rakuda.rakuda_robot.DigitSensor")
def test_actual_implementation(mock_digit, mock_camera, mock_pair_sys):
    """Test the actual implementation with mocks."""
    from robopy.config.robot_config.rakuda_config import RakudaConfig
    from robopy.robots.rakuda.rakuda_robot import RakudaRobot

    print("\n" + "=" * 60)
    print("ACTUAL IMPLEMENTATION TEST")
    print("=" * 60)

    # Setup realistic mocks
    processing_times = simulate_processing_times()

    # Mock pair system
    mock_pair_sys_instance = MagicMock()
    mock_pair_sys_instance.is_connected = True

    def mock_teleoperate_step():
        time.sleep(processing_times["arm"])
        return {
            "leader": np.random.random(6).astype(np.float32),
            "follower": np.random.random(6).astype(np.float32),
        }

    mock_pair_sys_instance.teleoperate_step = mock_teleoperate_step
    mock_pair_sys.return_value = mock_pair_sys_instance

    # Mock camera
    mock_camera_instance = MagicMock()
    mock_camera_instance.is_connected = True
    mock_camera_instance.name = "test_camera"

    def mock_camera_async_read(timeout_ms=10):
        time.sleep(processing_times["camera"])
        return np.random.random((3, 480, 640)).astype(np.float32)

    mock_camera_instance.async_read = mock_camera_async_read
    mock_camera.return_value = mock_camera_instance

    # Mock tactile
    mock_digit_instance = MagicMock()
    mock_digit_instance.is_connected = True
    mock_digit_instance.name = "test_tactile"

    def mock_tactile_async_read(timeout_ms=10):
        time.sleep(processing_times["tactile"] * 0.8)
        return np.random.random((240, 320, 3)).astype(np.float32)

    mock_digit_instance.async_read = mock_tactile_async_read
    mock_digit.return_value = mock_digit_instance

    # Test the actual implementation
    config = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
    robot = RakudaRobot(config)

    # Test parameters
    num_frames = 15
    target_fps = 30

    print(f"Testing record_parallel with {num_frames} frames at {target_fps}Hz")

    start_time = time.perf_counter()
    obs = robot.record_parallel(max_frame=num_frames, fps=target_fps)
    duration = time.perf_counter() - start_time

    frames_recorded = len(obs["arms"]["leader"])
    actual_fps = frames_recorded / duration
    avg_frame_time = duration / frames_recorded * 1000

    print("Results:")
    print(f"  Frames recorded: {frames_recorded}/{num_frames}")
    print(f"  Duration: {duration:.3f}s")
    print(f"  Actual FPS: {actual_fps:.1f}")
    print(f"  Average frame time: {avg_frame_time:.1f}ms")
    print(f"  30Hz achievable: {'Yes' if actual_fps >= 30 else 'No'}")


if __name__ == "__main__":
    print("RakudaRobot 30Hz Performance Analysis")
    print("=" * 60)

    # Simulate different approaches
    sequential_times = simulate_sequential_processing(num_frames=15, target_fps=30)
    parallel_times = simulate_parallel_processing(num_frames=15, target_fps=30)

    # Performance comparison
    seq_avg = np.mean(sequential_times) * 1000
    par_avg = np.mean(parallel_times) * 1000
    improvement = (seq_avg - par_avg) / seq_avg * 100

    print(f"\nPerformance improvement: {improvement:.1f}%")

    # Feasibility analysis
    demonstrate_30hz_feasibility()

    # Test actual implementation
    test_actual_implementation()
