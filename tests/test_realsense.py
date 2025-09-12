#!/usr/bin/env python3
"""Test script for RealsenseCamera implementation with non-blocking performance measurement"""

import logging
import statistics
import time
from typing import List

from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig
from robopy.sensors.visual.realsense_camera import RealsenseCamera

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class PerformanceMetrics:
    """Class to track performance metrics for camera operations"""

    def __init__(self):
        self.frame_times: List[float] = []
        self.processing_times: List[float] = []
        self.dropped_frames = 0
        self.total_frames = 0
        self.start_time = 0.0

    def add_frame_time(self, frame_time: float):
        """Add frame capture time"""
        self.frame_times.append(frame_time)

    def add_processing_time(self, processing_time: float):
        """Add processing time"""
        self.processing_times.append(processing_time)

    def increment_dropped_frames(self):
        """Increment dropped frame counter"""
        self.dropped_frames += 1

    def increment_total_frames(self):
        """Increment total frame counter"""
        self.total_frames += 1

    def get_statistics(self) -> dict:
        """Get performance statistics"""
        total_time = time.time() - self.start_time

        stats = {
            "total_runtime": total_time,
            "total_frames": self.total_frames,
            "successful_frames": len(self.frame_times),
            "dropped_frames": self.dropped_frames,
            "actual_fps": len(self.frame_times) / total_time if total_time > 0 else 0,
            "drop_rate": (
                (self.dropped_frames / self.total_frames * 100) if self.total_frames > 0 else 0
            ),
        }

        if self.frame_times:
            stats.update(
                {
                    "avg_frame_time": statistics.mean(self.frame_times),
                    "min_frame_time": min(self.frame_times),
                    "max_frame_time": max(self.frame_times),
                    "frame_time_std": (
                        statistics.stdev(self.frame_times) if len(self.frame_times) > 1 else 0
                    ),
                }
            )

        if self.processing_times:
            stats.update(
                {
                    "avg_processing_time": statistics.mean(self.processing_times),
                    "min_processing_time": min(self.processing_times),
                    "max_processing_time": max(self.processing_times),
                    "processing_time_std": (
                        statistics.stdev(self.processing_times)
                        if len(self.processing_times) > 1
                        else 0
                    ),
                }
            )

        return stats


def simulate_main_thread_work(work_duration: float = 0.01):
    """Simulate some CPU-intensive work on the main thread"""
    # Simulate computational work (matrix operations, calculations, etc.)
    start = time.time()
    result = 0.0
    while time.time() - start < work_duration:
        # Simple computation to consume CPU cycles
        for i in range(1000):
            result += i * 0.001
    return result


def test_non_blocking_performance():
    """Test RealsenseCamera non-blocking performance at 30fps"""

    # Configuration
    TARGET_FPS = 30
    TEST_DURATION = 10.0  # seconds
    FRAME_INTERVAL = 1.0 / TARGET_FPS
    MAIN_THREAD_WORK_DURATION = 0.005  # 5ms of work per iteration

    logger.info("Starting non-blocking performance test")
    logger.info(f"Target FPS: {TARGET_FPS}")
    logger.info(f"Test duration: {TEST_DURATION}s")
    logger.info(f"Main thread work per cycle: {MAIN_THREAD_WORK_DURATION * 1000:.1f}ms")

    # Create camera configuration
    config = RealsenseCameraConfig(fps=TARGET_FPS, width=640, height=480, color_mode="rgb")

    # Create camera instance
    camera = RealsenseCamera(config=config)
    metrics = PerformanceMetrics()

    try:
        # Connect to camera
        logger.info("Connecting to camera...")
        camera.connect()

        # Start metrics timing
        metrics.start_time = time.time()
        test_start_time = time.time()
        last_frame_time = time.time()

        logger.info("Starting performance test...")

        while time.time() - test_start_time < TEST_DURATION:
            current_time = time.time()

            # Check if it's time for next frame
            if current_time - last_frame_time >= FRAME_INTERVAL:
                metrics.increment_total_frames()

                try:
                    # Measure frame capture time
                    frame_start = time.time()
                    frame = camera.async_read(timeout_ms=50)  # Short timeout for non-blocking
                    frame_end = time.time()

                    frame_time = (frame_end - frame_start) * 1000  # Convert to ms
                    metrics.add_frame_time(frame_time)

                    # Simulate frame processing
                    processing_start = time.time()
                    # Do some lightweight processing (e.g., check frame properties)
                    _ = frame.shape  # Check frame shape
                    _ = frame.mean() if hasattr(frame, "mean") else 0  # Check frame mean
                    processing_end = time.time()

                    processing_time = (processing_end - processing_start) * 1000
                    metrics.add_processing_time(processing_time)

                    last_frame_time = current_time

                    # Log progress every 60 frames
                    if metrics.total_frames % 60 == 0:
                        elapsed = current_time - test_start_time
                        actual_fps = len(metrics.frame_times) / elapsed if elapsed > 0 else 0
                        logger.info(
                            f"Progress: {elapsed:.1f}s, "
                            f"Frames: {len(metrics.frame_times)}/{metrics.total_frames}, "
                            f"Actual FPS: {actual_fps:.1f}, "
                            f"Drop rate: {metrics.dropped_frames / metrics.total_frames * 100:.1f}%"
                        )

                except Exception as e:
                    # Frame capture failed (timeout, etc.)
                    metrics.increment_dropped_frames()
                    logger.debug(f"Frame capture failed: {e}")
                    last_frame_time = current_time

            # Simulate main thread work (this should not block frame capture)
            simulate_main_thread_work(MAIN_THREAD_WORK_DURATION)

            # Small sleep to prevent busy waiting
            remaining_time = FRAME_INTERVAL - (time.time() - last_frame_time)
            if remaining_time > 0.001:  # If more than 1ms remaining
                # Sleep for half the remaining time, max 1ms
                time.sleep(min(remaining_time * 0.5, 0.001))

        # Get final statistics
        stats = metrics.get_statistics()

        # Print results
        logger.info("=" * 60)
        logger.info("PERFORMANCE TEST RESULTS")
        logger.info("=" * 60)
        logger.info(f"Test Duration: {stats['total_runtime']:.2f}s")
        logger.info(f"Target FPS: {TARGET_FPS}")
        logger.info(f"Actual FPS: {stats['actual_fps']:.2f}")
        logger.info(f"Total Frames: {stats['total_frames']}")
        logger.info(f"Successful Frames: {stats['successful_frames']}")
        logger.info(f"Dropped Frames: {stats['dropped_frames']}")
        logger.info(f"Drop Rate: {stats['drop_rate']:.2f}%")

        if "avg_frame_time" in stats:
            logger.info(f"Average Frame Time: {stats['avg_frame_time']:.2f}ms")
            logger.info(f"Min Frame Time: {stats['min_frame_time']:.2f}ms")
            logger.info(f"Max Frame Time: {stats['max_frame_time']:.2f}ms")
            logger.info(f"Frame Time Std Dev: {stats['frame_time_std']:.2f}ms")

        if "avg_processing_time" in stats:
            logger.info(f"Average Processing Time: {stats['avg_processing_time']:.2f}ms")
            logger.info(f"Max Processing Time: {stats['max_processing_time']:.2f}ms")

        logger.info("=" * 60)

        # Performance evaluation
        success_rate = (
            (stats["successful_frames"] / stats["total_frames"]) * 100
            if stats["total_frames"] > 0
            else 0
        )
        fps_accuracy = (stats["actual_fps"] / TARGET_FPS) * 100 if TARGET_FPS > 0 else 0

        logger.info("PERFORMANCE EVALUATION")
        logger.info("=" * 60)
        logger.info(f"Success Rate: {success_rate:.1f}%")
        logger.info(f"FPS Accuracy: {fps_accuracy:.1f}%")

        if success_rate >= 95 and fps_accuracy >= 90:
            logger.info("✅ PERFORMANCE TEST PASSED - Excellent non-blocking performance")
        elif success_rate >= 90 and fps_accuracy >= 80:
            logger.info("⚠️  PERFORMANCE TEST MARGINAL - Acceptable but could be improved")
        else:
            logger.info("❌ PERFORMANCE TEST FAILED - Poor non-blocking performance")

        # Check if frame times are consistently low (< 10ms for non-blocking)
        if "avg_frame_time" in stats and stats["avg_frame_time"] < 10:
            logger.info("✅ Frame capture is truly non-blocking (avg < 10ms)")
        else:
            logger.info("⚠️  Frame capture may be blocking main thread")

        return stats

    except Exception as e:
        logger.error(f"Performance test failed: {e}")
        return None

    finally:
        # Cleanup
        logger.info("Disconnecting camera...")
        camera.disconnect()


def test_blocking_vs_nonblocking_comparison():
    """Compare blocking vs non-blocking read performance"""

    logger.info("Starting blocking vs non-blocking comparison test")

    config = RealsenseCameraConfig(fps=30, width=640, height=480, color_mode="rgb")

    camera = RealsenseCamera(config=config)

    try:
        camera.connect()

        # Test blocking reads
        logger.info("Testing blocking reads (get_observation)...")
        blocking_times = []
        for i in range(30):  # 30 frames
            start = time.time()
            _ = camera.read()  # Don't store unused frame
            end = time.time()
            blocking_times.append((end - start) * 1000)
            time.sleep(0.01)  # Small delay between reads

        # Test non-blocking reads
        logger.info("Testing non-blocking reads (async_read)...")
        nonblocking_times = []
        for i in range(30):  # 30 frames
            start = time.time()
            try:
                _ = camera.async_read(timeout_ms=50)  # Don't store unused frame
                end = time.time()
                nonblocking_times.append((end - start) * 1000)
            except Exception as e:
                logger.debug(f"Non-blocking read failed: {e}")
                nonblocking_times.append(50)  # Timeout value
            time.sleep(0.01)

        # Compare results
        avg_blocking = statistics.mean(blocking_times)
        avg_nonblocking = statistics.mean(nonblocking_times)

        logger.info("=" * 60)
        logger.info("BLOCKING VS NON-BLOCKING COMPARISON")
        logger.info("=" * 60)
        logger.info(
            f"Blocking (get_observation) - Average: {avg_blocking:.2f}ms, "
            f"Max: {max(blocking_times):.2f}ms"
        )
        logger.info(
            f"Non-blocking (async_read) - Average: {avg_nonblocking:.2f}ms, "
            f"Max: {max(nonblocking_times):.2f}ms"
        )
        improvement = (avg_blocking - avg_nonblocking) / avg_blocking * 100
        logger.info(f"Performance improvement: {improvement:.1f}%")

        if avg_nonblocking < avg_blocking * 0.5:
            logger.info("✅ Non-blocking reads are significantly faster")
        else:
            logger.info("⚠️  Non-blocking improvement is minimal")

    except Exception as e:
        logger.error(f"Comparison test failed: {e}")

    finally:
        camera.disconnect()


def test_realsense_camera():
    """Test basic RealsenseCamera functionality"""

    # Create camera configuration
    config = RealsenseCameraConfig(fps=30, width=640, height=480, color_mode="rgb")

    # Create camera instance
    camera = RealsenseCamera(config=config)

    try:
        # Test connection
        logger.info("Connecting to camera...")
        camera.connect()

        # Test synchronous reading
        logger.info("Testing synchronous read...")
        frame = camera.read()
        logger.info(f"Captured frame shape: {frame.shape}")

        # Test asynchronous reading
        logger.info("Testing asynchronous read...")
        async_frame = camera.async_read()
        logger.info(f"Async frame shape: {async_frame.shape}")

        # Test multiple async reads
        logger.info("Testing multiple async reads...")
        for i in range(5):
            frame = camera.async_read(timeout_ms=1 / 60 * 1000)
            logger.info(f"Frame {i + 1} shape: {frame.shape}")
            time.sleep(0.1)

        # Test depth reading if available
        if config.is_depth_camera:
            logger.info("Testing depth reading...")
            try:
                depth_frame = camera.read_depth()
                logger.info(f"Depth frame shape: {depth_frame.shape}")

                async_depth = camera.async_read_depth()
                logger.info(f"Async depth frame shape: {async_depth.shape}")
            except Exception as e:
                logger.warning(f"Depth reading failed: {e}")

        logger.info("Camera test completed successfully!")

    except Exception as e:
        logger.error(f"Camera test failed: {e}")

    finally:
        # Cleanup
        logger.info("Disconnecting camera...")
        camera.disconnect()


if __name__ == "__main__":
    # Run the new performance test
    test_non_blocking_performance()

    # Run comparison test
    test_blocking_vs_nonblocking_comparison()

    # Run basic functionality test
    test_realsense_camera()


def test_basic_functionality():
    """Test basic RealsenseCamera functionality for verification"""

    # Create camera configuration
    config = RealsenseCameraConfig(fps=30, width=640, height=480, color_mode="rgb")

    # Create camera instance
    camera = RealsenseCamera(config=config)

    try:
        # Test connection
        logger.info("Connecting to camera...")
        camera.connect()

        # Test synchronous reading
        logger.info("Testing synchronous read...")
        frame = camera.read()
        logger.info(f"Captured frame shape: {frame.shape}")

        # Test asynchronous reading
        logger.info("Testing asynchronous read...")
        async_frame = camera.async_read()
        logger.info(f"Async frame shape: {async_frame.shape}")

        logger.info("Basic functionality test completed successfully!")

    except Exception as e:
        logger.error(f"Basic functionality test failed: {e}")

    finally:
        # Cleanup
        logger.info("Disconnecting camera...")
        camera.disconnect()


if __name__ == "__main__":
    # Run the new performance test (main test)
    test_non_blocking_performance()

    # Run comparison test
    test_blocking_vs_nonblocking_comparison()

    # Run basic functionality test
    test_basic_functionality()
