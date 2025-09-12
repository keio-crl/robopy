#!/usr/bin/env python3
"""
Simple demo script showing RealSense camera usage with threading.

This demonstrates how the threading implementation prevents blocking
the main thread while capturing frames.
"""

import logging
import time

from robopy.config.sensor_config.visual_config.camera_config import RealsenseCameraConfig
from robopy.sensors.visual.realsense_camera import RealsenseCamera

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def simulate_main_thread_work():
    """Simulate some work being done in the main thread."""
    logger.info("Main thread: Doing important work...")
    time.sleep(2)
    logger.info("Main thread: Work completed!")


def main():
    """Main demo function."""
    logger.info("=== RealSense Camera Threading Demo ===")

    # Create camera with standard settings
    config = RealsenseCameraConfig(fps=30, width=640, height=480, color_mode="rgb")

    camera = RealsenseCamera(config=config)

    try:
        # Connect to camera (starts background thread)
        logger.info("Connecting to camera...")
        camera.connect()

        # Demonstrate that main thread is not blocked
        logger.info("Camera connected. Background thread is now capturing frames.")
        logger.info("Main thread continues without blocking...")

        # Do some work in main thread while camera captures in background
        simulate_main_thread_work()

        # Get a few frames asynchronously (non-blocking)
        logger.info("Reading frames asynchronously...")
        for i in range(5):
            frame = camera.async_read(timeout_ms=1000)
            logger.info(f"Got frame {i + 1}: shape={frame.shape}")

            # Simulate processing time
            time.sleep(0.5)

        # Show that we can still do sync reads if needed
        logger.info("Demonstrating synchronous read...")
        sync_frame = camera.read()
        logger.info(f"Sync frame shape: {sync_frame.shape}")

        # Test depth if available
        if config.is_depth_camera:
            logger.info("Reading depth frame...")
            depth = camera.async_read_depth(timeout_ms=1000)
            logger.info(f"Depth frame shape: {depth.shape}")

        logger.info("Demo completed successfully!")

    except Exception as e:
        logger.error(f"Demo failed: {e}")

    finally:
        # Always disconnect
        logger.info("Disconnecting camera...")
        camera.disconnect()
        logger.info("Demo finished.")


if __name__ == "__main__":
    main()
