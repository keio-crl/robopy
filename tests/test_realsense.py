#!/usr/bin/env python3
"""Test script for RealsenseCamera implementation"""

import logging
import time

from robopy.config.visual_config.camera_config import RealsenseCameraConfig
from robopy.sensors.visual.realsense_camera import RealsenseCamera

# Set up logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def test_realsense_camera():
    """Test basic RealsenseCamera functionality"""

    # Create camera configuration
    config = RealsenseCameraConfig(fps=30, width=640, height=480, color_mode="rgb")

    # Create camera instance
    camera = RealsenseCamera(index=0, name="test_camera", config=config)

    try:
        # Test connection
        logger.info("Connecting to camera...")
        camera.connect()

        # Test synchronous reading
        logger.info("Testing synchronous read...")
        frame = camera.get_observation()
        logger.info(f"Captured frame shape: {frame.shape}")

        # Test asynchronous reading
        logger.info("Testing asynchronous read...")
        async_frame = camera.async_read()
        logger.info(f"Async frame shape: {async_frame.shape}")

        # Test multiple async reads
        logger.info("Testing multiple async reads...")
        for i in range(5):
            frame = camera.async_read(timeout_ms=500)
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
    test_realsense_camera()
