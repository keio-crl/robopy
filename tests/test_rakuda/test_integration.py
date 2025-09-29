"""
Simple integration test for RakudaRobot parallel recording functionality.

This test verifies that the new record_parallel method works correctly
and can be integrated with existing systems.
"""

import time
import unittest
from unittest.mock import MagicMock, patch

import numpy as np

from robopy.config import RakudaConfig


class TestRakudaRobotIntegration(unittest.TestCase):
    """Integration tests for RakudaRobot record_parallel method."""

    def test_record_parallel_exists(self):
        """Test that record_parallel method exists and is callable."""
        from robopy.robots.rakuda.rakuda_robot import RakudaRobot

        # Create config with required parameters
        config = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
        robot = RakudaRobot(config)

        # Check method exists
        self.assertTrue(
            hasattr(robot, "record_parallel"), "RakudaRobot should have record_parallel method"
        )
        self.assertTrue(
            callable(getattr(robot, "record_parallel")), "record_parallel should be callable"
        )

    @patch("robopy.robots.rakuda.rakuda_robot.RakudaPairSys")
    @patch("robopy.robots.rakuda.rakuda_robot.RealsenseCamera")
    @patch("robopy.robots.rakuda.rakuda_robot.DigitSensor")
    def test_record_parallel_basic_functionality(self, mock_digit, mock_camera, mock_pair_sys):
        """Test basic functionality of record_parallel method."""
        from robopy.robots.rakuda.rakuda_robot import RakudaRobot

        # Setup mocks
        mock_pair_sys_instance = MagicMock()
        mock_pair_sys_instance.is_connected = True
        mock_pair_sys_instance.teleoperate_step.return_value = {
            "leader": np.random.random(6).astype(np.float32),
            "follower": np.random.random(6).astype(np.float32),
        }
        mock_pair_sys.return_value = mock_pair_sys_instance

        mock_camera_instance = MagicMock()
        mock_camera_instance.is_connected = True
        mock_camera_instance.async_read.return_value = np.random.random((3, 480, 640)).astype(
            np.float32
        )
        mock_camera_instance.name = "test_camera"
        mock_camera.return_value = mock_camera_instance

        mock_digit_instance = MagicMock()
        mock_digit_instance.is_connected = True
        mock_digit_instance.async_read.return_value = np.random.random((240, 320, 3)).astype(
            np.float32
        )
        mock_digit_instance.name = "test_tactile"
        mock_digit.return_value = mock_digit_instance

        # Create robot with config
        config = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
        robot = RakudaRobot(config)

        # Mock the internal state to avoid actual hardware connections
        robot._pair_sys = mock_pair_sys_instance
        robot._sensors = MagicMock()
        robot._sensors.cameras = [mock_camera_instance]
        robot._sensors.tactile = [mock_digit_instance]

        # Test recording
        num_frames = 3
        target_fps = 10  # Use lower FPS for testing

        start_time = time.perf_counter()
        obs = robot.record_parallel(max_frame=num_frames, fps=target_fps)
        end_time = time.perf_counter()

        duration = end_time - start_time

        # Verify basic structure
        self.assertIsNotNone(obs)
        self.assertIn("arms", obs)
        self.assertIn("sensors", obs)
        self.assertIn("leader", obs["arms"])
        self.assertIn("follower", obs["arms"])

        # Verify we got the expected number of frames
        self.assertEqual(len(obs["arms"]["leader"]), num_frames)
        self.assertEqual(len(obs["arms"]["follower"]), num_frames)

        print(f"Integration test: {num_frames} frames in {duration:.3f}s at {target_fps}Hz")

    def test_sensors_observation_optimized(self):
        """Test that optimized sensors_observation method works."""
        from robopy.robots.rakuda.rakuda_robot import RakudaRobot

        config = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
        robot = RakudaRobot(config)

        # Check that the method still exists and handles the async_read check
        self.assertTrue(
            hasattr(robot, "sensors_observation"), "sensors_observation method should exist"
        )

        # Test with mock sensors that have async_read
        mock_camera = MagicMock()
        mock_camera.is_connected = True
        mock_camera.async_read.return_value = np.random.random((3, 480, 640)).astype(np.float32)
        mock_camera.name = "test_camera"

        mock_tactile = MagicMock()
        mock_tactile.is_connected = True
        mock_tactile.async_read.return_value = np.random.random((240, 320, 3)).astype(np.float32)
        mock_tactile.name = "test_tactile"

        mock_sensors = MagicMock()
        mock_sensors.cameras = [mock_camera]
        mock_sensors.tactile = [mock_tactile]

        robot._sensors = mock_sensors
        robot._pair_sys = MagicMock()
        robot._pair_sys.is_connected = True

        # Test the method
        try:
            obs = robot.sensors_observation()
            self.assertIsNotNone(obs)
            # Verify that async_read was called with reduced timeout (5ms instead of 10ms)
            mock_camera.async_read.assert_called_with(timeout_ms=5)
            mock_tactile.async_read.assert_called_with(timeout_ms=5)
        except Exception as e:
            self.fail(f"sensors_observation should work with mocked sensors: {e}")


if __name__ == "__main__":
    unittest.main(verbosity=2)
