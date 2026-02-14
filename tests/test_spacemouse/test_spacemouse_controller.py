"""Tests for So101SpaceMouseController (mocked hardware)."""

from __future__ import annotations

import sys
import time
from collections import namedtuple
from unittest.mock import MagicMock, PropertyMock, patch

import numpy as np
import pytest

from robopy.config.input_config.spacemouse_config import SpaceMouseConfig
from robopy.input.spacemouse import SpaceMouseReader, SpaceMouseState
from robopy.kinematics import EEPose, IKResult


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _mock_reader(
    state: SpaceMouseState | None = None,
) -> MagicMock:
    """Create a mock SpaceMouseReader."""
    reader = MagicMock(spec=SpaceMouseReader)
    reader.is_running = True
    if state is None:
        state = SpaceMouseState()
    reader.get_state.return_value = state
    return reader


def _mock_robot(
    follower_joints: np.ndarray | None = None,
) -> MagicMock:
    """Create a mock So101Robot with FK/IK support."""
    if follower_joints is None:
        follower_joints = np.zeros(6, dtype=np.float32)

    robot = MagicMock()
    robot.is_connected = True

    # get_arm_observation → ArmObs.follower
    arm_obs = MagicMock()
    arm_obs.follower = follower_joints
    robot.get_arm_observation.return_value = arm_obs

    # forward_kinematics (classmethod → just use a normal return)
    robot.forward_kinematics = MagicMock(
        return_value=EEPose(x=0.0, y=0.0, z=0.2, pitch=0.0, roll=0.0)
    )

    # inverse_kinematics → IKResult
    def _ik(target, current, ik_config=None):
        return IKResult(
            joint_angles_rad=np.deg2rad(current[:5].astype(np.float64)),
            success=True,
            iterations=5,
            position_error=1e-5,
            orientation_error=1e-5,
        )

    robot.inverse_kinematics = MagicMock(side_effect=_ik)
    robot.send_frame_action = MagicMock()
    robot.sensors_observation = MagicMock(return_value={})

    return robot


# ---------------------------------------------------------------------------
# SpaceMouseConfig
# ---------------------------------------------------------------------------


class TestSpaceMouseConfig:
    def test_defaults(self) -> None:
        cfg = SpaceMouseConfig()
        assert cfg.linear_speed == 0.10
        assert cfg.angular_speed == 0.5
        assert cfg.gripper_speed == 50.0
        assert cfg.deadzone == 0.05
        assert cfg.control_hz == 50

    def test_custom(self) -> None:
        cfg = SpaceMouseConfig(linear_speed=0.2, deadzone=0.1)
        assert cfg.linear_speed == 0.2
        assert cfg.deadzone == 0.1


# ---------------------------------------------------------------------------
# _apply_deadzone
# ---------------------------------------------------------------------------


class TestApplyDeadzone:
    def test_below_deadzone(self) -> None:
        from robopy.robots.so101.so101_spacemouse import _apply_deadzone

        assert _apply_deadzone(0.03, 0.05) == 0.0
        assert _apply_deadzone(-0.03, 0.05) == 0.0

    def test_above_deadzone(self) -> None:
        from robopy.robots.so101.so101_spacemouse import _apply_deadzone

        assert _apply_deadzone(0.5, 0.05) == 0.5
        assert _apply_deadzone(-0.5, 0.05) == -0.5

    def test_at_deadzone(self) -> None:
        from robopy.robots.so101.so101_spacemouse import _apply_deadzone

        assert _apply_deadzone(0.05, 0.05) == 0.05
        assert _apply_deadzone(0.04999, 0.05) == 0.0


# ---------------------------------------------------------------------------
# So101SpaceMouseController._control_step
# ---------------------------------------------------------------------------


class TestControlStep:
    def test_zero_input_no_movement(self) -> None:
        """When SpaceMouse is centred, EE target should not change."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig()
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState())  # all zeros

        target_ee = np.array([0.0, 0.0, 0.2, 0.0, 0.0], dtype=np.float64)
        gripper = 50.0
        joints = np.zeros(6, dtype=np.float32)

        new_ee, new_grip, new_joints = ctrl._control_step(target_ee, gripper, joints, dt=0.02)

        np.testing.assert_allclose(new_ee, target_ee, atol=1e-10)
        assert new_grip == pytest.approx(50.0)
        robot.send_frame_action.assert_called_once()

    def test_positive_x_input(self) -> None:
        """Full positive x should advance target_ee[0]."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig(linear_speed=0.10, deadzone=0.0)
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState(x=1.0))

        target_ee = np.array([0.0, 0.0, 0.2, 0.0, 0.0], dtype=np.float64)
        dt = 0.02  # 50 Hz

        new_ee, _, _ = ctrl._control_step(target_ee.copy(), 0.0, np.zeros(6, dtype=np.float32), dt)

        expected_dx = 1.0 * 0.10 * 0.02  # 0.002 m
        assert abs(new_ee[0] - expected_dx) < 1e-10
        assert abs(new_ee[1]) < 1e-10  # y unchanged
        assert abs(new_ee[2] - 0.2) < 1e-10  # z unchanged

    def test_button_gripper_close(self) -> None:
        """Left button should decrease gripper angle."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig(gripper_speed=50.0, deadzone=0.0)
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState(buttons=[True, False]))

        target_ee = np.zeros(5, dtype=np.float64)
        dt = 0.02

        _, new_grip, _ = ctrl._control_step(target_ee.copy(), 50.0, np.zeros(6, dtype=np.float32), dt)

        expected = 50.0 - 50.0 * 0.02  # 49.0
        assert new_grip == pytest.approx(expected)

    def test_button_gripper_open(self) -> None:
        """Right button should increase gripper angle."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig(gripper_speed=50.0, deadzone=0.0)
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState(buttons=[False, True]))

        target_ee = np.zeros(5, dtype=np.float64)
        dt = 0.02

        _, new_grip, _ = ctrl._control_step(target_ee.copy(), 50.0, np.zeros(6, dtype=np.float32), dt)

        expected = 50.0 + 50.0 * 0.02  # 51.0
        assert new_grip == pytest.approx(expected)

    def test_gripper_clamp_min(self) -> None:
        """Gripper should not go below 0."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig(gripper_speed=1000.0, deadzone=0.0)
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState(buttons=[True, False]))

        target_ee = np.zeros(5, dtype=np.float64)

        _, new_grip, _ = ctrl._control_step(target_ee.copy(), 1.0, np.zeros(6, dtype=np.float32), 0.02)

        assert new_grip >= 0.0

    def test_gripper_clamp_max(self) -> None:
        """Gripper should not exceed 100."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig(gripper_speed=1000.0, deadzone=0.0)
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState(buttons=[False, True]))

        target_ee = np.zeros(5, dtype=np.float64)

        _, new_grip, _ = ctrl._control_step(target_ee.copy(), 99.0, np.zeros(6, dtype=np.float32), 0.02)

        assert new_grip <= 100.0

    def test_angular_input(self) -> None:
        """Pitch and roll inputs should move target orientation."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig(angular_speed=0.5, deadzone=0.0)
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState(pitch=1.0, roll=-1.0))

        target_ee = np.zeros(5, dtype=np.float64)
        dt = 0.02

        new_ee, _, _ = ctrl._control_step(target_ee.copy(), 0.0, np.zeros(6, dtype=np.float32), dt)

        expected_dpitch = 1.0 * 0.5 * 0.02
        expected_droll = -1.0 * 0.5 * 0.02
        assert abs(new_ee[3] - expected_dpitch) < 1e-10
        assert abs(new_ee[4] - expected_droll) < 1e-10

    def test_deadzone_filters_small_inputs(self) -> None:
        """Inputs below the deadzone should be zeroed."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        cfg = SpaceMouseConfig(deadzone=0.1)
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = cfg
        ctrl._reader = _mock_reader(SpaceMouseState(x=0.05, y=0.09, z=-0.08))

        target_ee = np.zeros(5, dtype=np.float64)
        dt = 0.02

        new_ee, _, _ = ctrl._control_step(target_ee.copy(), 0.0, np.zeros(6, dtype=np.float32), dt)

        np.testing.assert_allclose(new_ee, np.zeros(5), atol=1e-10)


# ---------------------------------------------------------------------------
# Teleoperation integration
# ---------------------------------------------------------------------------


class TestTeleoperation:
    def test_not_connected_raises(self) -> None:
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        robot.is_connected = False
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = SpaceMouseConfig()
        reader = _mock_reader()
        reader.is_running = False
        ctrl._reader = reader

        with pytest.raises(ConnectionError):
            ctrl.teleoperation(max_seconds=1.0)

    def test_timed_teleoperation(self) -> None:
        """Timed teleoperation should exit after max_seconds."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = SpaceMouseConfig(control_hz=100)
        ctrl._reader = _mock_reader()

        start = time.perf_counter()
        ctrl.teleoperation(max_seconds=0.15)
        elapsed = time.perf_counter() - start

        assert elapsed < 1.0  # should not take more than 1s
        assert robot.send_frame_action.call_count > 0


# ---------------------------------------------------------------------------
# record_parallel
# ---------------------------------------------------------------------------


class TestRecordParallel:
    def test_invalid_max_frame(self) -> None:
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = SpaceMouseConfig()
        ctrl._reader = _mock_reader()

        with pytest.raises(ValueError, match="max_frame"):
            ctrl.record_parallel(max_frame=0)

    def test_invalid_fps(self) -> None:
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = SpaceMouseConfig()
        ctrl._reader = _mock_reader()

        with pytest.raises(ValueError, match="fps"):
            ctrl.record_parallel(max_frame=10, fps=0)

    def test_record_returns_so101_obs(self) -> None:
        """A short recording should return valid So101Obs."""
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = SpaceMouseConfig(control_hz=200)
        ctrl._reader = _mock_reader()

        obs = ctrl.record_parallel(max_frame=3, fps=30, control_hz=200)

        assert obs.arms.leader.shape[0] == 3
        assert obs.arms.follower.shape[0] == 3
        assert obs.arms.leader.shape[1] == 6  # 5 joints + gripper
        assert isinstance(obs.cameras, dict)

    def test_record_not_connected(self) -> None:
        from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

        robot = _mock_robot()
        robot.is_connected = False
        ctrl = So101SpaceMouseController.__new__(So101SpaceMouseController)
        ctrl._robot = robot
        ctrl._sm_config = SpaceMouseConfig()
        reader = _mock_reader()
        reader.is_running = False
        ctrl._reader = reader

        with pytest.raises(ConnectionError):
            ctrl.record_parallel(max_frame=5)
