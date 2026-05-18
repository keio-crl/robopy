"""Hardware-free FK / IK round-trip tests for SO-101 and Koch.

These verify that ``So101Robot.forward_kinematics`` / ``inverse_kinematics``
and the Koch equivalents can be called as classmethods, without ever
instantiating the robot or touching hardware.
"""

from __future__ import annotations

import numpy as np

from robopy.kinematics import EEPose
from robopy.robots.koch.koch_robot import KochRobot
from robopy.robots.so101.so101_robot import So101Robot


def test_so101_fk_classmethod_no_hardware() -> None:
    pose = So101Robot.forward_kinematics(np.zeros(5, dtype=np.float32))
    assert isinstance(pose, EEPose)
    arr = pose.to_array()
    assert arr.shape == (5,)


def test_so101_ik_classmethod_no_hardware() -> None:
    """Round-trip: FK(q0) -> target; IK(target, q0) -> q_hat; FK(q_hat) ≈ target."""
    q0_deg = np.array([10.0, 20.0, -15.0, 5.0, 30.0], dtype=np.float32)
    target = So101Robot.forward_kinematics(q0_deg)

    result = So101Robot.inverse_kinematics(target, q0_deg)
    assert result.position_error < 1e-3  # < 1 mm
    assert result.orientation_error < 1e-2

    q_hat_rad = result.joint_angles_rad
    recovered = So101Robot.forward_kinematics(np.rad2deg(q_hat_rad).astype(np.float32))
    np.testing.assert_allclose(recovered.to_array()[:3], target.to_array()[:3], atol=1e-3)


def test_koch_fk_classmethod_no_hardware() -> None:
    pose = KochRobot.forward_kinematics(np.zeros(5, dtype=np.float32))
    assert isinstance(pose, EEPose)


def test_koch_ik_classmethod_no_hardware() -> None:
    q0_deg = np.array([5.0, 10.0, -10.0, 5.0, 15.0], dtype=np.float32)
    target = KochRobot.forward_kinematics(q0_deg)

    result = KochRobot.inverse_kinematics(target, q0_deg)
    assert result.position_error < 1e-3
    assert result.orientation_error < 1e-2


def test_so101_ik_callable_via_instance_too() -> None:
    """Backwards-compat: existing code that calls IK on an instance still works."""
    q0_deg = np.array([0.0, 0.0, 0.0, 0.0, 0.0], dtype=np.float32)
    target = So101Robot.forward_kinematics(q0_deg)

    from robopy.config.robot_config.so101_config import So101Config

    cfg = So101Config(
        leader_port=None, follower_port="/dev/null", calibration_path="/tmp/nonexistent.json"
    )
    robot = So101Robot(cfg)  # Note: no connect() -- no hardware touched.
    result = robot.inverse_kinematics(target, q0_deg)
    assert result.position_error < 1e-3
