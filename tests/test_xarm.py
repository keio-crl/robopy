"""Hardware-free smoke tests for the xArm integration.

These tests validate that the package can be imported, dataclasses construct
correctly, and basic type contracts hold even when the xArm SDK and hardware
are absent.
"""

from __future__ import annotations

import numpy as np

from robopy.config.robot_config import (
    GELLO_XARM7_DEFAULT,
    GelloArmConfig,
    XArmArmObs,
    XArmConfig,
    XArmObs,
    XArmSensorObs,
    XArmSensorParams,
    XArmWorkspaceBounds,
)
from robopy.robots.xarm import (
    XArmArm,
    XArmFollower,
    XArmLeader,
    XArmPairSys,
    XArmRobot,
)


def test_workspace_bounds_defaults() -> None:
    bounds = XArmWorkspaceBounds()
    assert bounds.min_x == 390.4
    assert bounds.max_x == 100000.0
    assert bounds.min_y == -257.5
    assert bounds.max_y == 314.0
    assert bounds.min_z == 25.0
    assert bounds.max_z == 10000.0


def test_gello_config_defaults() -> None:
    cfg = GELLO_XARM7_DEFAULT
    assert cfg.joint_ids == (1, 2, 3, 4, 5, 6, 7)
    assert cfg.gripper_id == 8
    assert cfg.motor_model == "xl330-m288"
    assert len(cfg.joint_offsets) == 7
    assert len(cfg.joint_signs) == 7


def test_gello_config_validation() -> None:
    try:
        GelloArmConfig(
            joint_ids=(1, 2, 3),
            joint_offsets=(0.0,),
            joint_signs=(1, 1, 1),
        )
    except ValueError:
        return
    raise AssertionError("GelloArmConfig should reject mismatched lengths")


def test_xarm_config_defaults() -> None:
    cfg = XArmConfig()
    assert cfg.follower_ip == "192.168.1.240"
    assert cfg.leader_port is None
    assert cfg.workspace_bounds is None
    assert cfg.control_frequency == 50.0
    assert cfg.max_delta == 0.05
    assert cfg.gripper_open == 800
    assert cfg.gripper_close == 0
    assert isinstance(cfg.gello, GelloArmConfig)


def test_xarm_obs_dataclass() -> None:
    arms = XArmArmObs(
        leader=np.zeros(8, dtype=np.float32),
        follower=np.zeros(8, dtype=np.float32),
        ee_pos_quat=np.zeros(7, dtype=np.float32),
    )
    sensors = XArmSensorObs(cameras={}, tactile={}, audio={})
    obs = XArmObs(arms=arms, sensors=sensors)
    assert obs.arms.leader.shape == (8,)
    assert obs.arms.ee_pos_quat.shape == (7,)
    assert obs.sensors is not None


def test_xarm_sensor_params_defaults() -> None:
    params = XArmSensorParams()
    assert params.cameras == []
    assert params.tactile == []
    assert params.audio == []


def test_xarm_classes_are_exported() -> None:
    # Just check the classes exist and the relationships are correct without
    # touching hardware.
    assert issubclass(XArmLeader, XArmArm)
    assert issubclass(XArmFollower, XArmArm)
    assert XArmPairSys is not None
    assert XArmRobot is not None


def test_xarm_follower_instantiation_without_hardware() -> None:
    """XArmFollower should construct without connecting to the robot."""
    cfg = XArmConfig(follower_ip="127.0.0.1")
    follower = XArmFollower(cfg)
    assert not follower.is_connected
    assert follower.port == "127.0.0.1"
    # get_joint_state returns the zero initial state (no hardware)
    state = follower.get_joint_state()
    assert state.shape == (8,)
