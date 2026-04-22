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
from robopy.config.robot_config.xarm_config import XARM_LEADER_MOTOR_NAMES
from robopy.robots.xarm import (
    SimXArmFollower,
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


def test_sim_xarm_follower_instantiation() -> None:
    """SimXArmFollower should construct without connecting."""
    sim = SimXArmFollower(host="127.0.0.1", port=6000)
    assert not sim.is_connected
    assert sim.port == "127.0.0.1:6000"
    assert issubclass(SimXArmFollower, XArmArm)


def test_xarm_config_sim_mode() -> None:
    cfg = XArmConfig(sim_mode=True, sim_host="192.168.1.100", sim_port=7000)
    assert cfg.sim_mode is True
    assert cfg.sim_host == "192.168.1.100"
    assert cfg.sim_port == 7000


class _FakeMotor:
    def __init__(self, resolution: int = 4096) -> None:
        self.resolution = resolution


class _FakeBus:
    """Mimics DynamixelBus.sync_read returning raw INT32 ticks (no calibration)."""

    def __init__(self, tick_values: dict[str, int], resolution: int = 4096) -> None:
        self._ticks = tick_values
        self.motors = {name: _FakeMotor(resolution) for name in tick_values}

    def sync_read(self, _item: object, motor_names: list[str]) -> dict[str, int]:
        return {name: self._ticks[name] for name in motor_names}


def test_xarm_leader_read_raw_radians_tick_to_rad_conversion() -> None:
    """Regression: _read_raw_radians must interpret sync_read output as raw
    ticks (calibration is never set on the bus), matching the legacy
    gello_software DynamixelDriver.get_joints() formula ``ticks / 2048 * pi``.

    Pre-fix, the method called np.deg2rad() on raw ticks and was wrong by
    2048/180 ≈ 11.38×.
    """
    cfg = XArmConfig(leader_port="/dev/null")
    leader = XArmLeader(cfg)
    # Inject fake bus with known tick values: 2048 ticks → pi rad (half turn).
    ticks = {name: 2048 for name in XARM_LEADER_MOTOR_NAMES}
    leader._motors = _FakeBus(ticks)  # type: ignore[assignment]

    rad = leader._read_raw_radians()
    assert rad.shape == (8,)
    expected = np.full(8, np.pi, dtype=np.float32)
    np.testing.assert_allclose(rad, expected, atol=1e-5)


def test_xarm_leader_read_raw_radians_handles_negative_ticks() -> None:
    """Negative ticks (multi-turn) must convert linearly, not wrap through
    deg2rad. At -1024 ticks, legacy formula gives -pi/2 rad."""
    cfg = XArmConfig(leader_port="/dev/null")
    leader = XArmLeader(cfg)
    ticks = {name: -1024 for name in XARM_LEADER_MOTOR_NAMES}
    leader._motors = _FakeBus(ticks)  # type: ignore[assignment]

    rad = leader._read_raw_radians()
    expected = np.full(8, -np.pi / 2.0, dtype=np.float32)
    np.testing.assert_allclose(rad, expected, atol=1e-5)
