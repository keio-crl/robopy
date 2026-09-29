"""Hardware-free smoke tests for the xArm integration.

These tests validate that the package can be imported, dataclasses construct
correctly, and basic type contracts hold even when the xArm SDK and hardware
are absent.
"""

from __future__ import annotations

from unittest.mock import Mock, call

import numpy as np
import pytest

from robopy.config.robot_config import (
    GELLO_XARM7_DEFAULT,
    XARM_WORKSPACE_PRESETS,
    GelloArmConfig,
    XArmAdmittanceConfig,
    XArmArmObs,
    XArmConfig,
    XArmObs,
    XArmSensorObs,
    XArmSensorParams,
    XArmWorkspaceBounds,
    XArmZFloorZone,
    resolve_workspace_bounds,
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
    assert bounds.min_y == -300.0
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
    assert cfg.gripper_open == 84.0
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


def test_restriction_default_preset_resolves_to_default_bounds() -> None:
    bounds = resolve_workspace_bounds(None, ["default"])
    assert bounds is not None
    default = XArmWorkspaceBounds()
    assert bounds.min_x == default.min_x
    assert bounds.max_x == default.max_x
    assert bounds.min_y == default.min_y
    assert bounds.max_y == default.max_y
    assert bounds.min_z == default.min_z
    assert bounds.max_z == default.max_z


def test_restriction_intersection_takes_tighter() -> None:
    """Combining 'default' (very wide) with 'drawer' (narrow) yields the drawer box."""
    bounds = resolve_workspace_bounds(None, ["default", "drawer"])
    assert bounds is not None
    drawer = XARM_WORKSPACE_PRESETS["drawer"]
    # Drawer is fully inside default on every axis, so intersection equals drawer.
    assert bounds.min_x == drawer.min_x
    assert bounds.max_x == drawer.max_x
    assert bounds.min_y == drawer.min_y
    assert bounds.max_y == drawer.max_y
    assert bounds.min_z == drawer.min_z
    assert bounds.max_z == drawer.max_z


def test_restriction_unknown_name_raises() -> None:
    try:
        resolve_workspace_bounds(None, ["nonexistent"])
    except ValueError:
        return
    raise AssertionError("Unknown restriction name must raise ValueError")


def test_restriction_combined_with_workspace_bounds() -> None:
    """An explicit ``workspace_bounds`` should also participate in the intersection."""
    explicit = XArmWorkspaceBounds(
        min_x=500.0,
        max_x=600.0,
        min_y=-100.0,
        max_y=100.0,
        min_z=100.0,
        max_z=300.0,
    )
    # 'drawer' is wider than ``explicit`` on every axis, so explicit dominates.
    bounds = resolve_workspace_bounds(explicit, ["drawer"])
    assert bounds is not None
    assert bounds.min_x == explicit.min_x
    assert bounds.max_x == explicit.max_x
    assert bounds.min_y == explicit.min_y
    assert bounds.max_y == explicit.max_y
    assert bounds.min_z == explicit.min_z
    assert bounds.max_z == explicit.max_z


def test_restriction_empty_intersection_raises() -> None:
    """If the explicit box does not overlap a preset on any axis, raise."""
    disjoint = XArmWorkspaceBounds(
        min_x=-1000.0,
        max_x=-500.0,  # entirely behind base, drawer requires x >= 400
        min_y=-100.0,
        max_y=100.0,
        min_z=100.0,
        max_z=300.0,
    )
    try:
        resolve_workspace_bounds(disjoint, ["drawer"])
    except ValueError:
        return
    raise AssertionError("Empty intersection must raise ValueError")


def test_restriction_none_returns_none() -> None:
    assert resolve_workspace_bounds(None, None) is None
    assert resolve_workspace_bounds(None, []) is None


def test_xarm_config_restriction_field() -> None:
    cfg = XArmConfig(restriction=["default"])
    assert cfg.restriction == ["default"]
    cfg_default = XArmConfig()
    assert cfg_default.restriction is None


def test_z_floor_zone_raises_min_z_inside_footprint() -> None:
    zone = XArmZFloorZone(min_x=400.0, max_x=600.0, min_y=-100.0, max_y=100.0, min_z=200.0)
    bounds = XArmWorkspaceBounds(min_z=25.0, z_floor_zones=(zone,))

    assert bounds.effective_min_z(500.0, 0.0) == 200.0
    assert bounds.effective_min_z(400.0, -100.0) == 200.0
    assert bounds.effective_min_z(700.0, 0.0) == 25.0
    assert bounds.effective_min_z(500.0, 200.0) == 25.0


def test_z_floor_zone_takes_max_when_overlapping() -> None:
    z1 = XArmZFloorZone(min_x=400.0, max_x=600.0, min_y=-100.0, max_y=100.0, min_z=150.0)
    z2 = XArmZFloorZone(min_x=500.0, max_x=700.0, min_y=-100.0, max_y=100.0, min_z=250.0)
    bounds = XArmWorkspaceBounds(min_z=25.0, z_floor_zones=(z1, z2))

    assert bounds.effective_min_z(450.0, 0.0) == 150.0
    assert bounds.effective_min_z(550.0, 0.0) == 250.0
    assert bounds.effective_min_z(650.0, 0.0) == 250.0


def test_resolve_workspace_bounds_unions_z_floor_zones() -> None:
    z1 = XArmZFloorZone(min_x=400.0, max_x=500.0, min_y=-50.0, max_y=50.0, min_z=200.0)
    z2 = XArmZFloorZone(min_x=550.0, max_x=650.0, min_y=-50.0, max_y=50.0, min_z=300.0)
    explicit = XArmWorkspaceBounds(z_floor_zones=(z1,))
    XARM_WORKSPACE_PRESETS["__test_zone"] = XArmWorkspaceBounds(z_floor_zones=(z2,))
    try:
        bounds = resolve_workspace_bounds(explicit, ["__test_zone"])
    finally:
        del XARM_WORKSPACE_PRESETS["__test_zone"]

    assert bounds is not None
    assert len(bounds.z_floor_zones) == 2
    assert bounds.effective_min_z(450.0, 0.0) == 200.0
    assert bounds.effective_min_z(600.0, 0.0) == 300.0


def test_z_floor_zone_invalid_footprint_raises() -> None:
    try:
        XArmZFloorZone(min_x=600.0, max_x=400.0, min_y=0.0, max_y=10.0, min_z=200.0)
    except ValueError:
        return
    raise AssertionError("Inverted footprint must raise ValueError")


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


def test_admittance_defaults() -> None:
    cfg = XArmAdmittanceConfig()
    assert cfg.translational_mass == 0.06
    assert cfg.rotational_inertia_mass_ratio == 0.01
    assert cfg.position_stiffness == 1200.0
    assert cfg.orientation_stiffness == 4.0
    assert cfg.damping == (0.0,) * 6
    assert cfg.reference_frame == 0
    assert cfg.compliant_axis == (0, 0, 1, 0, 0, 0)
    first, second = XArmConfig(), XArmConfig()
    assert isinstance(first.admittance, XArmAdmittanceConfig)
    assert first.admittance == cfg
    assert first.admittance is not second.admittance


@pytest.mark.parametrize("field, value", [
    ("translational_mass", value) for value in (-1, 0, 0.019, 1.001, float("nan"))
] + [
    ("rotational_inertia_mass_ratio", value)
    for value in (-1, 0, 0.001, 0.2, float("nan"))
] + [
    ("position_stiffness", -1), ("position_stiffness", 2001),
    ("orientation_stiffness", -1), ("orientation_stiffness", 21),
    ("damping", (0,) * 5), ("damping", (0,) * 7),
    ("damping", (0, 0, 0, 0, 0, -1)),
    ("reference_frame", -1), ("reference_frame", 2),
    ("compliant_axis", (0,) * 5), ("compliant_axis", (0,) * 7),
    ("compliant_axis", (0, 0, 0, 0, 0, 2)),
    ("compliant_axis", (-1, 0, 0, 0, 0, 0)),
])
def test_admittance_invalid_values(field, value) -> None:
    with pytest.raises(ValueError, match=field):
        XArmAdmittanceConfig(**{field: value})


@pytest.mark.parametrize("mass, ratio, position, orientation", [
    (0.02, 0.005, 0, 0),
    (1.0, 0.01, 2000, 20),
])
def test_admittance_sdk_boundaries(mass, ratio, position, orientation) -> None:
    XArmAdmittanceConfig(
        translational_mass=mass,
        rotational_inertia_mass_ratio=ratio,
        position_stiffness=position,
        orientation_stiffness=orientation,
    )


@pytest.mark.parametrize("custom", [False, True])
def test_enable_admittance_uses_config(monkeypatch, custom) -> None:
    values = dict(
        translational_mass=0.1,
        rotational_inertia_mass_ratio=0.02,
        position_stiffness=500.0,
        orientation_stiffness=8.0,
        damping=(1.0, 2.0, 3.0, 4.0, 5.0, 6.0),
        reference_frame=1,
        compliant_axis=(1, 0, 1, 0, 1, 0),
    ) if custom else {}
    admittance = XArmAdmittanceConfig(**values)
    cfg = XArmConfig(admittance=admittance)
    assert cfg.admittance is admittance
    for name, value in values.items():
        assert getattr(cfg.admittance, name) == value
    follower = XArmFollower(cfg)
    arm = Mock()
    for name in ("set_ft_sensor_admittance_parameters", "set_ft_sensor_enable",
                 "set_ft_sensor_mode", "set_state"):
        getattr(arm, name).return_value = 0
    follower._robot = arm
    follower._is_connected = True
    monkeypatch.setattr("robopy.robots.xarm.xarm_follower.time.sleep", lambda _: None)

    follower.enable_admittance_control()

    mass = [0.1] * 3 + [0.002] * 3 if custom else [0.06] * 3 + [0.06 * 0.01] * 3
    stiffness = [500.0] * 3 + [8.0] * 3 if custom else [1200.0] * 3 + [4.0] * 3
    damping = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0] if custom else [0.0] * 6
    axes = [1, 0, 1, 0, 1, 0] if custom else [0, 0, 1, 0, 0, 0]
    assert arm.mock_calls == [
        call.set_ft_sensor_admittance_parameters(mass, stiffness, damping),
        call.set_ft_sensor_admittance_parameters(1 if custom else 0, axes),
        call.set_ft_sensor_enable(1),
        call.set_ft_sensor_mode(1),
        call.set_state(0),
    ]
    assert follower._admittance_enabled is True
    assert follower._motion_paused is True
