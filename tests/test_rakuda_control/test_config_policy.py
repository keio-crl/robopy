"""Configuration semantics: torque policy keywords and the control section."""

from __future__ import annotations

from pathlib import Path

import pytest
import yaml

from robopy.config.dotrobopy import (
    apply_rakuda_dotconfig,
    get_rakuda_yaml_path,
    parse_rakuda_control_yaml,
    validate_rakuda_control,
)
from robopy.config.robot_config.rakuda_config import RAKUDA_JOINT_NAMES, RakudaConfig


def _write(tmp_path: Path, body: str) -> None:
    path = get_rakuda_yaml_path(tmp_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(body, encoding="utf-8")


def _config() -> RakudaConfig:
    return RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")


class TestTorquePolicySemantics:
    """``null``, omitted, explicit ``[]`` and ``all`` each keep their meaning."""

    def test_null_leaves_the_leader_default_in_place(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        out = apply_rakuda_dotconfig(_config())
        # The leader default (grippers only) is applied at connect time, so the
        # config keeps `None` and does not pretend to be an explicit list.
        assert out.leader_torque_enabled is None

    def test_null_resolves_the_follower_default_to_every_joint(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        out = apply_rakuda_dotconfig(_config())
        assert out.follower_torque_enabled == list(RAKUDA_JOINT_NAMES)

    def test_an_omitted_section_behaves_like_null(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        _write(tmp_path, "leader: {}\n")
        out = apply_rakuda_dotconfig(_config())
        assert out.leader_torque_enabled is None
        assert out.follower_torque_enabled == list(RAKUDA_JOINT_NAMES)

    def test_an_explicit_empty_list_means_no_torque(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        _write(tmp_path, "leader:\n  torque_enabled: []\nfollower:\n  torque_enabled: []\n")
        out = apply_rakuda_dotconfig(_config())
        assert out.leader_torque_enabled == []
        assert out.follower_torque_enabled == []

    def test_the_all_keyword_expands_to_every_joint(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        _write(tmp_path, "follower:\n  torque_enabled: all\n")
        out = apply_rakuda_dotconfig(_config())
        assert out.follower_torque_enabled == list(RAKUDA_JOINT_NAMES)

    @pytest.mark.parametrize("keyword", ["none", "off", "no", "'off'"])
    def test_the_none_and_off_keywords_mean_an_empty_list(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, keyword: str
    ) -> None:
        # A bare `off` or `no` is read as a YAML boolean before it ever reaches
        # the parser; it still has to mean "no torque".
        monkeypatch.chdir(tmp_path)
        _write(tmp_path, f"leader:\n  torque_enabled: {keyword}\n")
        assert apply_rakuda_dotconfig(_config()).leader_torque_enabled == []

    def test_a_bare_yes_is_rejected_as_ambiguous(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        _write(tmp_path, "leader:\n  torque_enabled: yes\n")
        with pytest.raises(ValueError, match="Write `all`"):
            apply_rakuda_dotconfig(_config())

    def test_an_unknown_joint_name_is_rejected(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        _write(tmp_path, "leader:\n  torque_enabled:\n    - not_a_joint\n")
        with pytest.raises(ValueError, match=r"Unknown joint name\(s\)"):
            apply_rakuda_dotconfig(_config())


class TestLeaderEmptyListIsHonoured:
    def test_an_empty_list_no_longer_force_enables_the_grippers(self) -> None:
        # The connect path used to torque-enable both grippers unconditionally,
        # ignoring an explicit empty list. Reproduced here through the
        # PairSys-level resolution, which is what filters outgoing writes.
        from robopy.robots.rakuda.rakuda_pair_sys import _filter_action_by_enabled_joints

        enabled: set[str] = set()
        assert _filter_action_by_enabled_joints({"l_arm_grip": 2600.0}, enabled) == {}

    def test_the_leader_connect_path_respects_an_empty_list(self) -> None:
        import inspect

        from robopy.robots.rakuda.rakuda_leader import RakudaLeader

        source = inspect.getsource(RakudaLeader.connect)
        # The unconditional enable is gone; enabling is driven by the list.
        assert "grippers_enabled" in source
        assert 'torque_enabled(specific_motor_names=["l_arm_grip", "r_arm_grip"])' not in source


class TestControlSection:
    def test_an_absent_section_keeps_the_legacy_behaviour(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        assert apply_rakuda_dotconfig(_config()).control is None

    def test_the_generated_template_does_not_enable_current_output(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        apply_rakuda_dotconfig(_config())
        text = get_rakuda_yaml_path(tmp_path).read_text(encoding="utf-8")
        assert "allow_hardware_current_output: false" in text
        # Everything in the control block is commented out, so a freshly
        # generated file changes nothing.
        assert yaml.safe_load(text).get("control") is None

    def test_a_control_section_is_parsed(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        _write(
            tmp_path,
            """
leader:
  torque_enabled: [torso_yaw]
control:
  mode: bilateral_joint
  control_period_s: 0.002
  stop_policy: hold_position
  bus_watchdog_counts: 5
  bilateral:
    coupled_motors: [torso_yaw]
    stiffness_nm_per_rad: 3.0
    damping_nm_s_per_rad: {torso_yaw: 0.2}
  follower_joint_calibration:
    torso_yaw:
      urdf_joint: torso_yaw_dof
      direction: -1
      zero_count: 2050
      validated: true
""".lstrip(),
        )
        out = apply_rakuda_dotconfig(_config())
        assert out.control is not None
        assert out.control.mode == "bilateral_joint"
        assert out.control.control_period_s == pytest.approx(0.002)
        assert out.control.stop_policy == "hold_position"
        assert out.control.bus_watchdog_counts == 5
        assert out.control.bilateral.stiffness_nm_per_rad == pytest.approx(3.0)
        assert out.control.bilateral.damping_nm_s_per_rad == {"torso_yaw": 0.2}
        spec = out.control.follower_joint_calibration["torso_yaw"]
        assert (spec.direction, spec.zero_count, spec.validated) == (-1, 2050, True)
        # Unmeasured values stay None rather than being filled in.
        assert spec.torque_constant_nm_per_a is None

    def test_an_unknown_mode_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="control.mode must be one of"):
            parse_rakuda_control_yaml({"mode": "teleport"})

    def test_an_unknown_stop_policy_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="control.stop_policy must be one of"):
            parse_rakuda_control_yaml({"stop_policy": "hope"})

    def test_a_collision_exclusion_needs_a_reason(self) -> None:
        with pytest.raises(ValueError, match="recorded decision"):
            parse_rakuda_control_yaml({"model": {"collision_exclusions": [["a", "b"]]}})

    def test_an_inverted_soft_limit_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="lower .* exceeds upper"):
            parse_rakuda_control_yaml({"model": {"soft_limits_rad": {"j": [1.0, -1.0]}}})

    def test_a_tcp_without_a_parent_frame_is_rejected(self) -> None:
        with pytest.raises(ValueError, match="parent_frame is required"):
            parse_rakuda_control_yaml({"model": {"left_tcp": {"translation_m": [0, 0, 0]}}})


class TestControlValidation:
    def _control(self, **bilateral: object) -> object:
        payload: dict[str, object] = {"mode": "bilateral_joint", "bilateral": bilateral}
        return parse_rakuda_control_yaml(payload)

    def test_a_coupled_joint_that_is_torque_off_is_rejected(self) -> None:
        control = self._control(coupled_motors=["torso_yaw"])
        with pytest.raises(ValueError, match="not narrowed automatically"):
            validate_rakuda_control(
                control,
                leader_torque_enabled=["l_arm_grip"],
                follower_torque_enabled=list(RAKUDA_JOINT_NAMES),
            )

    def test_a_gripper_cannot_be_coupled(self) -> None:
        control = self._control(coupled_motors=["l_arm_grip"])
        with pytest.raises(ValueError, match="must not include"):
            validate_rakuda_control(
                control, leader_torque_enabled=None, follower_torque_enabled=None
            )

    def test_a_head_joint_cannot_be_coupled(self) -> None:
        control = self._control(coupled_motors=["head_yaw"])
        with pytest.raises(ValueError, match="must not include"):
            validate_rakuda_control(
                control, leader_torque_enabled=None, follower_torque_enabled=None
            )

    def test_duplicates_are_rejected(self) -> None:
        control = self._control(coupled_motors=["torso_yaw", "torso_yaw"])
        with pytest.raises(ValueError, match="contains duplicates"):
            validate_rakuda_control(
                control, leader_torque_enabled=None, follower_torque_enabled=None
            )

    def test_bilateral_mode_needs_coupled_motors(self) -> None:
        control = parse_rakuda_control_yaml({"mode": "bilateral_joint"})
        with pytest.raises(ValueError, match="no coupled_motors are configured"):
            validate_rakuda_control(
                control, leader_torque_enabled=None, follower_torque_enabled=None
            )

    def test_cartesian_mode_needs_a_complete_model(self) -> None:
        control = parse_rakuda_control_yaml({"mode": "cartesian_teleop"})
        with pytest.raises(ValueError, match="the model is incomplete"):
            validate_rakuda_control(
                control, leader_torque_enabled=None, follower_torque_enabled=None
            )

    def test_two_motors_cannot_claim_one_urdf_joint(self) -> None:
        control = parse_rakuda_control_yaml(
            {
                "follower_joint_calibration": {
                    "torso_yaw": {"urdf_joint": "torso_yaw_dof"},
                    "head_yaw": {"urdf_joint": "torso_yaw_dof"},
                }
            }
        )
        with pytest.raises(ValueError, match="is assigned to both"):
            validate_rakuda_control(
                control, leader_torque_enabled=None, follower_torque_enabled=None
            )

    def test_an_unknown_motor_in_the_calibration_is_rejected(self) -> None:
        control = parse_rakuda_control_yaml(
            {"follower_joint_calibration": {"elbow": {"urdf_joint": "x"}}}
        )
        with pytest.raises(ValueError, match="unknown motor"):
            validate_rakuda_control(
                control, leader_torque_enabled=None, follower_torque_enabled=None
            )
