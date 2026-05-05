from __future__ import annotations

from pathlib import Path

import pytest

from robopy.config.dotrobopy import apply_rakuda_dotconfig, get_rakuda_yaml_path
from robopy.config.robot_config.rakuda_config import RAKUDA_JOINT_NAMES, RakudaConfig
from robopy.robots.rakuda.rakuda_pair_sys import _filter_action_by_enabled_joints


def test_dotrobopy_creates_default_yaml(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(tmp_path)

    cfg = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
    out = apply_rakuda_dotconfig(cfg)

    yaml_path = get_rakuda_yaml_path()
    assert yaml_path.exists()
    assert out.leader_torque_enabled is None
    assert out.follower_torque_enabled == list(RAKUDA_JOINT_NAMES)


def test_dotrobopy_supports_all_keyword(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(tmp_path)

    yaml_path = get_rakuda_yaml_path()
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(
        """
follower:
  torque_enabled: all
""".lstrip(),
        encoding="utf-8",
    )

    cfg = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
    out = apply_rakuda_dotconfig(cfg)
    assert out.follower_torque_enabled == list(RAKUDA_JOINT_NAMES)


def test_dotrobopy_applies_yaml_overrides(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(tmp_path)

    yaml_path = get_rakuda_yaml_path()
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(
        """
leader:
  torque_enabled:
    - l_arm_grip
    - r_arm_grip
follower:
  torque_enabled:
    - torso_yaw
    - head_yaw
""".lstrip(),
        encoding="utf-8",
    )

    cfg = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
    out = apply_rakuda_dotconfig(cfg)

    assert out.leader_torque_enabled == ["l_arm_grip", "r_arm_grip"]
    assert out.follower_torque_enabled == ["torso_yaw", "head_yaw"]


def test_dotrobopy_rejects_unknown_joint_names(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    monkeypatch.chdir(tmp_path)

    yaml_path = get_rakuda_yaml_path()
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(
        """
leader:
  torque_enabled:
    - not_a_joint
""".lstrip(),
        encoding="utf-8",
    )

    cfg = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
    with pytest.raises(ValueError, match=r"Unknown joint name\(s\)"):
        apply_rakuda_dotconfig(cfg)


def test_dotrobopy_allows_empty_list(tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.chdir(tmp_path)

    yaml_path = get_rakuda_yaml_path()
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    yaml_path.write_text(
        """
leader:
    torque_enabled: []
follower:
    torque_enabled: []
""".lstrip(),
        encoding="utf-8",
    )

    cfg = RakudaConfig(leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1")
    out = apply_rakuda_dotconfig(cfg)

    assert out.leader_torque_enabled == []
    assert out.follower_torque_enabled == []


def test_filter_action_by_enabled_joints() -> None:
    action = {"a": 1.0, "b": 2.0, "c": 3.0}
    enabled = {"b", "c"}
    assert _filter_action_by_enabled_joints(action, enabled) == {"b": 2.0, "c": 3.0}
