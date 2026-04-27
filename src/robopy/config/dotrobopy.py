from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import TYPE_CHECKING, Any

import yaml

if TYPE_CHECKING:
    from robopy.config.robot_config.rakuda_config import RakudaConfig


_UNSET: Any = object()


def _as_dict(value: Any) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ValueError(f"Expected a mapping in YAML, got {type(value).__name__}.")
    return value


def _as_str_list_or_none(value: Any, *, field_name: str) -> list[str] | None:
    if value is None:
        return None
    if isinstance(value, list):
        if not all(isinstance(x, str) for x in value):
            raise ValueError(f"{field_name} must be a list[str] or null.")
        return list(value)
    raise ValueError(f"{field_name} must be a list[str] or null.")


def _parse_torque_enabled_yaml(
    value: Any,
    *,
    field_name: str,
    all_joint_names: tuple[str, ...],
) -> list[str] | None:
    """Parse YAML torque_enabled into list[str] | None.

    Accepts:
    - null -> None (use default behavior)
    - list[str] -> explicit list (empty list allowed)
    - str keywords:
        - 'all' -> all_joint_names
        - 'default'/'null' -> None
        - 'none'/'off' -> []
    """

    if value is None:
        return None
    if isinstance(value, list):
        if not all(isinstance(x, str) for x in value):
            raise ValueError(f"{field_name} must be a list[str], a keyword, or null.")
        return list(value)
    if isinstance(value, str):
        v = value.strip().lower()
        if v in {"default", "null"}:
            return None
        if v in {"none", "off"}:
            return []
        if v == "all":
            return list(all_joint_names)
        raise ValueError(f"{field_name} must be a list[str], one of (all/default/none), or null.")
    raise ValueError(f"{field_name} must be a list[str], a keyword, or null.")


def get_dotrobopy_dir(base_dir: Path | None = None) -> Path:
    """Return the base `.robopy` directory.

    Default: current working directory.
    """

    return (base_dir or Path.cwd()) / ".robopy"


def get_rakuda_dotdir(base_dir: Path | None = None) -> Path:
    """Return the Rakuda config directory: `.robopy/rakuda`."""

    return get_dotrobopy_dir(base_dir) / "rakuda"


def get_rakuda_yaml_path(base_dir: Path | None = None) -> Path:
    """Return the Rakuda YAML config path: `.robopy/rakuda/config.yaml`."""

    return get_rakuda_dotdir(base_dir) / "config.yaml"


def ensure_rakuda_dotfiles(base_dir: Path | None = None) -> Path:
    """Ensure `.robopy/rakuda` exists and return its path."""

    dotdir = get_rakuda_dotdir(base_dir)
    dotdir.mkdir(parents=True, exist_ok=True)
    return dotdir


def ensure_rakuda_yaml_exists(base_dir: Path | None = None) -> Path:
    """Ensure `.robopy/rakuda/config.yaml` exists and return its path.

    This is the file-level entry point: callers should prefer this over checking
    directory existence.
    """

    from robopy.config.robot_config.rakuda_config import RAKUDA_JOINT_NAMES

    yaml_path = get_rakuda_yaml_path(base_dir)
    yaml_path.parent.mkdir(parents=True, exist_ok=True)
    ensure_default_rakuda_yaml(yaml_path, joint_names=RAKUDA_JOINT_NAMES)
    return yaml_path


def ensure_default_rakuda_yaml(path: Path, *, joint_names: tuple[str, ...]) -> None:
    """Create a default Rakuda YAML if it does not exist.

    The default file should be non-invasive: it must not change runtime behavior
    unless the user edits it.
    """

    if path.exists():
        return

    joint_list_comment = "\n".join([f"  # - {name}" for name in joint_names])

    content = "\n".join(
        [
            "# robopy user config (Rakuda)",
            "#",
            "# This file is created automatically. Edit it to customize Rakuda behavior.",
            "#",
            "# Semantics:",
            "# - leader.torque_enabled: joints to torque ON (null -> default: grippers only)",
            "# - follower.torque_enabled: joints to torque ON (null -> default: all joints)",
            "#",
            "# Available joint names:",
            joint_list_comment,
            "",
            "leader:",
            "  torque_enabled: null",
            "  # torque_enabled:",
            "  #   - l_arm_grip",
            "  #   - r_arm_grip",
            "",
            "follower:",
            "  torque_enabled: all",
            "  # torque_enabled:",
            "  #   - torso_yaw",
            "",
        ]
    )

    path.write_text(content, encoding="utf-8")


def load_yaml(path: Path) -> dict[str, Any]:
    """Load YAML file safely. Returns {} for empty files."""

    if not path.exists():
        return {}
    text = path.read_text(encoding="utf-8")
    if not text.strip():
        return {}
    loaded = yaml.safe_load(text)
    return _as_dict(loaded)


def validate_joint_names(
    names: list[str] | None,
    *,
    allowed: set[str],
    field_name: str,
) -> None:
    if names is None:
        return
    unknown = sorted(set(names) - allowed)
    if unknown:
        allowed_preview = ", ".join(sorted(allowed))
        raise ValueError(
            f"Unknown joint name(s) in {field_name}: {unknown}. Allowed: {allowed_preview}"
        )


def apply_rakuda_dotconfig(
    cfg: "RakudaConfig",
    *,
    base_dir: Path | None = None,
) -> "RakudaConfig":
    """Apply `.robopy/rakuda/config.yaml` overrides to a RakudaConfig.

    This also ensures the directory and default YAML exist.
    """

    # Local import to avoid circular dependency in robopy.config package.
    from robopy.config.robot_config.rakuda_config import RAKUDA_JOINT_NAMES, RakudaConfig

    if not isinstance(cfg, RakudaConfig):
        raise TypeError("apply_rakuda_dotconfig expects a RakudaConfig")

    yaml_path = ensure_rakuda_yaml_exists(base_dir)

    data = load_yaml(yaml_path)

    leader = _as_dict(data.get("leader"))
    follower = _as_dict(data.get("follower"))

    leader_torque_enabled = _parse_torque_enabled_yaml(
        leader.get("torque_enabled"),
        field_name="leader.torque_enabled",
        all_joint_names=RAKUDA_JOINT_NAMES,
    )
    follower_torque_enabled = _parse_torque_enabled_yaml(
        follower.get("torque_enabled"),
        field_name="follower.torque_enabled",
        all_joint_names=RAKUDA_JOINT_NAMES,
    )

    allowed = set(RAKUDA_JOINT_NAMES)
    validate_joint_names(leader_torque_enabled, allowed=allowed, field_name="leader.torque_enabled")
    validate_joint_names(
        follower_torque_enabled, allowed=allowed, field_name="follower.torque_enabled"
    )

    # Only override if YAML explicitly provides a non-null value.
    updates: dict[str, Any] = {}
    if leader_torque_enabled is not None:
        updates["leader_torque_enabled"] = leader_torque_enabled
    if follower_torque_enabled is not None:
        updates["follower_torque_enabled"] = follower_torque_enabled

    if not updates:
        return cfg

    return replace(cfg, **updates)


def update_rakuda_yaml_torque_enabled(
    *,
    leader: list[str] | None | Any = _UNSET,
    follower: list[str] | None | Any = _UNSET,
    base_dir: Path | None = None,
) -> Path:
    """Update `.robopy/rakuda/config.yaml` torque settings.

    - leader / follower:
        - _UNSET: do not modify the field
        - None: write YAML null (meaning: use default behavior)
        - list[str]: explicit joints to torque ON (empty list means torque OFF for all)
    """

    from robopy.config.robot_config.rakuda_config import RAKUDA_JOINT_NAMES

    yaml_path = ensure_rakuda_yaml_exists(base_dir)

    data = load_yaml(yaml_path)
    if not isinstance(data, dict):
        data = {}

    allowed = set(RAKUDA_JOINT_NAMES)
    if leader is not _UNSET:
        validate_joint_names(leader, allowed=allowed, field_name="leader.torque_enabled")
    if follower is not _UNSET:
        validate_joint_names(follower, allowed=allowed, field_name="follower.torque_enabled")

    leader_dict = _as_dict(data.get("leader"))
    follower_dict = _as_dict(data.get("follower"))

    if leader is not _UNSET:
        leader_dict["torque_enabled"] = leader
    if follower is not _UNSET:
        follower_dict["torque_enabled"] = follower

    data["leader"] = leader_dict
    data["follower"] = follower_dict

    yaml_path.write_text(
        yaml.safe_dump(data, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )
    return yaml_path
