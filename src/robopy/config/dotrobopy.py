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
    - bool False (which is what YAML 1.1 makes of a bare `off`/`no`) -> []
    """

    if value is None:
        return None
    if isinstance(value, bool):
        # YAML 1.1 turns a bare `off` / `no` into False (and `on` / `yes` into
        # True) before this parser ever sees a string, so the documented `off`
        # keyword arrives as a boolean. False means the same thing as `off`.
        if value is False:
            return []
        raise ValueError(
            f"{field_name}: a bare `on`/`yes`/`true` is ambiguous here because YAML reads it as a "
            "boolean. Write `all` to enable every joint, or list the joints explicitly."
        )
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
            "  torque_enabled: null",
            "  # torque_enabled:",
            "  #   - torso_yaw",
            "",
            "# ---------------------------------------------------------------",
            "# Dual-arm IK and bilateral control (optional).",
            "#",
            "# Leaving this section out keeps the legacy position teleoperation",
            "# behaviour exactly as it was: no kinematic model is loaded and no",
            "# current is ever commanded.",
            "#",
            "# Values that have not been measured on the actual machine are left",
            "# as null. They are NOT filled in with plausible-looking numbers:",
            "# a null makes the affected feature refuse to run, which is the",
            "# point. In particular `allow_hardware_current_output` stays false",
            "# until the calibration below has been measured and marked",
            "# validated: true, so a freshly generated config cannot start",
            "# driving current.",
            "# ---------------------------------------------------------------",
            "# control:",
            "#   mode: position_teleop      # position_teleop|cartesian_teleop|bilateral_joint",
            "#   control_period_s: 0.005",
            "#   ik_period_s: 0.02",
            "#   read_timeout_s: 0.02",
            "#   write_timeout_s: 0.02",
            "#   max_state_age_s: 0.05      # measured state staleness",
            "#   max_command_age_s: 0.05    # command staleness",
            "#   max_target_age_s: 0.2      # external Cartesian target staleness",
            "#   max_cross_bus_skew_s: 0.01",
            "#   diagnostics_period_s: 1.0",
            "#   stop_policy: zero_current  # hold_position|zero_current|torque_off",
            "#   bus_watchdog_counts: null  # 20 ms per count, or null to leave alone",
            "#   allow_hardware_current_output: false",
            "#",
            "#   model:",
            "#     urdf_path: null          # e.g. models/assembly_2/urdf/"
            "assembly_2_convex_collision.urdf",
            "#     package_dirs: []         # dirs that resolve package:// URIs",
            "#     torso_joint: torso_yaw_dof",
            "#     left_arm_joints: [shoulder_pitch_left_dof, shoulder_roll_left_dof,",
            "#                       elbow_yaw_left_dof, elbow_pitch_left_dof,",
            "#                       wrist_yaw_left_dof, wrist_pitch_left_dof]",
            "#     right_arm_joints: [shoulder_pitch_right_dof, shoulder_roll_right_dof,",
            "#                        elbow_yaw_right_dof, elbow_pitch_right_dof,",
            "#                        wrist_yaw_right_dof, wrist_pitch_right_dof]",
            "#     head_joints: []          # modelled, never driven by the arm IK",
            "#     build_collision: false",
            "#     geometry_only: true      # the CAD export's masses are not usable",
            "#     left_tcp:",
            "#       parent_frame: gripper_left_dof",
            "#       translation_m: [0.0, 0.0, 0.0]   # MEASURE THIS",
            "#       quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]",
            "#       validated: false",
            "#     right_tcp:",
            "#       parent_frame: gripper_right_dof",
            "#       translation_m: [0.0, 0.0, 0.0]   # MEASURE THIS",
            "#       quaternion_xyzw: [0.0, 0.0, 0.0, 1.0]",
            "#       validated: false",
            "#     soft_limits_rad:         # required for continuous joints",
            "#       torso_yaw_dof: [-1.5, 1.5]",
            "#     collision_exclusions: [] # [geom_a, geom_b, reason] triples",
            "#",
            "#   # Per-motor calibration. The URDF joint a motor drives is NOT",
            "#   # inferred from its name: r_arm_sh_pitch2, r_arm_el_yaw and",
            "#   # r_arm_wr_roll do not map onto the URDF names by any rule.",
            "#   follower_joint_calibration:",
            "#     torso_yaw:",
            "#       urdf_joint: torso_yaw_dof",
            "#       direction: 1",
            "#       zero_count: null       # MEASURE THIS",
            "#       lower_limit_rad: null",
            "#       upper_limit_rad: null",
            "#       torque_constant_nm_per_a: null",
            "#       current_limit_a: null",
            "#       validated: false",
            "#   leader_joint_calibration: {}",
            "#",
            "#   bilateral:",
            "#     coupled_motors: []       # at most the 13 arm+torso joints",
            "#     stiffness_nm_per_rad: 1.0",
            "#     damping_nm_s_per_rad: 0.05",
            "#     scale: 1.0",
            "#     offset_rad: 0.0",
            "#     max_torque_nm: 0.3",
            "#     max_torque_rate_nm_s: 10.0",
            "#     leader_current_limit_a: {}",
            "#     follower_current_limit_a: {}",
            "#     velocity_filter_hz: 20.0",
            "#     ramp_time_s: 1.0",
            "#     max_alignment_error_rad: 0.1",
            "#     allow_uncompensated: false",
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
    elif cfg.follower_torque_enabled is None:
        # The follower's documented default is "every joint", and it is resolved
        # here so that the loaded configuration states it rather than leaving it
        # implicit. This is the same set the follower already torque-enabled for
        # a null value, so no behaviour changes; it only makes the policy
        # readable from the config object (and is what
        # tests/test_rakuda_torque_config.py has always asserted).
        #
        # The leader default (grippers only) is deliberately *not* resolved
        # here: RakudaLeader distinguishes "no policy given" from an explicit
        # list, and an explicit empty list must stay empty.
        updates["follower_torque_enabled"] = list(RAKUDA_JOINT_NAMES)

    control = parse_rakuda_control_yaml(data.get("control"))
    if control is not None:
        validate_rakuda_control(
            control,
            leader_torque_enabled=(
                leader_torque_enabled
                if leader_torque_enabled is not None
                else cfg.leader_torque_enabled
            ),
            follower_torque_enabled=(
                follower_torque_enabled
                if follower_torque_enabled is not None
                else cfg.follower_torque_enabled
            ),
        )
        updates["control"] = control

    if not updates:
        return cfg

    return replace(cfg, **updates)


def _as_float_or_none(value: Any, *, field_name: str) -> float | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{field_name} must be a number or null.")
    return float(value)


def _as_int_or_none(value: Any, *, field_name: str) -> int | None:
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{field_name} must be an integer or null.")
    return int(value)


def _as_scalar_or_map(value: Any, *, field_name: str, default: Any) -> Any:
    if value is None:
        return default
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    if isinstance(value, dict):
        out: dict[str, float] = {}
        for key, item in value.items():
            if isinstance(item, bool) or not isinstance(item, (int, float)):
                raise ValueError(f"{field_name}.{key} must be a number.")
            out[str(key)] = float(item)
        return out
    raise ValueError(f"{field_name} must be a number, a mapping of numbers, or null.")


def _parse_tcp(value: Any, *, field_name: str) -> Any:
    from robopy.config.robot_config.rakuda_config import RakudaTcpSpec

    if value is None:
        return None
    data = _as_dict(value)
    parent = data.get("parent_frame")
    if not isinstance(parent, str) or not parent:
        raise ValueError(f"{field_name}.parent_frame is required and must be a frame name.")
    translation = data.get("translation_m", [0.0, 0.0, 0.0])
    quaternion = data.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0])
    if not isinstance(translation, list) or len(translation) != 3:
        raise ValueError(f"{field_name}.translation_m must be a list of three numbers.")
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        raise ValueError(
            f"{field_name}.quaternion_xyzw must be a list of four numbers in (x, y, z, w) order."
        )
    return RakudaTcpSpec(
        parent_frame=parent,
        translation_m=tuple(float(v) for v in translation),  # type: ignore[arg-type]
        quaternion_xyzw=tuple(float(v) for v in quaternion),  # type: ignore[arg-type]
        validated=bool(data.get("validated", False)),
    )


def _parse_calibration(value: Any, *, field_name: str) -> dict[str, Any]:
    from robopy.config.robot_config.rakuda_config import RakudaJointCalibrationSpec

    out: dict[str, Any] = {}
    for motor, entry in _as_dict(value).items():
        spec = _as_dict(entry)
        direction = spec.get("direction", 1)
        if direction not in (1, -1):
            raise ValueError(f"{field_name}.{motor}.direction must be +1 or -1.")
        out[str(motor)] = RakudaJointCalibrationSpec(
            urdf_joint=spec.get("urdf_joint"),
            direction=int(direction),
            zero_count=_as_int_or_none(
                spec.get("zero_count"), field_name=f"{field_name}.{motor}.zero_count"
            ),
            drive_mode=_as_int_or_none(
                spec.get("drive_mode"), field_name=f"{field_name}.{motor}.drive_mode"
            ),
            homing_offset=_as_int_or_none(
                spec.get("homing_offset"), field_name=f"{field_name}.{motor}.homing_offset"
            ),
            lower_limit_rad=_as_float_or_none(
                spec.get("lower_limit_rad"), field_name=f"{field_name}.{motor}.lower_limit_rad"
            ),
            upper_limit_rad=_as_float_or_none(
                spec.get("upper_limit_rad"), field_name=f"{field_name}.{motor}.upper_limit_rad"
            ),
            max_velocity_rad_s=_as_float_or_none(
                spec.get("max_velocity_rad_s"),
                field_name=f"{field_name}.{motor}.max_velocity_rad_s",
            ),
            max_acceleration_rad_s2=_as_float_or_none(
                spec.get("max_acceleration_rad_s2"),
                field_name=f"{field_name}.{motor}.max_acceleration_rad_s2",
            ),
            torque_constant_nm_per_a=_as_float_or_none(
                spec.get("torque_constant_nm_per_a"),
                field_name=f"{field_name}.{motor}.torque_constant_nm_per_a",
            ),
            current_limit_a=_as_float_or_none(
                spec.get("current_limit_a"), field_name=f"{field_name}.{motor}.current_limit_a"
            ),
            max_current_rate_a_s=_as_float_or_none(
                spec.get("max_current_rate_a_s"),
                field_name=f"{field_name}.{motor}.max_current_rate_a_s",
            ),
            validated=bool(spec.get("validated", False)),
            notes=str(spec.get("notes", "")),
        )
    return out


def parse_rakuda_control_yaml(value: Any) -> Any:
    """Parse the optional ``control:`` section into a ``RakudaControlConfig``.

    Args:
        value: The raw YAML value, or ``None`` when the section is absent.

    Returns:
        A ``RakudaControlConfig``, or ``None`` when the section is absent, which
        keeps the legacy behaviour unchanged.

    Raises:
        ValueError: On an unknown mode, an unknown stop policy, or a malformed
            entry.
    """
    from robopy.config.robot_config.rakuda_config import (
        RakudaBilateralConfig,
        RakudaControlConfig,
        RakudaModelConfig,
    )

    if value is None:
        return None
    data = _as_dict(value)

    mode = str(data.get("mode", "position_teleop"))
    allowed_modes = {"position_teleop", "cartesian_teleop", "bilateral_joint"}
    if mode not in allowed_modes:
        raise ValueError(f"control.mode must be one of {sorted(allowed_modes)}, got '{mode}'.")

    stop_policy = str(data.get("stop_policy", "zero_current"))
    allowed_stops = {"hold_position", "zero_current", "torque_off"}
    if stop_policy not in allowed_stops:
        raise ValueError(
            f"control.stop_policy must be one of {sorted(allowed_stops)}, got '{stop_policy}'."
        )

    model_data = _as_dict(data.get("model"))
    exclusions: list[tuple[str, str, str]] = []
    for entry in model_data.get("collision_exclusions") or []:
        if not isinstance(entry, list) or len(entry) != 3:
            raise ValueError(
                "control.model.collision_exclusions entries must be "
                "[geometry_a, geometry_b, reason] triples; a reason is required so that an "
                "exclusion is a recorded decision rather than a blanket filter."
            )
        exclusions.append((str(entry[0]), str(entry[1]), str(entry[2])))

    soft_limits: dict[str, tuple[float, float]] = {}
    for joint, bounds in _as_dict(model_data.get("soft_limits_rad")).items():
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError(
                f"control.model.soft_limits_rad.{joint} must be a [lower, upper] pair."
            )
        lower, upper = float(bounds[0]), float(bounds[1])
        if lower > upper:
            raise ValueError(
                f"control.model.soft_limits_rad.{joint}: lower {lower} exceeds upper {upper}."
            )
        soft_limits[str(joint)] = (lower, upper)

    model = RakudaModelConfig(
        urdf_path=model_data.get("urdf_path"),
        package_dirs=[str(d) for d in (model_data.get("package_dirs") or [])],
        torso_joint=model_data.get("torso_joint"),
        left_arm_joints=[str(j) for j in (model_data.get("left_arm_joints") or [])],
        right_arm_joints=[str(j) for j in (model_data.get("right_arm_joints") or [])],
        head_joints=[str(j) for j in (model_data.get("head_joints") or [])],
        left_tcp=_parse_tcp(model_data.get("left_tcp"), field_name="control.model.left_tcp"),
        right_tcp=_parse_tcp(model_data.get("right_tcp"), field_name="control.model.right_tcp"),
        soft_limits_rad=soft_limits,
        build_collision=bool(model_data.get("build_collision", False)),
        collision_exclusions=exclusions,
        geometry_only=bool(model_data.get("geometry_only", True)),
    )

    bilateral_data = _as_dict(data.get("bilateral"))
    bilateral = RakudaBilateralConfig(
        coupled_motors=[str(m) for m in (bilateral_data.get("coupled_motors") or [])],
        stiffness_nm_per_rad=_as_scalar_or_map(
            bilateral_data.get("stiffness_nm_per_rad"),
            field_name="control.bilateral.stiffness_nm_per_rad",
            default=1.0,
        ),
        damping_nm_s_per_rad=_as_scalar_or_map(
            bilateral_data.get("damping_nm_s_per_rad"),
            field_name="control.bilateral.damping_nm_s_per_rad",
            default=0.05,
        ),
        scale=_as_scalar_or_map(
            bilateral_data.get("scale"), field_name="control.bilateral.scale", default=1.0
        ),
        offset_rad=_as_scalar_or_map(
            bilateral_data.get("offset_rad"),
            field_name="control.bilateral.offset_rad",
            default=0.0,
        ),
        max_torque_nm=_as_scalar_or_map(
            bilateral_data.get("max_torque_nm"),
            field_name="control.bilateral.max_torque_nm",
            default=0.3,
        ),
        max_torque_rate_nm_s=_as_scalar_or_map(
            bilateral_data.get("max_torque_rate_nm_s"),
            field_name="control.bilateral.max_torque_rate_nm_s",
            default=10.0,
        ),
        leader_current_limit_a=_as_scalar_or_map(
            bilateral_data.get("leader_current_limit_a"),
            field_name="control.bilateral.leader_current_limit_a",
            default={},
        ),
        follower_current_limit_a=_as_scalar_or_map(
            bilateral_data.get("follower_current_limit_a"),
            field_name="control.bilateral.follower_current_limit_a",
            default={},
        ),
        velocity_filter_hz=_as_float_or_none(
            bilateral_data.get("velocity_filter_hz", 20.0),
            field_name="control.bilateral.velocity_filter_hz",
        ),
        ramp_time_s=float(bilateral_data.get("ramp_time_s", 1.0)),
        max_alignment_error_rad=float(bilateral_data.get("max_alignment_error_rad", 0.1)),
        allow_uncompensated=bool(bilateral_data.get("allow_uncompensated", False)),
    )

    return RakudaControlConfig(
        mode=mode,
        control_period_s=float(data.get("control_period_s", 0.005)),
        ik_period_s=float(data.get("ik_period_s", 0.02)),
        read_timeout_s=float(data.get("read_timeout_s", 0.02)),
        write_timeout_s=float(data.get("write_timeout_s", 0.02)),
        max_state_age_s=float(data.get("max_state_age_s", 0.05)),
        max_command_age_s=float(data.get("max_command_age_s", 0.05)),
        max_target_age_s=float(data.get("max_target_age_s", 0.2)),
        max_cross_bus_skew_s=float(data.get("max_cross_bus_skew_s", 0.01)),
        diagnostics_period_s=float(data.get("diagnostics_period_s", 1.0)),
        stop_policy=stop_policy,
        bus_watchdog_counts=_as_int_or_none(
            data.get("bus_watchdog_counts"), field_name="control.bus_watchdog_counts"
        ),
        leader_joint_calibration=_parse_calibration(
            data.get("leader_joint_calibration"), field_name="control.leader_joint_calibration"
        ),
        follower_joint_calibration=_parse_calibration(
            data.get("follower_joint_calibration"),
            field_name="control.follower_joint_calibration",
        ),
        model=model,
        bilateral=bilateral,
        allow_hardware_current_output=bool(data.get("allow_hardware_current_output", False)),
    )


def validate_rakuda_control(
    control: Any,
    *,
    leader_torque_enabled: list[str] | None,
    follower_torque_enabled: list[str] | None,
) -> None:
    """Cross-check the control section against the rest of the configuration.

    Args:
        control: The parsed ``RakudaControlConfig``.
        leader_torque_enabled: Effective leader torque policy, or ``None`` for
            the default (grippers only).
        follower_torque_enabled: Effective follower torque policy, or ``None``
            for the default (all joints).

    Raises:
        ValueError: If a motor name is unknown, a bilateral joint is not in the
            torque-enabled set, a gripper or head motor is coupled, or
            Cartesian mode is selected without a usable model.
    """
    from robopy.config.robot_config.rakuda_config import (
        RAKUDA_GRIPPER_MOTOR_NAMES,
        RAKUDA_HEAD_MOTOR_NAMES,
        RAKUDA_IK_MOTOR_NAMES,
        RAKUDA_JOINT_NAMES,
    )

    allowed = set(RAKUDA_JOINT_NAMES)
    for side, calibration in (
        ("leader", control.leader_joint_calibration),
        ("follower", control.follower_joint_calibration),
    ):
        unknown = sorted(set(calibration) - allowed)
        if unknown:
            raise ValueError(f"control.{side}_joint_calibration names unknown motor(s): {unknown}.")
        claimed: dict[str, str] = {}
        for motor, spec in calibration.items():
            if spec.urdf_joint is None:
                continue
            if spec.urdf_joint in claimed:
                raise ValueError(
                    f"control.{side}_joint_calibration: URDF joint '{spec.urdf_joint}' is "
                    f"assigned to both '{claimed[spec.urdf_joint]}' and '{motor}'."
                )
            claimed[spec.urdf_joint] = motor

    coupled = control.bilateral.coupled_motors
    if coupled:
        unknown = sorted(set(coupled) - allowed)
        if unknown:
            raise ValueError(f"control.bilateral.coupled_motors is unknown: {unknown}.")
        if len(set(coupled)) != len(coupled):
            raise ValueError("control.bilateral.coupled_motors contains duplicates.")
        forbidden = sorted(
            set(coupled) & (set(RAKUDA_GRIPPER_MOTOR_NAMES) | set(RAKUDA_HEAD_MOTOR_NAMES))
        )
        if forbidden:
            raise ValueError(
                f"control.bilateral.coupled_motors must not include {forbidden}. The head and "
                "the grippers are held or disabled by a separate policy and are never added to "
                "the bilateral set implicitly."
            )
        outside = sorted(set(coupled) - set(RAKUDA_IK_MOTOR_NAMES))
        if outside:
            raise ValueError(
                f"control.bilateral.coupled_motors contains {outside}, which are outside the "
                f"thirteen arm and torso joints {list(RAKUDA_IK_MOTOR_NAMES)}."
            )

        # A joint that the bilateral controller drives but that the torque
        # policy leaves off is a contradiction. The coupling set is not
        # adjusted, and the configured torque-OFF is not overridden.
        effective_leader = (
            ["l_arm_grip", "r_arm_grip"] if leader_torque_enabled is None else leader_torque_enabled
        )
        effective_follower = (
            list(RAKUDA_JOINT_NAMES) if follower_torque_enabled is None else follower_torque_enabled
        )
        for side, enabled in (("leader", effective_leader), ("follower", effective_follower)):
            disabled = sorted(set(coupled) - set(enabled))
            if disabled:
                raise ValueError(
                    f"control.bilateral.coupled_motors includes {disabled}, which {side}."
                    "torque_enabled leaves off. Fix the configuration: the coupling set is not "
                    "narrowed automatically and a configured torque-OFF is not overridden."
                )

    if control.mode == "bilateral_joint" and not coupled:
        raise ValueError("control.mode is bilateral_joint but no coupled_motors are configured.")

    if control.mode == "cartesian_teleop":
        model = control.model
        problems: list[str] = []
        if not model.urdf_path:
            problems.append("model.urdf_path is not set")
        if not model.torso_joint:
            problems.append("model.torso_joint is not set")
        if len(model.left_arm_joints) != 6:
            problems.append(
                f"model.left_arm_joints has {len(model.left_arm_joints)} entries, not 6"
            )
        if len(model.right_arm_joints) != 6:
            problems.append(
                f"model.right_arm_joints has {len(model.right_arm_joints)} entries, not 6"
            )
        if model.left_tcp is None or model.right_tcp is None:
            problems.append("model.left_tcp and model.right_tcp must both be defined")
        if problems:
            raise ValueError(
                "control.mode is cartesian_teleop but the model is incomplete: "
                + "; ".join(problems)
                + "."
            )


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
