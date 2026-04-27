import argparse
from logging import INFO, getLogger

logger = getLogger(__name__)
logger.setLevel(INFO)


def _parse_torque_enabled_arg(
    value: str | None,
    *,
    all_joint_names: list[str],
) -> list[str] | None:
    """Parse CLI arg into torque_enabled value.

    Returns:
        - _UNSET-like object when value is None (argument not provided)
        - None for default (YAML null)
        - [] for enable none
        - list[str] for explicit enable list
    """

    if value is None:
        return None

    v = value.strip()
    if v == "":
        return []
    v_lower = v.lower()
    if v_lower in {"default", "null"}:
        return None
    if v_lower in {"none", "off", "[]"}:
        return []
    if v_lower == "all":
        return list(all_joint_names)

    parts = [p.strip() for p in v.split(",") if p.strip()]
    return parts


def rakuda_teleoperate() -> None:
    from robopy.config.dotrobopy import (
        ensure_rakuda_yaml_exists,
        update_rakuda_yaml_torque_enabled,
    )
    from robopy.config.robot_config.rakuda_config import RAKUDA_JOINT_NAMES, RakudaConfig
    from robopy.robots.rakuda.rakuda_robot import RakudaRobot

    parser = argparse.ArgumentParser(description="Rakuda teleoperation with torque config")
    parser.add_argument("--leader-port", default="/dev/ttyUSB1")
    parser.add_argument("--follower-port", default="/dev/ttyUSB0")
    parser.add_argument(
        "--leader-torque-enabled",
        default=None,
        help=(
            "Comma-separated joints to torque ON. "
            "Use 'default'/'null' for default behavior, 'none' for all OFF, 'all' for all ON. "
            "If omitted, do not modify YAML."
        ),
    )
    parser.add_argument(
        "--follower-torque-enabled",
        default=None,
        help=(
            "Comma-separated joints to torque ON. "
            "Use 'default'/'null' for default behavior, 'none' for all OFF, 'all' for all ON. "
            "If omitted, do not modify YAML."
        ),
    )
    args = parser.parse_args()

    all_joints = list(RAKUDA_JOINT_NAMES)
    leader_val = _parse_torque_enabled_arg(args.leader_torque_enabled, all_joint_names=all_joints)
    follower_val = _parse_torque_enabled_arg(
        args.follower_torque_enabled, all_joint_names=all_joints
    )

    # Ensure `.robopy/rakuda/config.yaml` exists, and optionally update torque settings.
    # (The robot init also ensures this, but we do it here so the user can see/edit the file.)
    yaml_path = ensure_rakuda_yaml_exists()
    if args.leader_torque_enabled is not None or args.follower_torque_enabled is not None:
        if args.leader_torque_enabled is not None and args.follower_torque_enabled is not None:
            yaml_path = update_rakuda_yaml_torque_enabled(
                leader=leader_val,
                follower=follower_val,
            )
        elif args.leader_torque_enabled is not None:
            yaml_path = update_rakuda_yaml_torque_enabled(leader=leader_val)
        else:
            yaml_path = update_rakuda_yaml_torque_enabled(follower=follower_val)

    logger.info(f"Using Rakuda dotconfig: {yaml_path}")

    config = RakudaConfig(leader_port=args.leader_port, follower_port=args.follower_port)
    rakuda = RakudaRobot(config)

    try:
        rakuda.connect()
        rakuda.teleoperation()
    finally:
        try:
            rakuda.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    rakuda_teleoperate()
