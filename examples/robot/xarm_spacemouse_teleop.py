"""Teleoperate an xArm7 with a 3Dconnexion SpaceMouse (no GELLO required).

Usage:
    uv run python examples/robot/xarm_spacemouse_teleop.py

Prerequisites:
    * ``pyspacemouse`` installed (``uv add pyspacemouse``).
    * SpaceMouse connected via USB.
    * xArm7 reachable (real or UFactory Studio simulator).

Controls:
    * SpaceMouse 6-DOF → Cartesian relative EE movement (mm + rad).
    * Button 0 → toggle gripper open/closed.
    * Ctrl-C → stop and disconnect.
"""

from logging import INFO, basicConfig, getLogger

basicConfig(level=INFO)
logger = getLogger(__name__)


def main() -> None:
    from robopy.config.input_config.spacemouse_config import SpaceMouseConfig
    from robopy.config.robot_config import XArmConfig, XArmWorkspaceBounds
    from robopy.robots.xarm.spacemouse_agent import run_spacemouse_teleop
    from robopy.robots.xarm.xarm_follower import XArmFollower

    config = XArmConfig(
        follower_ip="192.168.1.240",
        workspace_bounds=XArmWorkspaceBounds(),
    )
    follower = XArmFollower(config)

    # linear_speed / angular_speed act as the sensitivity knobs. Lower these to
    # make the robot move slowly even when the SpaceMouse is fully deflected.
    sm_cfg = SpaceMouseConfig(
        linear_speed=0.10,  # m/s at full stick deflection
        angular_speed=0.5,  # rad/s at full stick deflection
        deadzone=0.05,
        control_hz=50,
        input_smoothing=0.5,  # EMA factor: higher = smoother but more lag
    )

    try:
        follower.connect()
        logger.info("xArm connected. Starting SpaceMouse teleop...")
        run_spacemouse_teleop(
            follower,
            cfg=sm_cfg,
            max_seconds=None,
            gripper_toggle_button=0,
        )
    except KeyboardInterrupt:
        logger.info("Interrupted.")
    finally:
        follower.disconnect()


if __name__ == "__main__":
    main()
