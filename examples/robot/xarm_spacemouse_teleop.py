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
    from robopy.config.robot_config import XArmConfig, XArmWorkspaceBounds
    from robopy.robots.xarm.spacemouse_agent import SpaceMouseConfig, run_spacemouse_teleop
    from robopy.robots.xarm.xarm_follower import XArmFollower

    config = XArmConfig(
        follower_ip="192.168.1.240",
        workspace_bounds=XArmWorkspaceBounds(),
    )
    follower = XArmFollower(config)

    try:
        follower.connect()
        logger.info("xArm connected. Starting SpaceMouse teleop...")
        run_spacemouse_teleop(
            follower,
            cfg=SpaceMouseConfig(pos_scale=1.0, rot_scale=0.5, hz=50.0),
            max_seconds=None,
        )
    except KeyboardInterrupt:
        logger.info("Interrupted.")
    finally:
        follower.disconnect()


if __name__ == "__main__":
    main()
