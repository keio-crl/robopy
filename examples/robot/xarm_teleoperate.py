"""Minimal xArm7 + GELLO teleoperation example.

Usage:
    uv run python examples/robot/xarm_teleoperate.py

Prerequisites:
    * UFactory xArm7 reachable at ``192.168.1.240`` (override via
      ``XArmConfig.follower_ip``).
    * GELLO Dynamixel controller connected via USB — the port is auto-detected
      from ``/dev/serial/by-id/*`` when ``leader_port`` is left as ``None``.

Simulator (recommended before running on the real robot):
    Launch UFactory Studio in simulation mode, select xArm7 as the virtual
    robot, then change ``follower_ip`` below to the simulator address
    (typically ``127.0.0.1``). No other change is required — the GELLO leader
    still connects to real USB hardware. See ``docs/robots/xarm.md`` for a
    full setup walk-through and the pre-flight checklist for switching back
    to the real robot.
"""

from logging import INFO, getLogger

import numpy as np

logger = getLogger(__name__)
logger.setLevel(INFO)


def xarm_teleoperate() -> None:
    from robopy.config.robot_config import XArmConfig, XArmWorkspaceBounds
    from robopy.robots.xarm import XArmRobot

    config = XArmConfig(
        follower_ip="192.168.1.240",
        leader_port=None,  # auto-detect
        workspace_bounds=XArmWorkspaceBounds(),
        start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
    )
    robot = XArmRobot(config)
    try:
        robot.connect()
        robot.teleoperation(max_seconds=30.0)
    except KeyboardInterrupt:
        logger.info("Teleoperation interrupted by user.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    xarm_teleoperate()
