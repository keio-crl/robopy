"""Teleoperate a simulated xArm7 via GELLO's sim_xarm (MuJoCo GUI).

Setup (two terminals):

    Terminal 1 — start the GELLO simulator:
        cd gello_software
        python experiments/launch_nodes.py --robot sim_xarm

    Terminal 2 — run this script:
        uv run python examples/robot/xarm_sim_teleop.py

The GELLO controller must be connected via USB. The simulated xArm
appears in the MuJoCo GUI launched by Terminal 1 and follows the GELLO
input in real time.
"""

from logging import INFO, basicConfig, getLogger

import numpy as np

basicConfig(level=INFO)
logger = getLogger(__name__)


def main() -> None:
    from robopy.config.robot_config import XArmConfig
    from robopy.robots.xarm import XArmRobot

    config = XArmConfig(
        sim_mode=True,
        sim_host="127.0.0.1",
        sim_port=6000,
        start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
    )
    robot = XArmRobot(config)
    try:
        robot.connect()
        logger.info("Connected to sim_xarm. Starting GELLO teleop...")
        robot.teleoperation(max_seconds=60.0)
    except KeyboardInterrupt:
        logger.info("Interrupted.")
    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
