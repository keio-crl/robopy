import numpy as np
from robopy.config.robot_config import GelloArmConfig, XArmConfig, XArmWorkspaceBounds
from robopy.robots.xarm import XArmRobot

config = XArmConfig(
    follower_ip="127.0.0.1",
    leader_port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0VKE-if00-port0",
    workspace_bounds=XArmWorkspaceBounds(),
    start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
    sim_mode=True,
    sim_host="127.0.0.1",
    sim_port=6001,
    gello=GelloArmConfig(
        joint_offsets=(
            np.pi,
            3 * np.pi / 2,
            0.0,
            np.pi,
            3 * np.pi / 2,
            np.pi,
            np.pi,
        ),
        joint_signs=(1, 1, 1, 1, 1, 1, 1),
        gripper_open_deg=204.5,
        gripper_close_deg=162.7,
    ),
)

robot = XArmRobot(config)
robot.connect()
print("leader raw:", robot.robot_system.leader.get_joint_state()[:7])
robot.teleoperation(max_seconds=30.0)
robot.disconnect()
