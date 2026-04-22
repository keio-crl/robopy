import numpy as np

from robopy.config.robot_config import XArmConfig, XArmWorkspaceBounds
from robopy.robots.xarm import XArmRobot

config = XArmConfig(
    follower_ip="127.0.0.1",
    leader_port="/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT8J0VKE-if00-port0",
    workspace_bounds=XArmWorkspaceBounds(),
    start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
    sim_mode=True,  # SimXArmFollower (ZMQ) を使用
    sim_host="127.0.0.1",
    sim_port=6001,
)

robot = XArmRobot(config)
robot.connect()  # 内部で XArmAPI("127.0.0.1", is_radian=True) が呼ばれる
robot.teleoperation(max_seconds=30.0)
robot.disconnect()
