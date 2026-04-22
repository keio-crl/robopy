import numpy as np
from robopy.config.robot_config import XArmConfig, XArmWorkspaceBounds
from robopy.robots.xarm import XArmRobot

config = XArmConfig(
    follower_ip="127.0.0.1",             # ← シミュレータの IP に変更
    leader_port=None,                    # GELLO は実機を USB 接続
    workspace_bounds=XArmWorkspaceBounds(),
    start_joints=np.deg2rad([0, -90, 90, -90, -90, 0, 0]).astype(np.float32),
)

robot = XArmRobot(config)
robot.connect()          # 内部で XArmAPI("127.0.0.1", is_radian=True) が呼ばれる
robot.teleoperation(max_seconds=30.0)
robot.disconnect()
