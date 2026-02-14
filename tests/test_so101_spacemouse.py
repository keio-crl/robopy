from robopy.config.input_config.spacemouse_config import SpaceMouseConfig
from robopy.config.robot_config.so101_config import So101Config
from robopy.robots.so101.so101_robot import So101Robot
from robopy.robots.so101.so101_spacemouse import So101SpaceMouseController

# Followerのみの設定（Leaderアーム不要）
config = So101Config(
    follower_port="/dev/tty.usbmodem5AA90174481",
    calibration_path="calibration/so101_calib.json",
)

# SpaceMouseの速度をカスタマイズ
spacemouse_config = SpaceMouseConfig(
    linear_speed=0.25,   # 並進速度: 0.15 m/s (デフォルト: 0.10)
    angular_speed=0.8,   # 回転速度: 0.8 rad/s (デフォルト: 0.5)
    gripper_speed=80.0,  # グリッパー速度: 80 deg/s (デフォルト: 50.0)
    deadzone=0.05,       # デッドゾーン (デフォルト: 0.05)
    control_hz=50,       # 制御周波数: 50 Hz (デフォルト: 50)
)

robot = So101Robot(cfg=config)
controller = So101SpaceMouseController(robot, spacemouse_config=spacemouse_config)

try:
    # ロボット接続 + SpaceMouse起動
    controller.connect()

    # SpaceMouseで30秒間テレオペレーション
    controller.teleoperation(max_seconds=30)

    # 時間制限なし（Ctrl+C で停止）
    # controller.teleoperation()

finally:
    controller.disconnect()
