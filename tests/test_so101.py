
from robopy.config.robot_config.so101_config import So101Config
from robopy.robots.so101.so101_robot import So101Robot




def main() -> None:
    try:
        # Leader付き設定（テレオペレーション用）
        config = So101Config(
            leader_port="/dev/tty.usbmodem5AA90170791",
            follower_port="/dev/tty.usbmodem5AA90174481",
            calibration_path="calibration/so101_calib.json",
        )

        robot = So101Robot(config)

        # 接続（初回はキャリブレーションが実行されます）
        robot.connect()

        # テレオペレーション（10秒間）
        robot.teleoperation(max_seconds=10)

    except Exception as e:
        print(f"エラーが発生しました: {e}")

    finally:
        robot.disconnect()


if __name__ == "__main__":
    main()
