from logging import INFO, getLogger

from robopy.config import RealsenseCameraConfig
from robopy.config.robot_config.koch_config import KochConfig, KochSensorConfig
from robopy.utils import KochExpHandler, MetaDataConfig

logger = getLogger(__name__)
logger.setLevel(level=INFO)


def main() -> None:
    try:
        koch_config = KochConfig(
            leader_port="/dev/ttyACM1",
            follower_port="/dev/ttyACM0",
            calibration_path="/home/fujii/.cache/calibration/curtain_new.pkl",
            sensors=KochSensorConfig(
                cameras={
                    "main": RealsenseCameraConfig(
                        width=640, height=480, fps=30, is_depth_camera=True
                    )
                }
            ),
        )
        # koch.connect()
        # obs = koch.get_observation()
        # print(obs.cameras["main.rgb"].shape)
        # print(obs.cameras["main.depth"].shape)
        # # koch.teleoperation(max_seconds=5)
        # koch.disconnect()

        handler = KochExpHandler(
            koch_config=koch_config,
            metadata_config=MetaDataConfig(
                task_name="test_koch_recording",
                description="Testing Koch robot recording functionality.",
                date="2024-06-01",
            ),
            fps=5,
        )

        handler.record_save(100, "test", if_async=True)
    except Exception as e:
        raise e


if __name__ == "__main__":
    main()
