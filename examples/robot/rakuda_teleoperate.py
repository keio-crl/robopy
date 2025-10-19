from logging import INFO, basicConfig

basicConfig(level=INFO)


def rakuda_teleoperate():
    from robopy.config.robot_config.rakuda_config import RakudaConfig
    from robopy.robots.rakuda.rakuda_robot import RakudaRobot

    config = RakudaConfig(leader_port="/dev/ttyUSB1", follower_port="/dev/ttyUSB0")
    rakuda = RakudaRobot(config)
    try:
        rakuda.connect()
        rakuda.teleoperation()
        # Disconnect
        rakuda.disconnect()
    except Exception as e:
        raise e
    except KeyboardInterrupt:
        rakuda.disconnect()


if __name__ == "__main__":
    rakuda_teleoperate()
