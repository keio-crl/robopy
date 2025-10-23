import time

from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import AudioParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot

# add audio sensor to Rakuda config
audio_params = AudioParams(name="main", fps=20)
sensor_params = RakudaSensorParams(
    cameras=[],
    tactile=[],
    audio=[audio_params],
)

config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=sensor_params,
)

# initialize RakudaRobot
robot = RakudaRobot(config)

# connect
robot.connect()

# test sensor observation
for i in range(5):
    obs = robot.get_observation()
    if obs.sensors and obs.sensors.audio:
        audio_data = obs.sensors.audio.get("main")
        if audio_data is not None:
            print(f"Audio data shape: {audio_data.shape}")
        else:
            print("No audio data")
    time.sleep(0.5)

# disconnect
robot.disconnect()
