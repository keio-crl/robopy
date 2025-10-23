from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import AudioParams, CameraParams, TactileParams
from robopy.robots.rakuda.rakuda_robot import RakudaRobot

# add all sensors to Rakuda config
audio_params = AudioParams(name="main", fps=20)
camera_params = CameraParams(name="main", width=640, height=480, fps=20)
tactile_params = TactileParams(serial_num="D20542", name="left", fps=30)

sensor_params = RakudaSensorParams(
    cameras=[camera_params], tactile=[tactile_params], audio=[audio_params]
)

config = RakudaConfig(
    leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1", sensors=sensor_params
)

robot = RakudaRobot(config)
robot.connect()

# test synchronized data collection
print("Starting synchronized data collection test...")
obs = robot.record_parallel(max_frame=10, fps=20)

# check results
print(f"Collected {len(obs.arms.leader)} frames")
if obs.sensors:
    if obs.sensors.cameras:
        for name, data in obs.sensors.cameras.items():
            if data is not None:
                print(f"Camera {name}: {data.shape}")
    if obs.sensors.tactile:
        for name, data in obs.sensors.tactile.items():
            if data is not None:
                print(f"Tactile {name}: {data.shape}")
    if obs.sensors.audio:
        for name, data in obs.sensors.audio.items():
            if data is not None:
                print(f"Audio {name}: {data.shape}")

robot.disconnect()
