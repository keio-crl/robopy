from robopy.config.robot_config.rakuda_config import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config.params_config import AudioParams, CameraParams, TactileParams
from robopy.utils.exp_interface.meta_data_config import MetaDataConfig
from robopy.utils.exp_interface.rakuda_exp_handler import RakudaExpHandler

# add audio sensor to Rakuda config
audio_params = AudioParams(name="main", fps=20)
camera_params = CameraParams(name="main", width=640, height=480, fps=20)
tactile_params = TactileParams(serial_num="D20542", name="left", fps=30)

sensor_params = RakudaSensorParams(
    cameras=[camera_params], tactile=[tactile_params], audio=[audio_params]
)

config = RakudaConfig(
    leader_port="/dev/ttyUSB0", follower_port="/dev/ttyUSB1", sensors=sensor_params
)

metadata_config = MetaDataConfig(
    experiment_name="audio_test", description="Testing audio synchronization"
)

# create experiment handler
handler = RakudaExpHandler(rakuda_config=config, metadata_config=metadata_config, fps=20)

# test data collection and saving
handler.record_save(max_frames=50, save_path="audio_test", save_gif=False)
