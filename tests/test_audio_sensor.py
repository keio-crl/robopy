import time

from robopy.config.sensor_config.params_config import AudioParams
from robopy.sensors.audio.audio_sensor import AudioSensor

# initialize audio sensor
audio_config = AudioParams(name="main", fps=20)
audio_sensor = AudioSensor(audio_config)

# connect
audio_sensor.connect()

# get data
for i in range(10):
    data = audio_sensor.read()
    if data is not None:
        print(f"Frame {i}: Shape={data.shape}, Min={data.min():.2f}, Max={data.max():.2f}")
    else:
        print(f"Frame {i}: No data")
    time.sleep(0.1)

# disconnect
audio_sensor.disconnect()
