from dataclasses import dataclass
from typing import Sequence

from robopy.sensors.audio.audio_sensor import AudioSensor
from robopy.sensors.tactile.digit_sensor import DigitSensor
from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.sensors.visual.web_camera import WebCamera


@dataclass
class Sensors:
    cameras: Sequence[RealsenseCamera | WebCamera] | None
    tactile: Sequence[DigitSensor] | None = None
    audio: Sequence[AudioSensor] | None = None
