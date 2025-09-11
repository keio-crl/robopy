from dataclasses import dataclass
from typing import List

from robopy.sensors.tactile.digit_sensor import DigitSensor
from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.sensors.visual.web_camera import WebCamera


@dataclass
class Sensors:
    cameras: List[RealsenseCamera | WebCamera] | None
    tactile: List[DigitSensor] | None = None
