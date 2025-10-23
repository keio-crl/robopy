from dataclasses import dataclass


@dataclass
class CameraParams:
    name: str
    width: int
    height: int
    fps: int


@dataclass
class TactileParams:
    serial_num: str
    name: str = "main"
    fps: int | None = 30


@dataclass
class AudioParams:
    name: str = "main"
    fps: int | None = 30


@dataclass
class SensorsParams:
    cameras: list[CameraParams] | None = None
    tactile: list[TactileParams] | None = None
    audio: list[AudioParams] | None = None
