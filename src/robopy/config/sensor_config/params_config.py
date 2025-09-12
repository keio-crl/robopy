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
