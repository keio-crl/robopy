from dataclasses import dataclass


@dataclass
class CameraParams:
    name: str
    width: int
    height: int
    fps: int


@dataclass
class TactileParams:
    name: str
    fps: int
