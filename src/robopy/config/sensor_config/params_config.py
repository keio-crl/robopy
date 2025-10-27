from dataclasses import dataclass


@dataclass
class CameraParams:
    name: str
    width: int
    height: int
    fps: int
    index: int = 0  # Add index attribute for RealSense camera


@dataclass
class TactileParams:
    serial_num: str
    name: str = "main"
    fps: int | None = 30


@dataclass
class AudioParams:
    name: str = "main"
    fps: int | None = 30
    # Audio processing parameters
    sample_rate: int = 44100
    n_mels: int = 64
    n_fft: int = 2048
    hop_length: int = 690
    fmax: int = 15000


@dataclass
class SensorsParams:
    cameras: list[CameraParams] | None = None
    tactile: list[TactileParams] | None = None
    audio: list[AudioParams] | None = None
