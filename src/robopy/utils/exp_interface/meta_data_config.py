from dataclasses import dataclass, field


@dataclass
class MetaDataConfig:
    record_fps: int = field(default=10)
    task_name: str = field(default="")
    description: str = field(default="")
    date: str = field(default="")
