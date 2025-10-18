from dataclasses import dataclass, field


@dataclass
class MetaDataConfig:
    task_name: str = field(default="")
    description: str = field(default="")
    date: str = field(default="")
