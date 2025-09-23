from abc import ABC, abstractmethod
from typing import Generic, TypeVar

import numpy as np
from numpy.typing import NDArray

T = TypeVar("T")


class ExpHandler(ABC, Generic[T]):
    @abstractmethod
    def record(self, max_frames: int, if_async: bool = True) -> T:
        pass

    @abstractmethod
    def recode_save(
        self,
        max_frames: int,
        save_path: str,
        if_async: bool = True,
        save_gif: bool = True,
        warmup_time: int = 5,
    ) -> None:
        pass

    @abstractmethod
    def send(self, max_frame: int, fps: int, leader_action: NDArray[np.float32]) -> None:
        pass
