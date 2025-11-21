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
    def record_save(
        self,
        max_frames: int,
        save_path: str,
        if_async: bool = True,
        save_gif: bool = True,
        warmup_time: int = 5,
    ) -> None:
        """New API name for saving recordings. Calls `recode_save` for compatibility.

        Subclasses may implement `recode_save` (old name) or override this
        method if they prefer the new name.
        """
        return self.record_save(max_frames, save_path, if_async, save_gif, warmup_time)

    @abstractmethod
    def send(self, max_frame: int, fps: int, leader_action: NDArray[np.float32]) -> None:
        pass

    @abstractmethod
    def save_metadata(self, save_path: str, data_shape: dict | None) -> None:
        pass
