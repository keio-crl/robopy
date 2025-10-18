from abc import ABC, abstractmethod
from typing import Generic, TypeVar

import numpy as np
from numpy.typing import NDArray

T = TypeVar("T")


class SaveWorker(ABC, Generic[T]):
    @abstractmethod
    def save_arm_datas(
        self, leader_obs: NDArray[np.float32], follower_obs: NDArray[np.float32], path: str
    ) -> None:
        pass

    @abstractmethod
    def save_all_obs(self, obs: T, save_path: str, save_gif: bool) -> None:
        pass
