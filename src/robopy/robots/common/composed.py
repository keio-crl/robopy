from typing import Generic, Protocol, TypeVar

import numpy as np
from numpy.typing import NDArray

S = TypeVar("S", covariant=True)
R = TypeVar("R", covariant=True)
Obs = TypeVar("Obs", covariant=True)


class ComposedRobot(Protocol, Generic[R, S, Obs]):
    """Protocol for composed robotic systems
    with leader and follower arms and sensors.
    """

    @property
    def robot_system(self) -> R: ...

    """Property to access the underlying robot system"""

    @property
    def sensors(self) -> S: ...

    @property
    def is_connected(self) -> bool: ...

    def get_observation(self) -> Obs: ...

    def teleoperation(self, max_seconds: float) -> None: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...

    def record_with_fixed_leader(
        self,
        max_frame: int,
        leader_action: NDArray[np.float32],
        fps: int = 20,
        teleop_hz: int = 100,
        max_processing_time_ms: float = 40,
    ) -> Obs: ...
