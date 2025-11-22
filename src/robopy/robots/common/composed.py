from typing import Generic, Protocol, TypeVar

T = TypeVar("T", covariant=True)


class ComposedRobot(Protocol, Generic[T]):
    """Protocol for composed robotic systems
    with leader and follower arms and sensors.
    """

    @property
    def robot_system(self) -> T: ...

    """Property to access the underlying robot system"""

    @property
    def sensors(self) -> T: ...

    @property
    def is_connected(self) -> bool: ...

    def get_observation(self) -> T: ...

    def teleoperation(self, max_seconds: float) -> None: ...

    def connect(self) -> None: ...

    def disconnect(self) -> None: ...
