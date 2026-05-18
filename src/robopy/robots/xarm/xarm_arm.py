"""Common base class for xArm leader and follower arms.

The leader (GELLO Dynamixel controller) and follower (UFactory xArm7) use very
different back-ends, so :class:`XArmArm` only exposes a thin life-cycle
interface compatible with :class:`robopy.robots.common.arm.Arm`. Concrete
subclasses override everything else.
"""

from abc import abstractmethod

import numpy as np
from numpy.typing import NDArray

from robopy.motor.dynamixel_bus import DynamixelBus
from robopy.robots.common.arm import Arm


class XArmArm(Arm):
    """Abstract base class shared by :class:`XArmLeader` and :class:`XArmFollower`."""

    @abstractmethod
    def connect(self) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def get_joint_state(self) -> NDArray[np.float32]:
        """Return the 8-DOF joint state: ``[joint_1..joint_7, gripper]``.

        Joint angles are in radians (xArm base frame) and the gripper position
        is normalized to ``[0, 1]`` where ``0`` is fully open and ``1`` is
        fully closed.
        """

    # --- Defaults so subclasses only override what they use ---
    @property
    def motors(self) -> DynamixelBus | None:
        return None

    @property
    def motor_names(self) -> list[str]:
        return []

    @property
    def motor_models(self) -> list[str]:
        return []
