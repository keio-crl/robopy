"""Rakuda robot modules."""

from typing import TYPE_CHECKING

from .rakuda_arm import RakudaArm
from .rakuda_follower import RakudaFollower
from .rakuda_leader import RakudaLeader
from .rakuda_pair_sys import RakudaPairSys
from .rakuda_robot import RakudaRobot

if TYPE_CHECKING:  # pragma: no cover - typing only
    from .rakuda_control import RakudaControlSystem

__all__ = [
    "RakudaArm",
    "RakudaControlSystem",
    "RakudaFollower",
    "RakudaLeader",
    "RakudaPairSys",
    "RakudaRobot",
]


def __getattr__(name: str) -> object:
    """Expose ``RakudaControlSystem`` lazily.

    It is imported on demand so that ``import robopy.robots.rakuda`` does not
    pull in the control stack -- and, through it, the optional ``kinematics``
    extra -- for code that only uses position teleoperation.
    """
    if name == "RakudaControlSystem":
        from .rakuda_control import RakudaControlSystem

        return RakudaControlSystem
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
