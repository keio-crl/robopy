"""Standing the Rakuda on a pedestal, so its hands reach a work surface.

The Rakuda cannot reach the surface it is bolted to.  Its shoulders sit 0.41 m
above its own base plate and each arm is about 0.30 m long, so a hand gets no
closer than **0.112 m** to the mounting plane.  Put the robot on a table and it
waves above everything on it.

The fix is the one CALVIN uses for its Franka: mount the robot at a height
chosen *relative to the work surface* rather than standing it on the floor and
hoping.  In ``calvin_D`` the arm's ``robot_base_position`` is ``z = 0.24`` while
the play table's surface is ``z = 0.46`` -- the base is fixed 0.22 m below the
surface it works on, on an implied mount with no geometry of its own.  This
module does the same for the Rakuda, with two differences: the offset is
measured from this robot's own reachable set rather than inherited, and the
pedestal is a real box, so the robot is visibly standing on something instead of
floating.

Where the offset comes from
---------------------------
Sampling the joint ranges and asking how often a hand lands in the 8 cm band
just above a horizontal surface, as a function of how far that surface sits
above the mounting plane:

======  ==================  ==================
offset  reachable, any side  reachable, in front
======  ==================  ==================
0.12    1.0%                0.2%
0.18    4.2%                1.1%
0.22    7.1%                2.0%
0.30    11.5%               3.3%
**0.34**  **12.4%**         **3.4%**
0.40    12.1%               3.7%
======  ==================  ==================

It peaks around 0.35, which is no surprise: the median hand height is 0.407
above the mounting plane, so a surface there sits in the middle of the
workspace, and a hand can come at it from above or below.  Anything under about
0.15 is close to useless, and below 0.112 is impossible.

:data:`WORK_OFFSET_ABOVE_MOUNT` is that number.  Everything else follows from
it: name the height of the work surface and the pedestal height is determined.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Tuple

__all__ = [
    "DEFAULT_WORK_SURFACE_Z",
    "GRASP_OFFSET_ABOVE_MOUNT",
    "HAND_FLOOR_ABOVE_MOUNT",
    "PLATE_CENTRE_XY",
    "PLATE_SIZE_XY",
    "RakudaMount",
    "STAND_HEIGHT",
    "WORK_OFFSET_ABOVE_MOUNT",
]

#: Distance from the mounting plane (the underside of the base plate) up to the
#: origin of the model's ``base`` body.  The CAD origin is at the waist, so a
#: Rakuda placed at ``z = 0`` is buried to the chest.
STAND_HEIGHT: float = 0.25752

#: Closest a hand can get to the mounting plane.  Measured, and the hard floor
#: under :data:`WORK_OFFSET_ABOVE_MOUNT`: a surface nearer than this cannot be
#: touched at all.
HAND_FLOOR_ABOVE_MOUNT: float = 0.1117

#: How far above the mounting plane to put a work surface.  See the table in the
#: module docstring; this is where the hands actually work.
WORK_OFFSET_ABOVE_MOUNT: float = 0.34

#: Centre and size of the base plate in the robot's own frame.  The plate is not
#: centred on the origin, so a pedestal drawn at ``x = y = 0`` would stick out at
#: the back and leave the front corner unsupported.
PLATE_CENTRE_XY: Tuple[float, float] = (0.0276, -0.0284)
PLATE_SIZE_XY: Tuple[float, float] = (0.300, 0.300)

#: Work surface height for the bundled tasks: an ordinary desk.
DEFAULT_WORK_SURFACE_Z: float = 0.75

#: Offset to use when the task has to *grasp* rather than touch.
#:
#: :data:`WORK_OFFSET_ABOVE_MOUNT` maximises how often a hand can get to a
#: surface, which is what reaching and pushing need.  Closing a gripper on
#: something on that surface is a different requirement: the hand has to arrive
#: pointing *down* at it, and that depends on where the object sits relative to
#: the shoulders.  Reaching out at shoulder height, the hand points forward.
#:
#: Sampling the arm and counting configurations that both reach a surface in
#: front of the robot and point the fingers down at it (world ``z < -0.7``):
#:
#: =========  ==================  =====================
#: offset     reachable in front  of those, pointing down
#: =========  ==================  =====================
#: 0.10       9660                952
#: **0.15**   9735                **408**
#: 0.20       8983                111
#: 0.25       7343                15
#: 0.30       4917                0
#: 0.34       2746                **0**
#: =========  ==================  =====================
#:
#: At the reaching offset a top-down grasp is not merely hard, it is impossible.
#: 0.15 leaves plenty of downward grasps and still keeps the surface clear of
#: :data:`HAND_FLOOR_ABOVE_MOUNT`, which the palm must respect even when the
#: fingers reach 42 mm past it.
GRASP_OFFSET_ABOVE_MOUNT: float = 0.15

#: Gap left between the pedestal's top and the robot's base plate.
#:
#: Both are fixed to the world, so resting one on the other is harmless but not
#: free: MuJoCo would find those contacts every step, and a scene that starts in
#: contact is harder to reason about when something later goes wrong.  Two
#: millimetres is invisible and keeps the robot's own geometry contact-free at
#: reset.
_PEDESTAL_GAP: float = 0.002

#: How much wider than the base plate to make the pedestal, per side.
_PEDESTAL_MARGIN: float = 0.02


@dataclass(frozen=True)
class RakudaMount:
    """Where to stand the robot, given the height of the work surface.

    The robot is always at ``x = y = 0`` facing ``+x``; only the height is
    derived.  Build one, then use :attr:`base_position` to place the robot,
    :meth:`pedestal` to put something under it, and :attr:`near_edge_x` to keep
    a table out of its way.

    Attributes:
        work_surface_z: World height of the surface the hands should work on.
        offset: How far the mounting plane sits below that surface.  The default
            is measured; lowering it below
            :data:`HAND_FLOOR_ABOVE_MOUNT` puts the surface out of reach
            entirely, which :meth:`validate` refuses.

    Example:
        >>> mount = RakudaMount(work_surface_z=0.75)
        >>> round(mount.top_z, 3), round(mount.base_position[2], 5)
        (0.41, 0.66752)
    """

    work_surface_z: float = DEFAULT_WORK_SURFACE_Z
    offset: float = WORK_OFFSET_ABOVE_MOUNT

    def __post_init__(self) -> None:
        self.validate()

    def validate(self) -> None:
        """Raise if the hands could not reach the work surface from here.

        Raises:
            ValueError: If the offset is at or below the closest a hand gets to
                the mounting plane, or if the pedestal would have to go
                underground.
        """
        if self.offset <= HAND_FLOOR_ABOVE_MOUNT:
            raise ValueError(
                f"a work surface {self.offset:.3f} m above the mounting plane is unreachable: "
                f"the Rakuda's hands stop {HAND_FLOOR_ABOVE_MOUNT:.3f} m above it. Raise the "
                "surface, or lower the robot."
            )
        if self.top_z < 0.0:
            raise ValueError(
                f"a work surface at z={self.work_surface_z:.3f} would put the robot's mounting "
                f"plane at z={self.top_z:.3f}, below the floor. Raise the work surface to at "
                f"least {self.offset:.3f}."
            )

    # -- placement --------------------------------------------------------- #

    @property
    def top_z(self) -> float:
        """World height of the pedestal's top, i.e. of the robot's mounting plane."""
        return self.work_surface_z - self.offset

    @property
    def base_position(self) -> Tuple[float, float, float]:
        """Where the robot's ``base`` body goes -- CALVIN's ``robot_base_position``."""
        return (0.0, 0.0, self.top_z + STAND_HEIGHT)

    @property
    def near_edge_x(self) -> float:
        """Smallest ``x`` a table may occupy without running into the pedestal."""
        return PLATE_CENTRE_XY[0] + PLATE_SIZE_XY[0] / 2.0 + _PEDESTAL_MARGIN

    def height_above_surface(self, z: float) -> float:
        """How far ``z`` sits above the mounting plane."""
        return z - self.top_z

    def reachable(self, z: float) -> bool:
        """Whether a hand can get down to the world height ``z`` at all."""
        return self.height_above_surface(z) >= HAND_FLOOR_ABOVE_MOUNT

    def to_world(self, point: Tuple[float, float, float]) -> Tuple[float, float, float]:
        """Lift a point given relative to the mounting plane into world coordinates."""
        return (point[0], point[1], point[2] + self.top_z)

    # -- scene geometry ---------------------------------------------------- #

    def pedestal(self, name: str = "rakuda_mount", color=(0.32, 0.34, 0.38)):
        """A box for the robot to stand on, sized and placed under its base plate.

        Args:
            name: Object name in the scene.
            color: RGB.

        Returns:
            A ``PrimitiveCubeCfg`` for the pedestal, and its ``pos`` from
            :meth:`pedestal_position`.

        Raises:
            ImportError: If MetaSim is not installed.
        """
        from metasim.constants import PhysicStateType
        from metasim.scenario.objects import PrimitiveCubeCfg

        return PrimitiveCubeCfg(
            name=name,
            size=self.pedestal_size,
            color=color,
            physics=PhysicStateType.GEOM,
            fix_base_link=True,
        )

    @property
    def pedestal_size(self) -> Tuple[float, float, float]:
        """``(x, y, z)`` of the pedestal box."""
        return (
            PLATE_SIZE_XY[0] + 2 * _PEDESTAL_MARGIN,
            PLATE_SIZE_XY[1] + 2 * _PEDESTAL_MARGIN,
            max(self.top_z - _PEDESTAL_GAP, 1e-3),
        )

    def pedestal_position(self) -> Tuple[float, float, float]:
        """Centre of the pedestal box: under the plate, resting on the floor."""
        return (PLATE_CENTRE_XY[0], PLATE_CENTRE_XY[1], self.pedestal_size[2] / 2.0)

    def table(
        self,
        name: str = "table",
        depth: float = 0.34,
        width: float = 0.60,
        color=(0.82, 0.76, 0.62),
    ):
        """A table in front of the robot whose top is the work surface.

        Its near edge starts at :attr:`near_edge_x`, so it never overlaps the
        pedestal.

        Args:
            name: Object name in the scene.
            depth: Extent along ``x``.
            width: Extent along ``y``.
            color: RGB.

        Returns:
            A ``PrimitiveCubeCfg`` for the table; place it at
            :meth:`table_position`.
        """
        from metasim.constants import PhysicStateType
        from metasim.scenario.objects import PrimitiveCubeCfg

        return PrimitiveCubeCfg(
            name=name,
            size=(depth, width, self.work_surface_z),
            color=color,
            physics=PhysicStateType.GEOM,
            fix_base_link=True,
        )

    def table_position(self, depth: float = 0.34) -> Tuple[float, float, float]:
        """Centre of the table box, resting on the floor in front of the robot."""
        return (self.near_edge_x + depth / 2.0, 0.0, self.work_surface_z / 2.0)

    def describe(self) -> Dict[str, float]:
        """The derived heights, for a log line or a test."""
        return {
            "work_surface_z": self.work_surface_z,
            "offset_above_mount": self.offset,
            "pedestal_top_z": self.top_z,
            "robot_base_z": self.base_position[2],
            "table_near_edge_x": self.near_edge_x,
        }
