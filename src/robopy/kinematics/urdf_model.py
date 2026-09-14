"""Whole-body URDF model: frames, joint indices, forward kinematics, Jacobians.

This module complements -- it does not replace -- :mod:`robopy.kinematics.chain`.
``KinematicChain`` describes a single serial arm with string axes and a 5-DOF
pose; that API and its users are untouched.  ``WholeBodyModel`` reads an entire
URDF tree, keeps arbitrary ``axis xyz`` directions and every fixed transform,
and works in SE(3).

Pinocchio is imported lazily, so ``import robopy`` keeps working without the
``kinematics`` extra installed.  The clear install message lives in
:func:`require_pinocchio`.

Configuration versus tangent space
----------------------------------
A ``continuous`` URDF joint is stored by Pinocchio as ``(cos q, sin q)``, so the
configuration dimension ``nq`` is larger than the velocity dimension ``nv``.
The two are never used interchangeably here: joint *positions* go through
:meth:`WholeBodyModel.positions_from_q` /
:meth:`WholeBodyModel.q_from_positions`, and configuration updates go through
:meth:`WholeBodyModel.integrate`, which calls Pinocchio's ``integrate`` rather
than adding a velocity to a configuration.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import TYPE_CHECKING, Any, Dict, Iterable, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

if TYPE_CHECKING:  # pragma: no cover - typing only
    import pinocchio as pin

__all__ = [
    "CollisionReport",
    "MissingKinematicsExtra",
    "WholeBodyModel",
    "require_pinocchio",
]

_INSTALL_HINT = (
    "The whole-body URDF model needs the optional 'kinematics' extra.\n"
    "Install it with one of:\n"
    "    uv sync --extra kinematics\n"
    "    uv pip install 'robopy[kinematics]'\n"
    "    pip install 'robopy[kinematics]'\n"
    "It pulls in 'pin' (the official Pinocchio distribution), 'pin-pink' (the "
    "official Pink distribution) and a QP solver. Beware of similarly named "
    "packages on PyPI: the correct names are exactly 'pin' and 'pin-pink'."
)


class MissingKinematicsExtra(ImportError):
    """Raised when the optional ``kinematics`` extra is needed but absent."""


def require_pinocchio() -> "pin":
    """Import and return Pinocchio, with an actionable error when it is missing.

    Returns:
        The ``pinocchio`` module.

    Raises:
        MissingKinematicsExtra: If Pinocchio is not installed.
    """
    try:
        import pinocchio  # noqa: PLC0415
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise MissingKinematicsExtra(_INSTALL_HINT) from exc
    return pinocchio


@dataclass(frozen=True)
class CollisionReport:
    """Self-collision distances at one configuration.

    Attributes:
        pair_names: ``(geometry_a, geometry_b)`` for each checked pair.
        distances: Signed distance for each pair, in metres.  Negative means
            the convex hulls interpenetrate.
        normals: ``(n_pairs, 3)`` unit vectors from the witness point on ``a``
            to the witness point on ``b``, in the world frame.
        witness_a: ``(n_pairs, 3)`` witness points on geometry ``a``, in world.
        witness_b: ``(n_pairs, 3)`` witness points on geometry ``b``, in world.
        joint_a: Parent joint index of geometry ``a`` for each pair.
        joint_b: Parent joint index of geometry ``b`` for each pair.
    """

    pair_names: Tuple[Tuple[str, str], ...]
    distances: NDArray[np.float64]
    normals: NDArray[np.float64]
    witness_a: NDArray[np.float64]
    witness_b: NDArray[np.float64]
    joint_a: Tuple[int, ...]
    joint_b: Tuple[int, ...]

    @property
    def min_distance(self) -> float:
        """Smallest distance over all checked pairs, or ``inf`` when none."""
        return float(np.min(self.distances)) if self.distances.size else float("inf")

    def closest_pair(self) -> Tuple[str, str] | None:
        """The pair achieving :attr:`min_distance`, or ``None``."""
        if not self.pair_names:
            return None
        return self.pair_names[int(np.argmin(self.distances))]


class WholeBodyModel:
    """A whole-robot kinematic model backed by Pinocchio.

    The base link is fixed: no free-flyer joint is added, because Rakuda's
    ``root`` is bolted down and a floating base would silently introduce six
    decision variables that no motor can realise.
    """

    def __init__(
        self,
        model: Any,
        *,
        collision_model: Any | None = None,
        source: str = "",
    ) -> None:
        """Wrap an already-built Pinocchio model.

        Prefer :meth:`from_urdf`; this constructor exists so a model built
        elsewhere can be reused.

        Args:
            model: A ``pinocchio.Model``.
            collision_model: An optional ``pinocchio.GeometryModel``.
            source: Human-readable provenance, used in error messages.
        """
        pin = require_pinocchio()
        self._pin = pin
        self._model = model
        self._data = model.createData()
        self._source = source
        self._collision_model = collision_model
        self._collision_data = collision_model.createData() if collision_model is not None else None

        self._movable_joint_names: Tuple[str, ...] = tuple(str(name) for name in model.names[1:])
        self._joint_id = {name: model.getJointId(name) for name in self._movable_joint_names}
        self._soft_lower: Dict[str, float] = {}
        self._soft_upper: Dict[str, float] = {}

    # -- construction ------------------------------------------------------

    @classmethod
    def from_urdf(
        cls,
        urdf_path: Path | str,
        *,
        package_dirs: Sequence[Path | str] = (),
        build_collision: bool = False,
        geometry_only: bool = False,
    ) -> "WholeBodyModel":
        """Load a URDF with a fixed base.

        Args:
            urdf_path: Path to the ``.urdf`` file.
            package_dirs: Directories used to resolve ``package://`` mesh URIs.
            build_collision: Also load the collision geometry.  Requires the
                meshes to resolve.
            geometry_only: Declare that this model's inertial data is not
                trustworthy.  It changes nothing about the kinematics; it marks
                the model so that anything needing dynamics refuses it instead
                of quietly using placeholder masses.

        Returns:
            The loaded :class:`WholeBodyModel`.

        Raises:
            FileNotFoundError: If the URDF does not exist.
            MissingKinematicsExtra: If Pinocchio is not installed.
        """
        pin = require_pinocchio()
        path = Path(urdf_path)
        if not path.exists():
            raise FileNotFoundError(f"URDF not found: {path}")
        dirs = [str(Path(d)) for d in package_dirs]

        model = pin.buildModelFromUrdf(str(path))
        collision_model = None
        if build_collision:
            if dirs:
                collision_model = pin.buildGeomFromUrdf(
                    model, str(path), pin.GeometryType.COLLISION, dirs
                )
            else:
                collision_model = pin.buildGeomFromUrdf(
                    model, str(path), pin.GeometryType.COLLISION
                )
        instance = cls(model, collision_model=collision_model, source=str(path))
        instance._geometry_only = geometry_only  # noqa: SLF001 - own attribute
        return instance

    # -- basic properties ---------------------------------------------------

    @property
    def model(self) -> Any:
        """The underlying ``pinocchio.Model``."""
        return self._model

    @property
    def data(self) -> Any:
        """The underlying ``pinocchio.Data``."""
        return self._data

    @property
    def collision_model(self) -> Any | None:
        """The underlying ``pinocchio.GeometryModel``, or ``None``."""
        return self._collision_model

    @property
    def source(self) -> str:
        """Where this model came from."""
        return self._source

    @property
    def nq(self) -> int:
        """Configuration-space dimension.  Larger than :attr:`nv` with continuous joints."""
        return int(self._model.nq)

    @property
    def nv(self) -> int:
        """Tangent- (velocity-) space dimension.  One per degree of freedom."""
        return int(self._model.nv)

    @property
    def movable_joint_names(self) -> Tuple[str, ...]:
        """Names of the movable joints, in Pinocchio's internal order."""
        return self._movable_joint_names

    @property
    def frame_names(self) -> Tuple[str, ...]:
        """Every frame name in the model, including fixed-joint frames."""
        return tuple(str(f.name) for f in self._model.frames)

    def has_frame(self, frame: str) -> bool:
        """Whether a frame of this name exists."""
        return bool(self._model.existFrame(frame))

    def has_joint(self, joint: str) -> bool:
        """Whether a *movable* joint of this name exists.

        A fixed joint exists as a frame but is not a movable joint, so this
        returns ``False`` for names such as ``gripper_left_dof``.
        """
        return joint in self._joint_id

    def is_continuous(self, joint: str) -> bool:
        """Whether ``joint`` is an unbounded (``continuous``) revolute joint.

        Continuous joints occupy two configuration entries and one tangent
        entry; their Pinocchio position "limits" of about ``+/-1.01`` bound the
        ``(cos, sin)`` pair and are not a joint range.
        """
        jid = self._require_joint(joint)
        return self._model.joints[jid].nq == 2

    def joint_v_index(self, joint: str) -> int:
        """Index of ``joint`` in the tangent (velocity) vector."""
        return int(self._model.joints[self._require_joint(joint)].idx_v)

    def joint_q_slice(self, joint: str) -> slice:
        """Slice of ``joint`` within the configuration vector."""
        j = self._model.joints[self._require_joint(joint)]
        return slice(int(j.idx_q), int(j.idx_q) + int(j.nq))

    def v_indices(self, joints: Sequence[str]) -> NDArray[np.int_]:
        """Tangent indices of ``joints``, in the given order."""
        return np.asarray([self.joint_v_index(name) for name in joints], dtype=int)

    def _require_joint(self, joint: str) -> int:
        if joint not in self._joint_id:
            available = ", ".join(self._movable_joint_names)
            hint = ""
            if self.has_frame(joint):
                hint = (
                    f" A frame named '{joint}' does exist, but it is a fixed joint; a name "
                    "ending in '_dof' is not evidence of a degree of freedom."
                )
            raise KeyError(
                f"'{joint}' is not a movable joint of {self._source or 'this model'}."
                f"{hint} Movable joints: {available}"
            )
        return int(self._joint_id[joint])

    # -- configuration handling ---------------------------------------------

    def neutral_q(self) -> NDArray[np.float64]:
        """The model's neutral configuration."""
        return np.asarray(self._pin.neutral(self._model), dtype=np.float64)

    def positions_from_q(self, q: NDArray[np.float64]) -> Dict[str, float]:
        """Extract per-joint angles in radians from a configuration vector.

        Continuous joints are decoded with ``atan2(sin, cos)``, so their value
        comes back in ``(-pi, pi]``.  That is a property of the representation,
        not a wrap applied to a measurement: measured multi-turn positions are
        handled in :mod:`robopy.control.joint_mapping` and are only mapped into
        the model here.
        """
        q = np.asarray(q, dtype=np.float64)
        self._check_q(q)
        out: Dict[str, float] = {}
        for name in self._movable_joint_names:
            sl = self.joint_q_slice(name)
            block = q[sl]
            if block.size == 2:
                out[name] = float(np.arctan2(block[1], block[0]))
            else:
                out[name] = float(block[0])
        return out

    def q_from_positions(
        self,
        positions_rad: Mapping[str, float],
        *,
        base: NDArray[np.float64] | None = None,
        require_all: bool = True,
    ) -> NDArray[np.float64]:
        """Build a configuration vector from per-joint angles.

        Args:
            positions_rad: ``{joint_name: radians}``.
            base: Configuration to start from; joints absent from
                ``positions_rad`` keep their value from it.  Defaults to
                :meth:`neutral_q`.
            require_all: Raise when a movable joint has no entry.

        Returns:
            A configuration vector of length :attr:`nq`.

        Raises:
            KeyError: On an unknown joint name, or a missing one when
                ``require_all`` is set.
        """
        q = self.neutral_q() if base is None else np.array(base, dtype=np.float64, copy=True)
        self._check_q(q)
        unknown = sorted(set(positions_rad) - set(self._movable_joint_names))
        if unknown:
            raise KeyError(f"Unknown movable joint(s): {unknown}")
        if require_all:
            missing = sorted(set(self._movable_joint_names) - set(positions_rad))
            if missing:
                raise KeyError(f"Missing joint position(s): {missing}")
        for name, angle in positions_rad.items():
            sl = self.joint_q_slice(name)
            if sl.stop - sl.start == 2:
                q[sl] = (np.cos(angle), np.sin(angle))
            else:
                q[sl] = angle
        return q

    def integrate(self, q: NDArray[np.float64], v: NDArray[np.float64]) -> NDArray[np.float64]:
        """Configuration after applying tangent displacement ``v`` (already times dt)."""
        self._check_q(q)
        self._check_v(v)
        return np.asarray(self._pin.integrate(self._model, q, v), dtype=np.float64)

    def difference(
        self, q_from: NDArray[np.float64], q_to: NDArray[np.float64]
    ) -> NDArray[np.float64]:
        """Tangent displacement taking ``q_from`` to ``q_to``."""
        self._check_q(q_from)
        self._check_q(q_to)
        return np.asarray(self._pin.difference(self._model, q_from, q_to), dtype=np.float64)

    def _check_q(self, q: NDArray[np.float64]) -> None:
        if np.asarray(q).shape != (self.nq,):
            raise ValueError(
                f"Configuration must have shape ({self.nq},) -- this model's nq -- got "
                f"{np.asarray(q).shape}. Note nq != nv ({self.nq} != {self.nv}) because of "
                "continuous joints."
            )

    def _check_v(self, v: NDArray[np.float64]) -> None:
        if np.asarray(v).shape != (self.nv,):
            raise ValueError(
                f"Tangent vector must have shape ({self.nv},) -- this model's nv -- got "
                f"{np.asarray(v).shape}."
            )

    # -- limits -------------------------------------------------------------

    def set_soft_limits(self, limits: Mapping[str, Tuple[float, float]]) -> None:
        """Record soft position limits in radians for named joints.

        Continuous joints carry no URDF range, so a real limit -- cable routing,
        for example -- has to be supplied here.  Without it, IK would happily
        take the shortest angular path through a region the machine cannot
        reach.

        Args:
            limits: ``{joint_name: (lower_rad, upper_rad)}``.

        Raises:
            KeyError: On an unknown joint name.
            ValueError: If a lower limit exceeds its upper limit.
        """
        for name, (lower, upper) in limits.items():
            self._require_joint(name)
            if lower > upper:
                raise ValueError(f"{name}: soft lower limit {lower} exceeds upper limit {upper}.")
            self._soft_lower[name] = float(lower)
            self._soft_upper[name] = float(upper)

    def position_limits(self, joints: Sequence[str]) -> Tuple[NDArray[np.float64], ...]:
        """Effective ``(lower, upper)`` position limits in radians for ``joints``.

        For a bounded joint the URDF limit is used unless a tighter soft limit
        was set.  For a continuous joint only the soft limit applies; when none
        was set the limit is infinite and the caller is responsible for knowing
        that.
        """
        lower: List[float] = []
        upper: List[float] = []
        for name in joints:
            if self.is_continuous(name):
                lo, hi = -np.inf, np.inf
            else:
                sl = self.joint_q_slice(name)
                lo = float(self._model.lowerPositionLimit[sl][0])
                hi = float(self._model.upperPositionLimit[sl][0])
            if name in self._soft_lower:
                lo = max(lo, self._soft_lower[name])
                hi = min(hi, self._soft_upper[name])
            lower.append(lo)
            upper.append(hi)
        return np.asarray(lower, dtype=np.float64), np.asarray(upper, dtype=np.float64)

    def unbounded_joints(self, joints: Sequence[str]) -> List[str]:
        """Joints in ``joints`` that still have no finite position limit."""
        lower, upper = self.position_limits(joints)
        return [
            name
            for i, name in enumerate(joints)
            if not (np.isfinite(lower[i]) and np.isfinite(upper[i]))
        ]

    # -- frames -------------------------------------------------------------

    def add_fixed_frame(
        self,
        name: str,
        parent_frame: str,
        transform: NDArray[np.float64],
    ) -> None:
        """Attach a new operational frame by a fixed transform from ``parent_frame``.

        This is how a TCP is defined.  The existing ``gripper_left_dof`` /
        ``gripper_right_dof`` frames are *not* assumed to sit at the grasp
        centre: the offset from them to the actual TCP is an explicit,
        verifiable input.

        Args:
            name: Name of the new frame.
            parent_frame: Existing frame the transform is relative to.
            transform: ``(4, 4)`` homogeneous transform from ``parent_frame`` to
                the new frame.

        Raises:
            KeyError: If ``parent_frame`` does not exist.
            ValueError: If ``name`` already exists or ``transform`` is malformed.
        """
        pin = self._pin
        if not self.has_frame(parent_frame):
            raise KeyError(f"Parent frame '{parent_frame}' does not exist in {self._source}.")
        if self.has_frame(name):
            raise ValueError(f"Frame '{name}' already exists; refusing to redefine it.")
        T = np.asarray(transform, dtype=np.float64)
        if T.shape != (4, 4):
            raise ValueError("transform must be a 4x4 homogeneous matrix.")

        parent_id = self._model.getFrameId(parent_frame)
        parent = self._model.frames[parent_id]
        placement = parent.placement * pin.SE3(T[:3, :3], T[:3, 3])
        frame = pin.Frame(
            name,
            parent.parentJoint,
            parent_id,
            placement,
            pin.FrameType.OP_FRAME,
        )
        self._model.addFrame(frame)
        self._data = self._model.createData()

    def forward_kinematics(self, q: NDArray[np.float64]) -> None:
        """Update the cached kinematics for configuration ``q``."""
        self._check_q(q)
        self._pin.forwardKinematics(self._model, self._data, np.asarray(q, dtype=np.float64))
        self._pin.updateFramePlacements(self._model, self._data)

    def frame_pose(self, q: NDArray[np.float64], frame: str) -> NDArray[np.float64]:
        """``(4, 4)`` pose of ``frame`` in the fixed base frame at configuration ``q``."""
        if not self.has_frame(frame):
            raise KeyError(f"Unknown frame '{frame}'.")
        self.forward_kinematics(q)
        placement = self._data.oMf[self._model.getFrameId(frame)]
        T = np.eye(4)
        T[:3, :3] = np.asarray(placement.rotation)
        T[:3, 3] = np.asarray(placement.translation)
        return T

    def frame_jacobian(
        self,
        q: NDArray[np.float64],
        frame: str,
        *,
        local: bool = True,
    ) -> NDArray[np.float64]:
        """``(6, nv)`` frame Jacobian at ``q``.

        Args:
            q: Configuration.
            frame: Frame name.
            local: When ``True`` the Jacobian is expressed in the frame itself
                (a *body* Jacobian), matching the convention Pink uses for its
                frame-task errors.  When ``False`` it is in the world-aligned
                local frame.

        Returns:
            A ``(6, nv)`` matrix whose first three rows are linear and last three
            angular, in Pinocchio's ordering.
        """
        pin = self._pin
        if not self.has_frame(frame):
            raise KeyError(f"Unknown frame '{frame}'.")
        self._check_q(q)
        q = np.asarray(q, dtype=np.float64)
        pin.computeJointJacobians(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        reference = pin.ReferenceFrame.LOCAL if local else pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
        return np.asarray(
            pin.getFrameJacobian(self._model, self._data, self._model.getFrameId(frame), reference),
            dtype=np.float64,
        )

    # -- collision ----------------------------------------------------------

    def add_all_collision_pairs(self, *, excluded: Iterable[Tuple[str, str]] = ()) -> int:
        """Register every collision pair except the approved exclusions.

        Args:
            excluded: Geometry-name pairs to skip, in either order.

        Returns:
            The number of pairs registered.

        Raises:
            RuntimeError: If the model was loaded without collision geometry.
        """
        if self._collision_model is None:
            raise RuntimeError(
                "This model was loaded without collision geometry. Rebuild it with "
                "from_urdf(..., build_collision=True)."
            )
        skip = {tuple(sorted(pair)) for pair in excluded}
        self._collision_model.removeAllCollisionPairs()
        names = [str(go.name) for go in self._collision_model.geometryObjects]
        count = 0
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                if tuple(sorted((names[i], names[j]))) in skip:
                    continue
                self._collision_model.addCollisionPair(self._pin.CollisionPair(i, j))
                count += 1
        self._collision_data = self._collision_model.createData()
        return count

    def classify_collision_pairs(
        self,
        q: NDArray[np.float64],
        *,
        touching_tolerance_m: float = 0.0,
    ) -> Dict[str, List[Tuple[str, str]]]:
        """Group candidate collision pairs by *why* they might need excluding.

        Excluding every adjacent or every nearby pair would remove real
        self-collision checks along with the spurious ones.  This method only
        categorises; deciding what to exclude stays an explicit, recorded choice
        passed to :meth:`add_all_collision_pairs`.

        Args:
            q: Configuration to evaluate the geometric categories at -- normally
                a known-good resting pose.
            touching_tolerance_m: Distance at or below which a pair counts as
                interfering at ``q``.

        Returns:
            A mapping with the keys:

            ``same_body``
                Both geometries hang off the same joint: duplicated parts or
                bolted-on hardware that can never collide.
            ``parent_child``
                Directly connected bodies, which touch at their shared joint by
                construction.
            ``interfering_at_q``
                Pairs whose convex hulls already overlap at ``q`` without being
                in either category above -- typically convex-hull
                over-approximation. These deserve a look before exclusion.
            ``other``
                Everything else: genuine self-collision candidates.
        """
        if self._collision_model is None:
            raise RuntimeError("This model was loaded without collision geometry.")
        previous = [
            (int(pair.first), int(pair.second)) for pair in self._collision_model.collisionPairs
        ]
        self.add_all_collision_pairs()
        report = self.collision_report(q)

        groups: Dict[str, List[Tuple[str, str]]] = {
            "same_body": [],
            "parent_child": [],
            "interfering_at_q": [],
            "other": [],
        }
        for k, names in enumerate(report.pair_names):
            ja, jb = report.joint_a[k], report.joint_b[k]
            if ja == jb:
                groups["same_body"].append(names)
            elif self._is_parent_child(ja, jb):
                groups["parent_child"].append(names)
            elif report.distances[k] <= touching_tolerance_m:
                groups["interfering_at_q"].append(names)
            else:
                groups["other"].append(names)

        # Restore whatever pair set the caller had registered before.
        self._collision_model.removeAllCollisionPairs()
        for first, second in previous:
            self._collision_model.addCollisionPair(self._pin.CollisionPair(first, second))
        self._collision_data = self._collision_model.createData()
        return groups

    def _is_parent_child(self, joint_a: int, joint_b: int) -> bool:
        """Whether two joints are directly connected in the kinematic tree."""
        parents = self._model.parents
        return int(parents[joint_a]) == joint_b or int(parents[joint_b]) == joint_a

    def collision_report(self, q: NDArray[np.float64]) -> CollisionReport:
        """Distances, normals and witness points for every registered pair at ``q``.

        Merely *loading* collision geometry avoids nothing.  This report is the
        input to the actual avoidance constraints in
        :class:`robopy.kinematics.dual_arm_ik.DualArmIK`.

        Raises:
            RuntimeError: If the model has no collision geometry.
        """
        pin = self._pin
        if self._collision_model is None or self._collision_data is None:
            raise RuntimeError("This model was loaded without collision geometry.")
        q = np.asarray(q, dtype=np.float64)
        self._check_q(q)
        pin.updateGeometryPlacements(
            self._model, self._data, self._collision_model, self._collision_data, q
        )
        pin.computeDistances(self._collision_model, self._collision_data)

        names: List[Tuple[str, str]] = []
        distances: List[float] = []
        normals: List[NDArray[np.float64]] = []
        witness_a: List[NDArray[np.float64]] = []
        witness_b: List[NDArray[np.float64]] = []
        joint_a: List[int] = []
        joint_b: List[int] = []
        geometries = self._collision_model.geometryObjects
        for k, pair in enumerate(self._collision_model.collisionPairs):
            result = self._collision_data.distanceResults[k]
            p1 = np.asarray(result.getNearestPoint1(), dtype=np.float64)
            p2 = np.asarray(result.getNearestPoint2(), dtype=np.float64)
            delta = p2 - p1
            norm = float(np.linalg.norm(delta))
            normal = delta / norm if norm > 1e-12 else np.zeros(3)
            names.append((str(geometries[pair.first].name), str(geometries[pair.second].name)))
            distances.append(float(result.min_distance))
            normals.append(normal)
            witness_a.append(p1)
            witness_b.append(p2)
            joint_a.append(int(geometries[pair.first].parentJoint))
            joint_b.append(int(geometries[pair.second].parentJoint))

        return CollisionReport(
            pair_names=tuple(names),
            distances=np.asarray(distances, dtype=np.float64),
            normals=np.asarray(normals, dtype=np.float64).reshape(-1, 3),
            witness_a=np.asarray(witness_a, dtype=np.float64).reshape(-1, 3),
            witness_b=np.asarray(witness_b, dtype=np.float64).reshape(-1, 3),
            joint_a=tuple(joint_a),
            joint_b=tuple(joint_b),
        )

    def distance_jacobian_row(
        self,
        q: NDArray[np.float64],
        point_a: NDArray[np.float64],
        joint_a: int,
        point_b: NDArray[np.float64],
        joint_b: int,
        normal: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        """Gradient of a pair distance with respect to the tangent velocity.

        For witness points ``p_a`` on body A and ``p_b`` on body B with unit
        normal ``n`` pointing from A to B, the rate of change of the distance is
        ``n^T (v_b - v_a)``, where each witness-point velocity is obtained from
        its parent joint's world-aligned Jacobian translated to that point.

        Returns:
            A ``(nv,)`` row such that ``row @ v`` is the distance rate.
        """
        pin = self._pin
        q = np.asarray(q, dtype=np.float64)
        self._check_q(q)
        pin.computeJointJacobians(self._model, self._data, q)

        def point_jacobian(joint_id: int, point: NDArray[np.float64]) -> NDArray[np.float64]:
            if joint_id == 0:
                return np.zeros((3, self.nv))
            J = np.asarray(
                pin.getJointJacobian(
                    self._model, self._data, joint_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                ),
                dtype=np.float64,
            )
            origin = np.asarray(self._data.oMi[joint_id].translation, dtype=np.float64)
            offset = np.asarray(point, dtype=np.float64) - origin
            # v_point = v_origin + omega x offset = J_lin - skew(offset) @ J_ang
            skew = np.array(
                [
                    [0.0, -offset[2], offset[1]],
                    [offset[2], 0.0, -offset[0]],
                    [-offset[1], offset[0], 0.0],
                ]
            )
            return J[:3, :] - skew @ J[3:, :]

        n = np.asarray(normal, dtype=np.float64)
        return n @ (point_jacobian(joint_b, point_b) - point_jacobian(joint_a, point_a))
