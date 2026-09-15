"""Everything the viewer page needs to know about a model, in JSON-friendly form.

The visual geometry is read from the URDF XML directly rather than through
Pinocchio's visual ``GeometryModel``: Pinocchio would build bounding-volume
hierarchies for every mesh at load (about 8 s for the 53 MB Rakuda export), and
the viewer only needs each shape's file or dimensions, colour and placement.
Placements are still evaluated by
:class:`~robopy.kinematics.urdf_model.WholeBodyModel`, so the page and the
controller agree on where every part is.
"""

from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Mapping, Sequence, Tuple

import numpy as np
from numpy.typing import NDArray

from robopy.kinematics.transforms import urdf_transform
from robopy.kinematics.urdf_audit import audit_urdf, resolve_package_path
from robopy.kinematics.urdf_model import WholeBodyModel
from robopy.models import is_lfs_pointer

__all__ = ["ModelBundle", "VisualGeometry", "matrix_to_pose"]

_DEFAULT_RGBA: Tuple[float, float, float, float] = (0.75, 0.75, 0.75, 1.0)


@dataclass(frozen=True)
class VisualGeometry:
    """One ``<visual>`` element of a link: a mesh or a primitive shape.

    Attributes:
        id: Stable identifier used by the page (``<link>#<index>``).
        link: Owning link name.  Its frame pose is what places the shape.
        shape: ``{"type": "mesh"}``, or a primitive such as
            ``{"type": "cylinder", "radius": r, "length": l}``,
            ``{"type": "box", "size": [x, y, z]}`` or
            ``{"type": "sphere", "radius": r}``.
        origin: ``(4, 4)`` transform from the link frame to the shape.
        scale: Per-axis scale (meshes only; primitives are already sized).
        rgba: Material colour, or the default grey when the URDF has none.
        mesh_path: Resolved path of the STL on disk (meshes only).
        mesh_url: URL path the server exposes the file under (meshes only).
    """

    id: str
    link: str
    shape: Dict[str, Any]
    origin: NDArray[np.float64]
    scale: Tuple[float, float, float]
    rgba: Tuple[float, float, float, float]
    mesh_path: Path | None = None
    mesh_url: str | None = None

    @property
    def is_mesh(self) -> bool:
        """Whether this visual is an STL rather than a primitive."""
        return self.shape.get("type") == "mesh"


def matrix_to_pose(T: NDArray[np.float64]) -> Dict[str, List[float]]:
    """``(4, 4)`` transform to ``{"p": [x, y, z], "q": [x, y, z, w]}``.

    The quaternion is ``xyzw``, the only order robopy uses.
    """
    R = T[:3, :3]
    trace = float(np.trace(R))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return {
        "p": [float(v) for v in T[:3, 3]],
        "q": [float(x), float(y), float(z), float(w)],
    }


def _floats(text: str | None, default: Sequence[float]) -> Tuple[float, ...]:
    if text is None:
        return tuple(default)
    parts = text.split()
    if len(parts) != len(default):
        return tuple(default)
    try:
        return tuple(float(v) for v in parts)
    except ValueError:
        return tuple(default)


def _parse_shape(
    element: ET.Element,
    search: Sequence[Path],
    mesh_count: int,
) -> Tuple[Dict[str, Any], Path | None, str | None, Tuple[float, float, float]] | None:
    """Interpret one ``<geometry>`` child; ``None`` means "not renderable"."""
    scale: Tuple[float, float, float] = (1.0, 1.0, 1.0)
    if element.tag == "mesh":
        uri = element.get("filename", "")
        resolved = resolve_package_path(uri, search) if uri else None
        if resolved is None or is_lfs_pointer(resolved):
            # A Git LFS pointer is not geometry; ModelBundle.load reports it.
            return None
        scale = _floats(element.get("scale"), scale)  # type: ignore[assignment]
        return {"type": "mesh"}, resolved, f"/mesh/{mesh_count}", scale
    if element.tag == "cylinder":
        shape = {
            "type": "cylinder",
            "radius": float(element.get("radius", "0")),
            "length": float(element.get("length", "0")),
        }
        return shape, None, None, scale
    if element.tag == "box":
        size = list(_floats(element.get("size"), (0.0, 0.0, 0.0)))
        return {"type": "box", "size": size}, None, None, scale
    if element.tag == "sphere":
        return {"type": "sphere", "radius": float(element.get("radius", "0"))}, None, None, scale
    return None


def _parse_visuals(
    urdf_path: Path,
    package_dirs: Sequence[Path],
    *,
    tag: str = "visual",
) -> List[VisualGeometry]:
    """Collect the renderable shapes under every ``<link>/<tag>`` element.

    ``tag`` is ``"visual"`` normally, or ``"collision"`` to draw the collision
    geometry instead -- the Rakuda export's convex URDF keeps the original
    (Git LFS) meshes under ``<visual>`` and puts the plain-git convex hulls
    under ``<collision>``, so a clone without LFS content can only be drawn
    from its collision elements.
    """
    root = ET.parse(urdf_path).getroot()
    materials: Dict[str, Tuple[float, float, float, float]] = {}
    for material_def in root.findall("material"):
        color = material_def.find("color")
        if color is not None and material_def.get("name"):
            materials[material_def.get("name", "")] = _floats(  # type: ignore[assignment]
                color.get("rgba"), _DEFAULT_RGBA
            )

    search = [
        *package_dirs,
        urdf_path.parent,
        urdf_path.parent.parent,
        urdf_path.parent.parent.parent,
    ]
    geometries: List[VisualGeometry] = []
    mesh_count = 0
    for link in root.findall("link"):
        link_name = link.get("name", "")
        for index, visual in enumerate(link.findall(tag)):
            geometry = visual.find("geometry")
            if geometry is None or len(geometry) == 0:
                continue
            parsed = _parse_shape(geometry[0], search, mesh_count)
            if parsed is None:
                continue
            shape, mesh_path, mesh_url, scale = parsed
            if mesh_path is not None:
                mesh_count += 1

            origin = visual.find("origin")
            xyz = _floats(origin.get("xyz") if origin is not None else None, (0.0, 0.0, 0.0))
            rpy = _floats(origin.get("rpy") if origin is not None else None, (0.0, 0.0, 0.0))

            rgba = _DEFAULT_RGBA
            material = visual.find("material")
            if material is not None:
                inline_color = material.find("color")
                if inline_color is not None:
                    rgba = _floats(inline_color.get("rgba"), rgba)  # type: ignore[assignment]
                elif material.get("name") in materials:
                    rgba = materials[material.get("name", "")]

            geometries.append(
                VisualGeometry(
                    id=f"{link_name}#{index}",
                    link=link_name,
                    shape=shape,
                    origin=urdf_transform(xyz, rpy),  # type: ignore[arg-type]
                    scale=scale,
                    rgba=rgba,
                    mesh_path=mesh_path,
                    mesh_url=mesh_url,
                )
            )
    return geometries


def _unavailable_meshes(
    urdf_path: Path,
    package_dirs: Sequence[Path],
    *,
    tag: str = "visual",
) -> Tuple[List[Path], List[str]]:
    """Meshes referenced under ``<link>/<tag>`` that cannot be drawn.

    Returns ``(pointers, absent)``: files that resolve to Git LFS pointers, and
    URIs that do not resolve to any file at all.
    """
    root = ET.parse(urdf_path).getroot()
    search = [
        *package_dirs,
        urdf_path.parent,
        urdf_path.parent.parent,
        urdf_path.parent.parent.parent,
    ]
    pointers: List[Path] = []
    absent: List[str] = []
    for element in root.iter(tag):
        for mesh in element.iter("mesh"):
            uri = mesh.get("filename")
            if not uri:
                continue
            resolved = resolve_package_path(uri, search)
            if resolved is None:
                if uri not in absent:
                    absent.append(uri)
            elif is_lfs_pointer(resolved) and resolved not in pointers:
                pointers.append(resolved)
    return pointers, absent


def _lfs_pointer_meshes(
    urdf_path: Path,
    package_dirs: Sequence[Path],
    *,
    tag: str = "visual",
) -> List[Path]:
    """Meshes referenced under ``<link>/<tag>`` that resolve to Git LFS pointers."""
    return _unavailable_meshes(urdf_path, package_dirs, tag=tag)[0]


@dataclass
class ModelBundle:
    """A loaded model plus the derived data the page consumes.

    Attributes:
        model: The kinematic model.
        urdf_path: Where it was loaded from.
        geometries: Visual shapes, in a stable order the page indexes by.
        joint_order: Movable joints in the order the page lists them.
        soft_limits: Soft limits applied to the model, for display.
        tcp_frames: ``{"left": frame, "right": frame}`` when TCPs are defined.
        warnings: Audit warnings, shown in the page's Info tab.
        geometry_source: Which URDF elements the shapes came from, ``"visual"``
            or ``"collision"``.
    """

    model: WholeBodyModel
    urdf_path: Path
    geometries: List[VisualGeometry]
    joint_order: Tuple[str, ...]
    soft_limits: Dict[str, Tuple[float, float]] = field(default_factory=dict)
    tcp_frames: Dict[str, str] = field(default_factory=dict)
    warnings: List[str] = field(default_factory=list)
    geometry_source: str = "visual"

    @classmethod
    def load(
        cls,
        urdf_path: Path | str,
        *,
        package_dirs: Sequence[Path | str] = (),
        soft_limits: Mapping[str, Tuple[float, float]] | None = None,
        tcp_offsets: Mapping[str, Tuple[str, NDArray[np.float64]]] | None = None,
        default_continuous_limit_rad: float = math.pi,
        geometry_source: str = "auto",
    ) -> "ModelBundle":
        """Load a URDF and prepare it for the viewer.

        Args:
            geometry_source: ``"visual"`` draws ``<visual>`` elements,
                ``"collision"`` draws ``<collision>`` elements, and ``"auto"``
                draws the visual geometry when its meshes are present and falls
                back to the collision geometry when they are Git LFS pointers
                (or when the file has no visual meshes at all).
            urdf_path: The URDF file.
            package_dirs: Directories that resolve ``package://`` URIs.
            soft_limits: ``{joint: (lower, upper)}`` measured limits.
            tcp_offsets: ``{"left"|"right": (parent_frame, (4,4) offset)}``.
                When omitted, TCPs are attached with a zero offset to the
                ``gripper_left_dof`` / ``gripper_right_dof`` frames if those
                exist -- clearly a placeholder, and reported as one.
            default_continuous_limit_rad: Slider range used for a continuous
                joint that has no soft limit.  This is a *display* range for
                the page's sliders, not a claim about the machine; the
                solver's own soft-limit requirement is unaffected.
        """
        path = Path(urdf_path)
        dirs = [Path(d) for d in package_dirs]
        audit = audit_urdf(path, package_dirs=dirs)
        model = WholeBodyModel.from_urdf(path, package_dirs=dirs, geometry_only=True)

        applied: Dict[str, Tuple[float, float]] = dict(soft_limits or {})
        if applied:
            model.set_soft_limits(applied)

        tcp_frames: Dict[str, str] = {}
        warnings = list(audit.warnings)
        offsets = dict(tcp_offsets or {})
        if not offsets:
            for side in ("left", "right"):
                parent = f"gripper_{side}_dof"
                if model.has_frame(parent):
                    offsets[side] = (parent, np.eye(4))
            if offsets:
                warnings.append(
                    "TCP frames are placed at gripper_left_dof / gripper_right_dof with a ZERO "
                    "offset. The real grasp-centre offset is a measurement; until it is made, "
                    "the end-effector pose shown here is the gripper frame, not the TCP."
                )
        for side, (parent, offset) in offsets.items():
            name = f"{side}_tcp"
            if not model.has_frame(name):
                model.add_fixed_frame(name, parent, offset)
            tcp_frames[side] = name

        # Sliders need a finite range even where the machine's real limit is
        # unknown; say so instead of pretending the range is measured.
        for name in model.movable_joint_names:
            if model.is_continuous(name) and name not in applied:
                warnings.append(
                    f"{name} is continuous with no soft limit; the slider shows "
                    f"+/-{default_continuous_limit_rad:.2f} rad as a display range only."
                )

        if geometry_source not in ("auto", "visual", "collision"):
            raise ValueError("geometry_source must be 'auto', 'visual' or 'collision'.")
        source = geometry_source
        pointers, absent = _unavailable_meshes(path, dirs, tag="visual")
        if source == "auto":
            visual = _parse_visuals(path, dirs, tag="visual")
            if visual:
                source, geometries = "visual", visual
            else:
                source, geometries = "collision", _parse_visuals(path, dirs, tag="collision")
                if pointers:
                    warnings.append(
                        f"{len(pointers)} visual mesh(es) are Git LFS pointers, so the collision "
                        "geometry (convex hulls) is drawn instead. Run `git lfs install && git "
                        "lfs pull` for the full meshes."
                    )
                if absent:
                    warnings.append(
                        f"{len(absent)} visual mesh file(s) are missing (e.g. {absent[0]}), so "
                        "the collision geometry (convex hulls) is drawn instead. Place the "
                        "assembly_2/meshes/*.stl files from Rakuda-2_simulation_ready.zip under "
                        "models/rakuda/assembly_2/meshes/ (see models/rakuda/README.md)."
                    )
        else:
            geometries = _parse_visuals(path, dirs, tag=source)
            if source == "visual" and pointers:
                warnings.append(
                    f"{len(pointers)} visual mesh(es) are Git LFS pointers, not geometry, and are "
                    "not drawn. Run `git lfs install && git lfs pull`, or draw the collision "
                    "geometry (geometry_source='collision')."
                )
            if source == "visual" and absent:
                warnings.append(
                    f"{len(absent)} visual mesh file(s) are missing (e.g. {absent[0]}) and are "
                    "not drawn. Copy them from Rakuda-2_simulation_ready.zip, or draw the "
                    "collision geometry (geometry_source='collision')."
                )
        if not geometries:
            warnings.append(
                f"No renderable {source} geometry was found in {path.name}; the page will show "
                "frames only."
            )

        return cls(
            model=model,
            urdf_path=path,
            geometries=geometries,
            joint_order=tuple(model.movable_joint_names),
            soft_limits=applied,
            tcp_frames=tcp_frames,
            warnings=warnings,
            geometry_source=source,
        )

    @property
    def meshes(self) -> List[VisualGeometry]:
        """Only the STL visuals, in the order ``/mesh/<index>`` refers to."""
        return [g for g in self.geometries if g.is_mesh]

    @property
    def static_links(self) -> set[str]:
        """Links rigidly attached to the world: no joint moves them.

        In Pinocchio those frames hang off the universe joint (index 0).  This
        is the robot's base -- in the Rakuda export the frame rail and the
        rubber feet -- and it is what the page stands the ground plane on,
        since the model origin is not on the floor (it sits about 26 cm above
        it in that export).
        """
        frames = self.model.model.frames
        return {str(f.name) for f in frames if int(f.parentJoint) == 0}

    # -- JSON views ---------------------------------------------------------

    def describe(self, *, default_continuous_limit_rad: float = math.pi) -> Dict[str, Any]:
        """The static description the page fetches once."""
        static = self.static_links
        lower, upper = self.model.position_limits(self.joint_order)
        joints = []
        for i, name in enumerate(self.joint_order):
            lo, hi = float(lower[i]), float(upper[i])
            display_only = False
            if not (math.isfinite(lo) and math.isfinite(hi)):
                lo, hi = -default_continuous_limit_rad, default_continuous_limit_rad
                display_only = True
            joints.append(
                {
                    "name": name,
                    "lower": lo,
                    "upper": hi,
                    "continuous": self.model.is_continuous(name),
                    "limit_is_display_only": display_only,
                    "group": _group_of(name),
                }
            )
        return {
            "urdf": str(self.urdf_path),
            "robot": self.urdf_path.stem,
            "nq": self.model.nq,
            "nv": self.model.nv,
            "joints": joints,
            "geometries": [
                {
                    "id": g.id,
                    "link": g.link,
                    "static": g.link in static,
                    "shape": g.shape,
                    "url": g.mesh_url,
                    "scale": list(g.scale),
                    "rgba": list(g.rgba),
                }
                for g in self.geometries
            ],
            "tcp_frames": dict(self.tcp_frames),
            "warnings": list(self.warnings),
            "geometry_source": self.geometry_source,
        }

    def positions_to_q(self, positions: Mapping[str, float]) -> NDArray[np.float64]:
        """Configuration from ``{joint: rad}``, with unknown names rejected."""
        unknown = sorted(set(positions) - set(self.joint_order))
        if unknown:
            raise KeyError(f"Unknown joint(s): {unknown}")
        full = {name: 0.0 for name in self.joint_order}
        full.update({k: float(v) for k, v in positions.items()})
        return self.model.q_from_positions(full)

    def poses(self, positions: Mapping[str, float]) -> Dict[str, Any]:
        """Geometry and TCP poses for one joint configuration."""
        q = self.positions_to_q(positions)
        link_names = sorted({g.link for g in self.geometries})
        wanted = link_names + list(self.tcp_frames.values())
        frames = self.model.frame_poses(q, wanted)
        geometry_poses = [matrix_to_pose(frames[g.link] @ g.origin) for g in self.geometries]
        tcp = {side: matrix_to_pose(frames[frame]) for side, frame in self.tcp_frames.items()}
        return {
            "joints": {name: float(v) for name, v in self.model.positions_from_q(q).items()},
            "geometries": geometry_poses,
            "tcp": tcp,
        }


def _group_of(joint: str) -> str:
    if "left" in joint:
        return "left"
    if "right" in joint:
        return "right"
    if "head" in joint:
        return "head"
    return "torso"
