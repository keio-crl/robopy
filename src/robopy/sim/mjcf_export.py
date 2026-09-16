"""Turn the Rakuda CAD export into an MJCF model MuJoCo-based simulators can load.

``models/rakuda/assembly_2/urdf/assembly_2_convex_collision.urdf`` is an Onshape
export.  It describes the machine's *geometry* faithfully and its *dynamics* not
at all, and several things in it stop MuJoCo from reading it directly.

``package://`` URIs
    MuJoCo has no notion of a ROS package, so the filenames are rewritten
    relative to a ``meshdir``.

Colliding mesh basenames
    A part's visual mesh and its convex hull share a filename
    (``meshes/X.stl`` and ``collision_meshes/X.stl``).  MuJoCo names a mesh
    asset after the file's basename, so loading the URDF as-is silently
    collapses each pair into one asset and *every* geom ends up using whichever
    was parsed first -- in practice the full visual mesh, which throws away the
    convex hulls the export went to the trouble of computing.  Each mesh is
    staged under a distinct name before MuJoCo sees it.

An over-large visual mesh
    ``Intel_RealSense_Depth_Camera_D435_1______1_1.stl`` has 203 886 triangles
    and MuJoCo's STL reader refuses anything above 200 000.  That one part falls
    back to its convex hull; see
    :attr:`MjcfExportReport.hulls_substituted_for_visual`.

No ``fusestatic``
    MuJoCo can fold each run of fixed links into the moving body above it,
    taking this model from 153 bodies to 16, and it is tempting to do so.  It is
    also wrong here: ``MjSpec.to_xml`` cannot serialise a fused model.  It
    relocates the fused *geoms* but writes each body's original ``pos`` and its
    original ``<inertial>``, so saving and reloading a fused Rakuda moves parts
    of it by up to 27 cm and loses nine tenths of its mass.  Compiling without
    fusion round-trips to within float32 rounding (2e-7 m), so that is what this
    converter does, and the fixed links stay as bodies.

    Nothing is lost by it.  MuJoCo never generates contacts between geoms whose
    bodies are welded together (they share a ``body_weldid``), which is exactly
    the set of pairs fusion would have removed, and the extra bodies carry no
    degrees of freedom.

Mass is not mass
    Every ``<mass>`` in the export is the part's **volume in cubic metres**, not
    its mass: the exporter ran with a density of 1.  This is measurable, not a
    guess -- comparing each ``<mass>`` against the enclosed volume of its own
    collision mesh gives a ratio that bottoms out at 1.008 for solid parts (the
    base plate, the servo bodies) and rises above 1 only for hollow parts, which
    is exactly what taking a convex hull does to them.  Summed over the machine
    it comes to 2.069 litres of material, and the inertia tensors follow the
    same convention.

    A mass is therefore recovered by multiplying both masses and inertias by a
    density.  :attr:`RakudaMjcfOptions.density_kg_m3` is that density.  Its
    default is aluminium, and it is a **modelling assumption, not a measurement
    of this machine**: one number applied to every part gets the relative
    distribution right (that much is the CAD's) and the absolute scale only
    approximately.  Weigh the real links if the absolute numbers matter.

What the exporter adds
    URDF carries no actuators, so a position servo is added per movable joint.
    Its force limit is the published DYNAMIXEL stall torque already recorded in
    :mod:`robopy.motor.dynamixel_control_table`, and which servo drives which
    joint is read off the CAD part names -- the Onshape parts are named after
    the servo bolted to them.  Named sites mark the two gripper frames and the
    head camera, so that nothing downstream has to know a CAD part number to
    find a hand.  The moving bodies and the pedestal are renamed after the joint
    each one carries, so that the fifteen names that matter read ``base``,
    ``elbow_pitch_left`` and so on rather than ``hn12_n101_1__``; the fixed
    links keep their CAD names.

The three continuous joints (``torso_yaw_dof`` and both
``shoulder_pitch_*_dof``) have no URDF range at all and MuJoCo needs a finite
one.  They get :data:`~robopy.config.robot_config.RAKUDA_MOTOR_TRAVEL_RAD`, the
travel the servos are driven over -- the same convention the viewer uses, and
the same caveat: it is what the motors allow, *not* a measurement of where this
machine actually stops.  A cable or a cover may end the travel much sooner.
"""

from __future__ import annotations

import math
import os
import struct
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import AbstractSet, Any, Dict, List, Sequence, Tuple

import numpy as np

from robopy.config.robot_config.rakuda_config import RAKUDA_MOTOR_TRAVEL_RAD
from robopy.kinematics.transforms import urdf_transform
from robopy.models import RAKUDA_PACKAGE_NAME, find_models_dir, find_rakuda_model

__all__ = [
    "MJCF_MODEL_NAME",
    "MjcfExportError",
    "MjcfExportReport",
    "RAKUDA_ACTUATED_JOINTS",
    "GRIPPER_NAME",
    "RAKUDA_BASE_BODY",
    "RAKUDA_FRAME_SITES",
    "RakudaMjcfOptions",
    "export_rakuda_mjcf",
]

#: ``<mujoco model="...">`` of the generated file, and therefore the prefix
#: dm_control gives every element when MetaSim attaches it to a scene.
MJCF_MODEL_NAME = "rakuda"

#: Name given to the pedestal body that everything else hangs off.
RAKUDA_BASE_BODY = "base"

#: The one borrowed gripper this exporter knows how to fit.
GRIPPER_NAME = "panda_longer_finger"

#: MuJoCo's STL reader rejects a mesh with more triangles than this.
_MAX_STL_FACES = 200_000

#: Movable joints of the Rakuda export, in the order
#: ``examples/config/rakuda_control.example.yaml`` lists them (torso, head, then
#: each arm shoulder-to-wrist).  This is the order of the simulated robot's
#: action vector.
RAKUDA_ACTUATED_JOINTS: Tuple[str, ...] = (
    "torso_yaw_dof",
    "head_yaw_dof",
    "head_pitch_dof",
    "shoulder_pitch_right_dof",
    "shoulder_roll_right_dof",
    "elbow_yaw_right_dof",
    "elbow_pitch_right_dof",
    "wrist_yaw_right_dof",
    "wrist_pitch_right_dof",
    "shoulder_pitch_left_dof",
    "shoulder_roll_left_dof",
    "elbow_yaw_left_dof",
    "elbow_pitch_left_dof",
    "wrist_yaw_left_dof",
    "wrist_pitch_left_dof",
)

#: Fixed URDF frames kept as MuJoCo sites: ``site name -> URDF link``.
#: ``gripper_*_dof`` are the *fixed* frames at the end of each arm -- the export
#: names them ``_dof`` but they are not degrees of freedom.  A grasp point is a
#: measured offset from these, not these themselves (see the
#: ``left_tcp``/``right_tcp`` block in
#: ``examples/config/rakuda_control.example.yaml``).
RAKUDA_FRAME_SITES: Dict[str, str] = {
    "gripper_left": "gripper_left_dof",
    "gripper_right": "gripper_right_dof",
    "head_camera": "head_camera_link",
}

#: Substring of a CAD link name -> the DYNAMIXEL bolted to that part.
_SERVO_BY_LINK_TOKEN: Tuple[Tuple[str, str], ...] = (
    ("xm540_w270", "xm540-w270"),
    ("xm430_w350", "xm430-w350"),
    ("xc330_t288", "xc330-t288"),
)

#: Joints whose neighbouring links name no servo.  ``torso_yaw_dof`` sits
#: between the waist base and a hinge horn, so the CAD cannot say which servo
#: turns it; :class:`~robopy.robots.rakuda.rakuda_follower.RakudaFollower` can,
#: and its motor carries exactly this joint's name.
_SERVO_BY_JOINT: Dict[str, str] = {
    "torso_yaw_dof": "xm540-w270",
}


class MjcfExportError(RuntimeError):
    """Raised when the CAD export cannot be turned into a usable MJCF."""


def _child(element: ET.Element, tag: str, context: str) -> ET.Element:
    """``element.find(tag)``, but a missing child is an error with a name on it."""
    found = element.find(tag)
    if found is None:
        raise MjcfExportError(f"the URDF is missing <{tag}> on {context}")
    return found


def _attr(element: ET.Element, name: str, context: str) -> str:
    """``element.get(name)``, but a missing attribute is an error with a name on it."""
    value = element.get(name)
    if value is None:
        raise MjcfExportError(f"the URDF is missing the {name!r} attribute on {context}")
    return value


def _link_of(joint: ET.Element, end: str) -> str:
    """The ``parent`` or ``child`` link name of a joint."""
    where = f"joint {joint.get('name', '?')!r}"
    return _attr(_child(joint, end, where), "link", f"<{end}> of {where}")


def _three(text: str) -> Tuple[float, float, float]:
    values = tuple(float(v) for v in text.split())
    if len(values) != 3:
        raise MjcfExportError(f"expected three numbers, got {text!r}")
    return (values[0], values[1], values[2])


@dataclass(frozen=True)
class RakudaMjcfOptions:
    """Knobs for :func:`export_rakuda_mjcf`.

    Attributes:
        density_kg_m3: Density the export's per-part volumes are multiplied by
            to get masses.  The default is aluminium.  **A modelling assumption,
            not a measurement** -- see the module docstring.
        continuous_joint_range_rad: Range given to the three joints the export
            leaves continuous.  Defaults to the motor travel, which is not a
            measurement of where the machine stops.
        saturation_error_rad: Position error at which a servo is asked for its
            full stall torque; this is what sets ``kp`` (``kp = stall_torque /
            saturation_error``).  A tuning choice, not a datasheet number.
        armature_kg_m2: Reflected rotor inertia added to every actuated joint.
            A geared servo's rotor dominates the joint's apparent inertia and
            leaving this at zero makes a stiff position servo ring.  A
            numerical-stability figure, not a measurement.
        joint_damping: Viscous damping on every actuated joint, for the same
            reason and with the same caveat.
        embed_meshes: Write geometry into the XML as inline vertex/face arrays
            instead of referencing STL files, giving one self-contained file
            that needs no ``models/`` directory beside it.  Implies
            ``visual_source="hull"``.
        visual_source: ``"mesh"`` draws the full visual STLs (53 MB, Git LFS);
            ``"hull"`` draws the convex hulls, which every checkout has.
        gripper: ``"panda_longer_finger"`` bolts a borrowed hand onto each
            gripper frame (see :mod:`robopy.sim.panda_gripper`), turning the
            15-joint robot into a 17-actuator one that can close on things.
            ``None``, the default, leaves the frames bare, which is what the CAD
            export actually describes.
    """

    density_kg_m3: float = 2700.0
    continuous_joint_range_rad: Tuple[float, float] = RAKUDA_MOTOR_TRAVEL_RAD
    saturation_error_rad: float = 0.2
    armature_kg_m2: float = 0.01
    joint_damping: float = 0.5
    embed_meshes: bool = False
    visual_source: str = "mesh"
    gripper: str | None = None


@dataclass
class MjcfExportReport:
    """What the export produced, so a caller can print it or assert on it."""

    output_path: Path
    urdf_path: Path
    density_kg_m3: float
    total_mass_kg: float = 0.0
    fixed_base_mass_kg: float = 0.0
    num_bodies: int = 0
    num_joints: int = 0
    num_geoms: int = 0
    num_meshes: int = 0
    actuators: List[str] = field(default_factory=list)
    sites: List[str] = field(default_factory=list)
    body_names: List[str] = field(default_factory=list)
    hulls_substituted_for_visual: List[str] = field(default_factory=list)
    continuous_joints_ranged: List[str] = field(default_factory=list)
    assembly_contacts_excluded: List[str] = field(default_factory=list)
    gripper: str | None = None
    gripper_actuators: List[str] = field(default_factory=list)
    servo_by_joint: Dict[str, str] = field(default_factory=dict)

    def summary(self) -> str:
        """A short human-readable report."""
        lines = [
            f"wrote {self.output_path}",
            f"  from {self.urdf_path.name}",
            f"  {self.num_bodies} bodies, {self.num_joints} joints, "
            f"{self.num_geoms} geoms, {self.num_meshes} meshes",
            f"  mass {self.total_mass_kg:.3f} kg at {self.density_kg_m3:g} kg/m^3 "
            f"({self.fixed_base_mass_kg:.3f} kg of it welded into the pedestal)",
            f"  {len(self.actuators)} position servos, {len(self.sites)} sites",
        ]
        if self.gripper:
            lines.append(
                f"  gripper {self.gripper!r}: {len(self.gripper_actuators)} more actuators "
                f"({', '.join(self.gripper_actuators)}) -- borrowed, not this machine's own"
            )
        if self.assembly_contacts_excluded:
            lines.append(
                "  contacts excluded as assembly interfaces: "
                + "; ".join(self.assembly_contacts_excluded)
            )
        if self.continuous_joints_ranged:
            low, high = RAKUDA_MOTOR_TRAVEL_RAD
            lines.append(
                f"  continuous joints given the motor travel ({math.degrees(low):.0f} to "
                f"{math.degrees(high):.0f} deg, NOT measured): "
                + ", ".join(self.continuous_joints_ranged)
            )
        if self.hulls_substituted_for_visual:
            lines.append(
                "  visual mesh over MuJoCo's triangle limit, convex hull drawn instead: "
                + ", ".join(self.hulls_substituted_for_visual)
            )
        return "\n".join(lines)


# --------------------------------------------------------------------------- #
# STL helpers
# --------------------------------------------------------------------------- #


def _stl_face_count(path: Path) -> int:
    """Triangle count of a binary STL, or 0 when the file is not one."""
    try:
        with path.open("rb") as handle:
            header = handle.read(84)
    except OSError:
        return 0
    if len(header) < 84:
        return 0
    return int(struct.unpack_from("<I", header, 80)[0])


def _read_obj(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(vertices, faces)`` of a Wavefront OBJ.

    Only ``v`` and ``f`` are read: the borrowed gripper meshes are plain
    triangle soup, and MuJoCo takes its materials from the MJCF rather than the
    OBJ's ``mtllib``. Faces are triangulated by fanning, which is exact for the
    quads these files contain.
    """
    vertices: List[Tuple[float, float, float]] = []
    faces: List[Tuple[int, int, int]] = []
    for line in path.read_text(errors="replace").splitlines():
        if line.startswith("v "):
            parts = line.split()
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
        elif line.startswith("f "):
            corners = [int(token.split("/")[0]) for token in line.split()[1:]]
            corners = [c - 1 if c > 0 else len(vertices) + c for c in corners]
            for i in range(1, len(corners) - 1):
                faces.append((corners[0], corners[i], corners[i + 1]))
    if not vertices or not faces:
        raise MjcfExportError(f"{path} has no usable geometry ({len(vertices)} verts)")
    return np.asarray(vertices, dtype=float), np.asarray(faces, dtype=int)


def _read_stl(path: Path) -> Tuple[np.ndarray, np.ndarray]:
    """Return ``(vertices, faces)`` of a binary STL, welding duplicate corners."""
    data = path.read_bytes()
    count = int(struct.unpack_from("<I", data, 80)[0])
    if len(data) != 84 + count * 50:
        raise MjcfExportError(
            f"{path} is not a binary STL, which is the only kind MuJoCo reads: expected "
            f"{84 + count * 50} bytes for {count} triangles, found {len(data)}"
        )
    records = np.frombuffer(data[84:], dtype=np.uint8).reshape(count, 50)
    corners = records[:, 12:48].copy().view(np.float32).reshape(count * 3, 3).astype(np.float64)
    vertices, inverse = np.unique(corners, axis=0, return_inverse=True)
    return vertices, np.asarray(inverse).reshape(count, 3)


# --------------------------------------------------------------------------- #
# URDF preparation
# --------------------------------------------------------------------------- #


def _mesh_relpath(filename: str) -> str:
    """``package://assembly_2/meshes/X.stl`` -> ``meshes/X.stl``."""
    prefix = f"package://{RAKUDA_PACKAGE_NAME}/"
    if not filename.startswith(prefix):
        raise MjcfExportError(
            f"mesh filename {filename!r} does not start with {prefix!r}; this converter only "
            "understands the Rakuda CAD export's own layout"
        )
    return filename[len(prefix) :]


def _stage_meshes(
    root: ET.Element,
    package_dir: Path,
    stage_dir: Path,
    options: RakudaMjcfOptions,
    substituted: List[str],
) -> Dict[str, Path]:
    """Point every ``<mesh>`` at a uniquely named symlink inside ``stage_dir``.

    MuJoCo names a mesh asset after the file's basename, so a visual mesh and
    its convex hull -- which share one -- have to be given different filenames
    before MuJoCo parses them, or they alias onto a single asset.  Nothing is
    copied; the staged files are symlinks.

    Returns:
        MuJoCo asset name -> the real STL it came from.
    """
    assembly = package_dir / RAKUDA_PACKAGE_NAME
    assets: Dict[str, Path] = {}
    for mesh in root.iter("mesh"):
        relative = _mesh_relpath(mesh.get("filename", ""))
        source = assembly / relative
        if not source.is_file():
            raise MjcfExportError(f"mesh {source}, referenced by the URDF, does not exist")

        is_visual = relative.startswith("meshes/")
        if is_visual and (
            options.visual_source == "hull" or _stl_face_count(source) > _MAX_STL_FACES
        ):
            hull_relative = "collision_meshes/" + relative[len("meshes/") :]
            hull = assembly / hull_relative
            if not hull.is_file():
                raise MjcfExportError(
                    f"{source.name} cannot be drawn (over {_MAX_STL_FACES} triangles, or hulls "
                    "were asked for) and has no convex hull to fall back on"
                )
            if options.visual_source != "hull":
                substituted.append(source.name)
            relative, source = hull_relative, hull

        asset_name = ("vis_" if is_visual else "col_") + Path(relative).stem
        staged = stage_dir / (asset_name + ".stl")
        if not staged.exists():
            staged.symlink_to(source.resolve())
        assets[asset_name] = source.resolve()
        mesh.set("filename", staged.name)
    return assets


def _scale_inertials(root: ET.Element, density: float) -> None:
    """Turn the export's per-part volumes into masses and inertias."""
    for inertial in root.iter("inertial"):
        mass = inertial.find("mass")
        if mass is not None:
            mass.set("value", repr(float(mass.get("value", "0")) * density))
        inertia = inertial.find("inertia")
        if inertia is not None:
            for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
                inertia.set(key, repr(float(inertia.get(key, "0")) * density))


def _bound_continuous_joints(root: ET.Element, span: Tuple[float, float]) -> List[str]:
    """Give the export's continuous joints a finite range; return their names."""
    lower, upper = span
    bounded: List[str] = []
    for joint in root.findall("joint"):
        if joint.get("type") != "continuous":
            continue
        joint.set("type", "revolute")
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        limit.set("lower", repr(float(lower)))
        limit.set("upper", repr(float(upper)))
        for placeholder in ("effort", "velocity"):
            if limit.get(placeholder) is None:
                limit.set(placeholder, "1")
        bounded.append(joint.get("name", "?"))
    return bounded


def _rename_links(root: ET.Element) -> Dict[str, str]:
    """Name the bodies a user will actually reach for.

    A moving body is the child link of exactly one actuated joint, so the joint
    names it: ``elbow_pitch_left_dof`` -> ``elbow_pitch_left``.  The root link
    becomes ``base``.  Every other link keeps its CAD name, which is fine --
    they are welded brackets and covers, and nothing addresses them.

    Returns:
        Old link name -> new link name, for the links that were renamed.
    """
    mapping: Dict[str, str] = {}
    for joint in root.findall("joint"):
        name = joint.get("name", "")
        if name in RAKUDA_ACTUATED_JOINTS:
            mapping[_link_of(joint, "child")] = name[: -len("_dof")]

    children = {_link_of(joint, "child") for joint in root.findall("joint")}
    link_names = [_attr(link, "name", "a <link>") for link in root.findall("link")]
    for name in link_names:
        if name not in children:
            mapping[name] = RAKUDA_BASE_BODY

    collisions = (set(link_names) - set(mapping)) & set(mapping.values())
    if collisions:
        raise MjcfExportError(f"renaming bodies would collide with existing links: {collisions}")

    for link in root.findall("link"):
        new = mapping.get(link.get("name", ""))
        if new:
            link.set("name", new)
    for joint in root.findall("joint"):
        for end in ("parent", "child"):
            element = _child(joint, end, f"joint {joint.get('name', '?')!r}")
            new = mapping.get(element.get("link", ""))
            if new:
                element.set("link", new)
    return mapping


# --------------------------------------------------------------------------- #
# Frames
# --------------------------------------------------------------------------- #


def _origin_matrix(element: ET.Element) -> np.ndarray:
    origin = element.find("origin")
    if origin is None:
        return np.eye(4)
    return urdf_transform(
        xyz=_three(origin.get("xyz", "0 0 0")), rpy=_three(origin.get("rpy", "0 0 0"))
    )


def _frame_on_surviving_body(
    root: ET.Element, link: str, surviving: AbstractSet[str]
) -> Tuple[str, np.ndarray]:
    """Locate ``link`` relative to the nearest body MuJoCo actually keeps.

    Every link is a body in the model this converter writes, so the answer is
    usually ``link`` itself and an identity pose.  The climb is kept because it
    is what makes the site placement independent of that: if a link is ever
    fused away, the site still lands in the right place on whatever body
    absorbed it, rather than failing or silently moving.

    Args:
        root: The prepared URDF, after :func:`_rename_links`.
        link: The frame to locate.
        surviving: Links that are bodies in the compiled model.

    Returns:
        ``(surviving link name, 4x4 pose of *link* expressed in that body)``.
    """
    by_child = {_link_of(j, "child"): j for j in root.findall("joint")}
    transform = np.eye(4)
    current = link
    while current not in surviving and current in by_child:
        joint = by_child[current]
        if joint.get("type") != "fixed":
            raise MjcfExportError(
                f"frame {link!r} is separated from body {current!r} by the movable joint "
                f"{joint.get('name')!r}; a site cannot span a degree of freedom"
            )
        transform = _origin_matrix(joint) @ transform
        current = _link_of(joint, "parent")
    if current not in surviving:
        raise MjcfExportError(f"frame {link!r} does not hang off any surviving body")
    return current, transform


def _quaternion_from_matrix(matrix: np.ndarray) -> Tuple[float, float, float, float]:
    """``(w, x, y, z)`` of a rotation matrix, in MuJoCo's order."""
    rotation = np.asarray(matrix)[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        values = (
            0.25 * scale,
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        other = [(axis + 1) % 3, (axis + 2) % 3]
        diagonal = (
            1.0 + rotation[axis, axis] - rotation[other[0], other[0]] - rotation[other[1], other[1]]
        )
        scale = math.sqrt(diagonal) * 2.0
        components = [0.0, 0.0, 0.0]
        components[axis] = 0.25 * scale
        components[other[0]] = (rotation[other[0], axis] + rotation[axis, other[0]]) / scale
        components[other[1]] = (rotation[other[1], axis] + rotation[axis, other[1]]) / scale
        w = (rotation[other[1], other[0]] - rotation[other[0], other[1]]) / scale
        values = (w, components[0], components[1], components[2])
    norm = math.sqrt(sum(v * v for v in values))
    return (values[0] / norm, values[1] / norm, values[2] / norm, values[3] / norm)


# --------------------------------------------------------------------------- #
# Servos
# --------------------------------------------------------------------------- #


def _servo_for_joint(joint: ET.Element, original_links: Dict[str, str]) -> str:
    """Which DYNAMIXEL drives this joint, read off the CAD part names.

    The Onshape parts are named after the servo bolted to them
    (``xm540_w270_1__...``), so a joint's own parent and child links say which
    one turns it.  ``torso_yaw_dof`` is the one joint whose neighbours name no
    servo; :data:`_SERVO_BY_JOINT` carries it, taken from the follower bus,
    whose motor has exactly this joint's name.

    Args:
        joint: The URDF ``<joint>``, possibly with renamed endpoints.
        original_links: New link name -> the CAD name it replaced.
    """
    name = joint.get("name", "")
    if name in _SERVO_BY_JOINT:
        return _SERVO_BY_JOINT[name]
    neighbours = [
        original_links.get(link, link) for link in (_link_of(joint, e) for e in ("parent", "child"))
    ]
    for link in neighbours:
        lowered = link.lower()
        for token, model in _SERVO_BY_LINK_TOKEN:
            if token in lowered:
                return model
    raise MjcfExportError(
        f"cannot tell which servo drives {name!r}: neither {neighbours[0]!r} nor "
        f"{neighbours[1]!r} names one, and it has no entry in _SERVO_BY_JOINT"
    )


def _stall_torque_nm(model: str) -> float:
    from robopy.motor.dynamixel_control_table import get_motor_capabilities

    stall = get_motor_capabilities(model).stall_torque_nm
    if stall is None:
        raise MjcfExportError(f"no stall torque is recorded for {model!r}")
    return stall


# --------------------------------------------------------------------------- #
# MJCF post-processing
# --------------------------------------------------------------------------- #


def _indent(element: ET.Element, level: int = 0) -> None:
    pad = "\n" + "  " * level
    if len(element):
        if not (element.text or "").strip():
            element.text = pad + "  "
        for child in element:
            _indent(child, level + 1)
        if not (element[-1].tail or "").strip():
            element[-1].tail = pad
    if level and not (element.tail or "").strip():
        element.tail = pad


def _check_single_root(mjcf: ET.Element) -> ET.Element:
    """Return the one root body, or explain why there is not exactly one.

    Everything downstream -- placing the robot, welding it down, letting it
    float -- assumes a single root, and a URDF that quietly grew a second tree
    would otherwise only show up as half a robot in the scene.
    """
    worldbody = mjcf.find("worldbody")
    if worldbody is None:
        raise MjcfExportError("the compiled model has no <worldbody>")
    bodies = worldbody.findall("body")
    if len(bodies) != 1:
        raise MjcfExportError(
            f"expected exactly one root body, found {len(bodies)}: "
            f"{[b.get('name') for b in bodies]}"
        )
    base = bodies[0]
    if base.get("name") != RAKUDA_BASE_BODY:
        raise MjcfExportError(
            f"the root body is {base.get('name')!r}, not {RAKUDA_BASE_BODY!r}; the link "
            "renaming did not reach it"
        )
    return base


def _add_defaults(mjcf: ET.Element) -> None:
    """Split the visual and collision geoms into named classes.

    MuJoCo's URDF importer already separates them -- ``group=1 contype=0`` for
    the ``<visual>`` geoms, the defaults for the ``<collision>`` ones -- but it
    repeats the attributes on every geom.  Naming the two sets makes the file
    readable and lets a renderer show or hide either.
    """
    default = ET.Element("default")
    visual = ET.SubElement(default, "default", {"class": "rakuda_visual"})
    ET.SubElement(
        visual,
        "geom",
        {"type": "mesh", "contype": "0", "conaffinity": "0", "group": "1", "density": "0"},
    )
    collision = ET.SubElement(default, "default", {"class": "rakuda_collision"})
    ET.SubElement(
        collision,
        "geom",
        {"type": "mesh", "contype": "1", "conaffinity": "1", "group": "3", "density": "0"},
    )
    mjcf.insert(0, default)

    for geom in mjcf.iter("geom"):
        mesh = geom.get("mesh", "")
        if mesh.startswith("vis_"):
            geom.set("class", "rakuda_visual")
        elif mesh.startswith("col_"):
            geom.set("class", "rakuda_collision")
            geom.attrib.pop("rgba", None)
        else:
            continue
        for attribute in ("contype", "conaffinity", "group", "density", "type"):
            geom.attrib.pop(attribute, None)


def _pin_inertials(mjcf: ET.Element, compiled: Any, mujoco: Any) -> List[str]:
    """Write out each body's inertial explicitly, as MuJoCo compiled it.

    The URDF's root link declares no ``<inertial>``, so none is written for it
    and a reload would fall back to inferring mass from the geoms -- which this
    converter deliberately sets ``density="0"`` on, so that geometry can never
    silently become mass.  The pedestal would come back weightless: harmless
    while it stays welded to the world, and a hard error the moment anyone lets
    the robot float.

    Writing every body's compiled values makes the file say plainly what it
    carries, and makes the round trip verifiable rather than assumed.

    Returns:
        Names of the bodies whose inertial was written.
    """
    pinned: List[str] = []
    for name, element in _bodies_by_name(mjcf).items():
        body_id = mujoco.mj_name2id(compiled, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            raise MjcfExportError(f"body {name!r} is in the XML but not in the compiled model")
        mass = float(compiled.body_mass[body_id])
        if mass <= 0.0:
            # A pure frame: the CAD export has 16 of them, plus the root link.
            # Leave it alone rather than inventing an inertia for it.
            continue
        for stale in element.findall("inertial"):
            element.remove(stale)
        inertial = ET.Element(
            "inertial",
            {
                "pos": " ".join(f"{v:.9g}" for v in compiled.body_ipos[body_id]),
                "quat": " ".join(f"{v:.9g}" for v in compiled.body_iquat[body_id]),
                "mass": f"{mass:.9g}",
                "diaginertia": " ".join(f"{v:.9g}" for v in compiled.body_inertia[body_id]),
            },
        )
        element.insert(0, inertial)
        pinned.append(name)
    return pinned


def _exclude_assembly_contacts(mjcf: ET.Element, compiled: Any, mujoco: Any) -> List[str]:
    """Stop MuJoCo colliding parts that the CAD has *assembled* together.

    Three pairs of parts overlap slightly in the as-designed pose: the waist
    thrust bearing against its housing, and the torso horn against the waist
    base.  A bearing touching its race is what a bearing is for, but MuJoCo
    cannot tell that from a collision, so it would spend every step resolving a
    half-millimetre penetration and quietly resist the torso turning.

    The pairs are found rather than listed: whatever is already in contact at
    the assembly pose, with the joints at zero, is assembled.  Poses the robot
    can *reach* are left alone -- around 880 body pairs touch somewhere in the
    joint ranges, and those are genuine self-collisions worth keeping, which is
    why this does not simply turn self-collision off.

    Returns:
        ``"body_a <-> body_b"`` for each excluded pair.
    """
    data = mujoco.MjData(compiled)
    mujoco.mj_forward(compiled, data)

    pairs = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        first = int(compiled.geom_bodyid[contact.geom1])
        second = int(compiled.geom_bodyid[contact.geom2])
        pairs.add((min(first, second), max(first, second)))

    if not pairs:
        return []
    section = ET.SubElement(mjcf, "contact")
    described: List[str] = []
    for first, second in sorted(pairs):
        names = [
            mujoco.mj_id2name(compiled, mujoco.mjtObj.mjOBJ_BODY, body) for body in (first, second)
        ]
        ET.SubElement(section, "exclude", {"body1": names[0], "body2": names[1]})
        described.append(f"{names[0]} <-> {names[1]}")
    return described


def _bodies_by_name(mjcf: ET.Element) -> Dict[str, ET.Element]:
    found: Dict[str, ET.Element] = {}

    def walk(element: ET.Element) -> None:
        for child in element:
            if child.tag == "body":
                name = child.get("name")
                if name:
                    found[name] = child
                walk(child)

    worldbody = mjcf.find("worldbody")
    if worldbody is not None:
        walk(worldbody)
    return found


def _add_sites(mjcf: ET.Element, urdf_root: ET.Element, sites: List[str]) -> None:
    """Hang a site at every frame in :data:`RAKUDA_FRAME_SITES`."""
    bodies = _bodies_by_name(mjcf)
    for site_name, link in RAKUDA_FRAME_SITES.items():
        host_link, pose = _frame_on_surviving_body(urdf_root, link, bodies.keys())
        host = bodies.get(host_link)
        if host is None:
            raise MjcfExportError(
                f"site {site_name!r} belongs on body {host_link!r}, which the compiled model "
                f"does not have; it has {sorted(bodies)}"
            )
        ET.SubElement(
            host,
            "site",
            {
                "name": site_name,
                "pos": " ".join(f"{v:.9g}" for v in pose[:3, 3]),
                "quat": " ".join(f"{v:.9g}" for v in _quaternion_from_matrix(pose)),
                "size": "0.005",
                "group": "4",
            },
        )
        sites.append(site_name)


def _add_actuators(
    mjcf: ET.Element,
    urdf_root: ET.Element,
    original_links: Dict[str, str],
    options: RakudaMjcfOptions,
    report: MjcfExportReport,
) -> None:
    """Add one position servo per movable joint, with the real stall torques."""
    actuator = ET.SubElement(mjcf, "actuator")
    joints = {joint.get("name"): joint for joint in urdf_root.findall("joint")}
    for joint_name in RAKUDA_ACTUATED_JOINTS:
        joint = joints.get(joint_name)
        if joint is None:
            raise MjcfExportError(f"the URDF has no joint named {joint_name!r}")
        model = _servo_for_joint(joint, original_links)
        stall = _stall_torque_nm(model)
        attributes = {
            "name": joint_name,
            "joint": joint_name,
            "kp": f"{stall / options.saturation_error_rad:.6g}",
            "dampratio": "1",
            "forcerange": f"{-stall:.6g} {stall:.6g}",
        }
        limit = joint.find("limit")
        lower = None if limit is None else limit.get("lower")
        upper = None if limit is None else limit.get("upper")
        if lower is not None and upper is not None:
            attributes["ctrlrange"] = f"{float(lower):.9g} {float(upper):.9g}"
        ET.SubElement(actuator, "position", attributes)
        report.actuators.append(joint_name)
        report.servo_by_joint[joint_name] = model


def _apply_joint_dynamics(mjcf: ET.Element, options: RakudaMjcfOptions) -> None:
    for joint in mjcf.iter("joint"):
        if joint.get("name") in RAKUDA_ACTUATED_JOINTS:
            joint.set("armature", f"{options.armature_kg_m2:.6g}")
            joint.set("damping", f"{options.joint_damping:.6g}")


def _embed_meshes(mjcf: ET.Element, assets: Dict[str, Path]) -> None:
    """Replace every ``<mesh file=...>`` with inline vertex and face arrays.

    A model with no external references can ship inside a wheel, which is what
    makes ``pip install robopy`` enough to simulate the robot.
    """
    asset_root = mjcf.find("asset")
    if asset_root is None:
        raise MjcfExportError("the compiled model has no <asset>")
    for mesh in asset_root.findall("mesh"):
        name = mesh.get("name", "")
        source = assets.get(name)
        if source is None:
            raise MjcfExportError(f"no source STL was recorded for mesh asset {name!r}")
        vertices, faces = (
            _read_obj(source) if source.suffix.lower() == ".obj" else _read_stl(source)
        )
        mesh.attrib.pop("file", None)
        # An inlined mesh has no file, so it has no content type either. Dropping
        # the attribute is also what makes this export reproducible: whether
        # MuJoCo writes ``content_type`` depends on what else has been loaded in
        # the process, so leaving it in makes the same model hash differently
        # depending on whether the other model was exported first.
        mesh.attrib.pop("content_type", None)
        mesh.set("vertex", " ".join(f"{v:.6g}" for v in vertices.reshape(-1)))
        mesh.set("face", " ".join(str(int(v)) for v in faces.reshape(-1)))


def _retarget_mesh_files(mjcf: ET.Element, assets: Dict[str, Path], output_dir: Path) -> None:
    """Point the mesh assets back at the real STLs, relative to the output file."""
    asset_root = mjcf.find("asset")
    if asset_root is None:
        raise MjcfExportError("the compiled model has no <asset>")
    for mesh in asset_root.findall("mesh"):
        name = mesh.get("name", "")
        if name.startswith("panda_finger_"):
            continue  # the gripper wrote its own path, already relative to here
        source = assets.get(name)
        if source is None:
            raise MjcfExportError(f"no source STL was recorded for mesh asset {name!r}")
        mesh.set("file", os.path.relpath(source, output_dir.resolve()))


# --------------------------------------------------------------------------- #
# Entry point
# --------------------------------------------------------------------------- #


def export_rakuda_mjcf(
    output_path: Path | str,
    urdf_path: Path | str | None = None,
    package_dir: Path | str | None = None,
    options: RakudaMjcfOptions | None = None,
) -> MjcfExportReport:
    """Write an MJCF model of the Rakuda to ``output_path``.

    Args:
        output_path: Where the ``.xml`` goes.  Its parent directory is created
            if needed, and mesh references are made relative to it.
        urdf_path: Source URDF.  Defaults to the committed convex-collision
            export, which every checkout has.
        package_dir: Directory ``package://assembly_2/...`` resolves against
            (that is, ``models/rakuda``).  Defaults to the committed one.
        options: See :class:`RakudaMjcfOptions`.

    Returns:
        A :class:`MjcfExportReport` describing what was written.

    Raises:
        MjcfExportError: If the model cannot be found, a mesh is unreadable, or
            the export contains something this converter does not understand.
        ImportError: If ``mujoco`` is not installed.  Compiling the model is
            also what validates it, so there is no export without it.
    """
    try:
        import mujoco
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError(
            "exporting MJCF needs MuJoCo: pip install 'robopy[sim]' (or just 'mujoco')"
        ) from exc

    options = options or RakudaMjcfOptions()
    if options.embed_meshes and options.visual_source != "hull":
        options = replace(options, visual_source="hull")

    if urdf_path is None or package_dir is None:
        model = find_rakuda_model()
        if model is None:
            raise MjcfExportError(
                "the Rakuda model was not found; pass urdf_path and package_dir, or point "
                "ROBOPY_MODELS_DIR at the repository's models/ directory"
            )
        urdf_path = urdf_path or model.convex_collision_urdf
        package_dir = package_dir or model.package_dir

    urdf_path = Path(urdf_path).resolve()
    package_dir = Path(package_dir).resolve()
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Taken from parse().getroot(), which is always an element; ElementTree's own
    # getroot() is Optional and would need checking at every use below.
    root = ET.parse(urdf_path).getroot()
    prepared = ET.ElementTree(root)

    report = MjcfExportReport(
        output_path=output_path, urdf_path=urdf_path, density_kg_m3=options.density_kg_m3
    )
    report.continuous_joints_ranged = _bound_continuous_joints(
        root, options.continuous_joint_range_rad
    )
    _scale_inertials(root, options.density_kg_m3)
    renamed = _rename_links(root)
    original_links = {new: old for old, new in renamed.items()}

    with tempfile.TemporaryDirectory(prefix="rakuda-mjcf-") as staging:
        stage_dir = Path(staging)
        assets = _stage_meshes(
            root, package_dir, stage_dir, options, report.hulls_substituted_for_visual
        )

        wrapper = ET.Element("mujoco")
        ET.SubElement(
            wrapper,
            "compiler",
            {
                "meshdir": str(stage_dir),
                "balanceinertia": "true",
                "discardvisual": "false",
                "strippath": "false",
                "fusestatic": "false",
            },
        )
        root.insert(0, wrapper)

        staged_urdf = stage_dir / "rakuda_prepared.urdf"
        prepared.write(staged_urdf, encoding="utf-8", xml_declaration=True)
        try:
            spec = mujoco.MjSpec.from_file(str(staged_urdf))
            compiled = spec.compile()
            xml_text = spec.to_xml()
        except ValueError as exc:
            raise MjcfExportError(f"MuJoCo refused the prepared URDF: {exc}") from exc

        mjcf = ET.fromstring(xml_text)
        mjcf.set("model", MJCF_MODEL_NAME)
        compiler = mjcf.find("compiler")
        if compiler is not None:
            compiler.attrib.pop("meshdir", None)

    _check_single_root(mjcf)
    report.assembly_contacts_excluded = _exclude_assembly_contacts(mjcf, compiled, mujoco)
    _pin_inertials(mjcf, compiled, mujoco)
    _add_defaults(mjcf)
    _apply_joint_dynamics(mjcf, options)
    _add_sites(mjcf, root, report.sites)
    _add_actuators(mjcf, root, original_links, options, report)
    _attach_gripper(mjcf, options, output_path, report)

    # After the gripper, so its meshes are embedded or retargeted with the rest.
    if options.embed_meshes:
        _embed_meshes(mjcf, {**assets, **_GRIPPER_ASSET_SOURCES})
    else:
        _retarget_mesh_files(mjcf, assets, output_path.parent)

    _indent(mjcf)

    # No "--" anywhere in here: XML forbids it inside a comment, and while MuJoCo
    # lets it pass, every conforming parser (ElementTree included) rejects the file.
    header = (
        "\n  Rakuda-2. GENERATED FROM THE CAD EXPORT, DO NOT EDIT BY HAND.\n\n"
        f"  source:      {urdf_path.name}\n"
        "  regenerate:  python -m robopy.sim.mjcf_export\n\n"
        "  The CAD export carries no dynamics. Every mass here is a CAD part volume\n"
        f"  multiplied by {options.density_kg_m3:g} kg/m^3: the relative distribution is the\n"
        "  CAD's, the absolute scale is an assumption. Joint armature and damping are\n"
        "  numerical-stability figures, not measurements. Servo force limits are the\n"
        "  published DYNAMIXEL stall torques. The ranges on torso_yaw and both\n"
        "  shoulder_pitch joints are the motor travel, not a measurement of where\n"
        "  this machine actually stops.\n"
    )
    output_path.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n'
        f"<!--{header}-->\n" + ET.tostring(mjcf, encoding="unicode").rstrip() + "\n",
        encoding="utf-8",
    )

    verified = mujoco.MjModel.from_xml_path(str(output_path))
    report.num_bodies = int(verified.nbody)
    report.num_joints = int(verified.njnt)
    report.num_geoms = int(verified.ngeom)
    report.num_meshes = int(verified.nmesh)
    report.total_mass_kg = float(verified.body_mass.sum())
    # The pedestal is a tree of welded links, not one body, so its mass is the
    # weld's -- the base body itself is only a frame and weighs nothing.
    base_id = mujoco.mj_name2id(verified, mujoco.mjtObj.mjOBJ_BODY, RAKUDA_BASE_BODY)
    weld = verified.body_weldid[base_id]
    report.fixed_base_mass_kg = float(
        sum(verified.body_mass[i] for i in range(verified.nbody) if verified.body_weldid[i] == weld)
    )
    report.body_names = [
        mujoco.mj_id2name(verified, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(verified.nbody)
    ]
    return report


def _attach_gripper(
    mjcf: ET.Element,
    options: RakudaMjcfOptions,
    output_path: Path,
    report: MjcfExportReport,
) -> None:
    """Bolt on a borrowed hand, when one was asked for."""
    if not options.gripper:
        return
    if options.gripper != "panda_longer_finger":
        raise MjcfExportError(
            f"unknown gripper {options.gripper!r}; the only one vendored is 'panda_longer_finger'"
        )

    from robopy.sim.panda_gripper import attach_panda_fingers, find_gripper_meshes

    models = find_models_dir()
    if models is None:
        raise MjcfExportError(
            "the gripper meshes live under models/gripper_panda, and no models/ directory "
            "was found; set ROBOPY_MODELS_DIR"
        )
    meshes = find_gripper_meshes(models)
    if options.embed_meshes:
        files = {kind: str(path) for kind, path in meshes.items()}
    else:
        files = {
            kind: os.path.relpath(path, output_path.parent.resolve())
            for kind, path in meshes.items()
        }
    report.gripper = options.gripper
    report.gripper_actuators = attach_panda_fingers(mjcf, meshes, files)
    mjcf.set("model", f"{MJCF_MODEL_NAME}_gripper")
    # The embedder looks its sources up by asset name.
    _GRIPPER_ASSET_SOURCES.update({f"panda_finger_{kind}": path for kind, path in meshes.items()})


#: Filled in by :func:`_attach_gripper` so the embedding pass can find the
#: borrowed meshes, which do not come from the CAD export's own staging.
_GRIPPER_ASSET_SOURCES: Dict[str, Path] = {}


def _default_outputs() -> List[Tuple[Path, RakudaMjcfOptions]]:
    """The models the repository keeps checked in."""
    from robopy.roboverse.assets import PACKAGED_RAKUDA_GRIPPER_MJCF, PACKAGED_RAKUDA_MJCF

    outputs: List[Tuple[Path, RakudaMjcfOptions]] = []
    model = find_rakuda_model()
    if model is not None:
        mjcf_dir = model.package_dir / RAKUDA_PACKAGE_NAME / "mjcf"
        outputs.append((mjcf_dir / "rakuda.xml", RakudaMjcfOptions()))
        outputs.append((mjcf_dir / "rakuda_gripper.xml", RakudaMjcfOptions(gripper=GRIPPER_NAME)))
    outputs.append((PACKAGED_RAKUDA_MJCF, RakudaMjcfOptions(embed_meshes=True)))
    outputs.append(
        (
            PACKAGED_RAKUDA_GRIPPER_MJCF,
            RakudaMjcfOptions(embed_meshes=True, gripper=GRIPPER_NAME),
        )
    )
    return outputs


def main(argv: Sequence[str] | None = None) -> int:
    """Regenerate the checked-in MJCF models."""
    import argparse

    parser = argparse.ArgumentParser(description="Export the Rakuda CAD model to MJCF.")
    parser.add_argument("-o", "--output", type=Path, default=None, help="write one model here")
    parser.add_argument("--urdf", type=Path, default=None, help="source URDF")
    parser.add_argument("--package-dir", type=Path, default=None, help="models/rakuda")
    parser.add_argument(
        "--density",
        type=float,
        default=RakudaMjcfOptions.density_kg_m3,
        help="kg/m^3 the CAD part volumes are multiplied by (default: aluminium)",
    )
    parser.add_argument("--embed-meshes", action="store_true", help="inline the geometry")
    parser.add_argument("--visual", choices=("mesh", "hull"), default="mesh")
    parser.add_argument(
        "--gripper",
        choices=(GRIPPER_NAME,),
        default=None,
        help="bolt on a borrowed hand; see robopy.sim.panda_gripper",
    )
    args = parser.parse_args(argv)

    if args.output is not None:
        jobs = [
            (
                args.output,
                RakudaMjcfOptions(
                    density_kg_m3=args.density,
                    embed_meshes=args.embed_meshes,
                    visual_source=args.visual,
                    gripper=args.gripper,
                ),
            )
        ]
    else:
        jobs = _default_outputs()

    for path, options in jobs:
        print(export_rakuda_mjcf(path, args.urdf, args.package_dir, options).summary())
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
