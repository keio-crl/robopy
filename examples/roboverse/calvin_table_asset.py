"""Turn CALVIN's play table into an MJCF, so RoboVerse can put it in a scene.

``calvin_table_D`` is the furniture the CALVIN benchmark builds its tasks around:
a bench with a sliding door, a drawer, a push button and a switch, plus an LED
and a light bulb.  The Rakuda work in this repository has had only a plain box
for a table, and this is what it takes to stand the robot in CALVIN's own scene
instead.

Unlike the Rakuda export, this one is nearly free.  The table is 8 links and 4
prismatic joints, its meshes are 11 files totalling 348 KB with no ``package://``
anywhere, and MuJoCo reads the URDF as-is once it is told where the meshes are.
There is no mass to invent either -- the upstream URDF has real inertials.

What does not come across
-------------------------
The LED and the bulb do not light.  ``calvin_scene_D.yaml`` wires them up with
``effect: led`` and ``effect: lightbulb``, but that is calvin_env's Python
reacting to the button and switch joints; the URDF only has the geometry.  A
task that wants them lit can read the joint and set the geoms' ``rgba``.

Textures, on the other hand, now do.  MuJoCo's URDF importer ignores ``mtllib``,
so the bench used to arrive with its texture coordinates intact and nothing to
put on them; the export reads the same MTLs PyBullet reads and writes MuJoCo
``<texture>``/``<material>`` pairs itself.  See :func:`_apply_materials`.

Neither did anything look right before that, for a second reason: the importer
puts visuals in geom group 1 and leaves collisions at group 0, and MuJoCo draws
both.  Every one of the 45 boxes below was being painted over the bench.  See
:data:`_COLLISION_GROUP`.

The bench, and why it is boxes
------------------------------
Upstream ships convex decompositions for the moving parts (``*_vhacd2.obj``)
but not for ``base_link``, whose ``<collision>`` is the visual mesh marked
``concave="yes"``.  PyBullet honours that; MuJoCo cannot, and hulls what it is
given.  The hull of this bench is a wedge running from the front edge of the
work surface up to the top of the back panel, so anything placed on the table
starts several centimetres *inside* the collision geometry and is flung out --
which is exactly what happened to CALVIN's three blocks.

So the export replaces that one collision mesh with boxes.  The bench is CAD
furniture, near enough axis-aligned, so its own face planes cut space into cells
that are each wholly inside or wholly outside the solid; testing one point per
cell and merging neighbours greedily gives an exact box decomposition, 45 of
them, in well under a second.  See :func:`decompose_to_boxes`.

The round details -- the drawer handle, the surround on the button -- are not
axis-aligned and come out slightly boxy, a little larger than they are.  Nothing
else changes: the bench top lands where the visual mesh says it does, and the
cavities the drawer and the sliding door move through are genuinely hollow.
"""

from __future__ import annotations

import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

__all__ = [
    "CALVIN_SCALE",
    "CALVIN_TABLE_JOINTS",
    "CALVIN_TABLE_SURFACE",
    "CALVIN_WORK_SURFACE_Z",
    "decompose_to_boxes",
    "CalvinTableExportReport",
    "export_calvin_table_mjcf",
    "export_calvin_table_urdf",
    "find_calvin_table",
]

#: ``global_scaling`` from ``calvin_scene_D.yaml``.  CALVIN shrinks the whole
#: scene by this before placing anything, so every position in that file -- the
#: robot's base included -- is already in scaled world coordinates.
CALVIN_SCALE: float = 0.8

#: The table's moving parts, and what each one is.
CALVIN_TABLE_JOINTS: Dict[str, str] = {
    "base__button": "push button",
    "base__switch": "switch",
    "base__slide": "sliding door",
    "base__drawer": "drawer",
}

#: ``surfaces.table`` from ``calvin_scene_D.yaml``: the rectangle CALVIN drops
#: objects into, as ``((x_min, y_min), (x_max, y_max))``.  Already in scaled
#: world coordinates, like everything else in that file.
CALVIN_TABLE_SURFACE: Tuple[Tuple[float, float], Tuple[float, float]] = (
    (0.0, -0.15),
    (0.35, -0.03),
)

#: Height of the bench over that rectangle, at :data:`CALVIN_SCALE`.
#:
#: Measured off the exported model rather than taken from the scene file, which
#: says 0.46 -- that is the height CALVIN *drops* from, two centimetres of clear
#: air above the bench.  ``test_calvin_table`` re-measures this on every export,
#: so if the scale or the meshes move, it fails rather than drifting.
CALVIN_WORK_SURFACE_Z: float = 0.44

MODEL_NAME = "calvin_table"
_URDF_NAME = "calvin_table_D.urdf"
#: The generated URDF, for the backends that cannot read an MJCF.
_SCALED_URDF_NAME = "calvin_table_scaled.urdf"

_URDF_HEADER = """
  CALVIN's play table, scaled and with the bench boxed.
  GENERATED, DO NOT EDIT BY HAND.

  source:      calvin_table_D.urdf (calvin_env, MIT)
  regenerate:  python examples/roboverse/calvin_table_asset.py

  This is the file the non-MuJoCo backends load; MuJoCo loads mjcf/calvin_table.xml,
  which is generated from the same two steps. CALVIN's global_scaling of 0.8 is
  baked in rather than left to the simulator, and base_link's concave collision
  mesh is replaced by an exact box decomposition, because every engine but
  PyBullet convex-hulls a collision mesh and the hull of this bench is a wedge
  that flings anything placed on it away.

  Textures come from the MTLs beside the OBJs, which Isaac Sim's importer reads.
"""


@dataclass
class CalvinTableExportReport:
    """What the export produced."""

    output_path: Path
    num_bodies: int = 0
    num_joints: int = 0
    num_geoms: int = 0
    num_meshes: int = 0
    total_mass_kg: float = 0.0
    scale: float = 1.0
    num_collision_boxes: int = 0
    joints: List[str] = field(default_factory=list)
    size_m: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    work_surface_z: float = 0.0
    materials: List[str] = field(default_factory=list)
    num_hidden_collision_geoms: int = 0

    def summary(self) -> str:
        """A short human-readable report."""
        return "\n".join(
            [
                f"wrote {self.output_path}",
                f"  {self.num_bodies} bodies, {self.num_joints} joints, "
                f"{self.num_geoms} geoms, {self.num_meshes} meshes",
                f"  {self.total_mass_kg:.2f} kg, "
                f"{self.size_m[0]:.3f} x {self.size_m[1]:.3f} x {self.size_m[2]:.3f} m "
                f"at scale {self.scale:g}",
                f"  {self.num_collision_boxes} boxes in place of {_BOXED_LINK}'s concave mesh",
                f"  {self.num_hidden_collision_geoms} collision geoms moved to group "
                f"{_COLLISION_GROUP} so they stop being drawn",
                f"  textured: {', '.join(self.materials) or 'nothing (no MTLs found)'}",
                f"  work surface at z = {self.work_surface_z:.3f}",
                f"  moving parts: {', '.join(self.joints)}",
            ]
        )


#: Where the vendored table lives: beside this file, under ``assets/``.
#:
#: Not in robopy's ``models/`` directory, even though that is where the Rakuda's
#: own meshes are.  Nothing in the library reads this table -- it is scenery for
#: an example -- and 321 KB of someone else's furniture has no business in the
#: wheel.
_ASSETS = Path(__file__).resolve().parent / "assets"


def find_calvin_table(assets_dir: Path | None = None) -> Path | None:
    """Locate the vendored table directory, or ``None``."""
    base = Path(assets_dir) if assets_dir is not None else _ASSETS
    table = base / MODEL_NAME
    return table if (table / "urdf" / _URDF_NAME).is_file() else None


#: Plane coordinates closer together than this are treated as one when building
#: the decomposition grid.  4 mm keeps the cell count in the thousands without
#: losing any real panel: the thinnest part of the bench is 25 mm.
_PLANE_MERGE_M: float = 0.004

#: Which link gets boxed, and why: the only one whose collision is a concave
#: mesh with no convex decomposition shipped alongside it.
_BOXED_LINK = "base_link"

#: Parts held by their joint limits rather than by contact with the cabinet.
#: See :func:`_exclude_cabinet_contacts`.
#:
#: The button is deliberately *not* here.  Its housing is what holds it up, and
#: cutting that contact drops it 0.02 m to the bottom of its travel, looking
#: permanently pressed.  The other three are all pushed the wrong way by it.
_SLIDING_PARTS = ("switch_link", "slide_link", "drawer_link")


def _read_obj(path: Path) -> Tuple[List[List[float]], List[List[int]]]:
    """Vertices and triangles of a Wavefront OBJ. Polygons are fanned."""
    verts: List[List[float]] = []
    faces: List[List[int]] = []
    for line in path.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(v) for v in line.split()[1:4]])
        elif line.startswith("f "):
            idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
            faces.extend([idx[0], idx[k], idx[k + 1]] for k in range(1, len(idx) - 1))
    return verts, faces


def _plane_grid(values, min_gap: float, np):
    """Distinct plane coordinates along one axis, near-duplicates merged outwards."""
    out: List[float] = []
    for value in np.unique(np.round(values, 6)):
        if not out or value - out[-1] >= min_gap:
            out.append(float(value))
        else:
            out[-1] = float(value)
    return np.array(out)


def decompose_to_boxes(
    obj_path: Path | str, min_gap: float = _PLANE_MERGE_M
) -> List[Tuple[Tuple[float, float, float], Tuple[float, float, float]]]:
    """Cut an axis-aligned mesh into boxes, as ``(low_corner, high_corner)`` pairs.

    The mesh's own face planes are extended into a non-uniform grid.  For an
    axis-aligned solid every resulting cell is entirely inside or entirely
    outside, so one point test per cell -- by counting how many triangles a
    vertical ray crosses below it -- classifies the lot exactly.  Neighbouring
    solid cells are then merged greedily, growing along ``x``, then ``y``, then
    ``z``, which turns a few thousand cells into a few dozen boxes.

    Faces that are not axis-aligned, such as a cylindrical handle, still get
    classified: their planes just enter the grid at the extremes, so the feature
    comes out as the smallest box-union the grid can draw around it.

    Args:
        obj_path: The mesh.
        min_gap: Merge plane coordinates nearer than this, to bound the grid.

    Returns:
        The boxes, in the mesh's own coordinates and units.

    Raises:
        ImportError: If NumPy is not installed.
    """
    try:
        import numpy as np
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError("decomposing a mesh into boxes needs NumPy") from exc

    raw_verts, raw_faces = _read_obj(Path(obj_path))
    verts = np.array(raw_verts)
    faces = np.array(raw_faces)
    grids = [_plane_grid(verts[:, axis], min_gap, np) for axis in range(3)]
    centres = [(g[:-1] + g[1:]) / 2 for g in grids]

    tri = verts[faces]
    a, b, c = tri[:, 0], tri[:, 1], tri[:, 2]
    # Twice the signed area of each triangle's shadow on the xy plane.  Zero
    # means the triangle stands vertically, and a vertical ray never crosses it.
    area = (b[:, 0] - a[:, 0]) * (c[:, 1] - a[:, 1]) - (c[:, 0] - a[:, 0]) * (b[:, 1] - a[:, 1])
    upright = np.abs(area) > 1e-12
    a, b, c, area = a[upright], b[upright], c[upright], area[upright]

    shape = tuple(len(centre) for centre in centres)
    inside = np.zeros(shape, dtype=bool)
    for i, x in enumerate(centres[0]):
        for j, y in enumerate(centres[1]):
            # Barycentric coordinates of (x, y) against every triangle at once.
            w0 = ((b[:, 0] - x) * (c[:, 1] - y) - (c[:, 0] - x) * (b[:, 1] - y)) / area
            w1 = ((c[:, 0] - x) * (a[:, 1] - y) - (a[:, 0] - x) * (c[:, 1] - y)) / area
            w2 = 1.0 - w0 - w1
            hit = (w0 >= 0) & (w1 >= 0) & (w2 >= 0)
            if not hit.any():
                continue
            crossings = w0[hit] * a[hit, 2] + w1[hit] * b[hit, 2] + w2[hit] * c[hit, 2]
            below = np.searchsorted(np.sort(crossings), centres[2])
            inside[i, j] = below % 2 == 1

    todo = inside.copy()
    gx, gy, gz = grids
    nx, ny, nz = shape
    boxes = []
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                if not todo[i, j, k]:
                    continue
                i1 = i
                while i1 + 1 < nx and todo[i1 + 1, j, k]:
                    i1 += 1
                j1 = j
                while j1 + 1 < ny and todo[i : i1 + 1, j1 + 1, k].all():
                    j1 += 1
                k1 = k
                while k1 + 1 < nz and todo[i : i1 + 1, j : j1 + 1, k1 + 1].all():
                    k1 += 1
                todo[i : i1 + 1, j : j1 + 1, k : k1 + 1] = False
                boxes.append(
                    (
                        (float(gx[i]), float(gy[j]), float(gz[k])),
                        (float(gx[i1 + 1]), float(gy[j1 + 1]), float(gz[k1 + 1])),
                    )
                )
    return boxes


#: MuJoCo renders geom groups 0, 1 and 2 by default and hides group 3, which is
#: the convention MuJoCo's own models use for collision geometry.  The URDF
#: importer knows nothing of it: it puts every visual in group 1 and leaves
#: collisions at the default 0, so the 45 boxes of the bench, and the convex
#: hulls of the drawer, the door and the switch, were all drawn *over* the
#: meshes they stand in for.  That is what made the table look like a pile of
#: white blocks.  The export moves them to group 3; MuJoCo's viewer still shows
#: them on demand, and the physics is untouched -- a group is a rendering hint.
_COLLISION_GROUP = "3"

#: Group the URDF importer gives a ``<visual>``.
_VISUAL_GROUP = "1"

#: Specular and shininess for the wood.  The URDF has no shading information at
#: all -- ``rgba 1 1 1 1`` on every textured link, which is the "modulate the
#: texture by white" convention -- so these are chosen, not imported: enough
#: sheen to read as varnished chipboard under a key light, not enough to look
#: like plastic.
_WOOD_SPECULAR = "0.25"
_WOOD_SHININESS = "0.35"


def _mtl_texture(mtl_path: Path) -> str | None:
    """The ``map_Kd`` of a Wavefront MTL, as a bare file name, or ``None``."""
    if not mtl_path.is_file():
        return None
    for line in mtl_path.read_text().splitlines():
        if line.strip().startswith("map_Kd"):
            return Path(line.split(maxsplit=1)[1].strip()).name
    return None


def _hide_collision_geoms(mjcf: ET.Element) -> int:
    """Move every geom the URDF importer left ungrouped into :data:`_COLLISION_GROUP`.

    Returns:
        How many geoms were moved.
    """
    moved = 0
    for geom in mjcf.iter("geom"):
        if geom.get("group") is None:
            geom.set("group", _COLLISION_GROUP)
            moved += 1
    return moved


def _apply_materials(mjcf: ET.Element, table: Path, output_dir: Path) -> List[str]:
    """Give the visual meshes the textures their MTLs name.

    MuJoCo's URDF importer ignores ``mtllib``: it reads the geometry of an OBJ
    and nothing else, so the bench arrived with its texture coordinates intact
    and no texture to put on them.  This walks the same MTLs PyBullet reads --
    one beside each OBJ -- and turns each ``map_Kd`` into a MuJoCo
    ``<texture>``/``<material>`` pair, then points the link's visual geom at it.

    Links whose mesh is an STL (the button, the LED, the bulb) have no texture
    coordinates and no MTL; they keep the URDF's flat ``rgba``, which is what
    CALVIN draws them in too.

    Args:
        mjcf: The parsed MJCF, after ``MjSpec.to_xml``.
        table: The vendored table directory.
        output_dir: Where the MJCF is being written, for relative asset paths.

    Returns:
        The names of the materials created, in the order they were added.
    """
    import os

    asset = mjcf.find("asset")
    if asset is None:
        return []

    # link name -> texture file name, read from the MTL beside its mesh.
    textures: Dict[str, str] = {}
    for mesh in asset.findall("mesh"):
        name = mesh.get("name") or ""
        texture = _mtl_texture(table / "meshes" / f"{name}.mtl")
        if texture is not None and (table / "textures" / texture).is_file():
            textures[name] = texture

    created: List[str] = []
    by_file: Dict[str, str] = {}
    for link, texture in textures.items():
        if texture not in by_file:
            tex_name = f"tex_{Path(texture).stem}"
            ET.SubElement(
                asset,
                "texture",
                {
                    "name": tex_name,
                    "type": "2d",
                    "file": os.path.relpath(table / "textures" / texture, output_dir),
                },
            )
            by_file[texture] = tex_name
        material = f"mat_{link}"
        ET.SubElement(
            asset,
            "material",
            {
                "name": material,
                "texture": by_file[texture],
                # The meshes carry their own UVs from Blender, so the texture
                # must follow them rather than be projected on a cube: with
                # texuniform="true" the wood grain would ignore the unwrap.
                "texuniform": "false",
                "specular": _WOOD_SPECULAR,
                "shininess": _WOOD_SHININESS,
            },
        )
        created.append(material)

    for geom in mjcf.iter("geom"):
        link = geom.get("mesh")
        if geom.get("group") == _VISUAL_GROUP and link in textures:
            geom.set("material", f"mat_{link}")
            # rgba multiplies the texture; the URDF's "white" is the identity,
            # but say so rather than leave the reader to check.
            geom.set("rgba", "1 1 1 1")
    return created


def _exclude_cabinet_contacts(mjcf: ET.Element) -> None:
    """Stop the cabinet from pushing its own drawer open.

    The drawer, the sliding door and the switch collide with the bench they move
    inside: upstream ships them as convex decompositions, which bulge a little
    past the real surface, and the box decomposition of the bench is
    conservative around anything not axis-aligned.  Left alone, and with nothing
    touching them, the drawer creeps 0.17 m open by itself, the door 0.02 m, and
    the switch is forced 0.015 m through its own lower limit.

    A drawer should be held by its joint limits, not by grinding against the
    cabinet, and PyBullet -- which is what CALVIN runs -- disables parent/child
    collision by default.  So this excludes those pairs explicitly, rather than
    relying on MuJoCo's ``filterparent``, which a simulator wrapper is free to
    turn off for the whole model.
    """
    contact = mjcf.find("contact")
    if contact is None:
        contact = ET.SubElement(mjcf, "contact")
    for child in _SLIDING_PARTS:
        ET.SubElement(contact, "exclude", {"body1": _BOXED_LINK, "body2": child})


def _boxify_collision(root: ET.Element, meshes: Path, link_name: str) -> int:
    """Swap one link's concave collision mesh for the box decomposition.

    Returns:
        How many boxes replaced it, or 0 if the link has no mesh collision.
    """
    link = next((el for el in root.findall("link") if el.get("name") == link_name), None)
    if link is None:
        return 0
    collisions = link.findall("collision")
    source = None
    for collision in collisions:
        mesh = collision.find("geometry/mesh")
        if mesh is not None:
            source = meshes / Path(mesh.get("filename", "")).name
    if source is None or not source.is_file():
        return 0

    boxes = decompose_to_boxes(source)
    for collision in collisions:
        link.remove(collision)
    for low, high in boxes:
        collision = ET.SubElement(link, "collision")
        centre = [(lo + hi) / 2.0 for lo, hi in zip(low, high)]
        size = [hi - lo for lo, hi in zip(low, high)]
        ET.SubElement(
            collision, "origin", {"xyz": " ".join(repr(v) for v in centre), "rpy": "0 0 0"}
        )
        geometry = ET.SubElement(collision, "geometry")
        ET.SubElement(geometry, "box", {"size": " ".join(repr(v) for v in size)})
    return len(boxes)


def _scale_urdf(root: ET.Element, scale: float) -> None:
    """Shrink the whole model by ``scale``, the way CALVIN's ``global_scaling`` does.

    Doing it here rather than asking the simulator to scale the object means the
    numbers in ``calvin_scene_D.yaml`` -- which are already in scaled world
    coordinates -- line up without anything else having to know.

    Everything with a length in it moves: link offsets, mesh sizes, and the
    prismatic travel, so the drawer still comes exactly as far out relative to
    the bench.  Mass goes as the cube and inertia as the fifth power, which
    matters little for furniture bolted to the floor but costs nothing to get
    right.
    """
    if scale == 1.0:
        return
    for origin in root.iter("origin"):
        xyz = origin.get("xyz")
        if xyz:
            origin.set("xyz", " ".join(repr(float(v) * scale) for v in xyz.split()))
    for mesh in root.iter("mesh"):
        existing = mesh.get("scale", "1 1 1").split()
        mesh.set("scale", " ".join(repr(float(v) * scale) for v in existing))
    for box in root.iter("box"):
        box.set("size", " ".join(repr(float(v) * scale) for v in box.get("size", "").split()))
    for mass in root.iter("mass"):
        mass.set("value", repr(float(mass.get("value", "0")) * scale**3))
    for inertia in root.iter("inertia"):
        for key in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz"):
            inertia.set(key, repr(float(inertia.get(key, "0")) * scale**5))
    for joint in root.findall("joint"):
        if joint.get("type") != "prismatic":
            continue
        limit = joint.find("limit")
        if limit is None:
            continue
        for key in ("lower", "upper"):
            value = limit.get(key)
            if value is not None:
                limit.set(key, repr(float(value) * scale))


def _prepared_urdf(table: Path, scale: float) -> Tuple[ET.ElementTree, int]:
    """The upstream URDF with the bench boxed and the whole model scaled.

    Both exports start here, so a table loaded through MJCF and a table loaded
    through URDF are the same table: same box decomposition standing in for the
    bench's concave collision mesh, same ``global_scaling`` baked in rather than
    left to the simulator.

    Args:
        table: The vendored table directory.
        scale: CALVIN's ``global_scaling``.

    Returns:
        The tree, and how many boxes replaced the concave mesh.
    """
    tree = ET.parse(table / "urdf" / _URDF_NAME)
    root = tree.getroot()
    # Boxes first, in the mesh's own units, then scale everything together.
    num_boxes = _boxify_collision(root, table / "meshes", _BOXED_LINK)
    _scale_urdf(root, scale)
    return tree, num_boxes


def export_calvin_table_urdf(
    output_path: Path | str | None = None,
    table_dir: Path | str | None = None,
    scale: float = CALVIN_SCALE,
) -> CalvinTableExportReport:
    """Write the scaled URDF the non-MuJoCo backends load.

    MuJoCo is the only backend in MetaSim that reads an MJCF.  Isaac Sim, and
    PyBullet, SAPIEN and Genesis with it, read URDF -- Isaac Sim by running
    Isaac Lab's ``UrdfConverter`` over it once and caching the USD beside it
    (``metasim/utils/isaacsim_asset_util.py``).  So a scene that is to run on
    more than MuJoCo needs the table as a URDF as well, and it has to be *this*
    URDF rather than the upstream one, for two reasons:

    **Scale.**  ``ArticulationObjCfg.scale`` does reach Isaac Sim's
    ``UsdFileCfg`` but not MuJoCo's MJCF loader.  Setting it would give a table
    0.8x on one backend and full size on the other, under block positions that
    are scaled either way.  Baking the scale into both exports is the only way
    the two agree.

    **The bench's collision mesh.**  Upstream marks it ``concave="yes"``, which
    PyBullet honours; PhysX, like MuJoCo, convex-hulls a collision mesh unless
    it is told to do SDF or mesh decomposition, and the hull of this bench is a
    wedge that flings anything placed on it away.  The box decomposition is
    exact and every engine understands boxes.

    The textures come across on this path too, and by a shorter route than the
    MJCF's: the URDF points at OBJs, the OBJs name their MTLs, and Isaac Sim's
    importer reads MTLs -- which is the one thing MuJoCo's URDF importer does
    not do.

    Args:
        output_path: Where the ``.urdf`` goes.  Defaults to
            ``urdf/calvin_table_scaled.urdf`` inside the vendored table.
        table_dir: The vendored table directory.  Found automatically by default.
        scale: CALVIN's ``global_scaling``, baked in.

    Returns:
        A :class:`CalvinTableExportReport`.  It is filled in by compiling the
        written URDF *in MuJoCo* and measuring it, which is what makes this an
        export and not a copy: the work surface and the bounding box in the
        report are measured off the URDF, and ``test_calvin_table`` checks they
        match the MJCF's to the millimetre.

    Raises:
        FileNotFoundError: If the vendored table cannot be found.
        ImportError: If ``mujoco`` is not installed.
    """
    try:
        import mujoco
        import numpy as np
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError(
            "verifying the exported URDF needs MuJoCo: pip install 'robopy[sim]'"
        ) from exc

    table = Path(table_dir) if table_dir is not None else find_calvin_table()
    if table is None:
        raise FileNotFoundError(
            "the CALVIN play table was not found; it is vendored beside this file "
            "directory as calvin_table/, so this is a broken checkout"
        )
    table = Path(table).resolve()
    output_path = Path(output_path) if output_path else table / "urdf" / _SCALED_URDF_NAME
    output_path.parent.mkdir(parents=True, exist_ok=True)

    tree, num_boxes = _prepared_urdf(table, scale)
    root = tree.getroot()
    root.set("name", MODEL_NAME)
    # A <mujoco> block inside a URDF is the standard way to tell MuJoCo's
    # importer what it cannot infer, and every other URDF parser ignores an
    # element it does not know. It is here so that *this* file, read straight
    # into MuJoCo, is the same table as the MJCF: without it the importer
    # discards the visual meshes and fuses the static links away, and the model
    # comes out 3.16 kg of collision boxes with no wood on them.
    wrapper = ET.Element("mujoco")
    ET.SubElement(
        wrapper,
        "compiler",
        {"balanceinertia": "true", "discardvisual": "false", "strippath": "false", "fusestatic": "false"},
    )
    root.insert(0, wrapper)
    _indent(root)
    output_path.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n'
        f"<!--{_URDF_HEADER}-->\n" + ET.tostring(root, encoding="unicode").rstrip() + "\n",
        encoding="utf-8",
    )

    # Measure what was written, the same way the MJCF export measures itself.
    verified = mujoco.MjModel.from_xml_path(str(output_path))
    data = mujoco.MjData(verified)
    mujoco.mj_kinematics(verified, data)
    low, high = _mesh_bounds(verified, data, np)
    # MuJoCo's URDF importer leaves collisions in group 0, where its ray-caster
    # was looking before the MJCF export moved them; measure them where they are.
    surface = _work_surface_height(verified, data, mujoco, np, group=0)
    return CalvinTableExportReport(
        output_path=output_path,
        scale=scale,
        num_collision_boxes=num_boxes,
        num_bodies=int(verified.nbody),
        num_joints=int(verified.njnt),
        num_geoms=int(verified.ngeom),
        num_meshes=int(verified.nmesh),
        total_mass_kg=float(verified.body_mass.sum()),
        joints=[
            mujoco.mj_id2name(verified, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(verified.njnt)
        ],
        size_m=tuple(float(v) for v in (high - low)),
        work_surface_z=surface,
        # No materials are written here: the OBJs name their own MTLs and the
        # importers that read this file read those. Report which links that
        # covers, so a missing MTL shows up in the export log either way.
        materials=[
            f"{mtl.stem} <- {texture}"
            for mtl, texture in sorted(
                (mtl, _mtl_texture(mtl)) for mtl in (table / "meshes").glob("*.mtl")
            )
            if texture
        ],
    )


def export_calvin_table_mjcf(
    output_path: Path | str | None = None,
    table_dir: Path | str | None = None,
    scale: float = CALVIN_SCALE,
) -> CalvinTableExportReport:
    """Write an MJCF of the play table beside its meshes.

    Args:
        output_path: Where the ``.xml`` goes.  Defaults to ``mjcf/calvin_table.xml``
            inside the vendored table directory.
        table_dir: The vendored table directory.  Found automatically by default.
        scale: Shrink the model by this, baked in rather than left to the
            simulator.  Defaults to CALVIN's own ``global_scaling``.

    Returns:
        A :class:`CalvinTableExportReport`.

    Raises:
        FileNotFoundError: If the vendored table cannot be found.
        ImportError: If ``mujoco`` is not installed; compiling the model is also
            what validates it.
    """
    try:
        import mujoco
        import numpy as np
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError(
            "exporting MJCF needs MuJoCo: pip install 'robopy[sim]' (or just 'mujoco')"
        ) from exc

    table = Path(table_dir) if table_dir is not None else find_calvin_table()
    if table is None:
        raise FileNotFoundError(
            "the CALVIN play table was not found; it is vendored beside this file "
            "directory as calvin_table/, so this is a broken checkout"
        )
    table = Path(table).resolve()
    output_path = Path(output_path) if output_path else table / "mjcf" / f"{MODEL_NAME}.xml"
    output_path.parent.mkdir(parents=True, exist_ok=True)

    tree, num_boxes = _prepared_urdf(table, scale)
    root = tree.getroot()
    # The URDF says "../meshes/x.obj"; MuJoCo gets a meshdir instead.
    for mesh in root.iter("mesh"):
        mesh.set("filename", mesh.get("filename", "").replace("../meshes/", ""))

    wrapper = ET.Element("mujoco")
    ET.SubElement(
        wrapper,
        "compiler",
        {
            "meshdir": str(table / "meshes"),
            "balanceinertia": "true",
            "discardvisual": "false",
            "strippath": "false",
            # No fusing: `MjSpec.to_xml` cannot serialise a fused model, the
            # same trap the Rakuda export documents at length.
            "fusestatic": "false",
        },
    )
    root.insert(0, wrapper)

    with tempfile.TemporaryDirectory(prefix="calvin-table-") as staging:
        staged = Path(staging) / _URDF_NAME
        tree.write(staged, encoding="utf-8", xml_declaration=True)
        spec = mujoco.MjSpec.from_file(str(staged))
        spec.compile()  # validates the model; to_xml needs it
        xml_text = spec.to_xml()

    mjcf = ET.fromstring(xml_text)
    mjcf.set("model", MODEL_NAME)
    compiler = mjcf.find("compiler")
    if compiler is not None:
        compiler.attrib.pop("meshdir", None)

    import os

    asset = mjcf.find("asset")
    if asset is not None:
        for mesh in asset.findall("mesh"):
            name = mesh.get("file") or f"{mesh.get('name')}.obj"
            source = table / "meshes" / Path(name).name
            if source.is_file():
                mesh.set("file", os.path.relpath(source, output_path.parent))

    materials = _apply_materials(mjcf, table, output_path.parent)
    hidden = _hide_collision_geoms(mjcf)
    _exclude_cabinet_contacts(mjcf)
    _indent(mjcf)
    header = (
        "\n  CALVIN's play table. GENERATED, DO NOT EDIT BY HAND.\n\n"
        f"  source:      {_URDF_NAME} (calvin_env, MIT)\n"
        "  regenerate:  python examples/roboverse/calvin_table_asset.py\n\n"
        "  The LED and the bulb do not light by themselves: that behaviour lives in\n"
        "  calvin_env's Python, not in the URDF. examples/roboverse/tasks/rakuda_calvin.py\n"
        "  drives them from the button and switch joints.\n\n"
        "  base_link's concave collision mesh has been replaced by an exact box\n"
        "  decomposition, because MuJoCo would otherwise hull it into a wedge and\n"
        f"  fling anything placed on the bench away. Those boxes are in geom group\n"
        f"  {_COLLISION_GROUP}, which MuJoCo hides; the wood grain comes from the MTLs\n"
        "  the URDF importer does not read.\n"
    )
    output_path.write_text(
        '<?xml version="1.0" encoding="utf-8"?>\n'
        f"<!--{header}-->\n" + ET.tostring(mjcf, encoding="unicode").rstrip() + "\n",
        encoding="utf-8",
    )

    verified = mujoco.MjModel.from_xml_path(str(output_path))
    data = mujoco.MjData(verified)
    mujoco.mj_kinematics(verified, data)
    low, high = _mesh_bounds(verified, data, np)

    return CalvinTableExportReport(
        output_path=output_path,
        scale=scale,
        num_collision_boxes=num_boxes,
        num_bodies=int(verified.nbody),
        num_joints=int(verified.njnt),
        num_geoms=int(verified.ngeom),
        num_meshes=int(verified.nmesh),
        total_mass_kg=float(verified.body_mass.sum()),
        joints=[
            mujoco.mj_id2name(verified, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(verified.njnt)
        ],
        size_m=tuple(float(v) for v in (high - low)),
        work_surface_z=_work_surface_height(verified, data, mujoco, np),
        materials=materials,
        num_hidden_collision_geoms=hidden,
    )


def _mesh_bounds(model, data, np):
    """World-space bounding box of every mesh geom, as ``(low, high)``."""
    low = np.full(3, np.inf)
    high = np.full(3, -np.inf)
    for geom in range(model.ngeom):
        mesh_id = model.geom_dataid[geom]
        if mesh_id < 0:
            continue
        start = model.mesh_vertadr[mesh_id]
        count = model.mesh_vertnum[mesh_id]
        verts = model.mesh_vert[start : start + count].reshape(-1, 3)
        world = verts @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
        low = np.minimum(low, world.min(axis=0))
        high = np.maximum(high, world.max(axis=0))
    return low, high


def _work_surface_height(model, data, mujoco, np, group: int | None = None) -> float:
    """Height of the bench where CALVIN drops objects, measured by ray-casting.

    Not the tallest point of the furniture, and not the top of any one link: the
    back panel, the shelf and the sliding door are all higher, and the plank the
    name suggests is a shelf at the back, outside the drop zone entirely.  What a
    block actually lands on is the top of ``base_link``'s collision hull over
    :data:`CALVIN_TABLE_SURFACE`, so that is what this drops a ray onto -- the
    one measurement that answers "will a block rest here".

    Only collision geoms take part, since the visual mesh is not what an object
    comes to rest on.  Which geom group that means depends on what is being
    measured: the MJCF export has just moved them to :data:`_COLLISION_GROUP`,
    while a URDF read straight into MuJoCo still has them in group 0 where the
    importer put them.  ``group`` says which, and defaults to the MJCF's.
    """
    (x0, y0), (x1, y1) = CALVIN_TABLE_SURFACE
    collision_only = np.zeros(6, dtype=np.uint8)
    collision_only[int(_COLLISION_GROUP) if group is None else group] = 1
    above = 5.0
    lowest = None
    for fx in (0.1, 0.5, 0.9):
        for fy in (0.1, 0.5, 0.9):
            point = np.array([x0 + (x1 - x0) * fx, y0 + (y1 - y0) * fy, above])
            hit = np.zeros(1, dtype=np.int32)
            drop = mujoco.mj_ray(
                model, data, point, np.array([0.0, 0.0, -1.0]), collision_only, 1, -1, hit
            )
            if hit[0] < 0:
                return 0.0
            surface = above - float(drop)
            lowest = surface if lowest is None else min(lowest, surface)
    return float(lowest or 0.0)


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


def main(argv: Sequence[str] | None = None) -> int:
    """Regenerate the checked-in play table model."""
    import argparse

    parser = argparse.ArgumentParser(description="Export CALVIN's play table to MJCF.")
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--table-dir", type=Path, default=None)
    parser.add_argument("--scale", type=float, default=CALVIN_SCALE, help="CALVIN's global_scaling")
    parser.add_argument(
        "--format",
        choices=("both", "mjcf", "urdf"),
        default="both",
        help="which asset to write; the scene needs both (MuJoCo reads the MJCF, everything else the URDF)",
    )
    args = parser.parse_args(argv)
    if args.output is not None and args.format == "both":
        parser.error("--output names one file; pass --format mjcf or --format urdf with it")
    if args.format in ("both", "mjcf"):
        print(export_calvin_table_mjcf(args.output, args.table_dir, args.scale).summary())
    if args.format in ("both", "urdf"):
        print(export_calvin_table_urdf(args.output, args.table_dir, args.scale).summary())
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
