"""Write the CALVIN floor as USD, so the Isaac Sim backend has a scene to load.

``ScenarioCfg.scene`` is the one asset MetaSim does *not* convert for you.  An
object or a robot that ships only an MJCF is run through Isaac Lab's converters
on first launch (``metasim/utils/isaacsim_asset_util.py``); a scene is not --
``IsaacsimHandler._load_scene`` reads ``scene_cfg.usd_path`` and, finding it
``None``, logs a warning and returns.  Worse, the handler only builds its own
terrain when ``scenario.scene is None``, so a scene set without a USD leaves
Isaac Sim with **no floor at all**: blocks fall through the world.

So the room exists twice, from one set of numbers:

    assets/calvin_room/calvin_room.xml    MuJoCo, hand-written
    assets/calvin_room/calvin_room.usda   Isaac Sim, written by this script

Both put CALVIN's ``checker_blue.png`` on a 30 m plane with a tile every 0.5 m
and the ``Kd 0.588`` of ``plane.mtl`` over it, because both are reading
:data:`FLOOR_SIZE_M`, :data:`FLOOR_TILE_M` and :data:`FLOOR_KD` below -- which
are in turn measured off ``calvin_env``'s own ``plane.obj`` and ``plane.mtl``.
``test_calvin_room`` re-measures the USD and fails if the two drift apart.

The sky is not in the USD.  MuJoCo needs a skybox texture because its
background is otherwise the clear colour; Isaac Sim's background comes from the
dome light in ``ScenarioCfg.lights``, which is a real image-based light rather
than a painted backdrop, so putting a second sky in the stage would only fight
with it.

    python examples/roboverse/calvin_room_asset.py
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Sequence, Tuple

__all__ = [
    "FLOOR_KD",
    "FLOOR_SIZE_M",
    "FLOOR_TEXTURE",
    "FLOOR_TILE_M",
    "CalvinRoomExportReport",
    "export_calvin_room_usd",
    "find_calvin_room",
]

#: Side of CALVIN's floor, metres.  ``plane/plane.obj`` spans -15..15 on both
#: axes.
FLOOR_SIZE_M: float = 30.0

#: Side of one checker square, metres.  ``plane.obj``'s UVs run 0..15 over those
#: 30 m, so the image tiles every 2 m, and the image is a 4x4 checker.
FLOOR_TILE_M: float = 0.5

#: ``Kd`` from ``plane/plane.mtl``.  The image is white on very light grey; this
#: is what keeps the floor from reading as a sheet of blown-out white.
FLOOR_KD: float = 0.588

#: The image itself, vendored beside the generated stage.
FLOOR_TEXTURE = "textures/checker_blue.png"

MODEL_NAME = "calvin_room"
_USD_NAME = f"{MODEL_NAME}.usda"

#: Where the room lives: beside this file, under ``assets/``, like the table.
_ASSETS = Path(__file__).resolve().parent / "assets"


def find_calvin_room(assets_dir: Path | None = None) -> Path | None:
    """Locate the room directory, or ``None``."""
    base = Path(assets_dir) if assets_dir is not None else _ASSETS
    room = base / MODEL_NAME
    return room if (room / FLOOR_TEXTURE).is_file() else None


@dataclass
class CalvinRoomExportReport:
    """What the export produced."""

    output_path: Path
    size_m: float = 0.0
    tile_m: float = 0.0
    uv_repeats: float = 0.0
    has_collider: bool = False

    def summary(self) -> str:
        """A short human-readable report."""
        return "\n".join([
            f"wrote {self.output_path}",
            f"  {self.size_m:g} x {self.size_m:g} m floor, tile {self.tile_m:g} m "
            f"({self.uv_repeats:g} texture repeats per side)",
            f"  collider: {'yes' if self.has_collider else 'NO -- objects will fall through'}",
        ])


def _quad(half: float) -> Tuple[list, list]:
    """A single-quad mesh in the z=0 plane, counter-clockwise seen from above."""
    points = [(-half, -half, 0.0), (half, -half, 0.0), (half, half, 0.0), (-half, half, 0.0)]
    return points, [0, 1, 2, 3]


def export_calvin_room_usd(
    output_path: Path | str | None = None,
    room_dir: Path | str | None = None,
) -> CalvinRoomExportReport:
    """Write ``calvin_room.usda``: CALVIN's floor, textured, with a collider.

    Args:
        output_path: Where the ``.usda`` goes.  Defaults to beside the texture.
        room_dir: The room asset directory.  Found automatically by default.

    Returns:
        A :class:`CalvinRoomExportReport`, filled in by reopening the written
        stage and reading it back.

    Raises:
        FileNotFoundError: If the room assets cannot be found.
        ImportError: If USD is not installed.
    """
    try:
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade, Vt
    except ImportError as exc:  # pragma: no cover - depends on the environment
        raise ImportError(
            "writing USD needs the USD Python bindings: pip install usd-core "
            "(Isaac Sim brings its own, so this is only needed to regenerate the asset)"
        ) from exc

    room = Path(room_dir) if room_dir is not None else find_calvin_room()
    if room is None:
        raise FileNotFoundError(
            "the room assets were not found; they are vendored beside this file as "
            "assets/calvin_room/, so this is a broken checkout"
        )
    room = Path(room).resolve()
    output_path = Path(output_path) if output_path else room / _USD_NAME
    output_path.parent.mkdir(parents=True, exist_ok=True)

    repeats = FLOOR_SIZE_M / (FLOOR_TILE_M * 4.0)  # the image is a 4x4 checker

    # CreateNew refuses an existing layer, and this is a regenerated asset.
    output_path.unlink(missing_ok=True)
    stage = Usd.Stage.CreateNew(str(output_path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    stage.SetMetadata(
        "comment",
        "CALVIN's floor. GENERATED, DO NOT EDIT BY HAND. "
        "regenerate: python examples/roboverse/calvin_room_asset.py",
    )

    root = UsdGeom.Xform.Define(stage, "/calvin_room")
    stage.SetDefaultPrim(root.GetPrim())

    # --- the floor ---------------------------------------------------------
    mesh = UsdGeom.Mesh.Define(stage, "/calvin_room/Floor")
    points, indices = _quad(FLOOR_SIZE_M / 2.0)
    mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(*p) for p in points]))
    mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4]))
    mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(indices))
    mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, 1.0)] * 4))
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    # A quad is flat; without this USD would smooth it as a subdivision cage and
    # Isaac Sim's tessellation would round the corners of a 30 m plane.
    mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    mesh.CreateExtentAttr(
        Vt.Vec3fArray([
            Gf.Vec3f(-FLOOR_SIZE_M / 2.0, -FLOOR_SIZE_M / 2.0, 0.0),
            Gf.Vec3f(FLOOR_SIZE_M / 2.0, FLOOR_SIZE_M / 2.0, 0.0),
        ])
    )
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar(
        "st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex
    )
    st.Set(
        Vt.Vec2fArray([
            Gf.Vec2f(0.0, 0.0),
            Gf.Vec2f(repeats, 0.0),
            Gf.Vec2f(repeats, repeats),
            Gf.Vec2f(0.0, repeats),
        ])
    )

    # PhysX needs to be told this is ground; a Mesh alone is scenery and every
    # block placed on the bench would fall past it to -inf.
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    mesh_collision = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    mesh_collision.CreateApproximationAttr(UsdPhysics.Tokens.none)

    # --- the material ------------------------------------------------------
    material = UsdShade.Material.Define(stage, "/calvin_room/Looks/CalvinFloor")

    reader = UsdShade.Shader.Define(stage, "/calvin_room/Looks/CalvinFloor/stReader")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    reader_out = reader.CreateOutput("result", Sdf.ValueTypeNames.Float2)

    texture = UsdShade.Shader.Define(stage, "/calvin_room/Looks/CalvinFloor/diffuseTexture")
    texture.CreateIdAttr("UsdUVTexture")
    texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(FLOOR_TEXTURE)
    texture.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader_out)
    texture.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    texture.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    # plane.mtl's Kd, folded into the texture rather than into diffuseColor,
    # which the shader below takes from this output.
    texture.CreateInput("scale", Sdf.ValueTypeNames.Float4).Set(
        Gf.Vec4f(FLOOR_KD, FLOOR_KD, FLOOR_KD, 1.0)
    )
    texture_out = texture.CreateOutput("rgb", Sdf.ValueTypeNames.Float3)

    surface = UsdShade.Shader.Define(stage, "/calvin_room/Looks/CalvinFloor/PreviewSurface")
    surface.CreateIdAttr("UsdPreviewSurface")
    surface.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(texture_out)
    surface.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.75)
    surface.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    surface.CreateInput("specular", Sdf.ValueTypeNames.Float).Set(0.1)
    material.CreateSurfaceOutput().ConnectToSource(
        surface.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    )
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(material)

    stage.GetRootLayer().Save()

    # --- read it back ------------------------------------------------------
    written = Usd.Stage.Open(str(output_path))
    floor = UsdGeom.Mesh(written.GetPrimAtPath("/calvin_room/Floor"))
    extent = floor.GetExtentAttr().Get()
    uvs = UsdGeom.PrimvarsAPI(floor).GetPrimvar("st").Get()
    side = float(extent[1][0] - extent[0][0])
    uv_repeats = float(max(uv[0] for uv in uvs))
    return CalvinRoomExportReport(
        output_path=output_path,
        size_m=side,
        tile_m=side / (uv_repeats * 4.0),
        uv_repeats=uv_repeats,
        has_collider=bool(written.GetPrimAtPath("/calvin_room/Floor").HasAPI(UsdPhysics.CollisionAPI)),
    )


def main(argv: Sequence[str] | None = None) -> int:
    """Regenerate the checked-in room stage."""
    import argparse

    parser = argparse.ArgumentParser(description="Export CALVIN's floor to USD.")
    parser.add_argument("-o", "--output", type=Path, default=None)
    parser.add_argument("--room-dir", type=Path, default=None)
    args = parser.parse_args(argv)
    print(export_calvin_room_usd(args.output, args.room_dir).summary())
    return 0


if __name__ == "__main__":  # pragma: no cover
    raise SystemExit(main())
