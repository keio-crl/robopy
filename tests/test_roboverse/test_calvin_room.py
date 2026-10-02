"""The room these scenes are filmed in, and the rig that lights them.

The room exists twice -- an MJCF for MuJoCo and a USD for Isaac Sim -- from one
set of measurements taken off ``calvin_env``'s own ``plane.obj`` and
``plane.mtl``.  Two files written by hand and by a script from the same numbers
is exactly the arrangement that drifts, so most of what is here is checking
that they still agree.
"""

from __future__ import annotations

import xml.etree.ElementTree as ET

import pytest

from calvin_room_asset import (
    FLOOR_KD,
    FLOOR_SIZE_M,
    FLOOR_TEXTURE,
    FLOOR_TILE_M,
    export_calvin_room_usd,
    find_calvin_room,
)

ROOM = find_calvin_room()
pytestmark = pytest.mark.skipif(ROOM is None, reason="the room is not vendored here")


class TestMjcf:
    """``calvin_room.xml``: what the MuJoCo backend loads as its scene."""

    @pytest.fixture(scope="class")
    def mjcf(self):
        return ET.parse(ROOM / "calvin_room.xml").getroot()

    def test_the_floor_texture_is_vendored(self, mjcf):
        for texture in mjcf.find("asset").findall("texture"):
            source = texture.get("file")
            if source is None:  # the skybox is procedural
                continue
            assert (ROOM / source).is_file(), f"missing texture {source}"

    def test_there_is_a_sky(self, mjcf):
        """The complaint this room was built for: a black upper half."""
        kinds = {texture.get("type") for texture in mjcf.find("asset").findall("texture")}
        assert "skybox" in kinds

    def test_the_floor_matches_calvins_own(self, mjcf):
        """Tile size and shade, both measured off ``plane.obj`` / ``plane.mtl``.

        With ``texuniform`` the unit of ``texrepeat`` is image repeats per metre,
        and the image is a 4x4 checker, so a tile is ``1 / (4 * texrepeat)``.
        """
        material = next(
            m for m in mjcf.find("asset").findall("material") if m.get("texture") == "calvin_floor"
        )
        assert material.get("texuniform") == "true"
        repeats = float(material.get("texrepeat").split()[0])
        assert 1.0 / (4.0 * repeats) == pytest.approx(FLOOR_TILE_M, abs=1e-6)
        assert float(material.get("rgba").split()[0]) == pytest.approx(FLOOR_KD, abs=1e-6)

    def test_the_floor_is_big_enough_to_have_no_visible_edge(self, mjcf):
        floor = next(g for g in mjcf.find("worldbody").iter("geom") if g.get("name") == "floor")
        assert floor.get("type") == "plane"
        assert float(floor.get("size").split()[0]) * 2 == pytest.approx(FLOOR_SIZE_M, abs=1e-6)

    def test_it_declares_no_lights(self, mjcf):
        """The rig lives in ``staging.LIGHTS`` so Isaac Sim gets it too.

        A ``<light>`` here would light the MuJoCo render and nothing else, and
        the two backends would quietly stop matching.
        """
        assert not list(mjcf.iter("light"))


class TestUsd:
    """``calvin_room.usda``: what the Isaac Sim backend loads as its scene."""

    @pytest.fixture(scope="class")
    def report(self, tmp_path_factory):
        pytest.importorskip("pxr", reason="needs usd-core (or Isaac Sim's own USD)")
        staging = tmp_path_factory.mktemp("calvin-room")
        (staging / "textures").symlink_to(ROOM / "textures", target_is_directory=True)
        return export_calvin_room_usd(staging / "calvin_room.usda", room_dir=staging)

    def test_the_floor_is_the_same_floor_as_the_mjcf(self, report):
        assert report.size_m == pytest.approx(FLOOR_SIZE_M, abs=1e-6)
        assert report.tile_m == pytest.approx(FLOOR_TILE_M, abs=1e-6)

    def test_it_has_a_collider(self, report):
        """Without one the floor is scenery and everything falls through it."""
        assert report.has_collider

    def test_the_checked_in_stage_is_current(self):
        pytest.importorskip("pxr")
        from pxr import Usd, UsdGeom

        committed = ROOM / "calvin_room.usda"
        if not committed.is_file():
            pytest.skip("run python examples/roboverse/calvin_room_asset.py first")
        stage = Usd.Stage.Open(str(committed))
        floor = UsdGeom.Mesh(stage.GetPrimAtPath("/calvin_room/Floor"))
        assert floor, "the stage has no /calvin_room/Floor"
        extent = floor.GetExtentAttr().Get()
        assert float(extent[1][0] - extent[0][0]) == pytest.approx(FLOOR_SIZE_M, abs=1e-6)
        # Isaac Sim resolves the texture relative to the stage, so it has to be
        # a relative path that exists beside it, not an absolute one from the
        # machine the asset happened to be generated on.
        assert not FLOOR_TEXTURE.startswith("/")
        assert (ROOM / FLOOR_TEXTURE).is_file()


class TestStaging:
    """``staging.staged``: the defaults every task in ``tasks/`` now gets."""

    @pytest.fixture(scope="class")
    def scenario(self):
        pytest.importorskip("metasim", reason="needs RoboVerse/MetaSim installed")
        from staging import staged

        return staged(objects=[], robots=[])

    def test_the_room_replaces_metasims_default_ground(self, scenario):
        """Both, or the two floors z-fight at exactly the same height."""
        assert scenario.scene is not None
        assert scenario.add_default_ground is False

    def test_the_scene_carries_both_assets(self, scenario):
        """MetaSim converts an object's asset on demand but never a scene's."""
        assert scenario.scene.mjcf_path is not None
        assert scenario.scene.usd_path is not None

    def test_mujoco_is_told_to_use_the_rig(self, scenario):
        """Off by MetaSim's default, and the whole rig is ignored without it."""
        assert scenario.sim_params.mujoco_use_scenario_lights is True

    def test_the_rig_has_a_key_and_a_fill_and_a_sky(self, scenario):
        from metasim.scenario.lights import DistantLightCfg, DomeLightCfg

        kinds = [type(light) for light in scenario.lights]
        assert kinds.count(DomeLightCfg) == 1
        assert kinds.count(DistantLightCfg) == 2

    def test_nothing_is_bright_enough_to_clip(self, scenario):
        """MuJoCo maps these to ``diffuse`` and anything over 1 burns out white.

        The constants are in ``metasim/sim/mujoco/lights.py``; this is the check
        that keeps someone raising an intensity for Isaac Sim from silently
        blowing out every MuJoCo render.
        """
        from metasim.sim.mujoco.lights import (
            AMBIENT_RATIO,
            DISTANT_INTENSITY_TO_DIFFUSE,
            DOME_INTENSITY_TO_AMBIENT,
        )
        from metasim.scenario.lights import DistantLightCfg, DomeLightCfg

        ambient = 0.0
        for light in scenario.lights:
            if isinstance(light, DistantLightCfg):
                diffuse = light.intensity * DISTANT_INTENSITY_TO_DIFFUSE
                assert diffuse <= 1.0, f"{light.name} maps to diffuse {diffuse:.2f}"
                ambient += diffuse * AMBIENT_RATIO
            elif isinstance(light, DomeLightCfg):
                ambient += light.intensity * DOME_INTENSITY_TO_AMBIENT
        assert ambient <= 1.0, f"global ambient {ambient:.2f} clips the frame to white"
