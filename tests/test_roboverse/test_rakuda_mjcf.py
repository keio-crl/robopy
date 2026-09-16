"""The exported MJCF, checked against the CAD export it came from.

The point of these tests is that the MJCF is *generated*, so nothing stops a
change to the exporter from producing a model that still loads and is quietly no
longer the same robot.  Two of the bugs this suite exists to catch were exactly
that:

* MuJoCo names a mesh asset after its file's basename, and the export's visual
  mesh and convex hull share one.  Loading the URDF naively collapsed each pair
  and gave every collision geom the full visual mesh, silently discarding the
  hulls.
* ``MjSpec.to_xml`` cannot serialise a ``fusestatic``-compiled model: it moves
  the fused geoms but writes each body's *original* ``pos`` and ``<inertial>``.
  The saved file loaded fine, and had parts up to 27 cm out of place and a tenth
  of the mass.

So the kinematics are re-derived here from the URDF by hand -- no MuJoCo, no
exporter code -- and compared against what the simulator actually loads.
"""

from __future__ import annotations

import hashlib
import math
from pathlib import Path
from typing import Dict, Tuple
from xml.etree import ElementTree

import numpy as np
import pytest

from robopy.models import find_rakuda_model

mujoco = pytest.importorskip("mujoco", reason="needs the 'sim' optional extra")

from robopy.roboverse.assets import PACKAGED_RAKUDA_MJCF  # noqa: E402
from robopy.sim.mjcf_export import (  # noqa: E402
    RAKUDA_ACTUATED_JOINTS,
    RAKUDA_BASE_BODY,
    RAKUDA_FRAME_SITES,
    RakudaMjcfOptions,
    export_rakuda_mjcf,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
# The models directory lives inside the package now, so let the library
# find it rather than guessing at a layout.
rakuda = find_rakuda_model()
pytestmark = pytest.mark.skipif(rakuda is None, reason="the Rakuda model is not present")

#: Total material volume of the CAD export, in m^3.  Every ``<mass>`` in the
#: export is a part volume (density 1), so this times the density is the mass.
CAD_VOLUME_M3 = 0.00206929


def _checkout_mjcf() -> Path:
    return rakuda.package_dir / "assembly_2" / "mjcf" / "rakuda.xml"


# --------------------------------------------------------------------------- #
# A hand-rolled URDF forward kinematics, used as the reference
# --------------------------------------------------------------------------- #


def _rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = (
        math.cos(roll), math.sin(roll), math.cos(pitch),
        math.sin(pitch), math.cos(yaw), math.sin(yaw),
    )
    return (
        np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        @ np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        @ np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    )


def _origin(element: ElementTree.Element) -> np.ndarray:
    origin = element.find("origin")
    transform = np.eye(4)
    if origin is None:
        return transform
    transform[:3, 3] = [float(v) for v in origin.get("xyz", "0 0 0").split()]
    transform[:3, :3] = _rpy(*[float(v) for v in origin.get("rpy", "0 0 0").split()])
    return transform


class _UrdfReference:
    """Poses of the CAD export's links with every joint at zero."""

    def __init__(self, urdf_path: Path) -> None:
        self.root = ElementTree.parse(urdf_path).getroot()
        self.by_child = {
            joint.find("child").get("link"): joint for joint in self.root.findall("joint")
        }
        self.child_of = {
            joint.get("name"): joint.find("child").get("link")
            for joint in self.root.findall("joint")
        }

    def pose(self, link: str) -> np.ndarray:
        transform = np.eye(4)
        current = link
        for _ in range(len(self.by_child) + 1):
            joint = self.by_child.get(current)
            if joint is None:
                return transform
            transform = _origin(joint) @ transform
            current = joint.find("parent").get("link")
        raise AssertionError(f"the URDF has a cycle above {link!r}")

    def masses(self) -> float:
        return sum(
            float(link.find("inertial/mass").get("value"))
            for link in self.root.findall("link")
            if link.find("inertial") is not None
        )


@pytest.fixture(scope="module")
def reference() -> _UrdfReference:
    return _UrdfReference(rakuda.convex_collision_urdf)


@pytest.fixture(scope="module")
def model():
    path = _checkout_mjcf()
    if not path.is_file():
        pytest.skip(f"{path} has not been generated; run python -m robopy.sim.mjcf_export")
    return mujoco.MjModel.from_xml_path(str(path))


class TestTheExportIsTheCadModel:
    def test_the_cad_masses_are_volumes(self, reference: _UrdfReference) -> None:
        """The premise the whole mass model rests on, stated as a test.

        If a future export ships real masses instead of volumes, the total will
        stop being ~2 litres and multiplying by a density would be badly wrong.
        """
        assert reference.masses() == pytest.approx(CAD_VOLUME_M3, rel=1e-4)

    def test_every_joint_frame_matches_the_urdf(self, model, reference: _UrdfReference) -> None:
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        for joint_name in RAKUDA_ACTUATED_JOINTS:
            link = reference.child_of[joint_name]
            body = joint_name[: -len("_dof")]
            body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body)
            assert body_id >= 0, f"the model has no body {body!r}"
            want = reference.pose(link)[:3, 3]
            # 1e-6 m, not exact: MuJoCo keeps positions in float32.
            assert np.allclose(data.xpos[body_id], want, atol=1e-6), (
                f"{body!r} is at {data.xpos[body_id]}, the CAD puts it at {want}"
            )

    def test_every_site_matches_the_urdf_frame_it_marks(
        self, model, reference: _UrdfReference
    ) -> None:
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        for site_name, link in RAKUDA_FRAME_SITES.items():
            site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_name)
            assert site_id >= 0, f"the model has no site {site_name!r}"
            want = reference.pose(link)[:3, 3]
            assert np.allclose(data.site_xpos[site_id], want, atol=1e-6)

    def test_total_mass_is_the_cad_volume_times_the_density(self, model) -> None:
        density = RakudaMjcfOptions().density_kg_m3
        assert model.body_mass.sum() == pytest.approx(CAD_VOLUME_M3 * density, rel=1e-3)

    def test_no_body_carrying_a_joint_is_massless(self, model) -> None:
        """A weightless body with a degree of freedom is a MuJoCo hard error."""
        for joint_id in range(model.njnt):
            body_id = model.jnt_bodyid[joint_id]
            weld = model.body_weldid[body_id]
            welded = model.body_mass[[i for i in range(model.nbody) if model.body_weldid[i] == weld]]
            assert welded.sum() > 1e-6


class TestTheHullsAreActuallyUsed:
    def test_visual_and_collision_meshes_did_not_alias(self, model) -> None:
        """The failure this catches was silent: 274 geoms sharing 137 meshes."""
        assert model.nmesh == 274, (
            f"{model.nmesh} mesh assets for {model.ngeom} geoms -- the visual meshes and the "
            "convex hulls have collapsed onto each other again"
        )

    def test_collision_geoms_use_hulls_and_visual_geoms_use_meshes(self, model) -> None:
        for geom_id in range(model.ngeom):
            mesh_id = model.geom_dataid[geom_id]
            if mesh_id < 0:
                continue
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
            collidable = model.geom_contype[geom_id] != 0
            assert collidable == name.startswith("col_"), (
                f"geom {geom_id} uses {name!r} but contype={model.geom_contype[geom_id]}"
            )

    def test_a_hull_really_is_simpler_than_its_visual_mesh(self, model) -> None:
        """Otherwise the hulls are there in name only."""
        pairs = 0
        for mesh_id in range(model.nmesh):
            name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_MESH, mesh_id)
            if not name.startswith("col_"):
                continue
            twin = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_MESH, "vis_" + name[4:])
            if twin < 0:
                continue
            pairs += 1
            assert model.mesh_facenum[mesh_id] <= model.mesh_facenum[twin]
        assert pairs > 100, f"only {pairs} hull/visual pairs found"


class TestItSimulates:
    def test_the_assembly_pose_is_contact_free(self, model) -> None:
        """Parts that touch as designed are excluded; nothing else should touch.

        A model that starts in contact with itself burns solver effort on the
        first step and can push a joint off its commanded position before a
        controller has done anything.
        """
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        touching = [
            (
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom1]),
                mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[c.geom2]),
            )
            for c in (data.contact[i] for i in range(data.ncon))
        ]
        assert data.ncon == 0, f"the robot starts in contact with itself: {touching}"

    def test_every_servo_holds_its_commanded_angle(self, model) -> None:
        """The whole robot, driven to a non-trivial pose and left to settle."""
        data = mujoco.MjData(model)
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)
        ]
        assert set(names) == set(RAKUDA_ACTUATED_JOINTS)

        commanded = {"torso_yaw_dof": 0.5, "elbow_pitch_right_dof": -0.9, "elbow_pitch_left_dof": 0.9}
        for i, name in enumerate(names):
            low, high = model.actuator_ctrlrange[i]
            data.ctrl[i] = np.clip(commanded.get(name, 0.0), low + 1e-3, high - 1e-3)
        # 1500 steps at the model's 2 ms timestep is 3 s of simulation, and the
        # slowest joint settles inside 1 s. Longer is just slower: mesh-on-mesh
        # collision over 274 geoms is not cheap.
        for _ in range(1500):
            mujoco.mj_step(model, data)

        assert np.all(np.isfinite(data.qpos))
        for i, name in enumerate(names):
            angle = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)]]
            # 0.02 rad: gravity leaves a standing error of stall/kp on a loaded
            # joint, which is about 0.007 rad on the worst of them.
            assert abs(angle - data.ctrl[i]) < 0.02, f"{name} settled at {angle}, asked for {data.ctrl[i]}"

    def test_the_continuous_joints_got_the_motor_travel(self, model) -> None:
        from robopy.config.robot_config.rakuda_config import RAKUDA_MOTOR_TRAVEL_RAD

        for name in ("torso_yaw_dof", "shoulder_pitch_left_dof", "shoulder_pitch_right_dof"):
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            assert tuple(model.jnt_range[joint_id]) == pytest.approx(RAKUDA_MOTOR_TRAVEL_RAD)


class TestBothCheckedInModels:
    def test_the_packaged_model_is_self_contained(self) -> None:
        """It has to load with no ``models/`` directory beside it, or a wheel is useless."""
        assert PACKAGED_RAKUDA_MJCF.is_file(), "the packaged model is missing from the source tree"
        text = PACKAGED_RAKUDA_MJCF.read_text(encoding="utf-8")
        assert 'file="' not in text, "the packaged model still references external mesh files"
        model = mujoco.MjModel.from_xml_path(str(PACKAGED_RAKUDA_MJCF))
        assert model.njnt == len(RAKUDA_ACTUATED_JOINTS)

    def test_the_two_models_are_the_same_robot(self, model) -> None:
        """Same bodies, joints and masses; they differ only in what is drawn."""
        packaged = mujoco.MjModel.from_xml_path(str(PACKAGED_RAKUDA_MJCF))
        assert packaged.nbody == model.nbody
        assert packaged.njnt == model.njnt
        assert packaged.body_mass.sum() == pytest.approx(model.body_mass.sum(), rel=1e-6)
        assert np.allclose(packaged.jnt_range, model.jnt_range)

        assert np.allclose(packaged.qpos0, model.qpos0, atol=1e-9)

        # Same kinematics, body for body -- the two differ only in what is drawn.
        poses = []
        for reference_model in (model, packaged):
            data = mujoco.MjData(reference_model)
            mujoco.mj_kinematics(reference_model, data)
            poses.append({
                mujoco.mj_id2name(reference_model, mujoco.mjtObj.mjOBJ_BODY, i): data.xpos[i].copy()
                for i in range(reference_model.nbody)
            })
        assert set(poses[0]) == set(poses[1])
        for name, position in poses[0].items():
            assert np.allclose(position, poses[1][name], atol=1e-6), f"{name} differs"

    def test_the_xml_is_well_formed(self) -> None:
        """MuJoCo tolerates ``--`` inside a comment; conforming parsers do not."""
        for path in (PACKAGED_RAKUDA_MJCF, _checkout_mjcf()):
            if path.is_file():
                ElementTree.parse(path)


class TestReExportIsStable:
    def test_exporting_again_reproduces_the_checked_in_model(self, tmp_path: Path) -> None:
        """A drifted checked-in model is worse than none: it is silently stale."""
        report = export_rakuda_mjcf(
            tmp_path / "rakuda.xml",
            rakuda.convex_collision_urdf,
            rakuda.package_dir,
            RakudaMjcfOptions(embed_meshes=True),
        )
        assert report.num_joints == len(RAKUDA_ACTUATED_JOINTS)
        assert report.total_mass_kg == pytest.approx(
            CAD_VOLUME_M3 * report.density_kg_m3, rel=1e-3
        )
        assert sorted(report.sites) == sorted(RAKUDA_FRAME_SITES)
        assert RAKUDA_BASE_BODY in report.body_names
        assert len(report.assembly_contacts_excluded) == 3

        # Compared by digest rather than by value: these are 2.8 MB files, and
        # letting pytest render a character diff of two of them wedges the run.
        fresh = hashlib.sha256((tmp_path / "rakuda.xml").read_bytes()).hexdigest()
        committed = hashlib.sha256(PACKAGED_RAKUDA_MJCF.read_bytes()).hexdigest()
        assert fresh == committed, (
            "the packaged model does not match a fresh export; re-run "
            "`python -m robopy.sim.mjcf_export` and commit the result"
        )

    def test_the_density_scales_the_masses(self, tmp_path: Path) -> None:
        report = export_rakuda_mjcf(
            tmp_path / "half.xml",
            rakuda.convex_collision_urdf,
            rakuda.package_dir,
            RakudaMjcfOptions(density_kg_m3=1350.0, embed_meshes=True),
        )
        assert report.total_mass_kg == pytest.approx(CAD_VOLUME_M3 * 1350.0, rel=1e-3)

class TestWhichModelGetsResolved:
    """Which of the two files :func:`resolve_rakuda_mjcf` hands back, and why.

    The mesh-referencing model is the better-looking one, but it is only usable
    when the visual meshes are actually on disk -- and they are the one part of
    the model directory that is *not* shipped: the wheel excludes them and
    ``robopy-models fetch`` pulls them on demand.  Handing it back without them
    yields a path MuJoCo cannot open, which is a confusing way to fail, so the
    self-contained model is used instead.
    """

    def test_it_refuses_the_mesh_model_when_the_meshes_are_missing(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        from robopy.roboverse import assets

        class _NoMeshes:
            package_dir = rakuda.package_dir
            visual_meshes_available = False

        monkeypatch.setattr("robopy.models.find_rakuda_model", lambda base=None: _NoMeshes())
        monkeypatch.delenv("ROBOPY_RAKUDA_MJCF", raising=False)
        for variant in ("plain", "gripper"):
            path, why = assets.resolve_rakuda_mjcf(variant)
            assert "roboverse/assets" in str(path), (
                f"{variant}: handed back {path}, which references meshes that are not there"
            )
            assert "hull" in why

    def test_it_uses_the_mesh_model_when_they_are_there(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        from robopy.roboverse import assets

        class _WithMeshes:
            package_dir = rakuda.package_dir
            visual_meshes_available = True

        monkeypatch.setattr("robopy.models.find_rakuda_model", lambda base=None: _WithMeshes())
        monkeypatch.delenv("ROBOPY_RAKUDA_MJCF", raising=False)
        if not _checkout_mjcf().is_file():
            pytest.skip("the mesh-referencing model has not been generated")
        path, why = assets.resolve_rakuda_mjcf("plain")
        assert path == _checkout_mjcf()
        assert "visual meshes" in why

    def test_the_override_still_wins(self, monkeypatch: pytest.MonkeyPatch, tmp_path) -> None:
        from robopy.roboverse import assets

        mine = tmp_path / "mine.xml"
        mine.write_text("<mujoco/>")
        monkeypatch.setenv("ROBOPY_RAKUDA_MJCF", str(mine))
        path, why = assets.resolve_rakuda_mjcf("gripper")
        assert path == mine and "ROBOPY_RAKUDA_MJCF" in why
