"""The Rakuda model committed under ``models/rakuda`` and the helper that finds it.

These tests run against the *real* export, not the synthetic fixture. They are
skipped when the model directory is absent (a wheel install) and, where noted,
when the ``kinematics`` extra is missing. Nothing here needs the Git LFS visual
meshes: the kinematics and the convex collision geometry are plain git files.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List

import numpy as np
import pytest

from robopy.kinematics.urdf_audit import audit_urdf
from robopy.models import find_models_dir, find_rakuda_model, is_lfs_pointer

REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIT_JSON = REPO_ROOT / "docs" / "robots" / "assets" / "rakuda_urdf_audit.json"

rakuda = find_rakuda_model(REPO_ROOT / "models")
pytestmark = pytest.mark.skipif(rakuda is None, reason="models/rakuda is not present")


class TestLocator:
    def test_the_committed_model_is_found_from_the_repo(self) -> None:
        assert find_models_dir(REPO_ROOT / "models") == REPO_ROOT / "models"
        assert rakuda is not None
        assert rakuda.package_dir == REPO_ROOT / "models" / "rakuda"
        assert rakuda.convex_collision_urdf.is_file()
        assert rakuda.visual_urdf.is_file()

    def test_env_var_is_tried_first_and_the_checkout_is_the_fallback(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        # A valid env dir wins; an env dir without the model is skipped and the
        # checkout is found by walking up from the package.
        monkeypatch.setenv("ROBOPY_MODELS_DIR", str(REPO_ROOT / "models"))
        assert find_models_dir() == REPO_ROOT / "models"
        monkeypatch.setenv("ROBOPY_MODELS_DIR", str(tmp_path))
        assert find_models_dir() == REPO_ROOT / "models"

    def test_lfs_pointer_detection(self, tmp_path: Path) -> None:
        pointer = tmp_path / "p.stl"
        pointer.write_text(
            "version https://git-lfs.github.com/spec/v1\n"
            "oid sha256:0000000000000000000000000000000000000000000000000000000000000000\n"
            "size 12345\n"
        )
        real = tmp_path / "r.stl"
        real.write_bytes(b"\x00" * 200)
        assert is_lfs_pointer(pointer) is True
        assert is_lfs_pointer(real) is False
        assert is_lfs_pointer(tmp_path / "missing.stl") is False

    def test_default_urdf_falls_back_to_convex_without_lfs_content(self, tmp_path: Path) -> None:
        # Build a fake layout whose visual meshes are pointers.
        pkg = tmp_path / "rakuda" / "assembly_2"
        (pkg / "urdf").mkdir(parents=True)
        (pkg / "meshes").mkdir()
        for name in (
            "assembly_2.urdf",
            "assembly_2_convex_collision.urdf",
            "assembly_2_mesh_collision.urdf",
        ):
            (pkg / "urdf" / name).write_text("<robot name='x'/>")
        (pkg / "meshes" / "a.stl").write_text("version https://git-lfs.github.com/spec/v1\n")
        files = find_rakuda_model(tmp_path)
        assert files is not None
        assert files.visual_meshes_available is False
        assert files.visual_mesh_status == "LFS_POINTERS"
        assert files.default_urdf == files.convex_collision_urdf
        assert files.missing_visual_meshes() == [pkg / "meshes" / "a.stl"]
        assert "git lfs pull" in (files.visual_mesh_hint() or "")

        # The same layout with no visual STL at all (meshes never added, or the
        # directory removed) is reported as ABSENT, with a different remedy.
        (pkg / "meshes" / "a.stl").unlink()
        absent = find_rakuda_model(tmp_path)
        assert absent is not None
        assert absent.visual_meshes_available is False
        assert absent.visual_mesh_status == "ABSENT"
        assert absent.missing_visual_meshes() == []
        assert "Rakuda-2_simulation_ready.zip" in (absent.visual_mesh_hint() or "")

        # And with real geometry it is PRESENT and the visual URDF is preferred.
        (pkg / "meshes" / "a.stl").write_bytes(b"\0" * 2000)
        present = find_rakuda_model(tmp_path)
        assert present is not None
        assert present.visual_mesh_status == "PRESENT"
        assert present.visual_meshes_available is True
        assert present.default_urdf == present.visual_urdf
        assert present.visual_mesh_hint() is None

    def test_the_convex_meshes_are_real_files_not_pointers(self) -> None:
        assert rakuda is not None
        collision = rakuda.package_dir / "assembly_2" / "collision_meshes"
        stls = list(collision.glob("*.stl"))
        assert len(stls) == 137
        assert not any(is_lfs_pointer(p) for p in stls)


class TestCommittedUrdf:
    def test_audit_matches_the_recorded_result(self) -> None:
        assert rakuda is not None
        audit = audit_urdf(rakuda.convex_collision_urdf, package_dirs=[rakuda.package_dir])
        recorded = json.loads(AUDIT_JSON.read_text(encoding="utf-8"))
        assert audit.n_links == recorded["n_links"] == 153
        assert audit.n_joints == recorded["n_joints"] == 152
        assert audit.joint_type_counts == recorded["joint_type_counts"]
        assert audit.movable_joint_names == recorded["movable_joint_names"]
        assert audit.total_mass_kg == pytest.approx(recorded["total_mass_kg"])
        # The recorded audit was taken with the visual meshes on disk. The audit
        # only asks whether a reference resolves to a file, so LFS pointer files
        # (a clone before `git lfs pull`) count as resolved; only a checkout
        # with no `meshes/*.stl` at all leaves the 137 visual references
        # unresolved. The convex hulls always resolve.
        if rakuda.visual_mesh_status == "ABSENT":
            assert len(audit.unresolved_meshes) == 137
            assert all(
                u.startswith("package://assembly_2/meshes/") for u in audit.unresolved_meshes
            )
        else:
            assert audit.unresolved_meshes == recorded["unresolved_meshes"] == []
        assert not any("collision_meshes" in u for u in audit.unresolved_meshes)
        assert audit.ambiguous_names == [
            "gripper_left_dof",
            "gripper_right_dof",
            "head_camera_link",
        ]


class TestCommittedModelKinematics:
    """Real-model regressions for the behaviour found while validating the export."""

    @pytest.fixture(scope="class")
    def model(self):  # type: ignore[no-untyped-def]
        pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
        from robopy.kinematics.urdf_model import WholeBodyModel

        assert rakuda is not None
        wb = WholeBodyModel.from_urdf(
            rakuda.convex_collision_urdf, package_dirs=[rakuda.package_dir], geometry_only=True
        )
        for side in ("left", "right"):
            wb.add_fixed_frame(f"{side}_tcp", f"gripper_{side}_dof", np.eye(4))
        wb.set_soft_limits(
            {
                "torso_yaw_dof": (-1.57, 1.57),
                "shoulder_pitch_left_dof": (-3.14, 3.14),
                "shoulder_pitch_right_dof": (-3.14, 3.14),
            }
        )
        return wb

    def test_shared_torso_and_independent_arms(self, model) -> None:
        q = model.neutral_q()
        torso = model.joint_v_index("torso_yaw_dof")
        for frame in ("left_tcp", "right_tcp"):
            J = model.frame_jacobian(q, frame)
            assert np.linalg.norm(J[:, torso]) > 1e-6
            for head in ("head_yaw_dof", "head_pitch_dof"):
                np.testing.assert_allclose(J[:, model.joint_v_index(head)], 0.0, atol=1e-12)
        J_right = model.frame_jacobian(q, "right_tcp")
        for name in model.movable_joint_names:
            if "left" in name:
                np.testing.assert_allclose(J_right[:, model.joint_v_index(name)], 0.0, atol=1e-12)

    def test_duplicate_frame_names_resolve(self, model) -> None:
        for name in ("gripper_left_dof", "gripper_right_dof", "head_camera_link"):
            assert len(model.frame_ids(name)) == 2
            model.frame_pose(model.neutral_q(), name)  # must not raise

    def test_right_elbow_zero_sits_on_its_upper_limit(self, model) -> None:
        lower, upper = model.position_limits(["elbow_pitch_right_dof"])
        assert upper[0] == pytest.approx(0.0)
        assert lower[0] == pytest.approx(-2.7925, abs=1e-3)

    def test_jogs_from_a_bent_pose_converge_in_both_orientation_modes(self, model) -> None:
        pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")
        from robopy.control.types import DualArmTarget, JointState, monotonic_ns
        from robopy.kinematics.dual_arm_ik import DualArmIK, DualArmIKConfig, DualArmIKStatus

        names = tuple(model.movable_joint_names)
        arm = {s: [n for n in names if s in n and "head" not in n] for s in ("left", "right")}
        ik = DualArmIK(
            model,
            left_frame="left_tcp",
            right_frame="right_tcp",
            torso_joint="torso_yaw_dof",
            left_arm_joints=arm["left"],
            right_arm_joints=arm["right"],
            head_joints=["head_yaw_dof", "head_pitch_dof"],
            config=DualArmIKConfig(
                max_joint_step_rad=0.05,
                compute_budget_s=5.0,
                max_state_age_s=5.0,
                max_joint_acceleration_rad_s2=None,
                damping=1e-3,
            ),
        )

        def state(pos):  # type: ignore[no-untyped-def]
            now = monotonic_ns()
            n = len(names)
            return JointState(
                joint_names=names,
                position_rad=np.asarray([pos[k] for k in names]),
                velocity_rad_s=np.zeros(n),
                current_a=np.zeros(n),
                valid=np.ones(n, dtype=bool),
                read_start_ns=now,
                read_end_ns=now,
                sequence=0,
                mode_generation=0,
            )

        # The elbows bend one way only in this export: left positive, right negative.
        bent = {k: 0.0 for k in names}
        bent["elbow_pitch_left_dof"] = 0.8
        bent["elbow_pitch_right_dof"] = -0.8
        q0 = model.q_from_positions(bent)
        right_hold = model.frame_pose(q0, "right_tcp")
        base_left = model.frame_pose(q0, "left_tcp")

        for orientation_cost in (0.15, 0.0):
            for axis, sign in ((0, 1), (0, -1), (1, 1), (1, -1), (2, 1), (2, -1)):
                target = base_left.copy()
                target[axis, 3] += sign * 0.03
                ik.reset(q0)
                ik.set_task_costs(orientation_cost=orientation_cost)
                current = dict(bent)
                result = None
                for _ in range(120):
                    result = ik.solve_step(
                        state(current),
                        DualArmTarget(left_target=target, right_target=right_hold),
                        0.02,
                    )
                    assert result.is_commandable, result.message
                    current.update(result.joint_targets_rad)
                    if result.status is DualArmIKStatus.CONVERGED:
                        break
                assert result is not None
                assert result.status is DualArmIKStatus.CONVERGED, (axis, sign, orientation_cost)
                assert result.left_position_error_m < 1e-3


def _visual_mesh_names(urdf: Path) -> List[str]:
    """Basenames of the ``<visual>`` meshes a URDF references."""
    import xml.etree.ElementTree as ET

    names: List[str] = []
    for visual in ET.parse(urdf).getroot().iter("visual"):
        for mesh in visual.iter("mesh"):
            uri = mesh.get("filename", "")
            if uri and uri.rsplit("/", 1)[-1] not in names:
                names.append(uri.rsplit("/", 1)[-1])
    return names


class TestGeometrySource:
    """Drawing the convex hulls when the visual meshes are unavailable."""

    @staticmethod
    def _clone_without_visual_meshes(tmp_path: Path) -> Path:
        # A copy of the committed layout minus whatever visual STLs it has.
        import shutil

        assert rakuda is not None
        clone = tmp_path / "models"
        shutil.copytree(rakuda.package_dir.parent, clone)
        meshes = clone / "rakuda" / "assembly_2" / "meshes"
        if meshes.is_dir():
            shutil.rmtree(meshes)
        return clone

    @pytest.fixture
    def pointer_clone(self, tmp_path: Path) -> Path:
        # What a clone looks like before `git lfs pull`: every visual STL the
        # URDF references exists but is an LFS pointer file.
        assert rakuda is not None
        clone = self._clone_without_visual_meshes(tmp_path)
        meshes = clone / "rakuda" / "assembly_2" / "meshes"
        meshes.mkdir()
        for name in _visual_mesh_names(rakuda.convex_collision_urdf):
            (meshes / name).write_text(
                "version https://git-lfs.github.com/spec/v1\noid sha256:00\nsize 1\n"
            )
        return clone

    @pytest.fixture
    def absent_clone(self, tmp_path: Path) -> Path:
        # What this repository looks like until the visual meshes are added
        # (see models/rakuda/README.md): no meshes/ directory at all.
        return self._clone_without_visual_meshes(tmp_path)

    def test_convex_urdf_draws_collision_hulls_when_meshes_are_absent(
        self, absent_clone: Path
    ) -> None:
        pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
        from robopy.viewer.model_bundle import ModelBundle

        files = find_rakuda_model(absent_clone)
        assert files is not None and files.visual_mesh_status == "ABSENT"
        assert files.default_urdf == files.convex_collision_urdf
        bundle = ModelBundle.load(files.convex_collision_urdf, package_dirs=[files.package_dir])
        assert bundle.geometry_source == "collision"
        assert len(bundle.geometries) == 137
        assert all(g.is_mesh and g.mesh_path.is_file() for g in bundle.geometries)
        assert any("missing" in w and "convex hulls" in w for w in bundle.warnings)
        assert not any("LFS pointers" in w for w in bundle.warnings)

    def test_convex_urdf_draws_collision_hulls_without_lfs_content(
        self, pointer_clone: Path
    ) -> None:
        pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
        from robopy.viewer.model_bundle import ModelBundle

        files = find_rakuda_model(pointer_clone)
        assert files is not None and files.visual_meshes_available is False
        bundle = ModelBundle.load(files.convex_collision_urdf, package_dirs=[files.package_dir])
        assert bundle.geometry_source == "collision"
        assert len(bundle.geometries) == 137
        assert all(g.is_mesh and not is_lfs_pointer(g.mesh_path) for g in bundle.geometries)
        assert any("convex hulls" in w for w in bundle.warnings)

    def test_visual_source_is_used_when_the_meshes_are_present(self) -> None:
        pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
        from robopy.viewer.model_bundle import ModelBundle

        assert rakuda is not None
        if not rakuda.visual_meshes_available:
            pytest.skip("visual meshes not fetched (git lfs pull)")
        bundle = ModelBundle.load(rakuda.convex_collision_urdf, package_dirs=[rakuda.package_dir])
        assert bundle.geometry_source == "visual"
        assert len(bundle.geometries) == 137
        assert not any("LFS" in w for w in bundle.warnings)

    def test_explicit_sources_and_a_bad_one(self, pointer_clone: Path) -> None:
        pytest.importorskip("pinocchio", reason="needs the 'kinematics' optional extra")
        from robopy.viewer.model_bundle import ModelBundle

        files = find_rakuda_model(pointer_clone)
        assert files is not None
        forced = ModelBundle.load(
            files.convex_collision_urdf, package_dirs=[files.package_dir], geometry_source="visual"
        )
        assert forced.geometries == [] and any("not drawn" in w for w in forced.warnings)
        absent = find_rakuda_model(
            self._clone_without_visual_meshes(files.package_dir.parent.parent / "b")
        )
        assert absent is not None and absent.visual_mesh_status == "ABSENT"
        forced_absent = ModelBundle.load(
            absent.convex_collision_urdf,
            package_dirs=[absent.package_dir],
            geometry_source="visual",
        )
        assert forced_absent.geometries == []
        assert any("missing" in w and "not drawn" in w for w in forced_absent.warnings)
        hulls = ModelBundle.load(
            files.convex_collision_urdf,
            package_dirs=[files.package_dir],
            geometry_source="collision",
        )
        assert len(hulls.geometries) == 137
        with pytest.raises(ValueError, match="geometry_source"):
            ModelBundle.load(
                files.convex_collision_urdf, package_dirs=[files.package_dir], geometry_source="x"
            )
