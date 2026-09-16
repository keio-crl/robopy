"""The Rakuda model bundled as package data and the helper that finds it.

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
from robopy.models import (
    BUNDLED_MODELS_DIR,
    fetch_visual_meshes,
    find_models_dir,
    find_rakuda_model,
    is_lfs_pointer,
    visual_mesh_cache_package_dir,
    visual_mesh_names,
)
from robopy.models import main as models_main

REPO_ROOT = Path(__file__).resolve().parents[2]
AUDIT_JSON = REPO_ROOT / "docs" / "robots" / "assets" / "rakuda_urdf_audit.json"

rakuda = find_rakuda_model()
pytestmark = pytest.mark.skipif(rakuda is None, reason="the bundled Rakuda model is not present")


@pytest.fixture(autouse=True)
def _isolated_cache(monkeypatch: pytest.MonkeyPatch, tmp_path: Path) -> None:
    # Whatever the developer has fetched into their own cache must not leak
    # into these tests; every test gets an empty cache.
    monkeypatch.setenv("ROBOPY_CACHE_DIR", str(tmp_path / "cache"))


class TestLocator:
    def test_the_bundled_model_is_found_inside_the_package(self) -> None:
        assert find_models_dir() == BUNDLED_MODELS_DIR
        assert BUNDLED_MODELS_DIR.name == "models" and BUNDLED_MODELS_DIR.parent.name == "robopy"
        assert rakuda is not None
        assert rakuda.package_dir == BUNDLED_MODELS_DIR / "rakuda"
        assert rakuda.convex_collision_urdf.is_file()
        assert rakuda.visual_urdf.is_file()
        assert rakuda.visual_mesh_count == 137
        assert rakuda.package_dirs[-1] == rakuda.package_dir

    def test_env_var_is_tried_first_and_the_package_is_the_fallback(
        self, monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        # A valid env dir wins; an env dir without the model is skipped and the
        # package data is used.
        import shutil

        elsewhere = tmp_path / "elsewhere"
        shutil.copytree(
            BUNDLED_MODELS_DIR / "rakuda" / "assembly_2" / "urdf",
            elsewhere / "rakuda" / "assembly_2" / "urdf",
        )
        monkeypatch.setenv("ROBOPY_MODELS_DIR", str(elsewhere))
        assert find_models_dir() == elsewhere
        monkeypatch.setenv("ROBOPY_MODELS_DIR", str(tmp_path / "empty"))
        assert find_models_dir() == BUNDLED_MODELS_DIR
        monkeypatch.delenv("ROBOPY_MODELS_DIR")
        assert find_models_dir(tmp_path / "empty") == BUNDLED_MODELS_DIR

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
        assert "robopy-models fetch" in (absent.visual_mesh_hint() or "")

        # And with real geometry it is PRESENT and the visual URDF is preferred.
        (pkg / "meshes" / "a.stl").write_bytes(b"\0" * 2000)
        present = find_rakuda_model(tmp_path)
        assert present is not None
        assert present.visual_mesh_status == "PRESENT"
        assert present.visual_meshes_available is True
        assert present.default_urdf == present.visual_urdf
        assert present.visual_mesh_hint() is None
        assert present.visual_mesh_dir == pkg / "meshes"
        assert present.package_dirs == [tmp_path / "rakuda"]

    def test_the_convex_meshes_are_real_files_not_pointers(self) -> None:
        assert rakuda is not None
        collision = rakuda.package_dir / "assembly_2" / "collision_meshes"
        stls = list(collision.glob("*.stl"))
        assert len(stls) == 137
        assert not any(is_lfs_pointer(p) for p in stls)


class TestCommittedUrdf:
    def test_audit_matches_the_recorded_result(self) -> None:
        assert rakuda is not None
        audit = audit_urdf(rakuda.convex_collision_urdf, package_dirs=rakuda.package_dirs)
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
            rakuda.convex_collision_urdf, package_dirs=rakuda.package_dirs, geometry_only=True
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
        # What an installed wheel looks like before `robopy-models fetch`: the
        # visual meshes are excluded from the wheel, so no meshes/ directory.
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


def _stl_bytes(triangles: int = 2) -> bytes:
    """A minimal binary STL of the given triangle count."""
    return b"\0" * 80 + triangles.to_bytes(4, "little") + b"\0" * (50 * triangles)


def _asset_zip(
    files: dict[str, bytes], *, manifest: bool = True, extra: dict[str, bytes] | None = None
) -> bytes:
    """A release asset as scripts/build_visual_mesh_asset.py would write it."""
    import hashlib
    import io
    import json
    import zipfile

    buffer = io.BytesIO()
    with zipfile.ZipFile(buffer, "w", zipfile.ZIP_DEFLATED) as archive:
        for name, data in files.items():
            archive.writestr(f"assembly_2/meshes/{name}", data)
        for name, data in (extra or {}).items():
            archive.writestr(name, data)
        if manifest:
            table = {
                n: {"sha256": hashlib.sha256(d).hexdigest(), "bytes": len(d)}
                for n, d in files.items()
            }
            archive.writestr("MANIFEST.json", json.dumps({"files": table}))
    return buffer.getvalue()


class TestFetchVisualMeshes:
    """Downloading the release asset into the cache, without a network."""

    def test_fetch_extracts_the_asset_and_the_locator_then_finds_it(self, tmp_path: Path) -> None:
        assert rakuda is not None
        names = visual_mesh_names(rakuda.convex_collision_urdf)
        assert len(names) == 137
        asset = _asset_zip({n: _stl_bytes() for n in names})
        requested: list[tuple[str, str | None]] = []

        def opener(url: str, token: str | None, timeout_s: float) -> bytes:
            requested.append((url, token))
            return asset

        # An installed wheel has no visual meshes: pretend by pointing the
        # locator at a copy without them.
        clone = TestGeometrySource._clone_without_visual_meshes(tmp_path)
        before = find_rakuda_model(clone)
        assert before is not None and before.visual_mesh_status == "ABSENT"
        assert "robopy-models fetch" in (before.visual_mesh_hint() or "")

        report = fetch_visual_meshes(opener=opener, token="")
        assert report.ok, report.failed[:3]
        assert len(report.downloaded) == 137 and report.skipped == []
        assert report.destination == visual_mesh_cache_package_dir() / "assembly_2" / "meshes"
        assert report.archive_bytes == len(asset)
        # One request, to the pinned release asset, unauthenticated.
        assert requested == [
            (
                "https://github.com/keio-crl/robopy/releases/download/rakuda-visual-meshes-v1/rakuda_visual_meshes.zip",
                None,
            )
        ]
        assert all((report.destination / n).stat().st_size == len(_stl_bytes()) for n in names)

        after = find_rakuda_model(clone)
        assert after is not None and after.visual_mesh_status == "PRESENT"
        assert after.visual_mesh_dir == report.destination
        # The cache is searched first, the package's own directory second.
        assert after.package_dirs == [visual_mesh_cache_package_dir(), clone / "rakuda"]

        # A second fetch does not even download; --force extracts again.
        again = fetch_visual_meshes(opener=opener, token="")
        assert again.downloaded == [] and len(again.skipped) == 137 and again.source is None
        assert len(requested) == 1
        forced = fetch_visual_meshes(opener=opener, token="", force=True, names=names[:2])
        assert forced.downloaded == names[:2] and len(requested) == 2

    def test_a_token_resolves_the_asset_through_the_api(self, tmp_path: Path) -> None:
        import json

        asset = _asset_zip({"a.stl": _stl_bytes()})
        calls: list[tuple[str, str | None]] = []

        def opener(url: str, token: str | None, timeout_s: float) -> bytes:
            calls.append((url, token))
            if url.startswith("https://api.github.com/repos/keio-crl/robopy/releases/tags/"):
                return json.dumps(
                    {
                        "assets": [
                            {
                                "name": "rakuda_visual_meshes.zip",
                                "url": "https://api.github.com/repos/keio-crl/robopy/releases/assets/42",
                            }
                        ]
                    }
                ).encode()
            assert url.endswith("/assets/42")
            return asset

        report = fetch_visual_meshes(
            tmp_path / "dest", names=["a.stl"], opener=opener, token="secret"
        )
        assert report.ok and report.downloaded == ["a.stl"]
        assert [c[0].rsplit("/", 1)[-1] for c in calls] == ["rakuda-visual-meshes-v1", "42"]
        assert all(token == "secret" for _, token in calls)

    def test_bad_archives_and_entries_are_reported_and_not_kept(self, tmp_path: Path) -> None:
        import urllib.error

        pointer = b"version https://git-lfs.github.com/spec/v1\noid sha256:00\nsize 1\n"
        good = _stl_bytes(3)
        archive = _asset_zip(
            {"a.stl": pointer, "c.stl": good},
            extra={"assembly_2/meshes/../evil.stl": good, "README.txt": b"hi"},
        )
        # Tamper with c's manifest entry so its checksum no longer matches.
        import io
        import json
        import zipfile

        tampered = io.BytesIO()
        with zipfile.ZipFile(io.BytesIO(archive)) as src, zipfile.ZipFile(tampered, "w") as dst:
            for info in src.infolist():
                data = src.read(info)
                if info.filename == "MANIFEST.json":
                    table = json.loads(data)
                    table["files"]["c.stl"]["sha256"] = "0" * 64
                    data = json.dumps(table).encode()
                dst.writestr(info, data)

        report = fetch_visual_meshes(
            tmp_path / "dest",
            names=["a.stl", "b.stl", "c.stl", "evil.stl"],
            opener=lambda url, token, timeout: tampered.getvalue(),
            token="",
        )
        assert not report.ok and report.downloaded == []
        reasons = dict(report.failed)
        assert "LFS pointer" in reasons["a.stl"]
        assert reasons["b.stl"] == "not in the archive"
        assert "SHA-256" in reasons["c.stl"]
        assert reasons["evil.stl"] == "not in the archive"  # traversal entry was ignored
        assert not (tmp_path / "dest" / "c.stl").exists()
        assert not list(tmp_path.glob("**/evil.stl"))

        html = fetch_visual_meshes(
            tmp_path / "dest2",
            names=["a.stl"],
            opener=lambda u, t, s: b"<html>login</html>",
            token="",
        )
        assert not html.ok and "not a zip" in html.failed[0][1] and "HTML" in html.failed[0][1]

        def missing(url: str, token: str | None, timeout: float) -> bytes:
            raise urllib.error.HTTPError(url, 404, "Not Found", None, None)  # type: ignore[arg-type]

        gone = fetch_visual_meshes(tmp_path / "dest3", names=["a.stl"], opener=missing, token="")
        assert (
            not gone.ok
            and "HTTP 404" in gone.failed[0][1]
            and "no such release" in gone.failed[0][1]
        )
        assert gone.source is not None and gone.source.endswith("/rakuda_visual_meshes.zip")

    def test_command_line_status_paths_and_fetch(
        self, capsys: pytest.CaptureFixture[str], monkeypatch: pytest.MonkeyPatch, tmp_path: Path
    ) -> None:
        import robopy.models as models

        assert models_main(["status"]) == 0
        out = capsys.readouterr().out
        assert "Rakuda model" in out and "visual meshes" in out and "release asset" in out
        assert models_main(["path", "urdf"]) == 0
        assert capsys.readouterr().out.strip().endswith("assembly_2_convex_collision.urdf")
        assert models_main(["path", "cache"]) == 0
        assert capsys.readouterr().out.strip().endswith("meshes")

        asset = _asset_zip({"a.stl": _stl_bytes(), "b.stl": _stl_bytes()})
        seen: list[str] = []

        def fake(url: str, token: str | None, timeout: float) -> bytes:
            seen.append(url)
            return asset

        monkeypatch.setattr(models, "_default_opener", fake)
        monkeypatch.setattr(models, "visual_mesh_names", lambda urdf: ["a.stl", "b.stl"])
        assert (
            models_main(
                [
                    "fetch",
                    "--dest",
                    str(tmp_path / "d"),
                    "--url",
                    "https://mirror.example/meshes.zip",
                ]
            )
            == 0
        )
        assert seen == ["https://mirror.example/meshes.zip"]
        assert "extracted 2" in capsys.readouterr().out
        assert (tmp_path / "d" / "a.stl").is_file()
