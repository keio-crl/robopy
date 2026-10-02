"""Moving inertials between CAD exports, and what must survive the move.

The import is a textual edit of three generated files, which is the sort of
thing that works on the example you tried it on and quietly mangles the rest.
These tests are the reason to believe it did not.
"""

from __future__ import annotations

import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict

import pytest

REPO_ROOT = Path(__file__).resolve().parents[2]
URDF_DIR = REPO_ROOT / "src" / "robopy" / "models" / "rakuda" / "assembly_2" / "urdf"
MODELS = ("assembly_2.urdf", "assembly_2_convex_collision.urdf", "assembly_2_mesh_collision.urdf")

#: The figure the re-export was meant to produce, from the model with densities
#: set.  Hard-coded so a later import that quietly changes the machine's weight
#: has to be argued for rather than absorbed.
EXPECTED_TOTAL_MASS_KG = 7.8828710378


def _load_script():
    """Import ``scripts/update_rakuda_inertials.py``, which is not a package."""
    path = REPO_ROOT / "scripts" / "update_rakuda_inertials.py"
    if not path.is_file():
        pytest.skip("the import script is not present in this checkout")
    spec = importlib.util.spec_from_file_location("update_rakuda_inertials", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


script = _load_script()

pytestmark = pytest.mark.skipif(
    not (URDF_DIR / MODELS[0]).is_file(), reason="the bundled Rakuda model is not present"
)


def _inertials(path: Path) -> Dict[str, Dict[str, str]]:
    """Every link's inertial numbers, parsed, keyed by link name."""
    out: Dict[str, Dict[str, str]] = {}
    for link in ET.parse(path).getroot().findall("link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        entry: Dict[str, str] = {}
        mass = inertial.find("mass")
        if mass is not None:
            entry["mass"] = mass.get("value", "")
        origin = inertial.find("origin")
        if origin is not None:
            entry["xyz"] = origin.get("xyz", "")
            entry["rpy"] = origin.get("rpy", "")
        tensor = inertial.find("inertia")
        if tensor is not None:
            entry.update({k: tensor.get(k, "") for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")})
        out[link.get("name", "")] = entry
    return out


class TestTheCommittedModels:
    """What the three files must agree about, however they were produced."""

    def test_all_three_carry_the_same_inertials(self) -> None:
        """The collision variants differ from the plain model in collision
        geometry alone.  A dynamics result must not depend on which one was
        loaded."""
        reference = _inertials(URDF_DIR / MODELS[0])
        for name in MODELS[1:]:
            assert _inertials(URDF_DIR / name) == reference, name

    def test_the_total_mass_is_the_re_exported_one(self) -> None:
        for name in MODELS:
            root = ET.parse(URDF_DIR / name).getroot()
            total = sum(
                float(link.find("inertial/mass").get("value"))  # type: ignore[union-attr]
                for link in root.findall("link")
                if link.find("inertial/mass") is not None
            )
            assert total == pytest.approx(EXPECTED_TOTAL_MASS_KG, abs=1e-9), name

    def test_the_collision_variants_still_have_their_collision_geometry(self) -> None:
        """The failure mode the import exists to avoid: replacing three files
        with copies of an export that has no collision geometry at all."""
        counts = {
            name: sum(
                len(link.findall("collision"))
                for link in ET.parse(URDF_DIR / name).getroot().findall("link")
            )
            for name in MODELS
        }
        assert counts[MODELS[0]] == 0
        assert counts[MODELS[1]] == 137
        assert counts[MODELS[2]] == 137

    def test_every_inertia_describes_a_rigid_body(self) -> None:
        from robopy.kinematics.urdf_audit import _inertia_problem

        for name in MODELS:
            for link in ET.parse(URDF_DIR / name).getroot().findall("link"):
                tensor = link.find("inertial/inertia")
                if tensor is None:
                    continue
                values = {
                    k: float(tensor.get(k, "0")) for k in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
                }
                assert _inertia_problem(link.get("name", ""), values) is None


class TestTheImport:
    """The script, run against a copy."""

    @pytest.fixture
    def sandbox(self, tmp_path: Path) -> Path:
        for name in MODELS:
            (tmp_path / name).write_text(
                (URDF_DIR / name).read_text(encoding="utf-8"), encoding="utf-8"
            )
        return tmp_path

    def test_re_importing_changes_nothing(self, sandbox: Path) -> None:
        """Idempotence.  The committed files already hold these numbers, so a
        second import must be a no-op rather than drifting the formatting."""
        text, _ = script.read_source(URDF_DIR / MODELS[0])
        source_root = ET.fromstring(text)
        wanted = script.inertials_by_link(text)
        for name in MODELS:
            before = (sandbox / name).read_text(encoding="utf-8")
            report = script.update_file(sandbox / name, source_root, wanted, dry_run=False)
            assert report.changed_links == []
            assert (sandbox / name).read_text(encoding="utf-8") == before

    def test_a_different_machine_is_refused(self, sandbox: Path) -> None:
        """A source whose joints do not match must not be merged in."""
        text, _ = script.read_source(URDF_DIR / MODELS[0])
        source_root = ET.fromstring(text)
        moved = source_root.find("joint")
        assert moved is not None
        origin = moved.find("origin")
        assert origin is not None
        origin.set("xyz", "9 9 9")

        with pytest.raises(ValueError, match="does not match the source export"):
            script.update_file(
                sandbox / MODELS[0], source_root, script.inertials_by_link(text), dry_run=True
            )

    def test_a_dry_run_writes_nothing(self, sandbox: Path) -> None:
        text, _ = script.read_source(URDF_DIR / MODELS[0])
        source_root = ET.fromstring(text)
        # Halve every mass so the import would be a visible change.
        for link in source_root.findall("link"):
            mass = link.find("inertial/mass")
            if mass is not None:
                mass.set("value", repr(float(mass.get("value", "0")) / 2.0))
        altered = ET.tostring(source_root, encoding="unicode")
        wanted = script.inertials_by_link(altered)

        before = (sandbox / MODELS[0]).read_text(encoding="utf-8")
        report = script.update_file(sandbox / MODELS[0], source_root, wanted, dry_run=True)
        assert report.written is False
        assert report.mass_after_kg == pytest.approx(EXPECTED_TOTAL_MASS_KG / 2.0, rel=1e-9)
        assert (sandbox / MODELS[0]).read_text(encoding="utf-8") == before

    def test_the_zip_member_is_read_by_name(self, tmp_path: Path) -> None:
        """Only the one URDF is taken out of an archive, not the whole tree."""
        import zipfile

        archive = tmp_path / "model.zip"
        payload = (URDF_DIR / MODELS[0]).read_bytes()
        with zipfile.ZipFile(archive, "w") as zf:
            zf.writestr(script.SOURCE_MEMBER, payload)
            zf.writestr("elsewhere/evil.urdf", b"<robot name='no'/>")

        text, digest = script.read_source(archive)
        assert "assembly_2" in text
        assert len(digest) == 64

    def test_a_zip_without_the_member_says_so(self, tmp_path: Path) -> None:
        import zipfile

        archive = tmp_path / "empty.zip"
        with zipfile.ZipFile(archive, "w") as zf:
            zf.writestr("something/else.txt", b"")
        with pytest.raises(FileNotFoundError, match="has no member"):
            script.read_source(archive)


class TestKinematicsAreUnaffected:
    """Inertias are dynamics; the arm reaches the same places either way."""

    def test_masses_do_not_move_the_tool_or_its_jacobian(self, tmp_path: Path) -> None:
        """A re-weighed model must give the same FK and Jacobian at the same q.

        This is what makes an inertial import safe to do without re-checking
        every IK result: mass does not enter either computation.  Built by
        re-weighing rather than by keeping the old file, so the guard survives
        the old file being gone.
        """
        pytest.importorskip("pinocchio", reason="needs the kinematics extra")
        import numpy as np

        from robopy.kinematics.urdf_model import WholeBodyModel

        original = URDF_DIR / MODELS[0]
        root = ET.parse(original).getroot()
        for link in root.findall("link"):
            mass = link.find("inertial/mass")
            if mass is not None:
                mass.set("value", repr(float(mass.get("value", "0")) * 3.0 + 0.01))
        heavier = tmp_path / "heavier.urdf"
        heavier.write_bytes(ET.tostring(root))

        package_dirs = [original.parent.parent.parent]
        a = WholeBodyModel.from_urdf(original, package_dirs=package_dirs, geometry_only=True)
        b = WholeBodyModel.from_urdf(heavier, package_dirs=package_dirs, geometry_only=True)
        assert a.nq == b.nq

        rng = np.random.default_rng(0)
        frames = [f for f in a.frame_names if f.endswith("_dof") and "gripper" in f]
        assert frames, "the gripper frames should exist in the committed model"
        for _ in range(4):
            q = rng.uniform(-0.8, 0.8, size=a.nq)
            for frame in frames:
                assert np.allclose(a.frame_pose(q, frame), b.frame_pose(q, frame), atol=0.0)
                assert np.allclose(a.frame_jacobian(q, frame), b.frame_jacobian(q, frame), atol=0.0)
