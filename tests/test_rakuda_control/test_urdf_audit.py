"""The URDF auditor, which runs without the optional kinematics extra."""

from __future__ import annotations

import zipfile
from pathlib import Path

import pytest

from robopy.kinematics.synthetic_dual_arm import (
    SYNTHETIC_ARM_JOINTS,
    SYNTHETIC_HEAD_JOINTS,
    SYNTHETIC_TCP_FRAMES,
    SYNTHETIC_TORSO_JOINT,
    write_synthetic_dual_arm_urdf,
)
from robopy.kinematics.urdf_audit import (
    audit_urdf,
    extract_model_archive,
    main,
    resolve_package_path,
)


class TestSyntheticModelAudit:
    def test_topology_matches_the_real_rakuda_export(self, synthetic_urdf: Path) -> None:
        audit = audit_urdf(synthetic_urdf)
        # One shared torso + 6 per arm + 2 head = 15 movable joints, with the
        # same joint-type mix the real export has.
        assert audit.n_movable == 15
        assert audit.joint_type_counts["continuous"] == 3
        assert audit.joint_type_counts["revolute"] == 12
        assert audit.root_links == ["root"]

    def test_arm_joint_names_are_present_in_shoulder_to_wrist_order(
        self, synthetic_urdf: Path
    ) -> None:
        audit = audit_urdf(synthetic_urdf)
        for side in ("left", "right"):
            for name in SYNTHETIC_ARM_JOINTS[side]:
                assert name in audit.movable_joint_names
        assert SYNTHETIC_TORSO_JOINT in audit.movable_joint_names
        for name in SYNTHETIC_HEAD_JOINTS:
            assert name in audit.movable_joint_names

    def test_gripper_frames_are_fixed_and_flagged(self, synthetic_urdf: Path) -> None:
        audit = audit_urdf(synthetic_urdf)
        for frame in SYNTHETIC_TCP_FRAMES.values():
            joint = audit.joint(frame)
            assert joint.joint_type == "fixed"
            assert not joint.is_movable
            assert frame not in audit.movable_joint_names
        assert any("is not a degree of freedom" in w for w in audit.warnings)

    def test_continuous_joints_are_flagged_as_needing_a_real_limit(
        self, synthetic_urdf: Path
    ) -> None:
        audit = audit_urdf(synthetic_urdf)
        warning = next(w for w in audit.warnings if "no URDF range" in w)
        assert SYNTHETIC_TORSO_JOINT in warning

    def test_json_and_summary_both_render(self, synthetic_urdf: Path) -> None:
        audit = audit_urdf(synthetic_urdf)
        assert '"n_movable": 15' in audit.to_json()
        assert "movable DOF : 15" in audit.summary()


class TestInertialPlausibility:
    def test_a_milligram_scale_model_is_rejected_for_dynamics(self, tmp_path: Path) -> None:
        # This is the shape of the real CAD export: correct geometry, a total
        # mass of about 2 mg. The auditor says so instead of it going unnoticed.
        urdf = tmp_path / "tiny.urdf"
        urdf.write_text(
            '<?xml version="1.0"?>\n'
            '<robot name="tiny">\n'
            '  <link name="root">\n'
            "    <inertial>\n"
            '      <mass value="0.0020693"/>\n'
            '      <inertia ixx="1e-9" ixy="0" ixz="0" iyy="1e-9" iyz="0" izz="1e-9"/>\n'
            "    </inertial>\n"
            "  </link>\n"
            "</robot>\n",
            encoding="utf-8",
        )
        audit = audit_urdf(urdf)
        assert not audit.usable_for_dynamics()
        assert any("geometry only" in w for w in audit.warnings)
        assert any("do not edit the masses" in w for w in audit.warnings)

    def test_the_synthetic_fixture_has_plausible_mass(self, synthetic_urdf: Path) -> None:
        assert audit_urdf(synthetic_urdf).usable_for_dynamics()


class TestPlaceholderLimits:
    def test_effort_one_velocity_one_is_flagged_as_an_exporter_placeholder(
        self, tmp_path: Path
    ) -> None:
        urdf = tmp_path / "placeholder.urdf"
        urdf.write_text(
            '<?xml version="1.0"?>\n'
            '<robot name="placeholder">\n'
            '  <link name="root"/>\n'
            '  <link name="a"/>\n'
            '  <joint name="j" type="revolute">\n'
            '    <parent link="root"/><child link="a"/>\n'
            '    <axis xyz="0 0 1"/>\n'
            '    <limit lower="-1" upper="1" effort="1" velocity="1"/>\n'
            "  </joint>\n"
            "</robot>\n",
            encoding="utf-8",
        )
        audit = audit_urdf(urdf)
        assert any("exporter placeholders" in w for w in audit.warnings)

    def test_the_fixture_does_not_use_placeholder_limits(self, synthetic_urdf: Path) -> None:
        audit = audit_urdf(synthetic_urdf)
        assert not any("exporter placeholders" in w for w in audit.warnings)


class TestMeshResolution:
    def test_an_unresolvable_package_uri_is_reported(self, tmp_path: Path) -> None:
        urdf = tmp_path / "meshy.urdf"
        urdf.write_text(
            '<?xml version="1.0"?>\n'
            '<robot name="meshy">\n'
            '  <link name="root">\n'
            "    <visual><geometry>\n"
            '      <mesh filename="package://assembly_2/meshes/base.stl"/>\n'
            "    </geometry></visual>\n"
            "  </link>\n"
            "</robot>\n",
            encoding="utf-8",
        )
        audit = audit_urdf(urdf)
        assert audit.unresolved_meshes == ["package://assembly_2/meshes/base.stl"]
        assert any("--package-dir" in w for w in audit.warnings)

    def test_a_package_uri_resolves_against_a_package_dir(self, tmp_path: Path) -> None:
        package = tmp_path / "pkgs" / "assembly_2" / "meshes"
        package.mkdir(parents=True)
        (package / "base.stl").write_bytes(b"solid\n")
        resolved = resolve_package_path(
            "package://assembly_2/meshes/base.stl", [tmp_path / "pkgs"]
        )
        assert resolved is not None and resolved.exists()


class TestArchiveExtraction:
    def test_extraction_writes_the_members(self, tmp_path: Path) -> None:
        archive = tmp_path / "model.zip"
        with zipfile.ZipFile(archive, "w") as zf:
            zf.writestr("assembly_2/urdf/robot.urdf", "<robot name='x'/>")
        destination = extract_model_archive(archive, tmp_path / "out")
        assert (destination / "assembly_2" / "urdf" / "robot.urdf").exists()

    def test_a_member_escaping_the_destination_is_refused(self, tmp_path: Path) -> None:
        archive = tmp_path / "evil.zip"
        with zipfile.ZipFile(archive, "w") as zf:
            zf.writestr("../escaped.txt", "nope")
        with pytest.raises(ValueError, match="would escape"):
            extract_model_archive(archive, tmp_path / "out")


class TestErrors:
    def test_a_missing_file_raises(self, tmp_path: Path) -> None:
        with pytest.raises(FileNotFoundError):
            audit_urdf(tmp_path / "nope.urdf")

    def test_a_non_urdf_document_raises(self, tmp_path: Path) -> None:
        path = tmp_path / "not.urdf"
        path.write_text("<mujoco/>", encoding="utf-8")
        with pytest.raises(ValueError, match="not a URDF"):
            audit_urdf(path)


class TestCli:
    def test_the_cli_reports_warnings_with_a_non_zero_status(
        self, tmp_path: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        urdf = write_synthetic_dual_arm_urdf(tmp_path / "syn.urdf")
        assert main([str(urdf)]) == 1
        assert "movable DOF : 15" in capsys.readouterr().out

    def test_json_output(self, tmp_path: Path, capsys: pytest.CaptureFixture[str]) -> None:
        urdf = write_synthetic_dual_arm_urdf(tmp_path / "syn.urdf")
        main([str(urdf), "--json"])
        assert '"robot_name": "synthetic_dual_arm"' in capsys.readouterr().out


class TestAmbiguousNames:
    def test_joint_and_link_sharing_a_name_is_flagged(self, synthetic_urdf: Path) -> None:
        audit = audit_urdf(synthetic_urdf)
        assert audit.ambiguous_names == [
            "gripper_left_dof",
            "gripper_right_dof",
            "head_camera_link",
        ]
        assert any("both a joint and a link" in w for w in audit.warnings)

    def test_unique_names_produce_no_flag(self, tmp_path: Path) -> None:
        path = write_synthetic_dual_arm_urdf(tmp_path / "u.urdf", joint_named_child_links=False)
        audit = audit_urdf(path)
        assert audit.ambiguous_names == []
