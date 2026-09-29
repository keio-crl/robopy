"""The URDF auditor, which runs without the optional kinematics extra."""

from __future__ import annotations

import json
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
        resolved = resolve_package_path("package://assembly_2/meshes/base.stl", [tmp_path / "pkgs"])
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


class TestConsistencyVersusValidation:
    """Two different questions, deliberately given two different answers.

    "Do these numbers describe a coherent machine" is arithmetic, and this
    module can answer it.  "Do they describe *the* machine" is a measurement,
    and nothing here can.  Conflating them is how a model that merely adds up
    ends up authorising current into a motor.
    """

    @staticmethod
    def _model(tmp_path, links: str, joints: str = "") -> Path:
        urdf = tmp_path / "model.urdf"
        urdf.write_text(f"<robot name='t'>{links}{joints}</robot>", encoding="utf-8")
        return urdf

    def test_a_static_root_without_an_inertial_is_not_a_fault(self, tmp_path: Path) -> None:
        """The world attachment carries no mass by definition."""
        urdf = self._model(
            tmp_path,
            links=(
                "<link name='root'/>"
                "<link name='arm'><inertial><mass value='1.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual></link>"
            ),
            joints=(
                "<joint name='j' type='revolute'><parent link='root'/><child link='arm'/>"
                "<axis xyz='0 0 1'/></joint>"
            ),
        )
        audit = audit_urdf(urdf)
        assert audit.links_without_inertial == ["root"]
        assert audit.root_links_without_inertial == ["root"]
        assert audit.massless_moving_parts == []
        assert audit.numerically_consistent(min_plausible_mass_kg=0.5)

    def test_a_moving_part_without_mass_is_a_fault(self, tmp_path: Path) -> None:
        """Geometry hanging off a joint that weighs nothing changes every torque."""
        urdf = self._model(
            tmp_path,
            links=(
                "<link name='root'/>"
                "<link name='arm'><inertial><mass value='1.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual></link>"
                "<link name='hand'><visual><geometry><box size='1 1 1'/></geometry></visual></link>"
            ),
            joints=(
                "<joint name='j' type='revolute'><parent link='root'/><child link='arm'/>"
                "<axis xyz='0 0 1'/></joint>"
                "<joint name='k' type='fixed'><parent link='arm'/><child link='hand'/></joint>"
            ),
        )
        audit = audit_urdf(urdf)
        assert "hand" in audit.massless_moving_links
        assert audit.massless_moving_parts == ["hand"]
        assert not audit.numerically_consistent(min_plausible_mass_kg=0.5)

    def test_a_frame_without_geometry_is_not_a_part(self, tmp_path: Path) -> None:
        """The same link, minus its geometry, is a coordinate frame and fine."""
        urdf = self._model(
            tmp_path,
            links=(
                "<link name='root'/>"
                "<link name='arm'><inertial><mass value='1.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual></link>"
                "<link name='tcp'/>"
            ),
            joints=(
                "<joint name='j' type='revolute'><parent link='root'/><child link='arm'/>"
                "<axis xyz='0 0 1'/></joint>"
                "<joint name='k' type='fixed'><parent link='arm'/><child link='tcp'/></joint>"
            ),
        )
        audit = audit_urdf(urdf)
        assert "tcp" in audit.massless_moving_links
        assert audit.massless_moving_parts == []
        assert audit.numerically_consistent(min_plausible_mass_kg=0.5)

    def test_consistent_arithmetic_never_implies_a_measured_machine(self, tmp_path: Path) -> None:
        urdf = self._model(
            tmp_path,
            links=(
                "<link name='root'/>"
                "<link name='arm'><inertial><mass value='9.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual></link>"
            ),
            joints=(
                "<joint name='j' type='revolute'><parent link='root'/><child link='arm'/>"
                "<axis xyz='0 0 1'/></joint>"
            ),
        )
        audit = audit_urdf(urdf)
        assert audit.numerically_consistent()
        assert audit.hardware_validated is False
        assert any("not been checked against the actual machine" in w for w in audit.warnings)

    def test_subtree_masses_do_not_charge_a_joint_for_the_base(self, tmp_path: Path) -> None:
        """A heavy fixed base must not appear as load on the arm.

        The Rakuda's base plate is most of its mass; adding it to what the
        torso holds up would overstate every gravity term computed from it.
        """
        urdf = self._model(
            tmp_path,
            links=(
                "<link name='root'/>"
                "<link name='base'><inertial><mass value='5.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial></link>"
                "<link name='arm'><inertial><mass value='2.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial></link>"
            ),
            joints=(
                "<joint name='fixed_base' type='fixed'><parent link='root'/><child link='base'/></joint>"
                "<joint name='shoulder' type='revolute'><parent link='base'/><child link='arm'/>"
                "<axis xyz='0 0 1'/></joint>"
            ),
        )
        audit = audit_urdf(urdf)
        assert audit.total_mass_kg == pytest.approx(7.0)
        assert audit.movable_subtree_masses_kg == {"shoulder": pytest.approx(2.0)}

    def test_an_impossible_inertia_is_caught(self, tmp_path: Path) -> None:
        """Principal moments must obey the triangle inequality."""
        urdf = self._model(
            tmp_path,
            links=(
                "<link name='root'/>"
                "<link name='arm'><inertial><mass value='1.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='9'/></inertial>"
                "<visual><geometry><box size='1 1 1'/></geometry></visual></link>"
            ),
            joints=(
                "<joint name='j' type='revolute'><parent link='root'/><child link='arm'/>"
                "<axis xyz='0 0 1'/></joint>"
            ),
        )
        audit = audit_urdf(urdf)
        assert audit.invalid_inertias and "triangle inequality" in audit.invalid_inertias[0]
        assert not audit.numerically_consistent()

    def test_the_old_name_still_answers(self, tmp_path: Path) -> None:
        """`usable_for_dynamics` is kept so existing callers and the recorded
        audit JSON keep working."""
        urdf = self._model(
            tmp_path,
            links=(
                "<link name='root'/>"
                "<link name='arm'><inertial><mass value='9.0'/>"
                "<inertia ixx='1' ixy='0' ixz='0' iyy='1' iyz='0' izz='1'/></inertial></link>"
            ),
            joints=(
                "<joint name='j' type='revolute'><parent link='root'/><child link='arm'/>"
                "<axis xyz='0 0 1'/></joint>"
            ),
        )
        audit = audit_urdf(urdf)
        assert audit.usable_for_dynamics() == audit.numerically_consistent()
        payload = json.loads(audit.to_json())
        assert payload["usable_for_dynamics"] == payload["numerically_consistent"]


class TestPrincipalMoments:
    """The pure-Python eigensolver, against the one everybody trusts."""

    def test_it_agrees_with_numpy(self) -> None:
        import numpy as np

        from robopy.kinematics.urdf_audit import _principal_moments

        rng = np.random.default_rng(0)
        for _ in range(200):
            a = rng.normal(size=(3, 3))
            tensor = a @ a.T  # symmetric positive semi-definite
            mine = _principal_moments(
                tensor[0, 0], tensor[0, 1], tensor[0, 2], tensor[1, 1], tensor[1, 2], tensor[2, 2]
            )
            assert np.allclose(sorted(mine), sorted(np.linalg.eigvalsh(tensor)), atol=1e-9)

    def test_a_sphere_has_three_equal_moments(self) -> None:
        from robopy.kinematics.urdf_audit import _principal_moments

        assert _principal_moments(2.0, 0.0, 0.0, 2.0, 0.0, 2.0) == pytest.approx((2.0, 2.0, 2.0))
