"""The borrowed hand: that it is bolted on correctly, and that it can hold something.

The gripper is not the Rakuda's own -- the CAD export has no gripper geometry at
all, so the fingers come from CALVIN's ``panda_longer_finger``.  See
:mod:`robopy.sim.panda_gripper`.  What these tests check is that the graft is
sound: the plain robot is untouched, the model compiles into something MetaSim
can drive, and a jaw that closes on a cube actually keeps hold of it when the
arm moves.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from robopy.models import find_rakuda_model

mujoco = pytest.importorskip("mujoco", reason="needs the 'sim' optional extra")

from robopy.roboverse.assets import (  # noqa: E402
    PACKAGED_RAKUDA_GRIPPER_MJCF,
    PACKAGED_RAKUDA_MJCF,
)
from robopy.sim.mjcf_export import RAKUDA_ACTUATED_JOINTS, GRIPPER_NAME  # noqa: E402
from robopy.sim.panda_gripper import (  # noqa: E402
    FINGER_STROKE_M,
    GRIPPER_JOINTS,
    PAD_GAP_CLOSED_M,
    PAD_GAP_OPEN_M,
    find_gripper_meshes,
    gripper_targets,
    pad_gap,
)

REPO_ROOT = Path(__file__).resolve().parents[2]
rakuda = find_rakuda_model(REPO_ROOT / "models")
pytestmark = pytest.mark.skipif(rakuda is None, reason="models/rakuda is not present")

ALL_FINGER_JOINTS = tuple(j for side in ("left", "right") for j in GRIPPER_JOINTS[side])


def _checkout(name: str) -> Path:
    return rakuda.package_dir / "assembly_2" / "mjcf" / name


@pytest.fixture(scope="module")
def model():
    path = _checkout("rakuda_gripper.xml")
    if not path.is_file():
        pytest.skip(f"{path} has not been generated; run python -m robopy.sim.mjcf_export")
    return mujoco.MjModel.from_xml_path(str(path))


class TestTheGraft:
    def test_the_meshes_are_vendored(self) -> None:
        """They ship with the repository, so a checkout needs no CALVIN nearby."""
        meshes = find_gripper_meshes(REPO_ROOT / "models")
        for kind, path in meshes.items():
            assert path.is_file(), kind
        licence = REPO_ROOT / "models" / "gripper_panda" / "LICENSE.txt"
        assert licence.is_file(), "the Apache licence must travel with the meshes"
        assert "Apache License" in licence.read_text()[:200]

    def test_the_plain_robot_is_untouched(self) -> None:
        """Adding a hand must not change the robot that has none."""
        plain = mujoco.MjModel.from_xml_path(str(PACKAGED_RAKUDA_MJCF))
        assert plain.njnt == len(RAKUDA_ACTUATED_JOINTS) == 15
        names = {
            mujoco.mj_id2name(plain, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(plain.njnt)
        }
        assert not any("finger" in n for n in names)

    def test_the_gripper_model_adds_exactly_four_joints(self, model) -> None:
        assert model.njnt == len(RAKUDA_ACTUATED_JOINTS) + 4 == 19
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
        for joint in ALL_FINGER_JOINTS:
            assert joint in names

    def test_every_actuator_is_named_after_its_joint(self, model) -> None:
        """MetaSim drives actuators by joint name; a mismatch is unreachable."""
        actuators = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)
        }
        joints = {
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)
        }
        assert actuators == joints

    def test_the_two_fingers_of_a_hand_are_one_degree_of_freedom(self, model) -> None:
        """The real bus has one gripper motor per arm, not two."""
        assert model.neq == 2, "expected one equality constraint per hand"
        data = mujoco.MjData(model)
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)
        ]
        # Command only one finger of a hand; the constraint should drag the other.
        first, second = GRIPPER_JOINTS["right"]
        data.ctrl[names.index(first)] = FINGER_STROKE_M
        data.ctrl[names.index(second)] = 0.0
        for _ in range(2000):
            mujoco.mj_step(model, data)
        a = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, first)]]
        b = data.qpos[model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, second)]]
        assert abs(a - b) < 1e-3, f"the fingers moved independently: {a:.4f} vs {b:.4f}"


class TestTheJaw:
    def test_it_opens_and_closes_to_the_measured_gaps(self, model) -> None:
        """The advertised 8 to 88 mm, re-measured off the model's own geometry."""
        data = mujoco.MjData(model)
        names = [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)
        ]
        palm = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "right_gripper_base")
        fingers = [
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"right_finger_{s}")
            for s in ("a", "b")
        ]

        def gap() -> float:
            rotation = data.xmat[palm].reshape(3, 3)
            spans = []
            for body in fingers:
                points = []
                for geom in range(model.ngeom):
                    if model.geom_bodyid[geom] != body or model.geom_contype[geom] == 0:
                        continue
                    mesh = model.geom_dataid[geom]
                    start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                    verts = model.mesh_vert[start : start + count].reshape(-1, 3)
                    points.append(
                        verts @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
                    )
                local = (np.vstack(points) - data.xpos[palm]) @ rotation
                spans.append((local[:, 1].min(), local[:, 1].max()))
            return spans[0][0] - spans[1][1]

        for opening, expected in (
            (0.0, PAD_GAP_CLOSED_M),
            (FINGER_STROKE_M, PAD_GAP_OPEN_M),
        ):
            mujoco.mj_resetData(model, data)
            for joint, target in gripper_targets(opening, "right").items():
                data.ctrl[names.index(joint)] = target
            for _ in range(1500):
                mujoco.mj_step(model, data)
            mujoco.mj_forward(model, data)
            assert gap() == pytest.approx(expected, abs=0.002), f"at opening {opening}"

    def test_pad_gap_matches_the_geometry(self) -> None:
        assert pad_gap(0.0) == pytest.approx(PAD_GAP_CLOSED_M)
        assert pad_gap(FINGER_STROKE_M) == pytest.approx(PAD_GAP_OPEN_M, abs=1e-3)

    def test_gripper_targets_refuses_nonsense(self) -> None:
        """Clamping silently would hide a controller asking for the impossible."""
        with pytest.raises(ValueError, match="outside the gripper's travel"):
            gripper_targets(FINGER_STROKE_M + 0.01)
        with pytest.raises(ValueError, match="left"):
            gripper_targets(0.0, "middle")
        assert set(gripper_targets(0.0)) == set(ALL_FINGER_JOINTS)
        assert set(gripper_targets(0.0, "right")) == set(GRIPPER_JOINTS["right"])

    def test_it_holds_a_cube_through_a_wrist_swing(self, tmp_path: Path) -> None:
        """The thing the gripper exists for, end to end.

        A jaw that closes but does not grip is worse than no jaw: a task built on
        it looks solvable and never is. This closes on a 30 mm cube and swings
        the wrist, and fails if the cube is left behind.
        """
        cube = 0.03
        base = PACKAGED_RAKUDA_GRIPPER_MJCF.read_text()
        scene = base.replace(
            "</mujoco>",
            f"""
  <worldbody>
    <geom name="floor" type="plane" size="3 3 0.05" pos="0 0 -1.2"/>
    <body name="cube" pos="0 0 0">
      <freejoint name="cube_free"/>
      <geom name="cube_geom" type="box" size="{cube / 2} {cube / 2} {cube / 2}" mass="0.03"
            friction="1.5 0.02 0.001"/>
    </body>
  </worldbody>
</mujoco>""",
        )
        path = tmp_path / "grasp.xml"
        path.write_text(scene)

        m = mujoco.MjModel.from_xml_path(str(path))
        d = mujoco.MjData(m)
        names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(m.nu)]
        palm = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "right_gripper_base")
        cube_body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube")
        cube_q = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "cube_free")]

        mujoco.mj_resetData(m, d)
        # Start the fingers just around the cube, so it is not dropped before
        # they arrive; the task of *getting* there belongs to a policy.
        around = (cube - PAD_GAP_CLOSED_M) / 2.0 + 0.004
        for joint, _ in gripper_targets(around, "right").items():
            index = names.index(joint)
            d.ctrl[index] = around
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, joint)]] = around
        mujoco.mj_forward(m, d)

        rotation = d.xmat[palm].reshape(3, 3)
        d.qpos[cube_q : cube_q + 3] = d.xpos[palm] + rotation @ np.array([0.0, 0.0, 0.045])
        d.qpos[cube_q + 3 : cube_q + 7] = [1, 0, 0, 0]
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)

        for joint in gripper_targets(0.0, "right"):
            d.ctrl[names.index(joint)] = 0.0
        for _ in range(1500):
            mujoco.mj_step(m, d)
        before = d.xmat[palm].reshape(3, 3).T @ (d.xpos[cube_body] - d.xpos[palm])

        wrist = "wrist_pitch_right_dof"
        index = names.index(wrist)
        address = m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, wrist)]
        low, high = m.actuator_ctrlrange[index]
        d.ctrl[index] = float(np.clip(d.qpos[address] + 0.9, low + 1e-3, high - 1e-3))
        for _ in range(2000):
            mujoco.mj_step(m, d)

        after = d.xmat[palm].reshape(3, 3).T @ (d.xpos[cube_body] - d.xpos[palm])
        drift = float(np.linalg.norm(after - before))
        assert drift < 0.02, (
            f"the cube shifted {drift:.4f} m in the hand while the wrist swung; the grip is "
            "too weak to build a manipulation task on"
        )
        assert d.xpos[cube_body][2] > -1.0, "the cube ended up on the floor"


class TestBothModelsStayInStep:
    def test_the_packaged_gripper_model_is_self_contained(self) -> None:
        assert PACKAGED_RAKUDA_GRIPPER_MJCF.is_file()
        assert 'file="' not in PACKAGED_RAKUDA_GRIPPER_MJCF.read_text(encoding="utf-8")

    def test_the_two_gripper_models_are_the_same_robot(self, model) -> None:
        packaged = mujoco.MjModel.from_xml_path(str(PACKAGED_RAKUDA_GRIPPER_MJCF))
        assert packaged.njnt == model.njnt
        assert packaged.nu == model.nu
        assert packaged.body_mass.sum() == pytest.approx(model.body_mass.sum(), rel=1e-6)

    def test_the_gripper_is_the_only_one_on_offer(self) -> None:
        assert GRIPPER_NAME == "panda_longer_finger"
