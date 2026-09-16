"""CALVIN's play table: the export, the box decomposition, and the scene.

The box decomposition tests need only NumPy and the vendored mesh.  The scene
tests need MetaSim, which a plain robopy checkout does not have.
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

from robopy.sim.calvin_table import (
    CALVIN_SCALE,
    CALVIN_TABLE_JOINTS,
    CALVIN_TABLE_SURFACE,
    CALVIN_WORK_SURFACE_Z,
    decompose_to_boxes,
    find_calvin_table,
)

TABLE = find_calvin_table()
pytestmark = pytest.mark.skipif(TABLE is None, reason="the play table is not vendored here")


def _mesh_bounds(path: Path):
    verts = [
        [float(v) for v in line.split()[1:4]]
        for line in path.read_text().splitlines()
        if line.startswith("v ")
    ]
    verts = np.array(verts)
    return verts.min(axis=0), verts.max(axis=0)


class TestBoxDecomposition:
    """``decompose_to_boxes`` on the bench that needs it."""

    @pytest.fixture(scope="class")
    def boxes(self):
        return decompose_to_boxes(TABLE / "meshes" / "base_link.obj")

    def test_it_produces_a_manageable_number(self, boxes):
        """A few dozen. Hundreds would mean the greedy merge is not merging."""
        assert 10 <= len(boxes) <= 80, f"{len(boxes)} boxes is not a decomposition"

    def test_every_box_has_positive_extent(self, boxes):
        for low, high in boxes:
            assert all(hi > lo for lo, hi in zip(low, high)), f"degenerate box {low} {high}"

    def test_boxes_do_not_overlap(self, boxes):
        """Greedy merging claims cells exclusively, so no two boxes may intersect."""
        for i, (low_a, high_a) in enumerate(boxes):
            for low_b, high_b in boxes[i + 1 :]:
                separated = any(
                    high_a[k] <= low_b[k] + 1e-9 or high_b[k] <= low_a[k] + 1e-9 for k in range(3)
                )
                assert separated, f"{low_a}-{high_a} overlaps {low_b}-{high_b}"

    def test_boxes_stay_inside_the_mesh(self, boxes):
        low_mesh, high_mesh = _mesh_bounds(TABLE / "meshes" / "base_link.obj")
        for low, high in boxes:
            assert np.all(np.array(low) >= low_mesh - 1e-6)
            assert np.all(np.array(high) <= high_mesh + 1e-6)

    def test_it_is_hollow(self, boxes):
        """The point of the exercise: a bench, not the solid block its hull is."""
        low_mesh, high_mesh = _mesh_bounds(TABLE / "meshes" / "base_link.obj")
        filled = sum(float(np.prod(np.array(hi) - np.array(lo))) for lo, hi in boxes)
        envelope = float(np.prod(high_mesh - low_mesh))
        assert filled < 0.4 * envelope, f"{filled:.3f} of {envelope:.3f} m^3 is not furniture"

    def test_the_bench_top_is_where_the_mesh_says(self, boxes):
        """Over CALVIN's drop rectangle, the boxes must reach the work surface."""
        (x0, y0), (x1, y1) = CALVIN_TABLE_SURFACE
        for fx in (0.05, 0.5, 0.95):
            for fy in (0.05, 0.5, 0.95):
                x = (x0 + (x1 - x0) * fx) / CALVIN_SCALE
                y = (y0 + (y1 - y0) * fy) / CALVIN_SCALE
                tops = [hi[2] for lo, hi in boxes if lo[0] <= x <= hi[0] and lo[1] <= y <= hi[1]]
                assert tops, f"nothing under ({x:.3f}, {y:.3f}) to rest a block on"
                assert max(tops) * CALVIN_SCALE == pytest.approx(CALVIN_WORK_SURFACE_Z, abs=1e-3)


class TestExport:
    """The generated MJCF, checked against the model it compiles to."""

    @pytest.fixture(scope="class")
    def model(self):
        mujoco = pytest.importorskip("mujoco")
        mjcf = TABLE / "mjcf" / "calvin_table.xml"
        if not mjcf.is_file():
            pytest.skip("run python -m robopy.sim.calvin_table first")
        return mujoco.MjModel.from_xml_path(str(mjcf))

    def test_the_joints_are_all_there(self, model):
        import mujoco

        names = {mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)}
        assert set(CALVIN_TABLE_JOINTS) <= names

    def test_the_scale_is_baked_in(self, model):
        """1.1 m of bench unscaled, 0.88 at CALVIN's global_scaling.

        Measured in the world, not off ``mesh_vert``: MuJoCo re-frames each mesh
        about its own centre of mass, so the raw vertex array says nothing about
        where a mesh sits or how wide the model is.
        """
        import mujoco

        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
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
        assert (high - low)[0] == pytest.approx(1.1 * CALVIN_SCALE, abs=5e-3)

    def test_the_travel_is_scaled_too(self, model):
        """A drawer that still comes as far out, relative to the bench."""
        import mujoco

        drawer = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "base__drawer")
        assert model.jnt_range[drawer][1] == pytest.approx(0.275 * CALVIN_SCALE, abs=1e-4)

    def test_the_work_surface_is_where_the_constant_says(self, model):
        """Ray-cast onto the collision geometry, which is what a block lands on."""
        import mujoco

        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        (x0, y0), (x1, y1) = CALVIN_TABLE_SURFACE
        collision_only = np.array([1, 0, 0, 0, 0, 0], dtype=np.uint8)
        for fx in (0.1, 0.5, 0.9):
            for fy in (0.1, 0.5, 0.9):
                point = np.array([x0 + (x1 - x0) * fx, y0 + (y1 - y0) * fy, 5.0])
                hit = np.zeros(1, dtype=np.int32)
                drop = mujoco.mj_ray(
                    model, data, point, np.array([0.0, 0.0, -1.0]), collision_only, 1, -1, hit
                )
                assert hit[0] >= 0, f"no collision geometry under {point[:2]}"
                assert 5.0 - drop == pytest.approx(CALVIN_WORK_SURFACE_Z, abs=1e-3)


class TestScene:
    """``rakuda.calvin_table`` as MetaSim instantiates it."""

    @pytest.fixture(scope="class")
    def rolled(self):
        pytest.importorskip("metasim", reason="needs RoboVerse/MetaSim installed")
        pytest.importorskip("mujoco", reason="needs the mujoco backend")
        import torch
        from metasim.task.registry import get_task_class

        cls = get_task_class("rakuda.calvin_table")
        env = cls(scenario=cls.scenario.update(num_envs=1, headless=True), device="cpu")
        robot = env.scenario.robots[0]
        env.reset()
        joints = env.handler.get_joint_names(robot.name, sort=True)
        hold = torch.tensor([[float(robot.default_joint_positions[j]) for j in joints]])
        states = None
        for _ in range(120):
            states, *_ = env.step(hold)
        yield states
        env.close()

    def test_the_robot_stands_at_the_pandas_base(self, rolled):
        from robopy.roboverse.mount import STAND_HEIGHT
        from robopy.roboverse.tasks.rakuda_calvin_table import PANDA_BASE_POSITION

        base = rolled.robots["rakuda"].root_state[0, :3].numpy()
        feet = base[2] - STAND_HEIGHT
        assert base[0] == pytest.approx(PANDA_BASE_POSITION[0], abs=1e-3)
        assert base[1] == pytest.approx(PANDA_BASE_POSITION[1], abs=1e-3)
        assert feet == pytest.approx(PANDA_BASE_POSITION[2], abs=1e-3)

    def test_the_blocks_stay_on_the_table(self, rolled):
        """The failure this scene was built through: blocks flung off the bench."""
        from robopy.roboverse.tasks.rakuda_calvin_table import (
            CALVIN_BLOCKS,
            block_rest_positions,
        )

        wanted = block_rest_positions()
        for name, (size, _) in CALVIN_BLOCKS.items():
            actual = rolled.objects[name].root_state[0, :3].numpy()
            start = np.array(wanted[name])
            assert np.linalg.norm(actual[:2] - start[:2]) < 0.01, f"{name} slid away"
            resting = CALVIN_WORK_SURFACE_Z + size[2] * CALVIN_SCALE / 2
            assert actual[2] == pytest.approx(resting, abs=3e-3), f"{name} is not on the bench"
