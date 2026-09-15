"""The Rakuda as RoboVerse sees it: discovery, the robot config, and the tasks.

Skipped wholesale when MetaSim is not installed, which is the normal state of a
robopy checkout -- RoboVerse brings its own environment.  Install into it with::

    pip install -e ".[roboverse]"
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pytest

pytest.importorskip("metasim", reason="needs RoboVerse/MetaSim installed")
pytest.importorskip("mujoco", reason="needs the mujoco backend")

import torch  # noqa: E402
from metasim.task.registry import get_task_class  # noqa: E402
from metasim.utils.setup_util import get_robot  # noqa: E402

from robopy.models import find_rakuda_model  # noqa: E402
from robopy.roboverse.robots import RAKUDA_ARM_JOINTS, RAKUDA_STAND_HEIGHT_M, RakudaCfg  # noqa: E402
from robopy.roboverse.tasks._common import (  # noqa: E402
    PEDESTAL_FOOTPRINT,
    REACH_TARGET_BOX,
    STAND_HEIGHT,
    WORK_SURFACE_TOP,
    hand_position,
)
from robopy.sim.mjcf_export import RAKUDA_ACTUATED_JOINTS  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
TASK_NAMES = ("rakuda.reach", "rakuda.bimanual_reach", "rakuda.push_cube")


class TestDiscovery:
    def test_installing_robopy_is_enough_to_find_the_robot(self) -> None:
        """The entry point is the whole install step; without it nothing else works."""
        from importlib.metadata import entry_points

        roots = {ep.value for ep in entry_points().select(group="metasim.packages")}
        assert "robopy.roboverse" in roots, (
            "robopy does not advertise its MetaSim content pack; reinstall it so the "
            "'metasim.packages' entry point is registered"
        )

    def test_get_robot_resolves_the_name(self) -> None:
        robot = get_robot("rakuda")
        assert isinstance(robot, RakudaCfg)
        assert robot.name == "rakuda"
        assert robot.num_joints == len(RAKUDA_ACTUATED_JOINTS) == 15

    @pytest.mark.parametrize("name", TASK_NAMES)
    def test_task_names_resolve(self, name: str) -> None:
        assert get_task_class(name) is not None


class TestTheRobotConfig:
    @pytest.fixture(scope="class")
    @classmethod
    def robot(cls) -> RakudaCfg:
        return get_robot("rakuda")

    def test_the_asset_exists(self, robot: RakudaCfg) -> None:
        assert Path(robot.mjcf_path).is_file()
        assert robot.asset_source

    def test_every_joint_has_an_actuator_a_limit_and_a_control_mode(self, robot: RakudaCfg) -> None:
        for joint in RAKUDA_ACTUATED_JOINTS:
            assert joint in robot.actuators, joint
            assert joint in robot.joint_limits, joint
            assert robot.control_type[joint] == "position"
            assert robot.actuators[joint].effort_limit_sim in (4.1, 7.3)

    def test_the_arm_joint_lists_cover_the_actuated_set(self, robot: RakudaCfg) -> None:
        """The arm lists are what a Cartesian controller drives; a typo here is silent."""
        listed = set(RAKUDA_ARM_JOINTS["left"]) | set(RAKUDA_ARM_JOINTS["right"])
        listed |= {"torso_yaw_dof", "head_yaw_dof", "head_pitch_dof"}
        assert listed == set(RAKUDA_ACTUATED_JOINTS)

    def test_the_home_pose_is_strictly_inside_every_limit(self, robot: RakudaCfg) -> None:
        """The CAD zero sits *on* elbow_pitch_right's upper limit; the home pose must not."""
        for joint, angle in robot.default_joint_positions.items():
            low, high = robot.joint_limits[joint]
            assert low < angle < high, f"{joint}={angle} is not inside ({low}, {high})"

    def test_self_collision_policy_is_the_sentinel(self, robot: RakudaCfg) -> None:
        """``True`` would make MetaSim disable filterparent and jam the robot solid."""
        assert robot.enabled_self_collisions == "mujoco_default"

    def test_the_stand_height_is_what_the_model_needs(self, robot: RakudaCfg) -> None:
        """Re-derived from the model, so a re-export cannot leave it stale."""
        import mujoco

        model = mujoco.MjModel.from_xml_path(robot.mjcf_path)
        data = mujoco.MjData(model)
        mujoco.mj_kinematics(model, data)
        lowest = np.inf
        for geom in range(model.ngeom):
            mesh = model.geom_dataid[geom]
            if mesh < 0:
                continue
            start, count = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
            vertices = model.mesh_vert[start : start + count].reshape(-1, 3)
            world = vertices @ data.geom_xmat[geom].reshape(3, 3).T + data.geom_xpos[geom]
            lowest = min(lowest, float(world[:, 2].min()))
        assert -lowest == pytest.approx(RAKUDA_STAND_HEIGHT_M, abs=1e-4)
        assert robot.default_position == (0.0, 0.0, RAKUDA_STAND_HEIGHT_M)


class TestTheWorkspaceFactsTheTasksRelyOn:
    """Each constant in ``tasks._common`` is re-measured here.

    The tasks are only solvable because their targets sit inside the arm's
    reachable set.  If a re-export changes the geometry, these fail rather than
    the tasks quietly becoming impossible.
    """

    @pytest.fixture(scope="class")
    @classmethod
    def reach_cloud(cls):
        import mujoco

        robot = get_robot("rakuda")
        model = mujoco.MjModel.from_xml_path(robot.mjcf_path)
        data = mujoco.MjData(model)
        names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)]
        low, high = model.jnt_range[:, 0].copy(), model.jnt_range[:, 1].copy()
        for frozen in ("head_yaw_dof", "head_pitch_dof"):
            index = names.index(frozen)
            low[index] = high[index] = 0.0

        generator = np.random.default_rng(7)
        hands = {
            side: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"gripper_{side}_dof")
            for side in ("left", "right")
        }
        samples = 8000
        cloud = {side: np.empty((samples, 3)) for side in hands}
        for i in range(samples):
            data.qpos[:] = generator.uniform(low, high)
            mujoco.mj_kinematics(model, data)
            for side, body in hands.items():
                cloud[side][i] = data.xpos[body]
        for side in cloud:
            cloud[side][:, 2] += STAND_HEIGHT
        return cloud

    @pytest.mark.parametrize("hand", ["left", "right"])
    def test_the_reach_targets_are_reachable(self, reach_cloud, hand: str) -> None:
        box = REACH_TARGET_BOX[hand]
        points = reach_cloud[hand]
        inside = np.ones(len(points), dtype=bool)
        for axis, index in zip("xyz", range(3)):
            low, high = box[axis]
            inside &= (points[:, index] >= low) & (points[:, index] <= high)
        fraction = inside.mean()
        assert fraction > 0.01, (
            f"only {fraction:.2%} of random poses put the {hand} hand in its target box; "
            "the reach task would be asking for somewhere the arm cannot go"
        )

    def test_the_robot_cannot_reach_its_own_standing_surface(self, reach_cloud) -> None:
        """The fact that forces the object tasks onto a raised table."""
        lowest = min(reach_cloud[side][:, 2].min() for side in reach_cloud)
        assert lowest > 0.10, f"a hand got to z={lowest:.3f}; the table height can be reconsidered"

    def test_the_work_surface_is_reachable_and_clear_of_the_pedestal(self, reach_cloud) -> None:
        for side, points in reach_cloud.items():
            band = points[
                (points[:, 2] > WORK_SURFACE_TOP + 0.01) & (points[:, 2] < WORK_SURFACE_TOP + 0.10)
            ]
            beyond = band[band[:, 0] > PEDESTAL_FOOTPRINT["x"][1] + 0.01]
            assert len(beyond) > 50, (
                f"the {side} hand can barely reach past the pedestal onto the table: "
                f"{len(beyond)} of {len(points)} poses"
            )

    def test_the_push_cube_start_is_over_the_table_and_clear_of_the_pedestal(self) -> None:
        from robopy.roboverse.tasks.rakuda_push_cube import (
            CUBE_SIZE,
            CUBE_START_X,
            GOAL_OFFSET_Y,
            TABLE_CENTRE,
            TABLE_SIZE,
        )

        near_edge = TABLE_CENTRE[0] - TABLE_SIZE[0] / 2.0
        far_edge = TABLE_CENTRE[0] + TABLE_SIZE[0] / 2.0
        assert near_edge > PEDESTAL_FOOTPRINT["x"][1], "the table overlaps the robot's pedestal"
        assert near_edge <= CUBE_START_X[0] and CUBE_START_X[1] <= far_edge
        # the goal must stay on the table too
        half_y = TABLE_SIZE[1] / 2.0
        assert 0.10 + GOAL_OFFSET_Y <= half_y, "the goal falls off the edge of the table"
        assert CUBE_SIZE < 0.1


class TestTheTasksRun:
    @pytest.mark.parametrize("name", TASK_NAMES)
    def test_reset_and_step(self, name: str) -> None:
        task_class = get_task_class(name)
        scenario = task_class.scenario.update(num_envs=1, headless=True)
        env = task_class(scenario=scenario, device="cpu")
        try:
            states, _ = env.reset(seed=0)
            joints = env.handler.get_joint_names("rakuda", sort=True)
            assert len(joints) == 15

            action = torch.zeros(1, len(joints))
            for i, joint in enumerate(joints):
                low, high = scenario.robots[0].joint_limits[joint]
                action[0, i] = min(max(0.0, low + 0.05), high - 0.05)

            for _ in range(10):
                states, reward, terminated, timeout, _ = env.step(action)
            assert torch.isfinite(reward).all()
            assert torch.isfinite(states.robots["rakuda"].joint_pos).all()
            assert not terminated.any(), "the task reports success before anything has moved"
        finally:
            env.close()

    def test_reach_reward_improves_as_the_hand_approaches(self) -> None:
        """A reward that does not respond to the thing it measures teaches nothing."""
        task_class = get_task_class("rakuda.reach")
        env = task_class(scenario=task_class.scenario.update(num_envs=1, headless=True), device="cpu")
        try:
            states, _ = env.reset(seed=3)
            target = env.targets["right"][0]
            start = hand_position(states, "right")[0]
            assert float(torch.linalg.norm(start - target)) > 0.05

            far = env._reward(states).clone()
            # Move the goal onto the hand: same states, strictly smaller distance.
            env._targets["right"] = start.unsqueeze(0).clone()
            near = env._reward(states)
            assert float(near) > float(far)
        finally:
            env.close()

    def test_episodes_are_seeded_reproducibly(self) -> None:
        task_class = get_task_class("rakuda.reach")
        first = []
        for _ in range(2):
            env = task_class(
                scenario=task_class.scenario.update(num_envs=1, headless=True), device="cpu"
            )
            try:
                env.reset(seed=11)
                first.append(env.targets["right"].clone())
            finally:
                env.close()
        assert torch.allclose(first[0], first[1])
