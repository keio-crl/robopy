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
from robopy.roboverse.mount import (  # noqa: E402
    HAND_FLOOR_ABOVE_MOUNT,
    WORK_OFFSET_ABOVE_MOUNT,
    RakudaMount,
)
from robopy.roboverse.tasks._common import (  # noqa: E402
    OBJECT_ZONE,
    PEDESTAL_FOOTPRINT,
    REACH_TARGET_BOX,
    STAND_HEIGHT,
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
        # Heights relative to the mounting plane: that is the frame the task
        # constants are written in, and it is independent of the pedestal.
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

    def test_the_robot_cannot_reach_the_surface_it_stands_on(self, reach_cloud) -> None:
        """The fact the whole pedestal arrangement exists to work around.

        The hands stop about 11 cm above the mounting plane, so a Rakuda bolted
        to a table waves above everything on it. If a re-export ever makes the
        arms longer this stops being true, and the mount can be reconsidered.
        """
        lowest = min(reach_cloud[side][:, 2].min() for side in reach_cloud)
        assert lowest == pytest.approx(HAND_FLOOR_ABOVE_MOUNT, abs=0.01), (
            f"a hand reaches {lowest:.4f} m above the mounting plane, not "
            f"{HAND_FLOOR_ABOVE_MOUNT:.4f}"
        )

    def test_the_chosen_work_offset_is_near_the_best_available(self, reach_cloud) -> None:
        """The offset is supposed to be measured, so re-measure it.

        Counts how often a hand lands in the band just above a work surface, as
        a function of how far that surface is above the mounting plane, and
        checks the configured offset is within a hair of the best.
        """

        def coverage(offset: float) -> float:
            hits = 0
            for points in reach_cloud.values():
                band = (points[:, 2] > offset + 0.005) & (points[:, 2] < offset + 0.085)
                hits += int(band.sum())
            return hits / sum(len(p) for p in reach_cloud.values())

        scores = {off: coverage(off) for off in (0.12, 0.18, 0.22, 0.30, 0.34, 0.40, 0.50)}
        best = max(scores, key=scores.get)
        assert scores[WORK_OFFSET_ABOVE_MOUNT] > 0.8 * scores[best], (
            f"the configured offset {WORK_OFFSET_ABOVE_MOUNT} reaches "
            f"{scores[WORK_OFFSET_ABOVE_MOUNT]:.3f} of poses but {best} reaches "
            f"{scores[best]:.3f}; re-derive it"
        )
        assert scores[0.12] < scores[WORK_OFFSET_ABOVE_MOUNT], "low surfaces should be worse"

    def test_an_object_in_the_zone_is_reachable_and_clear_of_the_pedestal(
        self, reach_cloud
    ) -> None:
        """The cube has to be somewhere a hand can get to, and off the pedestal."""
        assert OBJECT_ZONE["x"][0] > PEDESTAL_FOOTPRINT["x"][1], (
            "the object zone overlaps the robot's own base plate"
        )
        surface = WORK_OFFSET_ABOVE_MOUNT
        for side, points in reach_cloud.items():
            inside = (
                (points[:, 2] > surface)
                & (points[:, 2] < surface + 0.12)
                & (points[:, 0] >= OBJECT_ZONE["x"][0])
                & (points[:, 0] <= OBJECT_ZONE["x"][1])
                & (points[:, 1] >= OBJECT_ZONE["y"][0])
                & (points[:, 1] <= OBJECT_ZONE["y"][1])
            )
            assert inside.mean() > 0.002, (
                f"the {side} hand lands in the object zone in only {inside.mean():.3%} of poses"
            )

    def test_the_push_cube_table_clears_the_pedestal_and_holds_the_goal(self) -> None:
        from robopy.roboverse.tasks.rakuda_push_cube import (
            CUBE_SIZE,
            CUBE_START_X,
            GOAL_OFFSET_Y,
            MOUNT,
            TABLE_DEPTH,
            TABLE_WIDTH,
        )

        near_edge = MOUNT.near_edge_x
        far_edge = near_edge + TABLE_DEPTH
        assert near_edge > PEDESTAL_FOOTPRINT["x"][1], "the table overlaps the robot's pedestal"
        assert near_edge <= CUBE_START_X[0] and CUBE_START_X[1] <= far_edge
        # the goal must stay on the table too
        assert 0.10 + GOAL_OFFSET_Y <= TABLE_WIDTH / 2.0, "the goal falls off the edge of the table"
        assert CUBE_SIZE < 0.1


class TestTheMount:
    """The pedestal: derived heights, and the reachability they are for."""

    def test_the_default_offset_is_above_the_hand_floor(self) -> None:
        assert WORK_OFFSET_ABOVE_MOUNT > HAND_FLOOR_ABOVE_MOUNT

    def test_the_heights_follow_from_the_work_surface(self) -> None:
        mount = RakudaMount(work_surface_z=0.75)
        assert mount.top_z == pytest.approx(0.75 - WORK_OFFSET_ABOVE_MOUNT)
        assert mount.base_position[2] == pytest.approx(mount.top_z + STAND_HEIGHT)
        assert mount.base_position[:2] == (0.0, 0.0)

        # Move the work surface and the robot moves with it, by the same amount.
        higher = RakudaMount(work_surface_z=0.95)
        assert higher.base_position[2] - mount.base_position[2] == pytest.approx(0.20)

    def test_a_surface_the_hands_cannot_reach_is_refused(self) -> None:
        """Silently building an impossible scene is the failure worth preventing."""
        with pytest.raises(ValueError, match="unreachable"):
            RakudaMount(work_surface_z=0.75, offset=HAND_FLOOR_ABOVE_MOUNT - 0.01)
        with pytest.raises(ValueError, match="below the floor"):
            RakudaMount(work_surface_z=0.10)

    def test_the_pedestal_supports_the_whole_base_plate(self) -> None:
        mount = RakudaMount()
        size = mount.pedestal_size
        centre = mount.pedestal_position()
        for axis, index in (("x", 0), ("y", 1)):
            low = centre[index] - size[index] / 2.0
            high = centre[index] + size[index] / 2.0
            plate_low, plate_high = PEDESTAL_FOOTPRINT[axis]
            assert low <= plate_low and high >= plate_high, (
                f"the pedestal does not cover the base plate in {axis}"
            )

    def test_the_pedestal_rests_on_the_floor_and_does_not_touch_the_robot(self) -> None:
        mount = RakudaMount()
        size = mount.pedestal_size
        centre = mount.pedestal_position()
        assert centre[2] - size[2] / 2.0 == pytest.approx(0.0), "the pedestal floats or sinks"
        top = centre[2] + size[2] / 2.0
        gap = mount.top_z - top
        assert 0.0 < gap < 0.005, f"pedestal top is {gap:.4f} m from the robot; want a small gap"

    def test_the_table_starts_clear_of_the_pedestal(self) -> None:
        mount = RakudaMount()
        depth = 0.34
        centre = mount.table_position(depth)
        assert centre[0] - depth / 2.0 == pytest.approx(mount.near_edge_x)
        assert mount.near_edge_x > PEDESTAL_FOOTPRINT["x"][1]
        assert centre[2] == pytest.approx(mount.work_surface_z / 2.0)

    def test_reachable_agrees_with_the_hand_floor(self) -> None:
        mount = RakudaMount()
        assert mount.reachable(mount.work_surface_z)
        assert not mount.reachable(mount.top_z + HAND_FLOOR_ABOVE_MOUNT - 0.01)


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

class TestTheMountedScene:
    """The pedestal doing its job, in a task's actual scene."""

    def test_the_robot_stands_on_the_pedestal_touching_nothing(self) -> None:
        """A scene that starts in contact with itself is a scene with a bug in it."""
        import mujoco

        task_class = get_task_class("rakuda.push_cube")
        env = task_class(
            scenario=task_class.scenario.update(num_envs=1, headless=True), device="cpu"
        )
        try:
            states, _ = env.reset(seed=0)
            mount = env.mount

            names = list(states.robots["rakuda"].body_names)
            base = states.robots["rakuda"].body_state[0, names.index("base"), :3]
            assert float(base[2]) == pytest.approx(mount.base_position[2], abs=1e-4)

            model = env.handler.physics.model.ptr
            data = env.handler.physics.data.ptr
            touching = set()
            for index in range(data.ncon):
                contact = data.contact[index]
                pair = tuple(
                    mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom])
                    for geom in (contact.geom1, contact.geom2)
                )
                touching.add(pair)
            robot_contacts = [p for p in touching if any("rakuda/" in name for name in p)]
            assert not robot_contacts, f"the robot starts in contact: {robot_contacts}"
        finally:
            env.close()

    def test_a_hand_can_reach_the_cube_on_the_table(self) -> None:
        """The whole point of the pedestal, checked end to end.

        Searches the joint space for a configuration that puts the hand on the
        cube. Without the pedestal the cube would sit below the hands' floor and
        no configuration would exist.
        """
        import mujoco

        task_class = get_task_class("rakuda.push_cube")
        env = task_class(
            scenario=task_class.scenario.update(num_envs=1, headless=True), device="cpu"
        )
        try:
            states, _ = env.reset(seed=0)
            cube = states.objects["cube"].root_state[0, :3].numpy()
            assert env.mount.reachable(float(cube[2])), "the cube is below the hands' floor"

            model = env.handler.physics.model.ptr
            data = env.handler.physics.data.ptr
            prefix = env.handler._mujoco_robot_names[0]
            joints = env.handler.get_joint_names("rakuda", sort=True)
            ids = [
                mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, prefix + name)
                for name in joints
            ]
            address = [model.jnt_qposadr[i] for i in ids]
            low = np.array([model.jnt_range[i][0] for i in ids])
            high = np.array([model.jnt_range[i][1] for i in ids])
            hand = mujoco.mj_name2id(
                model, mujoco.mjtObj.mjOBJ_BODY, prefix + "gripper_right_dof"
            )

            def evaluate(q):
                """Distance from the hand to the cube, and any collision that is not it."""
                for a, v in zip(address, q):
                    data.qpos[a] = v
                mujoco.mj_forward(model, data)
                reach = float(np.linalg.norm(data.xpos[hand] - cube))
                blocked = 0
                for index in range(data.ncon):
                    contact = data.contact[index]
                    bodies = [
                        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, model.geom_bodyid[geom])
                        or ""
                        for geom in (contact.geom1, contact.geom2)
                    ]
                    if any("rakuda/" in b for b in bodies) and not any(
                        "cube" in b for b in bodies
                    ):
                        blocked += 1
                return reach, blocked

            # Searched with a penalty on collisions, so "reachable" means the arm
            # can get there without going through the table it is reaching over.
            def score(q) -> float:
                reach, blocked = evaluate(q)
                return reach + 0.5 * blocked

            generator = np.random.default_rng(0)
            best, best_q = np.inf, None
            for _ in range(8000):
                q = generator.uniform(low, high)
                value = score(q)
                if value < best:
                    best, best_q = value, q
            # Polish: random sampling alone rarely lands inside 5 cm.
            step = 0.25
            while step > 2e-3:
                improved = False
                for i in range(len(best_q)):
                    for delta in (step, -step):
                        trial = best_q.copy()
                        trial[i] = np.clip(trial[i] + delta, low[i], high[i])
                        value = score(trial)
                        if value < best - 1e-6:
                            best, best_q, improved = value, trial, True
                if not improved:
                    step *= 0.5

            reach, blocked = evaluate(best_q)
            assert reach < 0.02, (
                f"the closest a hand gets to the cube is {reach:.4f} m; the work surface is out "
                "of reach, so the mount height needs revisiting"
            )
            assert blocked == 0, (
                f"the hand reaches the cube but {blocked} other robot contacts come with it; "
                "the arm is going through the table or its own pedestal"
            )
        finally:
            env.close()
