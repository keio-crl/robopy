"""Head-only hardware mode: the headset drives two follower motors, nothing else."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import numpy as np
import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.motor.dynamixel_bus import DynamixelMotor  # noqa: E402
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint  # noqa: E402
from robopy.viewer.model_bundle import ModelBundle  # noqa: E402
from robopy.vr.backend import TeleopCommand  # noqa: E402
from robopy.vr.camera import RealsenseFrameSource, to_bgr_uint8  # noqa: E402
from robopy.vr.head_only import HeadMotor, HeadOnlyFollowerBackend, head_motor_mapping  # noqa: E402
from robopy.vr.head_tracking import HeadJointMapping, HeadTracker, HeadTrackingConfig  # noqa: E402
from robopy.vr.server import TeleopSession, VRServerConfig  # noqa: E402

from .test_vr_server import SOFT_LIMITS, head  # noqa: E402

COUNTS_PER_RAD = 4096 / (2 * math.pi)
MOTORS = ("torso_yaw", "head_yaw", "head_pitch", "r_arm_sh_pitch1", "l_arm_sh_pitch1")


@pytest.fixture(scope="module")
def bundle(synthetic_urdf: Path) -> ModelBundle:
    return ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)


def make_bus(start: dict[str, int] | None = None) -> SimulatedDynamixelBus:
    motors = {name: DynamixelMotor(i + 1, name, "xm430-w350") for i, name in enumerate(MOTORS)}
    bus = SimulatedDynamixelBus(
        motors, joints={n: SimulatedJoint() for n in MOTORS}, auto_step=False
    )
    for name, count in (start or {}).items():
        bus.registers(name).joint.position_rad = (count - 2048) / COUNTS_PER_RAD
    bus.torque_enabled()  # as RakudaFollower.connect() leaves the machine
    return bus


def make_backend(
    bundle: ModelBundle, bus: SimulatedDynamixelBus, **kw: Any
) -> HeadOnlyFollowerBackend:
    yaw = HeadMotor("head_yaw", urdf_joint="head_yaw_dof", direction=kw.pop("yaw_direction", 1))
    pitch = HeadMotor(
        "head_pitch", urdf_joint="head_pitch_dof", urdf_neutral_rad=0.1, range_rad=math.radians(35)
    )
    kw.setdefault("threaded", False)  # inline bus traffic, so assertions can follow apply()
    return HeadOnlyFollowerBackend(bus, model=bundle.model, yaw=yaw, pitch=pitch, **kw)


class TestHeadOnlyFollowerBackend:
    def test_start_pose_is_read_and_targets_are_written_relative_to_it(
        self, bundle: ModelBundle
    ) -> None:
        bus = make_bus({"head_yaw": 2200, "head_pitch": 1900})
        backend = make_backend(bundle, bus)
        assert backend.start_units == {"head_yaw": 2200, "head_pitch": 1900}
        assert backend.name == "hardware" and backend.describe()["units"] == "counts"
        writes = bus.transaction_count
        untouched = {n: bus.registers(n).goal_position_count for n in MOTORS if "head" not in n}
        report = backend.apply(
            TeleopCommand(head_targets_rad={"head_yaw": 0.25, "head_pitch": -0.1}, stamp_s=1.0)
        )
        goals = {m: bus.registers(m).goal_position_count for m in ("head_yaw", "head_pitch")}
        assert goals["head_yaw"] == round(2200 + 0.25 * COUNTS_PER_RAD)
        assert goals["head_pitch"] == round(1900 - 0.1 * COUNTS_PER_RAD)
        assert bus.transaction_count > writes and report.warnings == []
        # No other motor received a goal.
        for name, before in untouched.items():
            assert bus.registers(name).goal_position_count == before
        assert report.hand_poses["left"].shape == (4, 4)

    def test_travel_is_clamped_and_unknown_or_bad_targets_are_reported(
        self, bundle: ModelBundle
    ) -> None:
        bus = make_bus({"head_yaw": 100})  # near the encoder's low end
        backend = make_backend(bundle, bus)
        torso_before = bus.registers("torso_yaw").goal_position_count
        report = backend.apply(
            TeleopCommand(
                head_targets_rad={"head_yaw": -3.0, "head_pitch": float("nan"), "torso_yaw": 0.1},
                arm_target=None,
                gripper_targets_rad={"l_arm_grip": 0.5},
            )
        )
        # -3 rad is beyond the 60 deg range and would be below count 0: clamped twice.
        assert bus.registers("head_yaw").goal_position_count == 0
        assert bus.registers("torso_yaw").goal_position_count == torso_before
        assert any("non-finite" in w for w in report.warnings)
        assert any("torso_yaw" in w for w in report.warnings)
        assert any("head-only" in w for w in report.warnings)

    def test_joint_positions_follow_the_motors_for_the_twin(self, bundle: ModelBundle) -> None:
        bus = make_bus({"head_yaw": 2048, "head_pitch": 2048})
        backend = make_backend(
            bundle, bus, yaw_direction=-1, rest_positions_rad={"torso_yaw_dof": 0.3}
        )
        backend.apply(TeleopCommand(head_targets_rad={"head_yaw": 0.2, "head_pitch": 0.0}))
        # The head is never read back after the start pose: the commanded goal
        # is what the twin shows, so one write is one bus transaction.
        writes = bus.transaction_count
        backend.apply(TeleopCommand())  # nothing to write, nothing to read
        assert bus.transaction_count == writes
        joints = backend.joint_positions()
        assert joints["torso_yaw_dof"] == 0.3
        # Motor moved +0.2 rad; direction -1 => the model joint turned -0.2.
        assert joints["head_yaw_dof"] == pytest.approx(-0.2, abs=2e-3)  # one count of rounding
        assert joints["head_pitch_dof"] == pytest.approx(0.1, abs=1e-9)  # neutral, unmoved
        assert backend.describe()["motors"]["head_yaw"]["goal"] == round(
            2048 + 0.2 * COUNTS_PER_RAD
        )

    def test_a_silent_motor_is_an_error_at_start(self, bundle: ModelBundle) -> None:
        bus = make_bus()
        bus.sync_read = lambda item, names: {}  # type: ignore[method-assign]
        with pytest.raises(RuntimeError, match="did not answer"):
            make_backend(bundle, bus)

    def test_calibrated_bus_speaks_degrees(self, bundle: ModelBundle) -> None:
        bus = make_bus({"head_yaw": 2048, "head_pitch": 2048})
        bus.calibration = {"head_yaw": (0, False), "head_pitch": (0, False)}
        backend = make_backend(bundle, bus)
        assert backend.describe()["units"] == "deg"
        start = backend.start_units["head_yaw"]
        backend.apply(TeleopCommand(head_targets_rad={"head_yaw": math.radians(10)}))
        # Ten degrees more than the start reading, in the bus's calibrated
        # units (the simulated bus stores what it is given, unconverted).
        assert bus.registers("head_yaw").goal_position_count == pytest.approx(start + 10, abs=1)


class TestLeaderFollowing:
    def test_leader_drives_everything_but_the_head(self, bundle: ModelBundle) -> None:
        follower = make_bus({"head_yaw": 2048, "head_pitch": 2048})
        leader = make_bus({"torso_yaw": 1500, "head_yaw": 900, "r_arm_sh_pitch1": 3000})
        before = {n: follower.registers(n).goal_position_count for n in MOTORS}
        backend = make_backend(
            bundle,
            follower,
            leader_bus=leader,
            follower_writable=["torso_yaw", "head_yaw", "head_pitch", "r_arm_sh_pitch1"],
        )
        report = backend.apply(TeleopCommand(head_targets_rad={"head_yaw": 0.1, "head_pitch": 0.0}))
        assert report.warnings == []
        # Torso and right shoulder: copied from the leader, count for count.
        assert follower.registers("torso_yaw").goal_position_count == 1500
        assert follower.registers("r_arm_sh_pitch1").goal_position_count == 3000
        # Left shoulder: not torque-enabled on the follower, so not written.
        assert (
            follower.registers("l_arm_sh_pitch1").goal_position_count == before["l_arm_sh_pitch1"]
        )
        # Head: from the headset, never from the leader's own head.
        assert follower.registers("head_yaw").goal_position_count == round(
            2048 + 0.1 * COUNTS_PER_RAD
        )
        assert follower.registers("head_pitch").goal_position_count == 2048
        described = backend.describe()["leader"]
        assert described["follower_motors"] == ["r_arm_sh_pitch1", "torso_yaw"]
        assert described["last_goals"] == {"torso_yaw": 1500.0, "r_arm_sh_pitch1": 3000.0}
        # Controllers are reported as not in charge of the arms.
        report = backend.apply(TeleopCommand(gripper_targets_rad={"l_arm_grip": 0.3}))
        assert any("follow the leader" in w for w in report.warnings)

    def test_leader_mapping_and_unknown_motors(self, bundle: ModelBundle) -> None:
        follower = make_bus()
        leader = make_bus({"torso_yaw": 1000})
        backend = make_backend(
            bundle,
            follower,
            leader_bus=leader,
            leader_to_follower={"torso_yaw": "r_arm_sh_pitch1", "ghost": "torso_yaw"},
        )
        backend.apply(TeleopCommand())
        assert follower.registers("r_arm_sh_pitch1").goal_position_count == 1000
        assert backend.describe()["leader"]["follower_motors"] == ["r_arm_sh_pitch1"]


class TestBusThread:
    def test_a_cycle_is_one_leader_read_and_one_follower_write(self, bundle: ModelBundle) -> None:
        follower = make_bus({"head_yaw": 2048, "head_pitch": 2048})
        leader = make_bus({"torso_yaw": 1200})
        backend = make_backend(bundle, follower, leader_bus=leader)
        f0, l0 = follower.transaction_count, leader.transaction_count
        backend.apply(TeleopCommand(head_targets_rad={"head_yaw": 0.1, "head_pitch": 0.0}))
        assert (follower.transaction_count - f0, leader.transaction_count - l0) == (1, 1)
        goals = backend.describe()["leader"]["last_goals"]
        assert "head_yaw" not in goals and "head_pitch" not in goals and len(goals) == 3
        # Fifteen on the real machine: seventeen leader motors minus the head.
        from robopy.config.robot_config.rakuda_config import RAKUDA_MOTOR_MAPPING

        assert len([m for m in RAKUDA_MOTOR_MAPPING if m not in ("head_yaw", "head_pitch")]) == 15

    def test_apply_only_posts_and_the_thread_does_the_bus_work(self, bundle: ModelBundle) -> None:
        import time

        follower = make_bus({"head_yaw": 2048, "head_pitch": 2048})
        leader = make_bus({"torso_yaw": 1200})
        backend = make_backend(bundle, follower, leader_bus=leader, threaded=True, rate_hz=200.0)
        before = follower.transaction_count
        backend.apply(TeleopCommand(head_targets_rad={"head_yaw": 0.2, "head_pitch": 0.0}))
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and backend.describe()["bus_thread"]["cycles"] < 5:
            time.sleep(0.01)
        info = backend.describe()
        assert info["bus_thread"]["running"] and info["bus_thread"]["cycles"] >= 5
        assert follower.registers("head_yaw").goal_position_count == round(
            2048 + 0.2 * COUNTS_PER_RAD
        )
        assert follower.registers("torso_yaw").goal_position_count == 1200
        assert follower.transaction_count > before
        # Goals are posted once and the leader keeps being copied every cycle.
        writes = info["writes"]
        time.sleep(0.05)
        assert backend.describe()["writes"] > writes
        backend.close()
        assert not backend.describe()["bus_thread"]["running"]
        with pytest.raises(ValueError):
            make_backend(bundle, follower, threaded=True, rate_hz=0.0)


class TestMotorSpaceMapping:
    def test_signs_combine_urdf_and_direction_unless_overridden(self) -> None:
        yaw = HeadMotor("head_yaw", direction=-1)
        pitch = HeadMotor("head_pitch", range_rad=0.5)
        m = head_motor_mapping(yaw, pitch, yaw_sign_urdf=-1, pitch_sign_urdf=1)
        assert (m.yaw_sign, m.pitch_sign) == (1, 1)
        assert m.yaw_neutral_rad == 0.0 and m.pitch_limits_rad == (-0.5, 0.5)
        assert m.torso_joint is None and "URDF axis" in m.notes[0]
        forced = head_motor_mapping(
            yaw, pitch, yaw_sign_urdf=-1, pitch_sign_urdf=1, sign_overrides=(-1, -1)
        )
        assert (forced.yaw_sign, forced.pitch_sign) == (-1, -1)
        with pytest.raises(ValueError):
            HeadMotor("x", direction=2)
        with pytest.raises(ValueError):
            HeadMotor("x", range_rad=0.0)

    def test_session_turns_the_head_motors_from_the_headset(self, bundle: ModelBundle) -> None:
        bus = make_bus({"head_yaw": 2048, "head_pitch": 2048})
        backend = make_backend(bundle, bus)
        urdf = HeadJointMapping.from_model(
            bundle.model, "head_yaw_dof", "head_pitch_dof", camera_frame="head_camera_link"
        )
        mapping = head_motor_mapping(
            HeadMotor("head_yaw"),
            HeadMotor("head_pitch"),
            yaw_sign_urdf=urdf.yaw_sign,
            pitch_sign_urdf=urdf.pitch_sign,
        )
        session = TeleopSession(
            backend,
            head_tracker=HeadTracker(
                mapping, HeadTrackingConfig(filter_hz=None, max_rate_rad_s=100.0)
            ),
            arm_teleop=None,
            config=VRServerConfig(state_hz=1000.0),
            bundle=bundle,
        )
        hello = session.handle({"type": "hello"}, 0.0)
        assert hello is not None and hello["backend"] == "hardware" and hello["arms"] is None
        session.handle({"type": "pose", "head": head(0.0), "left": None, "right": None}, 0.0)
        state = session.handle(
            {"type": "pose", "head": head(0.3), "left": None, "right": None}, 0.1
        )
        assert state is not None
        assert state["head"]["targets_rad"]["head_yaw"] == pytest.approx(0.3 * mapping.yaw_sign)
        expected = round(2048 + 0.3 * mapping.yaw_sign * COUNTS_PER_RAD)
        assert bus.registers("head_yaw").goal_position_count == expected
        assert bus.registers("head_pitch").goal_position_count == 2048
        assert state["backend"] == "hardware"


class TestRealsenseFrameSource:
    def test_wraps_the_camera_and_tolerates_timeouts(self, monkeypatch: pytest.MonkeyPatch) -> None:
        calls: list[str] = []

        class FakeCamera:
            def __init__(self, config: Any) -> None:
                calls.append(f"init {config.name} {config.width}x{config.height}@{config.fps}")
                assert config.color_mode == "rgb" and not config.is_depth_camera

            def connect(self) -> None:
                calls.append("connect")

            def async_read(self, timeout_ms: float) -> Any:
                if len(calls) < 3:
                    calls.append("timeout")
                    raise TimeoutError("no frame")
                calls.append("frame")
                chw = np.zeros((3, 4, 6), dtype=np.float32)  # CHW float, as the camera returns
                chw[0] = 255.0  # red
                return chw

            def disconnect(self) -> None:
                calls.append("disconnect")

        monkeypatch.setattr("robopy.sensors.visual.realsense_camera.RealsenseCamera", FakeCamera)
        source = RealsenseFrameSource(1, width=6, height=4, fps=15)
        assert source.read() is None  # timeout -> keep the previous picture
        frame = source.read()
        assert frame is not None and frame.shape == (4, 6, 3) and frame.dtype == np.uint8
        assert tuple(frame[0, 0]) == (0, 0, 255)  # BGR: red last
        source.close()
        assert (
            calls[:3] == ["init realsense1 6x4@15", "connect", "timeout"]
            and calls[-1] == "disconnect"
        )
        assert to_bgr_uint8(np.zeros((3, 2, 2)), color="rgb").shape == (2, 2, 3)


class TestBilateralHead:
    """The arms coupled by the bilateral controller; the head through its loop."""

    COUPLED = ("torso_yaw", "r_arm_sh_pitch1", "l_arm_sh_pitch1")

    def _system(self) -> Any:
        from robopy.config.robot_config.rakuda_config import (
            RakudaBilateralConfig,
            RakudaControlConfig,
            RakudaJointCalibrationSpec,
        )
        from robopy.robots.rakuda.rakuda_control import RakudaControlSystem

        calibration = {
            name: RakudaJointCalibrationSpec(
                direction=1,
                zero_count=2048,
                lower_limit_rad=-2.0,
                upper_limit_rad=2.0,
                max_velocity_rad_s=3.0,
                torque_constant_nm_per_a=1.5,
                current_limit_a=1.0,
                validated=True,
            )
            for name in self.COUPLED
        }
        config = RakudaControlConfig(
            mode="bilateral_joint",
            control_period_s=0.002,
            leader_joint_calibration=dict(calibration),
            follower_joint_calibration=dict(calibration),
            bilateral=RakudaBilateralConfig(
                coupled_motors=list(self.COUPLED),
                stiffness_nm_per_rad=2.0,
                damping_nm_s_per_rad=0.05,
                max_torque_nm=1.0,
                max_torque_rate_nm_s=50.0,
                ramp_time_s=0.2,
                velocity_filter_hz=50.0,
                leader_current_limit_a={n: 0.5 for n in self.COUPLED},
                follower_current_limit_a={n: 1.0 for n in self.COUPLED},
                allow_uncompensated=True,
            ),
            allow_hardware_current_output=True,
        )
        leader = make_bus({"torso_yaw": 2048})
        follower = make_bus({"head_yaw": 2048, "head_pitch": 2048})
        system = RakudaControlSystem.from_buses(
            config,
            leader,
            follower,
            leader_torque_enabled=list(self.COUPLED),
            follower_torque_enabled=list(self.COUPLED) + ["head_yaw", "head_pitch"],
        )
        return system, leader, follower

    def test_head_goals_are_written_by_the_control_loop(self, bundle: ModelBundle) -> None:
        system, leader, follower = self._system()
        system.configure()
        system.align()
        backend = make_backend(bundle, follower, control_system=system, target_ttl_s=0.5)
        assert backend.describe()["mode"] == "head_only+bilateral"
        system.prepare_running()
        head_before = follower.registers("head_yaw").goal_position_count
        report = backend.apply(TeleopCommand(head_targets_rad={"head_yaw": 0.2, "head_pitch": 0.0}))
        assert report.warnings == []
        # Not written yet: the backend never touches the bus while the loop owns it.
        assert follower.registers("head_yaw").goal_position_count == head_before
        assert system.report()["direct_goal_counts"]["goals"] == {
            "head_yaw": round(2048 + 0.2 * COUNTS_PER_RAD),
            "head_pitch": 2048,
        }
        dt = system.loop.control_period_s
        for _ in range(5):
            leader.joint("torso_yaw").position_rad = 0.2
            leader.step(dt)
            follower.step(dt)
            system.loop.run_once(dt)
        assert follower.registers("head_yaw").goal_position_count == round(
            2048 + 0.2 * COUNTS_PER_RAD
        )
        assert system.report()["direct_goal_counts"]["writes"] == 5
        # The coupling is in charge of the torso: current mode, with a torque command.
        assert follower.registers("torso_yaw").operating_mode == 0
        assert follower.registers("torso_yaw").goal_current_raw != 0
        # The twin reports the commanded head pose (nothing is read back).
        assert backend.joint_positions()["head_yaw_dof"] == pytest.approx(0.2, abs=1e-3)
        system.stop()

    def test_goal_counts_are_refused_for_coupled_motors_and_other_modes(
        self, bundle: ModelBundle
    ) -> None:
        system, _, follower = self._system()
        with pytest.raises(ValueError, match="bilateral coupling"):
            system.set_direct_goal_counts({"torso_yaw": 2048})
        with pytest.raises(ValueError, match="not a follower motor"):
            system.set_direct_goal_counts({"nope": 2048})
        with pytest.raises(ValueError, match="non-finite"):
            system.set_direct_goal_counts({"head_yaw": float("nan")})
        with pytest.raises(ValueError):
            make_backend(bundle, follower, control_system=system, leader_bus=make_bus())
        from robopy.robots.rakuda.rakuda_control import RakudaControlSystem

        cartesian = RakudaControlSystem.__new__(RakudaControlSystem)
        from robopy.control.types import ControlMode

        cartesian._mode = ControlMode.CARTESIAN_TELEOP
        with pytest.raises(RuntimeError, match="bilateral_joint"):
            cartesian.set_direct_goal_counts({"head_yaw": 2048})

    def test_expired_goals_stop_being_written(self, bundle: ModelBundle) -> None:
        import time

        system, leader, follower = self._system()
        system.configure()
        system.align()
        backend = make_backend(bundle, follower, control_system=system, target_ttl_s=0.02)
        system.prepare_running()
        backend.apply(TeleopCommand(head_targets_rad={"head_yaw": 0.1}))
        dt = system.loop.control_period_s
        leader.step(dt)
        follower.step(dt)
        system.loop.run_once(dt)
        assert system.report()["direct_goal_counts"]["writes"] == 1
        time.sleep(0.03)
        leader.step(dt)
        follower.step(dt)
        system.loop.run_once(dt)
        assert system.report()["direct_goal_counts"]["writes"] == 1  # expired: not rewritten
        system.stop()
