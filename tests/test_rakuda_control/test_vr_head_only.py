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
        for _ in range(400):  # let the simulated servo settle on the goal
            bus.step(0.005)
        backend.apply(TeleopCommand())  # a read, no write
        joints = backend.joint_positions()
        assert joints["torso_yaw_dof"] == 0.3
        # Motor moved +0.2 rad; direction -1 => the model joint turned -0.2.
        assert joints["head_yaw_dof"] == pytest.approx(-0.2, abs=0.01)
        assert joints["head_pitch_dof"] == pytest.approx(0.1, abs=0.01)  # neutral, unmoved
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
