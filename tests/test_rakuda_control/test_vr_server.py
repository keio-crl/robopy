"""The teleoperation session, the VR server and the hardware-path backend.

Skipped without the ``kinematics`` extra.  Nothing here needs a headset: the
tests speak the WebSocket protocol directly, and the hardware path runs on
simulated Dynamixel buses.
"""

from __future__ import annotations

import json
import math
import threading
import time
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Iterator, Tuple

import numpy as np
import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.config.robot_config.rakuda_config import (  # noqa: E402
    RakudaControlConfig,
    RakudaJointCalibrationSpec,
    RakudaModelConfig,
    RakudaTcpSpec,
    RakudaTrajectoryConfig,
)
from robopy.kinematics.synthetic_dual_arm import (  # noqa: E402
    SYNTHETIC_ARM_JOINTS,
    SYNTHETIC_HEAD_JOINTS,
    SYNTHETIC_TCP_FRAMES,
    SYNTHETIC_TORSO_JOINT,
)
from robopy.motor.dynamixel_bus import DynamixelMotor  # noqa: E402
from robopy.motor.sim_dynamixel_bus import SimulatedDynamixelBus, SimulatedJoint  # noqa: E402
from robopy.robots.rakuda.rakuda_control import RakudaControlSystem  # noqa: E402
from robopy.viewer.model_bundle import ModelBundle  # noqa: E402
from robopy.viewer.server import IKSetup  # noqa: E402
from robopy.vr import websocket as ws  # noqa: E402
from robopy.vr.__main__ import STREAMING_IK_OVERRIDES  # noqa: E402
from robopy.vr.arm_teleop import ArmTeleopConfig, DualArmTeleop  # noqa: E402
from robopy.vr.backend import ControlSystemBackend, SimulationBackend, TeleopCommand  # noqa: E402
from robopy.vr.camera import FrameStreamer, SyntheticFrameSource  # noqa: E402
from robopy.vr.head_tracking import HeadJointMapping, HeadTracker, HeadTrackingConfig  # noqa: E402
from robopy.vr.server import (  # noqa: E402
    TeleopSession,
    VRServer,
    VRServerConfig,
    head_anchor_position,
)

from .test_vr_transport import RawClient  # noqa: E402

SOFT_LIMITS = {
    "torso_yaw_dof": (-1.5, 1.5),
    "shoulder_pitch_left_dof": (-2.0, 2.0),
    "shoulder_pitch_right_dof": (-2.0, 2.0),
}
HEAD0 = {"p": [0.0, 1.6, 0.0], "q": [0.0, 0.0, 0.0, 1.0]}


def q_about(axis: str, angle: float) -> list[float]:
    half = angle / 2.0
    s, c = math.sin(half), math.cos(half)
    return {"x": [s, 0, 0, c], "y": [0, s, 0, c], "z": [0, 0, s, c]}[axis]


def head(yaw: float = 0.0, pitch: float = 0.0) -> Dict[str, Any]:
    # Yaw about WebXR +Y (left positive), then pitch about the turned +X (up positive).
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    # q = q_yaw * q_pitch  (xyzw), q_yaw = (0, sy, 0, cy), q_pitch = (sp, 0, 0, cp)
    x = cy * sp
    y = sy * cp
    z = -sy * sp
    w = cy * cp
    return {"p": [0.0, 1.6, 0.0], "q": [x, y, z, w]}


def controller(
    x: float, y: float, z: float, *, clutch: bool, trigger: float = 0.0
) -> Dict[str, Any]:
    return {"p": [x, y, z], "q": [0, 0, 0, 1], "clutch": clutch, "trigger": trigger, "buttons": {}}


@pytest.fixture(scope="module")
def bundle(synthetic_urdf: Path) -> ModelBundle:
    return ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)


def xr_point(p_robot: Any) -> list[float]:
    """A robot-axes point (x forward, y left, z up) as WebXR coordinates."""
    x, y, z = (float(v) for v in p_robot)
    return [-y, z, -x]


def make_session(
    bundle: ModelBundle,
    *,
    arms: bool = True,
    head_tracking: bool = True,
    state_hz: float = 1000.0,
    mapping: str = "relative",
    config: VRServerConfig | None = None,
) -> Tuple[TeleopSession, SimulationBackend]:
    ik = IKSetup(bundle, config_overrides=STREAMING_IK_OVERRIDES)
    backend = SimulationBackend(bundle, ik)
    tracker = None
    if head_tracking:
        head_mapping = HeadJointMapping.from_model(
            bundle.model,
            "head_yaw_dof",
            "head_pitch_dof",
            camera_frame="head_camera_link",
            torso_joint="torso_yaw_dof",
        )
        tracker = HeadTracker(
            head_mapping, HeadTrackingConfig(filter_hz=None, max_rate_rad_s=100.0)
        )
    teleop = None
    if arms:
        fast = ArmTeleopConfig(
            mapping=mapping,  # type: ignore[arg-type]
            max_speed_m_s=100.0,
            max_angular_speed_rad_s=100.0,
        )
        teleop = DualArmTeleop(fast, fast)
    session = TeleopSession(
        backend,
        head_tracker=tracker,
        arm_teleop=teleop,
        config=config or VRServerConfig(state_hz=state_hz),
        bundle=bundle,
    )
    return session, backend


class TestAbsoluteMapping:
    """The operator's head stands in for the robot's; a pressed arm goes to the controller."""

    def test_hello_and_state_carry_the_anchor_clutch_and_twin(self, bundle: ModelBundle) -> None:
        session, backend = make_session(bundle, mapping="absolute")
        expected = head_anchor_position(
            bundle, backend.joint_positions(), session.head_tracker, "head_camera_link"
        )
        hello = session.handle({"type": "hello"}, 0.0)
        assert hello is not None
        assert hello["clutch_button"] == "a"
        assert hello["arms"]["left"]["mapping"] == "absolute"
        assert hello["robot_anchor_m"] == pytest.approx(list(expected))
        assert hello["twin_offset_m"] is None and hello["twin"] is None  # not re-centred yet
        state = session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.1)
        assert state is not None
        assert state["operator"]["recentred"] and state["operator"]["head_height_m"] == 1.6
        # The twin's head anchor is drawn where the headset is: base = head - anchor.
        assert state["twin"]["yaw"] == pytest.approx(0.0)
        assert state["twin"]["p"] == pytest.approx(list(np.array([0.0, 0.0, 1.6]) - expected))
        assert hello["arms"]["left"]["anchor"] is None
        assert session.arm_teleop is not None
        assert session.arm_teleop.describe()["left"]["anchor"]["operator_m"] == [0.0, 0.0, 1.6]

    def test_twin_follows_a_yawed_recentre(self, bundle: ModelBundle) -> None:
        session, _ = make_session(bundle, mapping="absolute")
        # The operator faces WebXR +X (a quarter turn to the right of -Z) from (1, 1.6, 2).
        turned = {"p": [1.0, 1.6, 2.0], "q": q_about("y", -math.pi / 2)}
        state = session.handle({"type": "pose", "head": turned, "left": None, "right": None}, 0.0)
        assert state is not None
        anchor = np.asarray(session.robot_anchor_m)
        yaw = state["twin"]["yaw"]
        p = np.asarray(state["twin"]["p"])
        # Head in robot axes: (-z, -x, y) = (-2, -1, 1.6).  The base is placed so
        # that anchor, turned by the operator's yaw, lands on the head.
        c, s_ = math.cos(yaw), math.sin(yaw)
        Rz = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
        assert np.allclose(p + Rz @ anchor, [-2.0, -1.0, 1.6], atol=1e-9)
        assert yaw == pytest.approx(-math.pi / 2)

    def test_explicit_twin_offset_disables_the_automatic_placement(
        self, bundle: ModelBundle
    ) -> None:
        cfg = VRServerConfig(state_hz=1000.0, twin_offset_m=(0.0, 0.0, 1.0))
        session, _ = make_session(bundle, mapping="absolute", config=cfg)
        hello = session.handle({"type": "hello"}, 0.0)
        assert hello is not None and hello["twin_offset_m"] == [0.0, 0.0, 1.0]
        state = session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.1)
        assert state is not None and state["twin"] is None

    def test_a_pressed_arm_goes_to_where_the_controller_is(self, bundle: ModelBundle) -> None:
        session, backend = make_session(bundle, mapping="absolute")
        session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.0)
        anchor = np.asarray(session.robot_anchor_m)
        head = np.array([0.0, 0.0, 1.6])
        start = backend.hand_pose("left")[:3, 3].copy()
        # Put the controller where the robot's hand "is" in the operator's body,
        # 4 cm further forward.  Nothing about the controller's motion matters:
        # it is held still and the hand comes to it.
        controller_robot = head + (start - anchor) + np.array([0.04, 0.0, 0.0])
        entry = {"p": xr_point(controller_robot), "q": [0, 0, 0, 1], "clutch": True, "trigger": 0.0}
        t = 0.0
        for _ in range(100):
            t += 1.0 / 60.0
            state = session.handle({"type": "pose", "head": HEAD0, "left": entry, "right": None}, t)
        assert state is not None
        assert state["arms"]["left"]["clutched"] and state["ik"]["commandable"]
        target = state["arms"]["left"]["target"]["p"]
        assert target == pytest.approx(list(start + [0.04, 0.0, 0.0]), abs=1e-9)
        moved = backend.hand_pose("left")[:3, 3] - start
        assert moved[0] == pytest.approx(0.04, abs=0.006)
        assert abs(moved[1]) < 0.01 and abs(moved[2]) < 0.01
        # The other arm was never pressed: held, its joints untouched by the solver.
        assert not state["arms"]["right"]["enabled"]

    def test_config_validation_and_anchor_lookup(self, bundle: ModelBundle) -> None:
        with pytest.raises(ValueError, match="clutch_button"):
            VRServerConfig(clutch_button="trigger")
        joints = {n: 0.0 for n in bundle.model.movable_joint_names}
        auto = head_anchor_position(bundle, joints, None, "auto")
        named = head_anchor_position(bundle, joints, None, "head_camera_link")
        assert np.allclose(auto, named)
        with pytest.raises(ValueError, match="not in the model"):
            head_anchor_position(bundle, joints, None, "no_such_frame")
        cfg = VRServerConfig(arm_anchor_frame="no_such_frame")
        with pytest.raises(ValueError, match="not in the model"):
            make_session(bundle, mapping="absolute", config=cfg)


class TestTeleopSession:
    def test_hello_describes_the_session(self, bundle: ModelBundle) -> None:
        session, _ = make_session(bundle)
        hello = session.handle({"type": "hello", "want_poses": True}, 0.0)
        assert hello is not None and hello["type"] == "hello"
        assert hello["backend"] == "simulation"
        assert hello["model"]["robot"] == "synthetic"
        assert hello["head"]["mapping"]["yaw_sign"] == 1
        assert hello["arms"]["left"]["gripper_available"] is False
        assert session.want_poses

    def test_first_pose_recentres_and_the_head_follows_with_model_signs(
        self, bundle: ModelBundle
    ) -> None:
        session, backend = make_session(bundle)
        # The operator starts looking somewhere arbitrary: that becomes forward.
        first = session.handle(
            {"type": "pose", "t": 1, "head": head(yaw=1.0), "left": None, "right": None}, 0.0
        )
        assert first is not None and first["operator"]["recentred"]
        assert first["head"]["yaw_input_rad"] == pytest.approx(0.0, abs=1e-9)
        assert first["joints"]["head_yaw_dof"] == pytest.approx(0.0, abs=1e-9)
        # Turn left by 0.3 and look up by 0.2 relative to that.
        state = session.handle(
            {"type": "pose", "t": 2, "head": head(yaw=1.3, pitch=0.2), "left": None, "right": None},
            1.0,
        )
        assert state is not None
        assert state["head"]["yaw_input_rad"] == pytest.approx(0.3, abs=1e-6)
        assert state["head"]["pitch_input_rad"] == pytest.approx(0.2, abs=1e-6)
        # Synthetic head: yaw axis +Z (sign +1), pitch axis +Y (sign -1).
        assert state["joints"]["head_yaw_dof"] == pytest.approx(0.3, abs=1e-6)
        assert state["joints"]["head_pitch_dof"] == pytest.approx(-0.2, abs=1e-6)
        assert backend.joint_positions()["head_yaw_dof"] == pytest.approx(0.3, abs=1e-6)
        assert state["t"] == 2

    def test_head_yaw_command_cancels_the_torso_yaw(self, bundle: ModelBundle) -> None:
        session, backend = make_session(bundle)
        # The torso is turned left by 0.4 rad (as the arm solver might leave it).
        backend.apply(TeleopCommand(head_targets_rad={}, arm_target=None))
        backend._positions["torso_yaw_dof"] = 0.4  # noqa: SLF001 - set the plant state directly
        session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.0)
        state = session.handle(
            {"type": "pose", "head": head(yaw=0.3), "left": None, "right": None}, 1.0
        )
        assert state is not None
        # signal (+0.3, sign +1) minus the torso (0.4): the camera heading is +0.3 in the base.
        assert state["head"]["torso_compensation_rad"] == pytest.approx(-0.4, abs=1e-9)
        assert state["joints"]["head_yaw_dof"] == pytest.approx(0.3 - 0.4, abs=1e-6)
        positions = backend.joint_positions()
        R = bundle.model.frame_pose(bundle.model.q_from_positions(positions), "head_camera_link")
        assert math.atan2(R[1, 0], R[0, 0]) == pytest.approx(0.3, abs=1e-6)

    def test_a_clutched_controller_moves_the_hand_forward(self, bundle: ModelBundle) -> None:
        session, backend = make_session(bundle)
        session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.0)
        start = backend.hand_pose("left")[:3, 3].copy()
        t = 0.0
        # Squeeze, then move the controller 4 cm forward (WebXR -Z) over 40 frames
        # and hold it there for another 60 so the solver can converge.
        for i in range(100):
            t += 1.0 / 60.0
            z = -0.4 - 0.04 * min(i, 40) / 40.0
            state = session.handle(
                {
                    "type": "pose",
                    "head": HEAD0,
                    "left": controller(-0.3, 1.2, z, clutch=True),
                    "right": None,
                },
                t,
            )
        assert state is not None
        assert state["arms"]["left"]["clutched"] and state["arms"]["left"]["enabled"]
        assert state["arms"]["right"]["tracked"] is False
        assert state["ik"]["commandable"]
        moved = backend.hand_pose("left")[:3, 3] - start
        assert moved[0] == pytest.approx(0.04, abs=0.006)  # forward = robot +X
        assert abs(moved[1]) < 0.01 and abs(moved[2]) < 0.01
        assert state["ik"]["errors"]["left_position_m"] < 0.005
        # Release: the target is latched and the hand is held, not driven.
        released = session.handle(
            {
                "type": "pose",
                "head": HEAD0,
                "left": controller(-0.3, 1.2, -0.44, clutch=False),
                "right": None,
            },
            t + 0.02,
        )
        assert released is not None
        assert not released["arms"]["left"]["clutched"] and not released["arms"]["left"]["enabled"]
        assert released["arms"]["left"]["target"] is not None

    def test_operator_yaw_is_removed_before_the_arms_see_the_controller(
        self, bundle: ModelBundle
    ) -> None:
        # The operator faces WebXR +X (turned right by 90 degrees). Pushing the
        # controller along WebXR +X is then "forward" and must move the hand +X.
        session, backend = make_session(bundle)
        turned = {"p": [0.0, 1.6, 0.0], "q": q_about("y", -math.pi / 2)}
        session.handle({"type": "pose", "head": turned, "left": None, "right": None}, 0.0)
        start = backend.hand_pose("left")[:3, 3].copy()
        t = 0.0
        for i in range(100):
            t += 1.0 / 60.0
            x = 0.4 + 0.04 * min(i, 40) / 40.0
            session.handle(
                {
                    "type": "pose",
                    "head": turned,
                    "left": controller(x, 1.2, 0.3, clutch=True),
                    "right": None,
                },
                t,
            )
        moved = backend.hand_pose("left")[:3, 3] - start
        assert moved[0] == pytest.approx(0.04, abs=0.006)
        assert abs(moved[1]) < 0.01

    def test_recenter_set_and_errors(self, bundle: ModelBundle) -> None:
        session, _ = make_session(bundle)
        early = session.handle({"type": "recenter"}, 0.0)
        assert early is not None and early["type"] == "error"
        assert session.handle({"type": "bogus"}, 0.0)["type"] == "error"  # type: ignore[index]
        session.handle(
            {
                "type": "pose",
                "head": head(yaw=0.5),
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            },
            0.0,
        )
        assert session.arm_teleop is not None and session.arm_teleop.arms["left"].clutched
        turned = session.handle(
            {
                "type": "pose",
                "head": head(yaw=1.0),
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            },
            1.0,
        )
        assert turned is not None and turned["head"]["yaw_input_rad"] == pytest.approx(
            0.5, abs=1e-6
        )
        recentred = session.handle({"type": "recenter"}, 2.0)
        assert recentred is not None and recentred["type"] == "state" and recentred["recentred"]
        assert not session.arm_teleop.arms["left"].clutched  # re-centring releases the clutch
        after = session.handle(
            {"type": "pose", "head": head(yaw=1.0), "left": None, "right": None}, 3.0
        )
        assert after is not None and after["head"]["yaw_input_rad"] == pytest.approx(0.0, abs=1e-6)
        bad = session.handle({"type": "set", "position_scale": 9.0}, 4.0)
        assert bad is not None and bad["type"] == "error"
        ok = session.handle(
            {"type": "set", "position_scale": 0.5, "arms_enabled": False, "head_enabled": False},
            4.0,
        )
        assert ok is not None and ok["type"] == "hello" and not ok["arms_enabled"]
        assert session.arm_teleop.arms["right"].config.position_scale == 0.5
        off = session.handle(
            {
                "type": "pose",
                "head": head(yaw=2.0),
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            },
            5.0,
        )
        assert off is not None and off["head"] == {"tracking": False}
        assert off["arms"]["left"]["clutched"] is False

    def test_state_messages_are_throttled(self, bundle: ModelBundle) -> None:
        session, _ = make_session(bundle, state_hz=10.0)
        assert (
            session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.0)
            is not None
        )
        assert (
            session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.01)
            is None
        )
        assert (
            session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.2)
            is not None
        )

    def test_want_poses_adds_geometry_poses_and_close_releases(self, bundle: ModelBundle) -> None:
        session, backend = make_session(bundle)
        session.handle({"type": "hello", "want_poses": True}, 0.0)
        state = session.handle(
            {
                "type": "pose",
                "head": HEAD0,
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            },
            0.0,
        )
        assert state is not None and len(state["geometries"]) == len(bundle.geometries)
        assert set(state["tcp"]) == {"left", "right"}
        assert session.arm_teleop is not None and session.arm_teleop.arms["left"].clutched
        session.close()
        assert not session.arm_teleop.arms["left"].clutched

    def test_head_only_and_arms_only_sessions(self, bundle: ModelBundle) -> None:
        head_only, _ = make_session(bundle, arms=False)
        state = head_only.handle(
            {
                "type": "pose",
                "head": head(0.2),
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            },
            0.0,
        )
        assert state is not None and state["arms"] == {} and state["head"]["tracking"]
        arms_only, backend = make_session(bundle, head_tracking=False)
        state = arms_only.handle(
            {"type": "pose", "head": head(0.5), "left": None, "right": None}, 0.0
        )
        assert state is not None and state["head"] == {"tracking": False}
        assert backend.joint_positions()["head_yaw_dof"] == 0.0


@pytest.fixture
def server(bundle: ModelBundle) -> Iterator[VRServer]:
    ik = IKSetup(bundle, config_overrides=STREAMING_IK_OVERRIDES)
    mapping = HeadJointMapping.from_model(
        bundle.model, "head_yaw_dof", "head_pitch_dof", camera_frame="head_camera_link"
    )
    fast = ArmTeleopConfig(max_speed_m_s=100.0)
    srv = VRServer(
        bundle,
        ik=ik,
        backend=SimulationBackend(bundle, ik),
        # No filter and a huge rate limit: two poses a millisecond apart must
        # still land exactly, or the socket tests would depend on timing.
        head_tracker=HeadTracker(mapping, HeadTrackingConfig(filter_hz=None, max_rate_rad_s=1e6)),
        arm_teleop=DualArmTeleop(fast, fast),
        camera=FrameStreamer(SyntheticFrameSource(64, 48), fps=60.0),
        host="127.0.0.1",
        port=0,
        config=VRServerConfig(state_hz=1000.0, teleop_timeout_s=1.5),
    )
    assert srv.camera is not None
    srv.camera.start()
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    yield srv
    srv.shutdown()
    srv.server_close()


def _get(url: str) -> Tuple[int, Any]:
    try:
        with urllib.request.urlopen(url, timeout=10) as res:
            raw = res.read()
            ctype = res.headers.get("Content-Type", "")
            return res.status, (json.loads(raw) if "json" in ctype else raw)
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b"{}")


def _wait_until(predicate: Any, timeout_s: float) -> bool:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.02)
    return predicate()


class TestVRServer:
    def test_pages_and_status(self, server: VRServer) -> None:
        base = server.url.rstrip("/")
        status, page = _get(base + "/vr")
        assert status == 200 and b"robopy VR teleop" in page
        status, script = _get(base + "/vr/static/vr.js")
        assert status == 200 and b"/ws/teleop" in script
        status, model = _get(base + "/api/model")
        assert status == 200 and model["vr"] is True and model["simulation_only"] is True
        status, info = _get(base + "/api/vr/status")
        assert status == 200
        assert info["backend"]["name"] == "simulation"
        assert info["session"] is None and info["camera"]["running"]
        assert info["tls"] is False
        status, _ = _get(base + "/ws/teleop")
        assert status == 400
        status, _ = _get(base + "/vr/static/../server.py")
        assert status == 404

    def test_teleop_socket_round_trip_and_release_on_close(self, server: VRServer) -> None:
        host, port = server.server_address[0], server.server_address[1]
        client = RawClient(host, port, "/ws/teleop")
        assert b"101" in client.status
        client.send_json({"type": "hello", "want_poses": False})
        hello = client.recv_json()
        assert hello["type"] == "hello" and hello["backend"] == "simulation"
        client.send_json(
            {
                "type": "pose",
                "t": 5,
                "head": head(0.3),
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            }
        )
        state = client.recv_json()
        assert state["type"] == "state" and state["t"] == 5
        assert state["arms"]["left"]["clutched"]
        client.send_json(
            {
                "type": "pose",
                "t": 6,
                "head": head(0.6),
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            }
        )
        state = client.recv_json()
        assert state["joints"]["head_yaw_dof"] == pytest.approx(0.3, abs=1e-6)
        assert server.session is not None and server.status()["session"]["messages"] == 3
        client.send_json("not an object")
        assert client.recv_json()["type"] == "error"
        assert server.arm_teleop is not None and server.arm_teleop.arms["left"].clutched
        client.close()
        assert _wait_until(lambda: server.session is None, 3.0)
        assert not server.arm_teleop.arms["left"].clutched

    def test_binary_frames_reach_the_recorder(self, server: VRServer, tmp_path: Path) -> None:
        import cv2
        import numpy as np

        from robopy.vr.recording import CameraTap, PushedFrames, SessionRecorder

        pushed = PushedFrames()
        server.recorder = SessionRecorder(tmp_path, operator_view=CameraTap(pushed, fps=20.0))
        host, port = server.server_address[0], server.server_address[1]
        client = RawClient(host, port, "/ws/teleop")
        try:
            client.send_json({"type": "hello"})
            client.recv_json()
            client.send_json({"type": "record", "action": "start"})
            assert client.recv_json()["recording"]["active"]
            ok, buf = cv2.imencode(".jpg", np.zeros((8, 8, 3), dtype=np.uint8))
            assert ok
            client.send(ws.OP_BINARY, bytes(buf))
            client.send_json({"type": "record"})  # a status round trip orders the frame first
            assert client.recv_json()["type"] == "recording"
            assert pushed.pushed == 1
            client.send_json({"type": "record", "action": "stop"})
            assert not client.recv_json()["recording"]["active"]
        finally:
            client.close()
            server.recorder = None

    def test_second_operator_is_refused(self, server: VRServer) -> None:
        host, port = server.server_address[0], server.server_address[1]
        first = RawClient(host, port, "/ws/teleop")
        first.send_json({"type": "hello"})
        first.recv_json()
        second = RawClient(host, port, "/ws/teleop")
        refused = second.recv_json()
        assert refused["type"] == "error" and "another operator" in refused["message"]
        opcode, payload = second.recv()
        assert opcode == ws.OP_CLOSE
        assert server.status()["refused_operators"] == 1
        second.sock.close()
        first.close()
        assert _wait_until(lambda: server.session is None, 3.0)

    def test_camera_socket_streams_jpeg_frames(self, server: VRServer) -> None:
        host, port = server.server_address[0], server.server_address[1]
        client = RawClient(host, port, "/ws/camera")
        meta = client.recv_json()
        assert meta["type"] == "camera" and meta["fov_deg"] == 69.0
        frames = [client.recv() for _ in range(2)]
        for opcode, payload in frames:
            assert opcode == ws.OP_BINARY and payload[:2] == b"\xff\xd8"
        assert server.status()["camera_clients"] == 1
        client.close()
        assert _wait_until(lambda: server.status()["camera_clients"] == 0, 3.0)

    def test_pinging_operator_outlives_the_idle_timeout(self, server: VRServer) -> None:
        # The page pings once a second before Enter VR; the server must keep
        # the socket (the timeout is for operators who vanished, not for ones
        # still reading the page).
        host, port = server.server_address[0], server.server_address[1]
        client = RawClient(host, port, "/ws/teleop")
        try:
            client.send_json({"type": "hello"})
            assert client.recv_json()["type"] == "hello"
            deadline = time.monotonic() + 2.5 * server.vr_config.teleop_timeout_s
            while time.monotonic() < deadline:
                client.send_json({"type": "ping", "t": 1.0})
                reply = client.recv_json()
                assert reply == {"type": "pong", "t": 1.0}
                time.sleep(server.vr_config.teleop_timeout_s / 3)
            client.send_json({"type": "pose", "head": HEAD0, "left": None, "right": None})
            assert client.recv_json()["type"] == "state"  # still driving
        finally:
            client.close()

    def test_silent_operator_times_out_and_the_arms_hold(self, server: VRServer) -> None:
        host, port = server.server_address[0], server.server_address[1]
        client = RawClient(host, port, "/ws/teleop")
        client.send_json(
            {
                "type": "pose",
                "head": HEAD0,
                "left": controller(0, 1, -0.3, clutch=True),
                "right": None,
            }
        )
        client.recv_json()
        assert server.arm_teleop is not None and server.arm_teleop.arms["left"].clutched
        # Say nothing for longer than teleop_timeout_s.
        assert _wait_until(lambda: server.session is None, 4.0)
        assert not server.arm_teleop.arms["left"].clutched
        client.sock.close()


# --- hardware path on simulated buses -------------------------------------------

MOTOR_TO_URDF: Dict[str, str | None] = {
    "torso_yaw": SYNTHETIC_TORSO_JOINT,
    **{f"l_arm_{i}": name for i, name in enumerate(SYNTHETIC_ARM_JOINTS["left"])},
    **{f"r_arm_{i}": name for i, name in enumerate(SYNTHETIC_ARM_JOINTS["right"])},
    "head_yaw": SYNTHETIC_HEAD_JOINTS[0],
    "head_pitch": SYNTHETIC_HEAD_JOINTS[1],
    "l_arm_grip": None,
    "r_arm_grip": None,
}


def _bus() -> SimulatedDynamixelBus:
    motors = {
        name: DynamixelMotor(index + 1, name, "xm430-w350")
        for index, name in enumerate(MOTOR_TO_URDF)
    }
    return SimulatedDynamixelBus(
        motors, joints={name: SimulatedJoint() for name in MOTOR_TO_URDF}, auto_step=False
    )


def _config(urdf: Path, mode: str = "cartesian_teleop") -> RakudaControlConfig:
    calibration = {
        motor: RakudaJointCalibrationSpec(
            urdf_joint=urdf_joint,
            direction=1,
            zero_count=2048,
            lower_limit_rad=-3.0,
            upper_limit_rad=3.0,
            max_velocity_rad_s=3.0,
            validated=True,
        )
        for motor, urdf_joint in MOTOR_TO_URDF.items()
    }
    return RakudaControlConfig(
        mode=mode,
        control_period_s=0.01,
        ik_period_s=5.0,
        max_state_age_s=5.0,
        max_cross_bus_skew_s=5.0,
        leader_joint_calibration=dict(calibration),
        follower_joint_calibration=dict(calibration),
        model=RakudaModelConfig(
            urdf_path=str(urdf),
            torso_joint=SYNTHETIC_TORSO_JOINT,
            left_arm_joints=list(SYNTHETIC_ARM_JOINTS["left"]),
            right_arm_joints=list(SYNTHETIC_ARM_JOINTS["right"]),
            head_joints=list(SYNTHETIC_HEAD_JOINTS),
            left_tcp=RakudaTcpSpec(
                parent_frame=SYNTHETIC_TCP_FRAMES["left"],
                translation_m=(0, 0, -0.02),
                validated=True,
            ),
            right_tcp=RakudaTcpSpec(
                parent_frame=SYNTHETIC_TCP_FRAMES["right"],
                translation_m=(0, 0, -0.02),
                validated=True,
            ),
            soft_limits_rad=dict(SOFT_LIMITS),
            geometry_only=True,
        ),
        # Cartesian mode refuses to start without every ceiling of the hand
        # reference; these are the simulated plant's, not measurements.
        trajectory=RakudaTrajectoryConfig(
            max_linear_velocity_m_s=0.25,
            max_linear_acceleration_m_s2=1.0,
            max_angular_velocity_rad_s=1.5,
            max_angular_acceleration_rad_s2=6.0,
            lag_tolerance_m=0.02,
        ),
    )


@pytest.fixture
def hardware(synthetic_urdf: Path):  # type: ignore[no-untyped-def]
    leader, follower = _bus(), _bus()
    system = RakudaControlSystem.from_buses(_config(synthetic_urdf), leader, follower)
    system.configure()
    system.align()
    system.prepare_running()
    yield system, follower
    system.stop()


def _run(system: RakudaControlSystem, bus: SimulatedDynamixelBus, steps: int) -> None:
    dt = system.loop.control_period_s
    for _ in range(steps):
        bus.step(dt)
        system.loop.run_once(dt)


class TestControlSystemBackend:
    def test_head_and_gripper_targets_reach_the_follower_bus(self, hardware) -> None:
        system, bus = hardware
        backend = ControlSystemBackend(system, target_ttl_s=5.0)
        assert backend.name == "hardware"
        assert set(backend.joint_positions()) == set(system.model.movable_joint_names)
        assert backend.hand_pose("left").shape == (4, 4)
        report = backend.apply(
            TeleopCommand(
                head_targets_rad={"head_yaw_dof": 0.4, "head_pitch_dof": -0.2},
                gripper_targets_rad={"l_arm_grip": 0.7},
                stamp_s=1.0,
            )
        )
        assert report.warnings == [] and report.gripper_positions_rad == {"l_arm_grip": 0.7}
        _run(system, bus, 150)
        assert bus.joint("head_yaw").position_rad == pytest.approx(0.4, abs=0.03)
        assert bus.joint("head_pitch").position_rad == pytest.approx(-0.2, abs=0.03)
        assert bus.joint("l_arm_grip").position_rad == pytest.approx(0.7, abs=0.03)
        # The arms were never given a target and did not move.
        assert abs(bus.joint("l_arm_2").position_rad) < 1e-6
        assert backend.joint_positions()["head_yaw_dof"] == pytest.approx(0.4, abs=0.03)

    def test_direct_targets_are_validated(self, hardware) -> None:
        system, _bus_ = hardware
        with pytest.raises(ValueError, match="belongs to the arm IK"):
            system.set_direct_targets({"l_arm_0": 0.1})
        with pytest.raises(ValueError, match="not a follower motor"):
            system.set_direct_targets({"nope": 0.1})
        with pytest.raises(ValueError, match="non-finite"):
            system.set_direct_targets({"head_yaw": float("nan")})
        with pytest.raises(ValueError, match="ttl_s"):
            system.set_direct_targets({"head_yaw": 0.1}, ttl_s=0.0)
        backend = ControlSystemBackend(system)
        report = backend.apply(TeleopCommand(head_targets_rad={"not_a_joint": 0.1}))
        assert any("not_a_joint" in w for w in report.warnings)

    def test_direct_targets_expire(self, hardware, monkeypatch: pytest.MonkeyPatch) -> None:
        system, bus = hardware
        calls: list[Dict[str, float]] = []
        original = system.follower.command_positions_rad

        def record(targets: Dict[str, float], **kwargs: Any) -> None:
            calls.append(dict(targets))
            original(targets, **kwargs)

        monkeypatch.setattr(system.follower, "command_positions_rad", record)
        system.set_direct_targets({"head_yaw": 0.2}, ttl_s=0.05)
        _run(system, bus, 1)
        assert calls and calls[-1] == {"head_yaw": 0.2}
        time.sleep(0.08)
        before = len(calls)
        _run(system, bus, 3)
        assert len(calls) == before  # expired: nothing is commanded, the servo holds

    def test_hold_and_mode_check(self, hardware, synthetic_urdf: Path) -> None:
        system, _bus_ = hardware
        backend = ControlSystemBackend(system)
        backend.hold()
        assert system._target is not None  # noqa: SLF001 - observing the effect of hold()
        assert not system._target.left_enabled and not system._target.right_enabled  # noqa: SLF001
        info = backend.describe()
        assert info["mode"] == "cartesian_teleop"
        assert info["motor_for_joint"]["head_yaw_dof"] == "head_yaw"
        assert info["motor_for_joint"]["head_pitch_dof"] == "head_pitch"
        assert "gripper_left_dof" not in info["motor_for_joint"]  # grippers map to no URDF joint
        position_only = RakudaControlSystem.from_buses(
            _config(synthetic_urdf, mode="position_teleop"), _bus(), _bus()
        )
        with pytest.raises(ValueError, match="cartesian_teleop"):
            ControlSystemBackend(position_only)

    def test_a_session_on_the_hardware_backend_drives_the_head_through_the_loop(
        self, hardware
    ) -> None:
        system, bus = hardware
        backend = ControlSystemBackend(system, target_ttl_s=5.0)
        mapping = HeadJointMapping.from_model(
            system.model, "head_yaw_dof", "head_pitch_dof", camera_frame="head_camera_link"
        )
        session = TeleopSession(
            backend,
            head_tracker=HeadTracker(
                mapping, HeadTrackingConfig(filter_hz=None, max_rate_rad_s=100.0)
            ),
            arm_teleop=None,
            config=VRServerConfig(state_hz=1000.0),
        )
        session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.0)
        state = session.handle(
            {"type": "pose", "head": head(yaw=0.5), "left": None, "right": None}, 1.0
        )
        assert state is not None and state["backend"] == "hardware"
        _run(system, bus, 150)
        assert bus.joint("head_yaw").position_rad == pytest.approx(0.5, abs=0.03)
        assert np.isfinite(backend.hand_pose("right")).all()
