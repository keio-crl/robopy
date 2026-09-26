"""Hand tracking in :mod:`robopy.vr`: gestures, frames, and the session end to end.

The gesture and frame tests are pure numpy.  The session tests need the
``kinematics`` extra and feed :class:`~robopy.vr.server.TeleopSession` the
messages the page sends for a tracked hand.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Dict

import numpy as np
import pytest

from robopy.vr.hand_tracking import (
    HAND_JOINTS,
    HandFrame,
    HandInput,
    HandReading,
    HandTrackingConfig,
    TwoHandGestures,
)
from robopy.vr.xr_math import OperatorFrame, xr_pose_to_robot

# --------------------------------------------------------------- a synthetic hand
# Laid out in WebXR axes (+Y up, -Z forward): the fingers point forward from
# the wrist, 3 cm apart along +X, 10 cm knuckle to tip when straight.
FINGER_X = {"index": -0.03, "middle": 0.0, "ring": 0.03, "pinky": 0.06}
KNUCKLE_Z = -0.09
SEGMENTS_M = (0.04, 0.03, 0.03)


def hand_joints(
    wrist: Any = (0.0, 1.2, -0.3),
    *,
    pinch: bool = False,
    middle_pinch: bool = False,
    curl: float = 0.0,
    q: Any = (0.0, 0.0, 0.0, 1.0),
    drop: tuple[str, ...] = (),
) -> Dict[str, Any]:
    """The ``joints`` dictionary the page sends for one hand.

    ``curl`` 0 lays the middle, ring and little fingers straight, 1 folds them
    into the palm (each joint bent by 100 degrees); ``pinch`` puts the thumb
    tip 5 mm from the index tip, ``middle_pinch`` 5 mm from the middle tip.
    """
    w = np.asarray(wrist, dtype=np.float64)
    joints: Dict[str, Any] = {"wrist": {"p": list(w), "q": list(q)}}
    for finger, x in FINGER_X.items():
        joints[f"{finger}-finger-metacarpal"] = list(w + [x, 0.0, KNUCKLE_Z / 2])
        c = curl if finger != "index" else 0.0
        # Three segments; each joint bends the next segment by another
        # c * 100 degrees, from -Z (forward) towards -Y (into the palm).
        chain = [w + [x, 0.0, KNUCKLE_Z]]
        for k, length in enumerate(SEGMENTS_M):
            angle = math.radians(100.0) * c * k
            chain.append(chain[-1] + length * np.array([0.0, -math.sin(angle), -math.cos(angle)]))
        for name, point in zip(
            ("phalanx-proximal", "phalanx-intermediate", "phalanx-distal", "tip"), chain
        ):
            joints[f"{finger}-finger-{name}"] = list(point)
    thumb_tip = w + [0.09, 0.0, -0.03]  # well away from every fingertip
    if pinch:
        thumb_tip = np.asarray(joints["index-finger-tip"]) + [-0.005, 0.0, 0.0]
    if middle_pinch:
        thumb_tip = np.asarray(joints["middle-finger-tip"]) + [0.005, 0.0, 0.0]
    joints["thumb-metacarpal"] = list(w + [0.03, 0.0, -0.02])
    joints["thumb-phalanx-proximal"] = list(w + [0.045, 0.0, -0.035])
    joints["thumb-phalanx-distal"] = list((thumb_tip + w + [0.045, 0.0, -0.035]) / 2)
    joints["thumb-tip"] = list(thumb_tip)
    for name in drop:
        joints.pop(name)
    return joints


def hand_entry(**kwargs: Any) -> Dict[str, Any]:
    """The ``left``/``right`` entry of a pose message for a tracked hand."""
    return {"hand": {"joints": hand_joints(**kwargs)}}


def reading(*, tracked: bool = True, middle_pinch: bool = False) -> HandReading:
    return HandReading(sample=None, tracked=tracked, middle_pinch=middle_pinch)


class TestConfig:
    def test_defaults_and_required_joints(self) -> None:
        cfg = HandTrackingConfig()
        assert cfg.clutch_gesture == "pinch" and cfg.gripper_gesture == "curl"
        required = cfg.required_joints
        assert "wrist" in required and "thumb-tip" in required
        assert "pinky-finger-tip" in required and "index-finger-phalanx-distal" not in required
        assert set(required) <= set(HAND_JOINTS)
        lean = HandTrackingConfig(gripper_gesture="none", reference="wrist").required_joints
        assert set(lean) == {"wrist", "thumb-tip", "index-finger-tip", "middle-finger-tip"}

    def test_validation(self) -> None:
        with pytest.raises(ValueError, match="both the clutch and the gripper"):
            HandTrackingConfig(clutch_gesture="pinch", gripper_gesture="pinch")
        HandTrackingConfig(clutch_gesture="always", gripper_gesture="pinch")
        with pytest.raises(ValueError, match="pinch_on_m"):
            HandTrackingConfig(pinch_on_m=0.04, pinch_off_m=0.03)
        with pytest.raises(ValueError, match="pinch_open_m"):
            HandTrackingConfig(pinch_open_m=0.03)
        with pytest.raises(ValueError, match="curl"):
            HandTrackingConfig(curl_open_ratio=0.4, curl_closed_ratio=0.5)
        with pytest.raises(ValueError, match="reference"):
            HandTrackingConfig(reference="elbow")  # type: ignore[arg-type]
        with pytest.raises(ValueError, match="record_hold_s"):
            HandTrackingConfig(record_hold_s=0.0)
        assert HandTrackingConfig().describe()["reference"] == "palm"


class TestHandFrame:
    def test_parses_the_page_message_into_robot_axes(self) -> None:
        frame = HandFrame.from_message({"joints": hand_joints(wrist=(0.1, 1.2, -0.3))})
        assert frame is not None
        # WebXR (x, y, z) -> robot (-z, -x, y).
        assert np.allclose(frame.positions["wrist"], [0.3, -0.1, 1.2])
        # A fingertip 10 cm + 9 cm ahead of the wrist (WebXR -Z) is at robot +X.
        tip = frame.positions["middle-finger-tip"]
        assert np.allclose(tip, [0.3 + 0.19, -0.1, 1.2])
        assert np.allclose(frame.wrist_rotation, np.eye(3))
        # Bare [x, y, z] lists are accepted for the other joints; junk is dropped.
        joints = hand_joints()
        joints["thumb-tip"] = joints["thumb-tip"]  # already a list
        joints["ring-finger-tip"] = {"p": [0.0, float("nan"), 0.0]}
        joints["pinky-finger-tip"] = "nope"
        frame = HandFrame.from_message({"joints": joints})
        assert frame is not None
        assert "thumb-tip" in frame.positions
        assert (
            "ring-finger-tip" not in frame.positions and "pinky-finger-tip" not in frame.positions
        )

    def test_rejects_entries_without_a_usable_wrist(self) -> None:
        assert HandFrame.from_message(None) is None
        assert HandFrame.from_message({"joints": {}}) is None
        assert HandFrame.from_message({"joints": {"wrist": [0, 1, 2]}}) is None  # no q
        assert (
            HandFrame.from_message({"joints": {"wrist": {"p": [0, 1], "q": [0, 0, 0, 1]}}}) is None
        )
        assert (
            HandFrame.from_message({"joints": {"wrist": {"p": [0, 1, 2], "q": [0, 0, 0, 0]}}})
            is None
        )

    def test_reference_points_and_rigid_transform(self) -> None:
        frame = HandFrame.from_message({"joints": hand_joints(wrist=(0.0, 1.0, 0.0), pinch=True)})
        assert frame is not None
        wrist = frame.reference_pose("wrist")
        palm = frame.reference_pose("palm")
        pinch = frame.reference_pose("pinch")
        assert np.allclose(wrist[:3, 3], [0.0, 0.0, 1.0])
        # Palm: halfway to the middle knuckle, 9 cm ahead -> 4.5 cm ahead.
        assert np.allclose(palm[:3, 3], [0.045, 0.0, 1.0])
        # Pinch point: between the index tip and the thumb tip (5 mm outside it).
        assert np.allclose(pinch[:3, 3], [0.19, 0.03 + 0.0025, 1.0], atol=1e-9)
        with pytest.raises(ValueError):
            frame.reference_pose("elbow")
        # Transforming the frame transforms every joint and the wrist rotation.
        operator = OperatorFrame()
        operator.recenter(xr_pose_to_robot([1.0, 1.6, 2.0], [0, 0, 0, 1]))
        T = operator.to_operator(np.eye(4))
        moved = frame.transformed(T)
        for name, p in frame.positions.items():
            assert np.allclose(moved.positions[name], T[:3, :3] @ p + T[:3, 3])
        assert np.allclose(moved.wrist_rotation, T[:3, :3] @ frame.wrist_rotation)
        assert moved.pinch_distance("index") == pytest.approx(frame.pinch_distance("index"))
        assert moved.curl_ratio("ring") == pytest.approx(frame.curl_ratio("ring"))

    def test_curl_ratio_spans_straight_to_folded(self) -> None:
        straight = HandFrame.from_message({"joints": hand_joints(curl=0.0)})
        folded = HandFrame.from_message({"joints": hand_joints(curl=1.0)})
        assert straight is not None and folded is not None
        assert straight.curl_ratio("middle") == pytest.approx(1.0)
        assert folded.curl_ratio("middle") < 0.45
        assert folded.curl_ratio("index") == pytest.approx(1.0)  # the index never curls here


def frame_of(**kwargs: Any) -> HandFrame:
    frame = HandFrame.from_message({"joints": hand_joints(**kwargs)})
    assert frame is not None
    return frame


class TestHandInput:
    def test_pinch_is_the_clutch_with_hysteresis(self) -> None:
        hand = HandInput("left", HandTrackingConfig(pinch_on_m=0.02, pinch_off_m=0.035))
        opened = hand.update(frame_of(pinch=False), 0.0)
        assert opened.tracked and not opened.pinch and opened.sample is not None
        assert not opened.sample.clutch and opened.pinch_m is not None and opened.pinch_m > 0.05
        pinched = hand.update(frame_of(pinch=True), 0.1)
        assert pinched.pinch and pinched.sample is not None and pinched.sample.clutch
        assert pinched.sample.buttons == {"pinch": True, "middle_pinch": False}
        # Drift out to 3 cm: still inside the release threshold, still held.
        joints = hand_joints(pinch=True)
        joints["thumb-tip"] = list(np.asarray(joints["index-finger-tip"]) + [0.03, 0.0, 0.0])
        frame = HandFrame.from_message({"joints": joints})
        held = hand.update(frame, 0.2)
        assert held.pinch and held.pinch_m == pytest.approx(0.03)
        # 3 cm would not have engaged a fresh pinch.
        fresh = HandInput("left", HandTrackingConfig(pinch_on_m=0.02, pinch_off_m=0.035))
        assert not fresh.update(frame, 0.0).pinch
        # Past 3.5 cm it lets go.
        joints["thumb-tip"] = list(np.asarray(joints["index-finger-tip"]) + [0.04, 0.0, 0.0])
        released = hand.update(HandFrame.from_message({"joints": joints}), 0.3)
        assert not released.pinch

    def test_curl_of_the_free_fingers_is_the_gripper(self) -> None:
        hand = HandInput("right")
        open_hand = hand.update(frame_of(pinch=True, curl=0.0), 0.0)
        assert open_hand.curl == pytest.approx(0.0) and open_hand.gripper == 0.0
        fist = hand.update(frame_of(pinch=True, curl=1.0), 0.1)
        assert fist.curl == pytest.approx(1.0) and fist.gripper == 1.0
        assert fist.sample is not None and fist.sample.trigger == 1.0 and fist.sample.clutch
        half = hand.update(frame_of(pinch=True, curl=0.5), 0.2)
        assert half.curl is not None and 0.05 < half.curl < 0.95

    def test_always_clutch_with_the_pinch_as_gripper(self) -> None:
        cfg = HandTrackingConfig(
            clutch_gesture="always", gripper_gesture="pinch", pinch_open_m=0.08
        )
        hand = HandInput("left", cfg)
        wide = hand.update(frame_of(pinch=False), 0.0)
        assert wide.sample is not None and wide.sample.clutch and wide.curl is None
        assert wide.gripper == 0.0  # the thumb is > 8 cm from the index tip
        tight = hand.update(frame_of(pinch=True), 0.1)
        assert tight.sample is not None and tight.sample.clutch
        assert tight.gripper == 1.0  # 5 mm < pinch_on_m
        none = HandInput("left", HandTrackingConfig(gripper_gesture="none"))
        assert none.update(frame_of(pinch=True, curl=1.0), 0.0).gripper == 0.0

    def test_reference_point_is_the_sample_pose(self) -> None:
        for reference in ("palm", "wrist", "pinch"):
            hand = HandInput("left", HandTrackingConfig(reference=reference))  # type: ignore[arg-type]
            frame = frame_of(pinch=True)
            out = hand.update(frame, 0.0)
            assert out.sample is not None and out.sample.pose is not None
            assert np.allclose(out.sample.pose, frame.reference_pose(reference))

    def test_lost_or_partial_hand_releases_everything(self) -> None:
        hand = HandInput("left")
        assert hand.update(frame_of(pinch=True, middle_pinch=False), 0.0).pinch
        gone = hand.update(None, 0.1)
        assert not gone.tracked and gone.sample is None and not gone.pinch
        partial = hand.update(frame_of(pinch=True, drop=("pinky-finger-tip",)), 0.2)
        assert not partial.tracked and partial.sample is None
        assert partial.problem is not None and "pinky-finger-tip" in partial.problem
        # Back, and pinching: engages again from scratch.
        assert hand.update(frame_of(pinch=True), 0.3).pinch
        with pytest.raises(ValueError):
            HandInput("middle")

    def test_middle_pinch_is_reported_separately(self) -> None:
        hand = HandInput("left")
        out = hand.update(frame_of(middle_pinch=True), 0.0)
        assert out.middle_pinch and not out.pinch
        assert out.describe()["middle_pinch"] is True


class TestTwoHandGestures:
    def test_both_middle_pinches_recentre_on_the_edge(self) -> None:
        g = TwoHandGestures()
        assert not g.update({"left": reading(middle_pinch=True), "right": reading()}, 0.0).recenter
        first = g.update(
            {"left": reading(middle_pinch=True), "right": reading(middle_pinch=True)}, 0.1
        )
        assert first.recenter and not first.record_toggle
        again = g.update(
            {"left": reading(middle_pinch=True), "right": reading(middle_pinch=True)}, 0.2
        )
        assert not again.recenter  # held, not repeated
        g.update({"left": reading(), "right": reading()}, 0.3)
        assert g.update(
            {"left": reading(middle_pinch=True), "right": reading(middle_pinch=True)}, 0.4
        ).recenter
        # An untracked side never counts as pinching.
        g.reset()
        assert not g.update({"left": reading(middle_pinch=True), "right": None}, 0.5).recenter
        assert not g.update(
            {
                "left": reading(middle_pinch=True),
                "right": reading(tracked=False, middle_pinch=True),
            },
            0.6,
        ).recenter

    def test_one_hand_held_toggles_the_recording_once(self) -> None:
        g = TwoHandGestures(HandTrackingConfig(record_hold_s=1.0))
        held = {"left": reading(middle_pinch=True), "right": reading()}
        assert not g.update(held, 0.0).record_toggle
        assert not g.update(held, 0.9).record_toggle
        assert g.update(held, 1.0).record_toggle
        assert not g.update(held, 5.0).record_toggle  # once per hold
        g.update({"left": reading(), "right": reading()}, 5.1)
        assert not g.update(held, 5.2).record_toggle
        assert g.update(held, 6.3).record_toggle

    def test_the_two_handed_gesture_cancels_and_outlives_a_hold(self) -> None:
        g = TwoHandGestures(HandTrackingConfig(record_hold_s=1.0))
        g.update({"left": reading(middle_pinch=True), "right": reading()}, 0.0)
        both = g.update(
            {"left": reading(middle_pinch=True), "right": reading(middle_pinch=True)}, 0.5
        )
        assert both.recenter and not both.record_toggle
        # The right hand lets go; the left keeps pinching for a long time: no toggle.
        after = g.update({"left": reading(middle_pinch=True), "right": reading()}, 3.0)
        assert not after.record_toggle and not after.recenter
        assert not g.update(
            {"left": reading(middle_pinch=True), "right": reading()}, 9.0
        ).record_toggle


# --------------------------------------------------------------- the session
pink = pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.viewer.model_bundle import ModelBundle  # noqa: E402
from robopy.vr.server import VRServerConfig  # noqa: E402

from .test_vr_server import HEAD0, SOFT_LIMITS, make_session, xr_point  # noqa: E402


@pytest.fixture(scope="module")
def bundle(synthetic_urdf: Path) -> ModelBundle:
    return ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)


def pose(t: float = 0.0, **sides: Any) -> Dict[str, Any]:
    msg: Dict[str, Any] = {"type": "pose", "t": t, "head": HEAD0, "left": None, "right": None}
    msg.update(sides)
    return msg


class TestHandSession:
    def test_hello_and_state_describe_the_hands(self, bundle: ModelBundle) -> None:
        session, _ = make_session(bundle, mapping="absolute")
        hello = session.handle({"type": "hello"}, 0.0)
        assert hello is not None and hello["hands"]["clutch_gesture"] == "pinch"
        assert session.recording_metadata()["hands"]["reference"] == "palm"
        state = session.handle(pose(left=hand_entry(pinch=False)), 0.1)
        assert state is not None
        left = state["arms"]["left"]
        assert left["input"] == "hand" and left["tracked"] and not left["clutched"]
        assert left["hand"]["tracked"] and left["hand"]["pinch"] is False
        assert state["arms"]["right"]["input"] is None and state["arms"]["right"]["hand"] is None

    def test_a_pinched_hand_drives_the_arm_like_a_pressed_controller(
        self, bundle: ModelBundle
    ) -> None:
        session, backend = make_session(bundle, mapping="absolute")
        session.handle(pose(), 0.0)
        anchor = np.asarray(session.robot_anchor_m)
        head = np.array([0.0, 0.0, 1.6])
        start = backend.hand_pose("left")[:3, 3].copy()
        # The palm (4.5 cm ahead of the wrist) goes where the robot's hand is in
        # the operator's body, 4 cm further forward; the wrist is placed so.
        palm_robot = head + (start - anchor) + np.array([0.04, 0.0, 0.0])
        wrist_robot = palm_robot - np.array([0.045, 0.0, 0.0])
        entry = hand_entry(wrist=xr_point(wrist_robot), pinch=True)
        t = 0.0
        for _ in range(100):
            t += 1.0 / 60.0
            state = session.handle(pose(t, left=entry), t)
        assert state is not None
        left = state["arms"]["left"]
        assert left["input"] == "hand" and left["clutched"] and left["enabled"]
        assert left["hand"]["pinch"] and state["ik"]["commandable"]
        assert left["target"]["p"] == pytest.approx(list(start + [0.04, 0.0, 0.0]), abs=1e-9)
        moved = backend.hand_pose("left")[:3, 3] - start
        assert moved[0] == pytest.approx(0.04, abs=0.006)
        assert abs(moved[1]) < 0.01 and abs(moved[2]) < 0.01
        # The recording sees the palm under the same correspondence as the target.
        assert session._last_controller_base["left"] == pytest.approx(
            list(start + [0.04, 0.0, 0.0]), abs=1e-9
        )
        # Open the hand: the clutch releases and the target is latched.
        released = session.handle(
            pose(t + 0.02, left=hand_entry(wrist=xr_point(wrist_robot))), t + 0.02
        )
        assert released is not None
        assert not released["arms"]["left"]["clutched"] and not released["arms"]["left"]["enabled"]
        assert released["arms"]["left"]["target"] is not None
        # Lose the hand altogether: untracked, still held.
        lost = session.handle(pose(t + 0.04), t + 0.04)
        assert lost is not None and not lost["arms"]["left"]["tracked"]
        assert lost["arms"]["left"]["input"] is None

    def test_a_hand_and_a_controller_can_mix(self, bundle: ModelBundle) -> None:
        session, _ = make_session(bundle, mapping="relative")
        session.handle(pose(), 0.0)
        controller = {"p": [0.3, 1.2, -0.4], "q": [0, 0, 0, 1], "clutch": True, "trigger": 0.0}
        state = session.handle(pose(0.1, left=hand_entry(pinch=True), right=controller), 0.1)
        assert state is not None
        assert state["arms"]["left"]["input"] == "hand" and state["arms"]["left"]["clutched"]
        assert state["arms"]["right"]["input"] == "controller"
        assert state["arms"]["right"]["clutched"] and state["arms"]["right"]["hand"] is None

    def test_both_middle_pinches_recentre_and_release(self, bundle: ModelBundle) -> None:
        session, _ = make_session(bundle, mapping="absolute")
        session.handle(pose(), 0.0)
        session.handle(pose(0.1, left=hand_entry(pinch=True)), 0.1)
        assert session.arm_teleop is not None and session.arm_teleop.arms["left"].clutched
        # Turn the head 0.3 rad left, then pinch the middle fingers on both hands.
        turned = {"p": [0.0, 1.6, 0.0], "q": [0.0, math.sin(0.15), 0.0, math.cos(0.15)]}
        both = {"left": hand_entry(middle_pinch=True), "right": hand_entry(middle_pinch=True)}
        state = session.handle({"type": "pose", "t": 0.2, "head": turned, **both}, 0.2)
        assert state is not None and state.get("recentred") is True
        assert state["operator"]["yaw_offset_rad"] == pytest.approx(0.3, abs=1e-6)
        assert not session.arm_teleop.arms["left"].clutched
        # Holding the gesture does not re-centre again; the turned head now reads as level.
        again = session.handle({"type": "pose", "t": 0.3, "head": turned, **both}, 0.3)
        assert again is not None and "recentred" not in again
        assert again["head"]["yaw_input_rad"] == pytest.approx(0.0, abs=1e-6)

    def test_hands_can_be_ignored(self, bundle: ModelBundle) -> None:
        cfg = VRServerConfig(state_hz=1000.0, hands=None)
        session, _ = make_session(bundle, mapping="relative", config=cfg)
        hello = session.handle({"type": "hello"}, 0.0)
        assert hello is not None and hello["hands"] is None
        state = session.handle(pose(0.1, left=hand_entry(pinch=True)), 0.1)
        assert state is not None
        assert state["arms"]["left"]["input"] == "hand"
        assert not state["arms"]["left"]["tracked"] and not state["arms"]["left"]["clutched"]

    def test_hand_gripper_reaches_the_gripper_target(self, bundle: ModelBundle) -> None:
        from robopy.vr.arm_teleop import ArmTeleopConfig, DualArmTeleop

        session, _ = make_session(bundle, mapping="relative")
        assert session.arm_teleop is not None
        session.arm_teleop = DualArmTeleop(
            ArmTeleopConfig(
                mapping="relative",
                max_speed_m_s=100.0,
                gripper_motor="l_grip",
                gripper_open_rad=0.0,
                gripper_closed_rad=1.0,
            ),
            ArmTeleopConfig(mapping="relative"),
        )
        session.handle(pose(), 0.0)
        fist = session.handle(pose(0.1, left=hand_entry(pinch=True, curl=1.0)), 0.1)
        assert fist is not None and fist["arms"]["left"]["gripper_rad"] == pytest.approx(1.0)
        opened = session.handle(pose(0.2, left=hand_entry(pinch=True, curl=0.0)), 0.2)
        assert opened is not None and opened["arms"]["left"]["gripper_rad"] == pytest.approx(0.0)
