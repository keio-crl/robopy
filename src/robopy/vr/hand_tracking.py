"""Hand-tracked poses -> the same clutch-based arm samples a controller gives.

A Meta Quest can track the operator's bare hands (WebXR Hand Input).  The page
then streams the 25 joint poses of each hand instead of a controller pose, and
this module turns them into the :class:`~robopy.vr.arm_teleop.ControllerSample`
that :class:`~robopy.vr.arm_teleop.ArmTeleop` already consumes, so the two
mappings (absolute, relative), the slew limits, the hold on tracking loss and
the recording are all shared with the controllers.  Nothing on the page
interprets the hand: every gesture is decided here, where it can be tested.

What stands in for the controller's buttons:

* **clutch** -- a *pinch* of the thumb and index fingertips (with hysteresis:
  engaged below ``pinch_on_m``, released above ``pinch_off_m``), or, with
  ``clutch_gesture="always"``, the mere fact that the hand is tracked;
* **gripper (the trigger)** -- how far the middle, ring and little fingers are
  *curled* into the palm (``"curl"``, the default: pinch to hold the arm, close
  the rest of the hand to close the gripper), or the index pinch itself
  (``"pinch"``, for the always-on clutch), or nothing;
* **re-centre (both thumbstick clicks)** -- a pinch of the thumb and *middle*
  fingertips on both hands at once;
* **record (B / Y)** -- the same middle pinch on one hand, held for
  ``record_hold_s``.

The hand's *pose* is the wrist joint's orientation with a chosen reference
point as position: the centre of the palm (default), the wrist joint, or the
pinch point between thumb and index tips.  As with the controllers the
orientation is applied relatively, so the wrist's axis convention does not
matter.

The joint names are those of the WebXR Hand Input specification; the page sends
each joint's position in the session's reference space, and the wrist's
orientation.  Frames are converted with :mod:`robopy.vr.xr_math`.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Callable, Dict, Literal, Mapping, Tuple

import numpy as np
from numpy.typing import NDArray

from .arm_teleop import ControllerSample
from .xr_math import xr_pose_to_robot, xr_vector_to_robot

__all__ = [
    "CURL_FINGERS",
    "FINGERS",
    "HAND_JOINTS",
    "HandEvents",
    "HandFrame",
    "HandInput",
    "HandReading",
    "HandTrackingConfig",
    "TwoHandGestures",
    "operator_transform",
]

#: The 25 joints of a WebXR hand, in the specification's order.
HAND_JOINTS: Tuple[str, ...] = (
    "wrist",
    "thumb-metacarpal",
    "thumb-phalanx-proximal",
    "thumb-phalanx-distal",
    "thumb-tip",
    "index-finger-metacarpal",
    "index-finger-phalanx-proximal",
    "index-finger-phalanx-intermediate",
    "index-finger-phalanx-distal",
    "index-finger-tip",
    "middle-finger-metacarpal",
    "middle-finger-phalanx-proximal",
    "middle-finger-phalanx-intermediate",
    "middle-finger-phalanx-distal",
    "middle-finger-tip",
    "ring-finger-metacarpal",
    "ring-finger-phalanx-proximal",
    "ring-finger-phalanx-intermediate",
    "ring-finger-phalanx-distal",
    "ring-finger-tip",
    "pinky-finger-metacarpal",
    "pinky-finger-phalanx-proximal",
    "pinky-finger-phalanx-intermediate",
    "pinky-finger-phalanx-distal",
    "pinky-finger-tip",
)

#: The joints of each finger from the knuckle to the tip (the metacarpal does
#: not flex, so it takes no part in the curl).
FINGERS: Dict[str, Tuple[str, ...]] = {
    "index": (
        "index-finger-phalanx-proximal",
        "index-finger-phalanx-intermediate",
        "index-finger-phalanx-distal",
        "index-finger-tip",
    ),
    "middle": (
        "middle-finger-phalanx-proximal",
        "middle-finger-phalanx-intermediate",
        "middle-finger-phalanx-distal",
        "middle-finger-tip",
    ),
    "ring": (
        "ring-finger-phalanx-proximal",
        "ring-finger-phalanx-intermediate",
        "ring-finger-phalanx-distal",
        "ring-finger-tip",
    ),
    "pinky": (
        "pinky-finger-phalanx-proximal",
        "pinky-finger-phalanx-intermediate",
        "pinky-finger-phalanx-distal",
        "pinky-finger-tip",
    ),
}

#: Fingers whose curl is the gripper signal: the ones a pinch leaves free.
CURL_FINGERS: Tuple[str, ...] = ("middle", "ring", "pinky")


@dataclass
class HandTrackingConfig:
    """How the hands are read.

    Attributes:
        clutch_gesture: ``"pinch"`` (thumb and index tips together drive the
            arm; the default) or ``"always"`` (the arm follows whenever the
            hand is tracked -- with the absolute mapping the hand then goes
            to the operator's the moment it is seen).
        gripper_gesture: ``"curl"`` (middle, ring and little fingers closed
            into the palm = closed gripper), ``"pinch"`` (the index pinch, fully
            open at ``pinch_open_m``) or ``"none"``.  As with the trigger the
            gripper only moves when its travel has been measured.
        reference: Which point is the hand's position: ``"palm"`` (between the
            wrist and the middle knuckle), ``"wrist"`` or ``"pinch"`` (between
            the thumb and index tips).
        pinch_on_m: Thumb-to-index (or middle) tip distance below which a pinch
            engages.
        pinch_off_m: Distance above which an engaged pinch releases.  The gap
            to ``pinch_on_m`` is the hysteresis that keeps a held pinch from
            chattering.
        pinch_open_m: For ``gripper_gesture="pinch"``: the tip distance at
            which the gripper is fully open (fully closed at ``pinch_on_m``).
        curl_open_ratio: Tip-to-knuckle distance over the finger's length at
            which a finger counts as straight (gripper open).
        curl_closed_ratio: The ratio at which it counts as fully curled.
        record_hold_s: How long a one-handed middle pinch is held to toggle
            the recording.
    """

    clutch_gesture: Literal["pinch", "always"] = "pinch"
    gripper_gesture: Literal["curl", "pinch", "none"] = "curl"
    reference: Literal["palm", "wrist", "pinch"] = "palm"
    pinch_on_m: float = 0.02
    pinch_off_m: float = 0.035
    pinch_open_m: float = 0.08
    curl_open_ratio: float = 0.9
    curl_closed_ratio: float = 0.45
    record_hold_s: float = 1.0

    def __post_init__(self) -> None:
        if self.clutch_gesture not in ("pinch", "always"):
            raise ValueError("clutch_gesture must be 'pinch' or 'always'.")
        if self.gripper_gesture not in ("curl", "pinch", "none"):
            raise ValueError("gripper_gesture must be 'curl', 'pinch' or 'none'.")
        if self.reference not in ("palm", "wrist", "pinch"):
            raise ValueError("reference must be 'palm', 'wrist' or 'pinch'.")
        if self.clutch_gesture == "pinch" and self.gripper_gesture == "pinch":
            raise ValueError(
                "the index pinch cannot be both the clutch and the gripper; use "
                "gripper_gesture='curl' (or 'none'), or clutch_gesture='always'."
            )
        if not 0.0 < self.pinch_on_m < self.pinch_off_m:
            raise ValueError("Need 0 < pinch_on_m < pinch_off_m.")
        if self.pinch_open_m <= self.pinch_off_m:
            raise ValueError("pinch_open_m must be above pinch_off_m.")
        if not 0.0 < self.curl_closed_ratio < self.curl_open_ratio <= 1.0:
            raise ValueError("Need 0 < curl_closed_ratio < curl_open_ratio <= 1.")
        if self.record_hold_s <= 0.0:
            raise ValueError("record_hold_s must be positive.")

    @property
    def required_joints(self) -> Tuple[str, ...]:
        """The joints a hand must report for this configuration to read it."""
        joints = ["wrist", "thumb-tip", "index-finger-tip", "middle-finger-tip"]
        if self.reference == "palm":
            joints.append("middle-finger-phalanx-proximal")
        if self.gripper_gesture == "curl":
            for finger in CURL_FINGERS:
                joints.extend(FINGERS[finger])
        return tuple(dict.fromkeys(joints))

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly settings, for the page and the recording header."""
        return {
            "clutch_gesture": self.clutch_gesture,
            "gripper_gesture": self.gripper_gesture,
            "reference": self.reference,
            "pinch_on_m": self.pinch_on_m,
            "pinch_off_m": self.pinch_off_m,
            "pinch_open_m": self.pinch_open_m,
            "curl_open_ratio": self.curl_open_ratio,
            "curl_closed_ratio": self.curl_closed_ratio,
            "record_hold_s": self.record_hold_s,
        }


def _finite_triplet(value: Any) -> NDArray[np.float64] | None:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        return None
    if not all(isinstance(v, (int, float)) and np.isfinite(v) for v in value):
        return None
    return np.asarray(value, dtype=np.float64)


@dataclass(frozen=True)
class HandFrame:
    """One hand's joints, in some right-handed metric frame.

    Attributes:
        positions: ``{joint: (3,)}`` for the joints that were reported.
        wrist_rotation: ``(3, 3)`` orientation of the wrist joint.
    """

    positions: Mapping[str, NDArray[np.float64]]
    wrist_rotation: NDArray[np.float64]

    @classmethod
    def from_message(cls, entry: Any) -> HandFrame | None:
        """Parse the page's ``{"joints": {name: {"p": [3], "q": [4]?}}}`` into robot axes.

        Joint positions may also be bare ``[x, y, z]`` lists.  The wrist must
        carry an orientation.  Returns ``None`` when the entry is not a hand
        at all (no usable wrist); other malformed joints are simply dropped.
        """
        if not isinstance(entry, dict):
            return None
        joints = entry.get("joints")
        if not isinstance(joints, dict):
            return None
        wrist = joints.get("wrist")
        if not isinstance(wrist, dict):
            return None
        p, q = _finite_triplet(wrist.get("p")), wrist.get("q")
        if p is None or not isinstance(q, (list, tuple)) or len(q) != 4:
            return None
        if not all(isinstance(v, (int, float)) and np.isfinite(v) for v in q):
            return None
        try:
            T_wrist = xr_pose_to_robot(list(p), list(q))
        except ValueError:
            return None
        positions: Dict[str, NDArray[np.float64]] = {"wrist": T_wrist[:3, 3].copy()}
        for name, joint in joints.items():
            if name == "wrist" or not isinstance(name, str):
                continue
            raw = joint.get("p") if isinstance(joint, dict) else joint
            triplet = _finite_triplet(raw)
            if triplet is not None:
                positions[name] = xr_vector_to_robot(triplet.tolist())
        return cls(positions=positions, wrist_rotation=T_wrist[:3, :3].copy())

    def transformed(self, T: NDArray[np.float64]) -> HandFrame:
        """The same hand expressed through the rigid transform ``T`` (``(4, 4)``)."""
        T = np.asarray(T, dtype=np.float64)
        R, t = T[:3, :3], T[:3, 3]
        return HandFrame(
            positions={name: R @ p + t for name, p in self.positions.items()},
            wrist_rotation=R @ self.wrist_rotation,
        )

    def has(self, *joints: str) -> bool:
        """Whether every named joint was reported."""
        return all(j in self.positions for j in joints)

    def distance(self, a: str, b: str) -> float:
        """Distance between two joints (``KeyError`` when one is missing)."""
        return float(np.linalg.norm(self.positions[a] - self.positions[b]))

    def pinch_distance(self, finger: str = "index") -> float:
        """Thumb tip to the named finger's tip."""
        return self.distance("thumb-tip", f"{finger}-finger-tip")

    def curl_ratio(self, finger: str) -> float:
        """Tip-to-knuckle distance over the finger's length: ~1 straight, small when curled."""
        chain = FINGERS[finger]
        length = sum(self.distance(a, b) for a, b in zip(chain[:-1], chain[1:]))
        if length < 1e-6:
            return 1.0
        return self.distance(chain[0], chain[-1]) / length

    def reference_pose(self, reference: str) -> NDArray[np.float64]:
        """``(4, 4)`` hand pose: the wrist's orientation at the chosen reference point."""
        if reference == "wrist":
            p = self.positions["wrist"]
        elif reference == "palm":
            p = 0.5 * (self.positions["wrist"] + self.positions["middle-finger-phalanx-proximal"])
        elif reference == "pinch":
            p = 0.5 * (self.positions["thumb-tip"] + self.positions["index-finger-tip"])
        else:
            raise ValueError(f"unknown hand reference {reference!r}")
        T = np.eye(4)
        T[:3, :3] = self.wrist_rotation
        T[:3, 3] = p
        return T


@dataclass(frozen=True)
class HandReading:
    """What one hand said this step.

    Attributes:
        sample: The controller-equivalent sample, or ``None`` when the hand is
            not tracked (or not readable), which releases the arm's clutch.
        tracked: Whether a usable hand frame arrived.
        pinch: Whether the index pinch is engaged (with hysteresis).
        pinch_m: Thumb-to-index tip distance, if measured.
        middle_pinch: Whether the middle pinch is engaged.
        middle_pinch_m: Thumb-to-middle tip distance, if measured.
        curl: Mean curl of the free fingers in ``[0, 1]``, if measured.
        gripper: The trigger-equivalent in ``[0, 1]`` (1 = closed).
        problem: Why the hand could not be read, if it could not.
    """

    sample: ControllerSample | None
    tracked: bool
    pinch: bool = False
    pinch_m: float | None = None
    middle_pinch: bool = False
    middle_pinch_m: float | None = None
    curl: float | None = None
    gripper: float = 0.0
    problem: str | None = None

    def describe(self) -> Dict[str, Any]:
        """JSON-friendly state for the page's HUD and the recording."""
        return {
            "tracked": self.tracked,
            "pinch": self.pinch,
            "pinch_m": self.pinch_m,
            "middle_pinch": self.middle_pinch,
            "middle_pinch_m": self.middle_pinch_m,
            "curl": self.curl,
            "gripper": self.gripper,
            "problem": self.problem,
        }


class _Pinch:
    """A pinch with hysteresis."""

    def __init__(self, on_m: float, off_m: float) -> None:
        self.on_m, self.off_m = on_m, off_m
        self.engaged = False

    def update(self, distance_m: float | None) -> bool:
        if distance_m is None:
            self.engaged = False
        elif self.engaged:
            self.engaged = distance_m <= self.off_m
        else:
            self.engaged = distance_m < self.on_m
        return self.engaged


class HandInput:
    """One hand -> :class:`~robopy.vr.arm_teleop.ControllerSample`, with gesture state."""

    def __init__(self, side: str, config: HandTrackingConfig | None = None) -> None:
        """Bind the reader to one hand.

        Args:
            side: ``"left"`` or ``"right"``; for reporting.
            config: Gesture settings; defaults otherwise.
        """
        if side not in ("left", "right"):
            raise ValueError("side must be 'left' or 'right'.")
        self.side = side
        self.config = config or HandTrackingConfig()
        self._pinch = _Pinch(self.config.pinch_on_m, self.config.pinch_off_m)
        self._middle = _Pinch(self.config.pinch_on_m, self.config.pinch_off_m)
        self.last = HandReading(sample=None, tracked=False)

    def reset(self) -> None:
        """Forget any engaged gesture (the hand was lost, or the operator re-centred)."""
        self._pinch.engaged = False
        self._middle.engaged = False
        self.last = HandReading(sample=None, tracked=False)

    def update(self, frame: HandFrame | None, now_s: float) -> HandReading:
        """Read the hand for this step.

        Args:
            frame: The hand in the operator frame, or ``None`` when it is not
                tracked.  A lost hand releases every gesture at once: the arm
                must not keep following a pose that is no longer measured.
            now_s: Monotonic time, seconds.
        """
        c = self.config
        if frame is None:
            self.reset()
            return self.last
        missing = [j for j in c.required_joints if j not in frame.positions]
        if missing:
            self.reset()
            self.last = HandReading(
                sample=None, tracked=False, problem=f"joints not tracked: {', '.join(missing)}"
            )
            return self.last
        pinch_m = frame.pinch_distance("index")
        middle_m = frame.pinch_distance("middle")
        pinch = self._pinch.update(pinch_m)
        middle = self._middle.update(middle_m)
        curl: float | None = None
        gripper = 0.0
        if c.gripper_gesture == "curl":
            span = c.curl_open_ratio - c.curl_closed_ratio
            curls = [
                min(1.0, max(0.0, (c.curl_open_ratio - frame.curl_ratio(f)) / span))
                for f in CURL_FINGERS
            ]
            curl = float(np.mean(curls))
            gripper = curl
        elif c.gripper_gesture == "pinch":
            span = c.pinch_open_m - c.pinch_on_m
            gripper = min(1.0, max(0.0, (c.pinch_open_m - pinch_m) / span))
        clutch = pinch if c.clutch_gesture == "pinch" else True
        sample = ControllerSample(
            pose=frame.reference_pose(c.reference),
            clutch=clutch,
            trigger=gripper,
            buttons={"pinch": pinch, "middle_pinch": middle},
            stamp_s=now_s,
        )
        self.last = HandReading(
            sample=sample,
            tracked=True,
            pinch=pinch,
            pinch_m=pinch_m,
            middle_pinch=middle,
            middle_pinch_m=middle_m,
            curl=curl,
            gripper=gripper,
        )
        return self.last


@dataclass(frozen=True)
class HandEvents:
    """Session-level gestures raised this step."""

    recenter: bool = False
    record_toggle: bool = False


@dataclass
class _Hold:
    started_s: float | None = None
    fired: bool = False


class TwoHandGestures:
    """Both hands' readings -> re-centre and record events.

    Re-centre fires on the edge where both hands come to pinch thumb and middle
    fingertips together.  Record toggles when one hand alone holds that pinch
    for ``record_hold_s``; a hold that turns into the two-handed gesture, or
    outlives it, does not also toggle the recording.
    """

    def __init__(self, config: HandTrackingConfig | None = None) -> None:
        self.config = config or HandTrackingConfig()
        self._both = False
        self._holds: Dict[str, _Hold] = {"left": _Hold(), "right": _Hold()}

    def reset(self) -> None:
        """Forget every gesture in progress."""
        self._both = False
        self._holds = {"left": _Hold(), "right": _Hold()}

    def update(self, readings: Mapping[str, HandReading | None], now_s: float) -> HandEvents:
        """Advance one step with this frame's readings (a missing side is untracked)."""
        pinched = {
            side: bool(r is not None and r.tracked and r.middle_pinch)
            for side in ("left", "right")
            for r in (readings.get(side),)
        }
        both = pinched["left"] and pinched["right"]
        recenter = both and not self._both
        self._both = both
        record = False
        for side, hold in self._holds.items():
            if not pinched[side]:
                hold.started_s, hold.fired = None, False
                continue
            if both:
                # The two-handed gesture takes precedence, and stays taken
                # until this hand lets go.
                hold.started_s, hold.fired = None, True
                continue
            if hold.fired:
                continue
            if hold.started_s is None:
                hold.started_s = now_s
            elif now_s - hold.started_s >= self.config.record_hold_s:
                hold.fired = True
                record = True
        return HandEvents(recenter=recenter, record_toggle=record)


def operator_transform(
    to_operator: Callable[[NDArray[np.float64]], NDArray[np.float64]],
) -> NDArray[np.float64]:
    """The ``(4, 4)`` matrix of a rigid pose map such as ``OperatorFrame.to_operator``.

    Applying the map to the identity yields its rotation and translation, so
    ``to_operator(P) == operator_transform(to_operator) @ P`` for any pose.
    """
    return np.asarray(to_operator(np.eye(4)), dtype=np.float64)
