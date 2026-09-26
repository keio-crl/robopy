"""Browser-level test of the VR page in desktop-preview mode.

Headless Chromium has no WebXR device, so what can be verified without a
headset is: the page loads without script errors, connects both sockets,
draws the robot twin, receives camera frames, and -- in "desktop preview" --
streams the window camera's orientation as a head pose that the server turns
into head joint targets.  Skipped unless Playwright is importable (see
``test_viewer_browser.py`` for how to run it).
"""

from __future__ import annotations

import os
import threading
from pathlib import Path

import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")
playwright_api = pytest.importorskip("playwright.sync_api", reason="browser tests need playwright")

from robopy.viewer.model_bundle import ModelBundle  # noqa: E402
from robopy.viewer.server import IKSetup  # noqa: E402
from robopy.vr.__main__ import STREAMING_IK_OVERRIDES  # noqa: E402
from robopy.vr.arm_teleop import DualArmTeleop  # noqa: E402
from robopy.vr.backend import SimulationBackend  # noqa: E402
from robopy.vr.camera import FrameStreamer, SyntheticFrameSource  # noqa: E402
from robopy.vr.head_tracking import HeadJointMapping, HeadTracker, HeadTrackingConfig  # noqa: E402
from robopy.vr.server import VRServer, VRServerConfig  # noqa: E402

SOFT_LIMITS = {
    "torso_yaw_dof": (-1.5, 1.5),
    "shoulder_pitch_left_dof": (-2.0, 2.0),
    "shoulder_pitch_right_dof": (-2.0, 2.0),
}
OVERLAY_HIDDEN = "() => { const o = document.querySelector('#overlay'); return !!o && o.hidden; }"


@pytest.fixture(scope="module")
def server(synthetic_urdf: Path):  # type: ignore[no-untyped-def]
    bundle = ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)
    ik = IKSetup(bundle, config_overrides=STREAMING_IK_OVERRIDES)
    mapping = HeadJointMapping.from_model(
        bundle.model, "head_yaw_dof", "head_pitch_dof", camera_frame="head_camera_link"
    )
    srv = VRServer(
        bundle,
        ik=ik,
        backend=SimulationBackend(bundle, ik),
        head_tracker=HeadTracker(mapping, HeadTrackingConfig(filter_hz=None, max_rate_rad_s=100.0)),
        arm_teleop=DualArmTeleop(),
        camera=FrameStreamer(SyntheticFrameSource(160, 120), fps=30.0),
        host="127.0.0.1",
        port=0,
        config=VRServerConfig(state_hz=60.0, teleop_timeout_s=2.0),
    )
    assert srv.camera is not None
    srv.camera.start()
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    yield srv
    srv.shutdown()
    srv.server_close()


@pytest.fixture(scope="module")
def browser():  # type: ignore[no-untyped-def]
    kwargs = {}
    chromium = os.environ.get("ROBOPY_CHROMIUM")
    if chromium:
        kwargs["executable_path"] = chromium
    with playwright_api.sync_playwright() as p:
        try:
            b = p.chromium.launch(**kwargs)
        except Exception as exc:  # pragma: no cover - environment dependent
            pytest.skip(f"cannot launch chromium: {exc}")
        yield b
        b.close()


def test_preview_mode_streams_head_poses_and_draws_the_twin(server: VRServer, browser) -> None:  # type: ignore[no-untyped-def]
    page = browser.new_page(viewport={"width": 1200, "height": 800})
    errors: list[str] = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(server.vr_url)
    page.wait_for_function(OVERLAY_HIDDEN, timeout=60_000)
    page.wait_for_function("() => window.__robopy_vr.frames > 0", timeout=20_000)
    page.click("#preview")
    page.wait_for_function("() => window.__robopy_vr.received > 5", timeout=20_000)
    # Orbit the desktop camera: the head must follow the window's yaw.
    page.mouse.move(600, 400)
    page.mouse.down()
    page.mouse.move(450, 400, steps=10)
    page.mouse.up()
    page.wait_for_function(
        "() => { const s = window.__robopy_vr.lastState; return !!(s && s.head && s.head.tracking"
        " && Math.abs(s.head.yaw_input_rad) > 0.1); }",
        timeout=20_000,
    )
    info = page.evaluate(
        """() => { const s = window.__robopy_vr; const st = s.lastState;
          return { meshes: s.meshes.filter(Boolean).length, frames: s.frames, sent: s.sent,
                   backend: st.backend, yaw_input: st.head.yaw_input_rad,
                   yaw_joint: st.joints.head_yaw_dof, recentred: st.operator.recentred,
                   xrText: document.querySelector('#xr-support').textContent }; }"""
    )
    try:
        assert errors == []
        assert info["meshes"] == len(server.bundle.geometries)
        assert info["frames"] > 0 and info["sent"] > 5
        assert info["backend"] == "simulation" and info["recentred"]
        # Synthetic head: yaw sign +1, so the joint follows the input directly.
        assert info["yaw_joint"] == pytest.approx(info["yaw_input"], abs=1e-6)
        # Headless Chromium reports the check ran; a headset browser would say "available".
        assert any(
            text in info["xrText"]
            for text in ("WebXR", "navigator.xr", "immersive-vr", "secure context")
        )
    finally:
        page.close()


def test_idle_page_keeps_its_socket_until_the_operator_enters_vr(server: VRServer, browser) -> None:  # type: ignore[no-untyped-def]
    # Poses only flow while presenting or previewing.  A page that just sits
    # there (the operator putting the headset on) must not be dropped by the
    # server's idle timeout, or Enter VR later drives a dead socket.
    page = browser.new_page(viewport={"width": 1200, "height": 800})
    errors: list[str] = []
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.goto(server.vr_url)
    page.wait_for_function(OVERLAY_HIDDEN, timeout=60_000)
    page.wait_for_timeout(3 * server.vr_config.teleop_timeout_s * 1000)
    try:
        assert errors == []
        assert page.evaluate("() => window.__robopy_vr.connected") is True
        assert server.status()["session"] is not None, "the server dropped the idle operator"
        page.click("#preview")
        page.wait_for_function("() => window.__robopy_vr.received > 5", timeout=20_000)
        assert page.evaluate("() => window.__robopy_vr.mirrorOn") is True
    finally:
        page.close()


def test_hand_joints_are_sent_the_way_the_server_reads_them(server: VRServer, browser) -> None:  # type: ignore[no-untyped-def]
    """The page's hand entry, built from a stand-in XRFrame, parses on the server.

    Headless Chromium has no hands to track, so the WebXR objects are faked:
    a map-like ``hand`` of joint names and a frame whose ``getJointPose``
    returns a pose per joint.  What matters is the contract: the message the
    page builds is what :class:`robopy.vr.hand_tracking.HandFrame` expects.
    """
    from robopy.vr.hand_tracking import HAND_JOINTS, HandFrame, HandInput

    page = browser.new_page()
    errors: list[str] = []
    page.on("pageerror", lambda exc: errors.append(str(exc)))
    page.goto(server.vr_url)
    page.wait_for_function(OVERLAY_HIDDEN, timeout=30_000)
    entry = page.evaluate(
        """(names) => {
          const hand = new Map(names.map((n) => [n, { name: n }]));
          const frame = {
            getJointPose: (space) => {
              if (space.name === 'ring-finger-tip') return null;   // an untracked joint
              const i = names.indexOf(space.name);
              return {
                radius: 0.009,
                transform: {
                  position: { x: 0.1 * i, y: 1.2, z: -0.3 - 0.001 * i },
                  orientation: { x: 0, y: 0, z: 0, w: 1 },
                },
              };
            },
          };
          return window.__robopy_vr.handEntry(frame, null, { hand, handedness: 'left' });
        }""",
        list(HAND_JOINTS),
    )
    assert errors == []
    joints = entry["hand"]["joints"]
    assert set(joints) == set(HAND_JOINTS) - {"ring-finger-tip"}
    assert joints["wrist"] == {"p": [0.0, 1.2, -0.3], "q": [0, 0, 0, 1]}
    assert "q" not in joints["thumb-tip"]
    frame = HandFrame.from_message(entry["hand"])
    assert frame is not None and frame.has("wrist", "thumb-tip", "pinky-finger-tip")
    # The missing joint is exactly what the server reports back.
    reading = HandInput("left").update(frame, 0.0)
    assert not reading.tracked and reading.problem == "joints not tracked: ring-finger-tip"
    # And the joint spheres were drawn (all but the untracked one).
    visible = page.evaluate(
        "() => Object.values(window.__robopy_vr.hands.left.joints).filter((m) => m.visible).length"
    )
    assert visible == len(HAND_JOINTS) - 1
    page.close()
