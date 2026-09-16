"""Browser-level regression tests for the viewer page.

These drive the real page in headless Chromium through Playwright, which is
not a project dependency, so they are skipped unless it is importable::

    uv run --extra kinematics --with playwright \
        pytest tests/test_rakuda_control/test_viewer_browser.py

The environment variable ``ROBOPY_CHROMIUM`` may point at a Chromium binary
(otherwise Playwright's own download is used).
"""

from __future__ import annotations

import os
import threading
import time
from pathlib import Path

import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")
playwright_api = pytest.importorskip("playwright.sync_api", reason="browser tests need playwright")

from robopy.models import find_rakuda_model  # noqa: E402
from robopy.viewer.model_bundle import ModelBundle  # noqa: E402
from robopy.viewer.server import IKSetup, ViewerServer  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parents[2]
SOFT_LIMITS = {
    "torso_yaw_dof": (-1.57, 1.57),
    "shoulder_pitch_left_dof": (-3.14, 3.14),
    "shoulder_pitch_right_dof": (-3.14, 3.14),
}
OVERLAY_HIDDEN = "() => { const o = document.querySelector('#overlay'); return !!o && o.hidden; }"
# Lowest point of the geometry no joint moves -- the robot's base -- recomputed
# from the drawn meshes rather than from placeGround(), so the assertion does
# not simply echo the code it checks. THREE is not in the page's scope, so the
# Vector3 class is taken from an object that already holds one.
STATIC_MIN_Z = """() => {
  const s = window.__robopy_state;
  let min = Infinity;
  s.model.geometries.forEach((g, i) => {
    const obj = s.meshes[i];
    if (!g.static || !obj || !obj.geometry) return;
    obj.geometry.computeBoundingBox();
    const bb = obj.geometry.boundingBox;
    const Vector3 = obj.position.constructor;
    for (const x of [bb.min.x, bb.max.x]) {
      for (const y of [bb.min.y, bb.max.y]) {
        for (const z of [bb.min.z, bb.max.z]) {
          min = Math.min(min, new Vector3(x, y, z).applyMatrix4(obj.matrixWorld).z);
        }
      }
    }
  });
  return min;
}"""
MESHES_AT_IDENTITY = """() => {
  const s = window.__robopy_state;
  if (!s || !s.model) return -1;
  let n = 0;
  s.model.geometries.forEach((g, i) => {
    const m = s.meshes[i];
    if (!m || (m.position.lengthSq() < 1e-12 && Math.abs(m.quaternion.w - 1) < 1e-9)) n += 1;
  });
  return n;
}"""


@pytest.fixture(scope="module")
def server():  # type: ignore[no-untyped-def]
    rakuda = find_rakuda_model()
    if rakuda is None:
        pytest.skip("committed Rakuda model not found")
    bundle = ModelBundle.load(
        rakuda.convex_collision_urdf, package_dirs=rakuda.package_dirs, soft_limits=SOFT_LIMITS
    )
    srv = ViewerServer(bundle, host="127.0.0.1", port=0, ik=IKSetup(bundle))
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


def _open(  # type: ignore[no-untyped-def]
    browser,
    url: str,
    *,
    mesh_delay_s: float,
    device_scale_factor: float = 1.0,
):
    page = browser.new_page(
        viewport={"width": 1200, "height": 800}, device_scale_factor=device_scale_factor
    )
    errors: list[str] = []
    page.on("pageerror", lambda e: errors.append(str(e)))

    if mesh_delay_s > 0:

        def slow(route):  # type: ignore[no-untyped-def]
            time.sleep(mesh_delay_s)
            route.continue_()

        page.route("**/mesh/*", slow)
    page.goto(url)
    page.wait_for_function(OVERLAY_HIDDEN, timeout=120_000)
    page.wait_for_timeout(500)
    return page, errors


class TestMeshPlacement:
    """Every mesh must carry the FK pose whatever order downloads and FK finish in."""

    def test_meshes_that_arrive_after_the_first_fk_reply_are_still_placed(
        self, server: ViewerServer, browser
    ) -> None:
        # Delaying each of the 137 mesh downloads guarantees the /api/fk reply
        # (a few ms) lands first. Before the fix those meshes stayed at the
        # identity pose and the robot appeared to fall apart.
        page, errors = _open(browser, server.url, mesh_delay_s=0.01)
        try:
            assert errors == []
            assert page.evaluate(MESHES_AT_IDENTITY) == 0
            n_meshes = page.evaluate("() => window.__robopy_state.meshes.filter(Boolean).length")
            assert n_meshes == 137
        finally:
            page.close()

    def test_fast_load_also_places_every_mesh(self, server: ViewerServer, browser) -> None:
        page, errors = _open(browser, server.url, mesh_delay_s=0.0)
        try:
            assert errors == []
            assert page.evaluate(MESHES_AT_IDENTITY) == 0
        finally:
            page.close()

    def test_a_joint_change_moves_the_hand(self, server: ViewerServer, browser) -> None:
        page, errors = _open(browser, server.url, mesh_delay_s=0.0)
        try:
            before = page.evaluate("() => window.__robopy_state.ee.left.current.p")
            plus = page.locator("[data-joint='elbow_pitch_left_dof'] button.jog[data-dir='1']")
            for _ in range(12):  # 12 x 5 deg
                plus.click()
            page.wait_for_function(
                "(b) => { const p = window.__robopy_state.ee.left.current.p;"
                " return Math.hypot(p[0]-b[0], p[1]-b[1], p[2]-b[2]) > 0.05; }",
                arg=before,
                timeout=10_000,
            )
            assert errors == []
            assert page.evaluate(MESHES_AT_IDENTITY) == 0
        finally:
            page.close()


def _bend_the_elbows(page) -> None:  # type: ignore[no-untyped-def]
    """Leave the singular start pose: at q = 0 the arms hang fully extended."""
    for joint, deg in (("elbow_pitch_left_dof", 45), ("elbow_pitch_right_dof", -45)):
        field = page.locator(f".joint[data-joint={joint}] input.num")
        field.fill(str(deg))
        field.press("Enter")
    page.locator(".tab[data-tab=ee]").click()
    page.locator("#ee-capture-all").click()
    page.wait_for_timeout(200)


class TestCanvasSizing:
    """The canvas needs its CSS size set, not only its backing store."""

    def test_the_canvas_does_not_cover_the_panel_on_a_hidpi_screen(
        self, server: ViewerServer, browser
    ) -> None:
        # Sized through the width/height attributes alone, the canvas laid out
        # at one layout pixel per device pixel: at devicePixelRatio 2 -- a
        # HiDPI screen, or any browser zoom off 100% -- it came out twice as
        # wide as its box, spilled out of #viewport and hid the controls.
        page, errors = _open(browser, server.url, mesh_delay_s=0.0, device_scale_factor=2)
        try:
            box = page.evaluate(
                """() => {
                  const canvas = document.querySelector('#viewport canvas');
                  const viewport = document.querySelector('#viewport');
                  const c = canvas.getBoundingClientRect();
                  const p = document.querySelector('#panel').getBoundingClientRect();
                  return {
                    ratio: window.devicePixelRatio, buffer: canvas.width,
                    canvasRight: c.right, canvasWidth: c.width,
                    panelLeft: p.left, panelWidth: p.width,
                    viewportWidth: viewport.clientWidth,
                  };
                }"""
            )
            assert box["ratio"] == 2
            assert box["panelWidth"] > 0
            assert box["canvasRight"] <= box["panelLeft"] + 1
            assert box["canvasWidth"] == pytest.approx(box["viewportWidth"], abs=1)
            # ... while still drawing at the screen's full resolution.
            assert box["buffer"] == pytest.approx(box["viewportWidth"] * 2, abs=2)
            assert errors == []
        finally:
            page.close()


class TestGroundPlane:
    """The grid is the floor: it belongs under the base, not on the origin."""

    def test_the_grid_sits_on_the_bottom_of_the_base(self, server: ViewerServer, browser) -> None:
        # This export's origin is not on the floor -- the base plate's
        # underside is about 26 cm below it -- so a grid drawn at z = 0 cut
        # through the middle of the torso.
        page, errors = _open(browser, server.url, mesh_delay_s=0.0)
        try:
            ground = page.evaluate("() => window.__robopy_state.groundZ")
            assert ground == pytest.approx(page.evaluate(STATIC_MIN_Z), abs=1e-6)
            assert ground < -0.2
            assert errors == []
        finally:
            page.close()


class TestTargetBars:
    """Each end-effector target component has a bar that drives the solver."""

    def test_dragging_a_bar_moves_the_hand_towards_the_target(
        self, server: ViewerServer, browser
    ) -> None:
        page, errors = _open(browser, server.url, mesh_delay_s=0.0)
        try:
            _bend_the_elbows(page)
            bar = page.locator(".side[data-side=left] input[data-bar=x]")
            start = float(bar.input_value())
            before = page.evaluate("() => window.__robopy_state.ee.left.current.p")
            for step in range(1, 7):  # a drag is a stream of input events
                bar.fill(str(int(start + step * 10)))
                page.wait_for_timeout(120)
            # 60 mm asked for; the wrist has two axes, so the solver settles
            # for the closest pose it can reach -- most of the way there.
            page.wait_for_function(
                "(b) => window.__robopy_state.ee.left.current.p[0] - b[0] > 0.04",
                arg=before,
                timeout=15_000,
            )
            assert page.evaluate(
                "() => window.__robopy_state.ee.left.target.p[0]"
            ) == pytest.approx((start + 60) / 1000, abs=1e-6)
            assert errors == []
        finally:
            page.close()


class TestDraggingTheHand:
    """The handle on a target can be grabbed in the 3D view and dragged."""

    def _drag(self, page, start, dx, dy) -> None:  # type: ignore[no-untyped-def]
        page.mouse.move(start["x"], start["y"])
        page.mouse.down()
        for step in range(1, 6):
            page.mouse.move(start["x"] + step * dx / 5, start["y"] + step * dy / 5)
            page.wait_for_timeout(80)
        page.mouse.up()

    def test_dragging_the_handle_moves_that_hand(self, server: ViewerServer, browser) -> None:
        page, errors = _open(browser, server.url, mesh_delay_s=0.0)
        try:
            _bend_the_elbows(page)
            handle = page.evaluate("() => window.__robopy_state.handleScreen('left')")
            assert handle is not None
            page.mouse.move(handle["x"], handle["y"])
            cursor = "() => document.querySelector('#viewport canvas').style.cursor"
            assert page.evaluate(cursor) == "grab"

            before = page.evaluate("() => window.__robopy_state.ee.left.current.p")
            # Where the other hand's handle sits on screen only moves if the
            # camera does: the press must drag, not orbit underneath.
            right_before = page.evaluate("() => window.__robopy_state.handleScreen('right')")
            self._drag(page, handle, 50, 0)
            page.wait_for_function(
                "(b) => { const p = window.__robopy_state.ee.left.current.p;"
                " return Math.hypot(p[0]-b[0], p[1]-b[1], p[2]-b[2]) > 0.02; }",
                arg=before,
                timeout=15_000,
            )
            right_after = page.evaluate("() => window.__robopy_state.handleScreen('right')")
            assert abs(right_after["x"] - right_before["x"]) < 1.0
            assert abs(right_after["y"] - right_before["y"]) < 1.0
            assert errors == []
        finally:
            page.close()

    def test_an_untracked_hand_keeps_no_handle(self, server: ViewerServer, browser) -> None:
        # A hand nothing tracks must not leave a grabbable target behind: its
        # arm holds its joints and its TCP goes where the torso takes it.
        page, errors = _open(browser, server.url, mesh_delay_s=0.0)
        try:
            _bend_the_elbows(page)
            assert page.evaluate("() => window.__robopy_state.handleScreen('left')") is not None
            page.locator(".side[data-side=left] .ee-enable").uncheck()
            page.wait_for_timeout(300)
            assert page.evaluate("() => window.__robopy_state.handleScreen('left')") is None
            assert page.evaluate("() => window.__robopy_state.handleScreen('right')") is not None
            assert errors == []
        finally:
            page.close()

    def test_dragging_anywhere_else_still_orbits_the_view(
        self, server: ViewerServer, browser
    ) -> None:
        page, errors = _open(browser, server.url, mesh_delay_s=0.0)
        try:
            _bend_the_elbows(page)
            before = page.evaluate("() => window.__robopy_state.ee.left.current.p")
            handle_before = page.evaluate("() => window.__robopy_state.handleScreen('left')")
            self._drag(page, {"x": 120.0, "y": 700.0}, 120, 0)
            page.wait_for_timeout(500)
            handle_after = page.evaluate("() => window.__robopy_state.handleScreen('left')")
            after = page.evaluate("() => window.__robopy_state.ee.left.current.p")
            # The camera turned (the handle is elsewhere on screen) and nothing
            # was commanded (the hand is where it was).
            assert abs(handle_after["x"] - handle_before["x"]) > 5.0
            assert after == before
            assert errors == []
        finally:
            page.close()
