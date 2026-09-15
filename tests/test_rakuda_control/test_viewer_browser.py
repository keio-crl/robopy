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
    rakuda = find_rakuda_model(REPO_ROOT / "models")
    if rakuda is None:
        pytest.skip("committed Rakuda model not found")
    bundle = ModelBundle.load(
        rakuda.convex_collision_urdf, package_dirs=[rakuda.package_dir], soft_limits=SOFT_LIMITS
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


def _open(browser, url: str, *, mesh_delay_s: float):  # type: ignore[no-untyped-def]
    page = browser.new_page(viewport={"width": 1200, "height": 800})
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
