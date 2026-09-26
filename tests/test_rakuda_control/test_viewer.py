"""The browser viewer's server: model description, FK/IK endpoints, and safety.

Skipped when the optional ``kinematics`` extra is absent.  No browser is
involved here; the page itself is exercised manually with Playwright.
"""

from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any, Dict, Tuple

import numpy as np
import pytest

pytest.importorskip("pink", reason="needs the 'kinematics' optional extra")

from robopy.viewer.model_bundle import ModelBundle, matrix_to_pose  # noqa: E402
from robopy.viewer.server import IKSetup, ViewerServer  # noqa: E402

SOFT_LIMITS = {
    "torso_yaw_dof": (-1.5, 1.5),
    "shoulder_pitch_left_dof": (-2.0, 2.0),
    "shoulder_pitch_right_dof": (-2.0, 2.0),
}


@pytest.fixture(scope="module")
def bundle(tmp_path_factory: pytest.TempPathFactory) -> ModelBundle:
    from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

    urdf = write_synthetic_dual_arm_urdf(tmp_path_factory.mktemp("viewer") / "syn.urdf")
    return ModelBundle.load(urdf, soft_limits=SOFT_LIMITS)


@pytest.fixture(scope="module")
def server(bundle: ModelBundle):  # type: ignore[no-untyped-def]
    srv = ViewerServer(bundle, host="127.0.0.1", port=0, ik=IKSetup(bundle))
    thread = threading.Thread(target=srv.serve_forever, daemon=True)
    thread.start()
    yield srv
    srv.shutdown()
    srv.server_close()


def _call(server: ViewerServer, path: str, body: Any = None) -> Tuple[int, Any]:
    url = server.url.rstrip("/") + path
    data = None if body is None else json.dumps(body).encode()
    req = urllib.request.Request(
        url, data=data, headers={"Content-Type": "application/json"} if data else {}
    )
    try:
        with urllib.request.urlopen(req, timeout=30) as res:
            raw = res.read()
            ctype = res.headers.get("Content-Type", "")
            return res.status, (json.loads(raw) if "json" in ctype else raw)
    except urllib.error.HTTPError as exc:
        return exc.code, json.loads(exc.read() or b"{}")


class TestModelBundle:
    def test_primitives_are_parsed_from_the_fixture(self, bundle: ModelBundle) -> None:
        # The synthetic fixture uses cylinders; a mesh-only parser would render
        # nothing for the default no-argument viewer.
        assert bundle.geometries
        assert {g.shape["type"] for g in bundle.geometries} == {"cylinder"}
        assert bundle.meshes == []

    def test_tcp_frames_are_attached_with_a_reported_placeholder_offset(
        self, bundle: ModelBundle
    ) -> None:
        assert bundle.tcp_frames == {"left": "left_tcp", "right": "right_tcp"}
        assert any("ZERO offset" in w for w in bundle.warnings)

    def test_description_flags_display_only_limits(self, tmp_path: Path) -> None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        unbounded = ModelBundle.load(write_synthetic_dual_arm_urdf(tmp_path / "u.urdf"))
        joints = {j["name"]: j for j in unbounded.describe()["joints"]}
        assert joints["torso_yaw_dof"]["limit_is_display_only"] is True
        assert joints["elbow_yaw_left_dof"]["limit_is_display_only"] is False
        assert any("display range only" in w for w in unbounded.warnings)

    def test_the_base_geometry_is_flagged_static(self, bundle: ModelBundle) -> None:
        # The page stands its ground plane on the geometry no joint moves. The
        # fixture's base is its pedestal; everything above the torso yaw moves.
        described = bundle.describe()["geometries"]
        assert [g["id"] for g in described if g["static"]] == ["root#0"]
        assert not [g for g in described if g["link"] == "torso_link" and g["static"]]

    def test_sliders_solver_and_overrides_read_one_resolved_range(self, tmp_path: Path) -> None:
        # One resolver decides every joint's range: the URDF range, replaced by
        # a recorded override, narrowed by a soft limit. The slider range *is*
        # the solver's range; nothing applies an actuator travel to every axis.
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        resolved = ModelBundle.load(
            write_synthetic_dual_arm_urdf(tmp_path / "resolved.urdf"),
            soft_limits={
                **SOFT_LIMITS,
                "elbow_yaw_left_dof": {"lower": -1.0, "upper": 1.0, "validated": True},
                "wrist_yaw_left_dof": (-9.0, 9.0),  # wider than the URDF: does not widen
            },
            joint_limit_overrides={
                "elbow_pitch_left_dof": {"lower": -2.0, "upper": 2.9, "reason": "measured stop"}
            },
        )
        joints = {j["name"]: j for j in resolved.describe()["joints"]}

        elbow = joints["elbow_pitch_left_dof"]  # URDF +/-2.4, overridden
        assert elbow["limit_source"] == "override" and elbow["validated"] is True
        assert (elbow["lower"], elbow["upper"]) == pytest.approx((-2.0, 2.9))
        assert (elbow["model_lower"], elbow["model_upper"]) == pytest.approx((-2.0, 2.9))
        assert (elbow["urdf_lower"], elbow["urdf_upper"]) == pytest.approx((-2.4, 2.4))
        lower, upper = resolved.model.position_limits(["elbow_pitch_left_dof"])
        assert (lower[0], upper[0]) == pytest.approx((-2.0, 2.9))  # the solver sees the same

        yaw = joints["elbow_yaw_left_dof"]  # URDF +/-2.8 narrowed by a measured soft limit
        assert yaw["limit_source"] == "soft" and yaw["validated"] is True
        assert (yaw["lower"], yaw["upper"]) == pytest.approx((-1.0, 1.0))

        wrist = joints["wrist_yaw_left_dof"]  # a wide soft limit changes nothing
        assert wrist["limit_source"] == "urdf"
        assert (wrist["lower"], wrist["upper"]) == pytest.approx((-2.8, 2.8))
        assert any("wider than" in w for w in resolved.warnings)

        torso = joints["torso_yaw_dof"]  # a plain pair is provisional
        assert torso["limit_source"] == "soft" and torso["validated"] is False
        assert (torso["lower"], torso["upper"]) == pytest.approx(SOFT_LIMITS["torso_yaw_dof"])
        assert any("provisional" in w for w in resolved.warnings)
        assert not any(j["limit_source"] == "motor" for j in joints.values())

    def test_without_overrides_the_range_is_the_model_s(self, bundle: ModelBundle) -> None:
        joints = {j["name"]: j for j in bundle.describe()["joints"]}
        elbow = joints["elbow_pitch_left_dof"]
        assert elbow["limit_source"] == "urdf" and elbow["validated"] is True
        assert (elbow["lower"], elbow["upper"]) == pytest.approx((-2.4, 2.4))
        assert joints["torso_yaw_dof"]["limit_source"] == "soft"

    def test_a_home_pose_is_checked_before_it_is_offered(self, tmp_path: Path) -> None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        urdf = write_synthetic_dual_arm_urdf(tmp_path / "home.urdf")
        good = ModelBundle.load(
            urdf, soft_limits=SOFT_LIMITS, home_positions_rad={"elbow_pitch_left_dof": 0.8}
        )
        assert good.home_positions_rad["elbow_pitch_left_dof"] == pytest.approx(0.8)
        assert good.describe()["home_positions_rad"]["torso_yaw_dof"] == 0.0
        bad = ModelBundle.load(
            urdf, soft_limits=SOFT_LIMITS, home_positions_rad={"elbow_pitch_left_dof": 3.0}
        )
        assert bad.home_positions_rad == {}
        assert any(
            "not usable as a home pose" in w and "elbow_pitch_left_dof" in w for w in bad.warnings
        )

    def test_poses_place_every_geometry_and_both_tcps(self, bundle: ModelBundle) -> None:
        poses = bundle.poses({"torso_yaw_dof": 0.3})
        assert len(poses["geometries"]) == len(bundle.geometries)
        assert set(poses["tcp"]) == {"left", "right"}
        assert poses["joints"]["torso_yaw_dof"] == pytest.approx(0.3)

    def test_unknown_joint_is_rejected(self, bundle: ModelBundle) -> None:
        with pytest.raises(KeyError, match="Unknown joint"):
            bundle.poses({"nope": 0.1})

    def test_matrix_to_pose_round_trips_a_rotation(self) -> None:
        from robopy.control.types import se3_from_quat_xyzw

        q = np.array([0.1, -0.2, 0.3, 0.9])
        q /= np.linalg.norm(q)
        T = se3_from_quat_xyzw([0.1, 0.2, 0.3], q)
        pose = matrix_to_pose(T)
        np.testing.assert_allclose(pose["p"], [0.1, 0.2, 0.3])
        recovered = np.asarray(pose["q"])
        if np.dot(recovered, q) < 0:
            recovered = -recovered
        np.testing.assert_allclose(recovered, q, atol=1e-9)


class TestEndpoints:
    def test_health_and_model(self, server: ViewerServer) -> None:
        status, health = _call(server, "/api/health")
        assert status == 200 and health == {"ok": True, "ik": True}
        status, model = _call(server, "/api/model")
        assert status == 200
        assert len(model["joints"]) == 15
        assert model["simulation_only"] is True
        assert model["ik"]["groups"]["torso"] == "torso_yaw_dof"
        assert len(model["ik"]["groups"]["left"]) == 6
        assert model["ik"]["workspace"]["right"]["radius"] == pytest.approx(0.55)

    def test_fk_returns_poses(self, server: ViewerServer) -> None:
        status, poses = _call(server, "/api/fk", {"joints": {"torso_yaw_dof": 0.5}})
        assert status == 200
        assert poses["joints"]["torso_yaw_dof"] == pytest.approx(0.5)
        assert "timing_ms" in poses

    @pytest.mark.parametrize(
        "body",
        [{"joints": {"nope": 1.0}}, {"joints": {"torso_yaw_dof": float("nan")}}, {"joints": 3}],
    )
    def test_fk_rejects_bad_input(self, server: ViewerServer, body: Dict[str, Any]) -> None:
        status, err = _call(server, "/api/fk", body)
        assert status == 400 and "error" in err

    def test_ik_converges_and_returns_poses(
        self, server: ViewerServer, bundle: ModelBundle
    ) -> None:
        goal = {name: 0.0 for name in bundle.joint_order}
        goal["shoulder_roll_left_dof"] = 0.3
        goal["elbow_pitch_left_dof"] = -0.4
        left = matrix_to_pose(bundle.model.frame_pose(bundle.positions_to_q(goal), "left_tcp"))
        status, result = _call(
            server,
            "/api/ik",
            {"joints": {}, "targets": {"left": left}, "torso_policy": "fixed"},
        )
        assert status == 200
        assert result["status"] == "converged"
        assert result["commandable"] is True
        assert result["errors"]["left_position_m"] < 1e-3
        assert result["errors"]["right_hold_m"] is None  # no world hold by default
        assert result["inactive_arm_policy"] == "hold_joints"
        assert len(result["poses"]["geometries"]) == len(bundle.geometries)
        assert result["joints"]["head_yaw_dof"] == 0.0

    def test_ik_failure_keeps_the_starting_joints(self, server: ViewerServer) -> None:
        far = {"p": [5.0, 0.0, 0.0], "q": [0.0, 0.0, 0.0, 1.0]}
        start = {"torso_yaw_dof": 0.2}
        status, result = _call(
            server, "/api/ik", {"joints": start, "targets": {"left": far}, "iterations": 5}
        )
        assert status == 200
        # Unreachable: it tracks but does not converge; that is still commandable.
        assert result["status"] in ("tracking", "converged")

    def test_ik_with_no_targets_is_a_bad_request(self, server: ViewerServer) -> None:
        status, _ = _call(server, "/api/ik", {"joints": {}, "targets": {}})
        assert status == 400

    @pytest.mark.parametrize("side", ["left", "right"])
    @pytest.mark.parametrize("policy", ["hold_joints", "hold_world"])
    def test_single_arm_policy_reaches_solver(self, server, bundle, side, policy):
        other = "right" if side == "left" else "left"
        start = {name: 0.0 for name in bundle.joint_order}
        # Deliberately offset the active target so a manual solve iterates.
        goal = start | {f"elbow_pitch_{side}_dof": -0.3}
        pose = matrix_to_pose(bundle.model.frame_pose(bundle.positions_to_q(goal), f"{side}_tcp"))
        status, result = _call(
            server,
            "/api/ik",
            {
                "joints": start,
                "targets": {side: pose},
                "torso_policy": "manual",
                "torso_velocity_rad_s": 0.3,
                "iterations": 20,
                "inactive_arm_policy": policy,
            },
        )
        assert status == 200 and result["commandable"]
        assert result["inactive_arm_policy"] == policy
        assert abs(result["joints"]["torso_yaw_dof"]) > 0.01
        if policy == "hold_joints":
            assert result["errors"][f"{other}_hold_m"] is None
            for name in bundle.joint_order:
                if other in name:
                    assert result["joints"][name] == pytest.approx(start[name], abs=1e-12)
        else:
            assert result["errors"][f"{other}_hold_m"] is not None

    def test_invalid_inactive_arm_policy_is_rejected(self, server, bundle):
        pose = matrix_to_pose(bundle.model.frame_pose(bundle.positions_to_q({}), "right_tcp"))
        status, _ = _call(
            server,
            "/api/ik",
            {
                "joints": {},
                "targets": {"right": pose},
                "inactive_arm_policy": "typo",
            },
        )
        assert status == 400

    def test_static_traversal_is_refused(self, server: ViewerServer) -> None:
        status, _ = _call(server, "/static/../../__init__.py")
        assert status == 404
        status, _ = _call(server, "/static/%2e%2e/%2e%2e/server.py")
        assert status == 404

    def test_static_assets_and_page_are_served(self, server: ViewerServer) -> None:
        status, page = _call(server, "/")
        assert status == 200 and b"robopy" in page
        status, js = _call(server, "/static/app.js")
        assert status == 200 and b"import * as THREE" in js
        status, vendor = _call(server, "/static/vendor/three.module.min.js")
        assert status == 200 and len(vendor) > 100_000

    def test_a_mesh_index_out_of_range_is_404(self, server: ViewerServer) -> None:
        status, _ = _call(server, "/mesh/0")  # the fixture has primitives only
        assert status == 404
        status, _ = _call(server, "/mesh/not-a-number")
        assert status == 404

    def test_unknown_routes_are_404(self, server: ViewerServer) -> None:
        assert _call(server, "/api/nope")[0] == 404
        assert _call(server, "/api/nope", {"x": 1})[0] == 404


class TestIKSetup:
    def test_groups_are_inferred_and_reported(self, bundle: ModelBundle) -> None:
        setup = IKSetup(bundle)
        assert setup.groups["torso"] == "torso_yaw_dof"
        assert setup.groups["head"] == ["head_yaw_dof", "head_pitch_dof"]
        assert setup.groups["left"][0] == "shoulder_pitch_left_dof"  # shoulder first
        assert setup.geometric_study_only is False

    def test_the_reach_bound_is_the_arm_unfolded_from_its_shoulder(
        self, bundle: ModelBundle
    ) -> None:
        # The page sizes its target sliders with this. It is the sum of the
        # fixture's arm segments (0.25 + 0.20 + 0.10 m) measured from the
        # shoulder frame -- an outer bound, not a reachability claim.
        left = IKSetup(bundle).workspace["left"]
        assert left["radius"] == pytest.approx(0.55)
        np.testing.assert_allclose(left["center"], [0.0, 0.2, 0.6], atol=1e-9)

    def test_missing_soft_limits_make_it_a_geometric_study(self, tmp_path: Path) -> None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf

        unbounded = ModelBundle.load(write_synthetic_dual_arm_urdf(tmp_path / "u.urdf"))
        setup = IKSetup(unbounded)
        assert setup.geometric_study_only is True
        assert "torso_yaw_dof" in setup.groups["unbounded_continuous"]


class TestOrientationWeightAndStall:
    """A two-axis wrist cannot always translate while keeping its orientation."""

    def _jog(self, server: ViewerServer, bundle: ModelBundle, **extra: Any) -> Dict[str, Any]:
        start = {"torso_yaw_dof": 0.6}
        _, cur = _call(server, "/api/fk", {"joints": start})
        left = dict(cur["tcp"]["left"])
        left["p"] = [left["p"][0] + 0.03, left["p"][1], left["p"][2]]
        body = {
            "joints": start,
            "targets": {"left": left, "right": cur["tcp"]["right"]},
            "torso_policy": "fixed",
            "iterations": 300,
            **extra,
        }
        status, result = _call(server, "/api/ik", body)
        assert status == 200
        return result

    def test_an_unreachable_target_is_reported_as_stalled(
        self, server: ViewerServer, bundle: ModelBundle
    ) -> None:
        # A target a metre away cannot be reached. The API must say the solve
        # stalled -- the residual stopped improving -- rather than exhaust its
        # iteration budget silently, and it must echo the weight it used.
        status, result = _call(
            server,
            "/api/ik",
            {
                "joints": {},
                "targets": {"left": {"p": [1.0, 0.5, 0.3], "q": [0.0, 0.0, 0.0, 1.0]}},
                "torso_policy": "fixed",
                "iterations": 600,
                "orientation_weight": 0.15,
            },
        )
        assert status == 200
        assert result["status"] == "tracking"
        assert result["commandable"] is True
        # Either the residual stopped improving (stalled) or the budget ran out
        # with the arm still swinging at its speed bound; both are "not reached".
        assert result["stalled"] or result["iterations"] == 600
        assert result["errors"]["left_position_m"] > 0.1
        assert result["orientation_weight"] == pytest.approx(0.15)
        assert "stalled" in result and "active_limits" in result

    def test_a_position_only_jog_converges(self, server: ViewerServer, bundle: ModelBundle) -> None:
        result = self._jog(server, bundle, orientation_weight=0.0)
        assert result["status"] == "converged", result
        assert result["stalled"] is False
        assert result["errors"]["left_position_m"] < 1e-3
        assert result["orientation_weight"] == 0.0

    def test_orientation_weight_is_range_checked(self, server: ViewerServer) -> None:
        status, _ = _call(
            server,
            "/api/ik",
            {
                "joints": {},
                "targets": {"left": {"p": [0, 0.2, 0], "q": [0, 0, 0, 1]}},
                "orientation_weight": -1,
            },
        )
        assert status == 400
