"""The viewer's read-only mirror of the real follower."""

from __future__ import annotations

import json
import threading
import urllib.request
from pathlib import Path
from typing import Any, List

import pytest

pytest.importorskip("pinocchio")

from robopy.config.robot_config.rakuda_config import RakudaJointCalibrationSpec  # noqa: E402
from robopy.robots.rakuda.calibrate import simulated_buses  # noqa: E402
from robopy.robots.rakuda.rakuda_control import build_joint_map  # noqa: E402
from robopy.viewer.machine_mirror import MachineMirror  # noqa: E402

MOTORS = ("torso_yaw", "r_arm_sh_pitch1", "l_arm_sh_pitch1")


class NoWriteBus:
    """A bus that fails the test on anything but a read."""

    def __init__(self, bus: Any) -> None:
        self._bus = bus
        self.motors = bus.motors
        self.reads: List[List[str]] = []

    def sync_read(self, item: Any, names: List[str]) -> Any:
        self.reads.append(list(names))
        return self._bus.sync_read(item, names)

    def __getattr__(self, name: str) -> Any:
        raise AssertionError(f"the mirror must only read; it called {name}")


def _mirror() -> tuple[MachineMirror, Any, NoWriteBus]:
    sim = simulated_buses(MOTORS)["follower"]
    calibration = {
        "torso_yaw": RakudaJointCalibrationSpec(
            urdf_joint="torso_yaw_dof", direction=-1, zero_count=2048
        ),
        "r_arm_sh_pitch1": RakudaJointCalibrationSpec(
            urdf_joint="shoulder_pitch_right_dof", direction=1, zero_count=2048
        ),
        # l_arm_sh_pitch1 has no entry: it is on the bus but not mirrored.
    }
    bus = NoWriteBus(sim)
    joint_map = build_joint_map(sim.motors, calibration, side="follower")
    return MachineMirror(bus, joint_map), sim, bus


class TestMachineMirror:
    def test_reads_calibrated_motors_as_urdf_angles_and_writes_nothing(self) -> None:
        mirror, sim, bus = _mirror()
        sim.joint("torso_yaw").position_rad = 0.5
        sim.joint("r_arm_sh_pitch1").position_rad = -0.25
        mirror.read_once()
        snap = mirror.snapshot()
        assert snap["available"] and snap["read_only"] and snap["error"] is None
        assert snap["joints"]["torso_yaw_dof"] == pytest.approx(-0.5, abs=2e-3)  # direction -1
        assert snap["joints"]["shoulder_pitch_right_dof"] == pytest.approx(-0.25, abs=2e-3)
        assert snap["unmapped"] == {"l_arm_sh_pitch1": "no urdf_joint"}
        assert bus.reads == [["torso_yaw", "r_arm_sh_pitch1"]]
        assert snap["motors"]["torso_yaw"] == "torso_yaw_dof"

    def test_a_read_error_is_reported_and_polling_goes_on(self) -> None:
        mirror, sim, _ = _mirror()

        def broken(*_: Any) -> Any:
            raise OSError("port gone")

        mirror.bus = type("Broken", (), {"sync_read": broken})()
        mirror.read_once()
        assert "port gone" in mirror.snapshot()["error"]
        closed = []
        mirror = MachineMirror(
            NoWriteBus(sim), mirror.joint_map, rate_hz=200.0, close=lambda: closed.append(1)
        )
        mirror.start()
        try:
            for _ in range(200):
                if mirror.snapshot()["reads"] >= 3:
                    break
                threading.Event().wait(0.01)
        finally:
            mirror.stop()
        assert mirror.snapshot()["reads"] >= 3 and closed == [1]
        with pytest.raises(ValueError):
            MachineMirror(sim, mirror.joint_map, rate_hz=0.0)

    def test_the_viewer_serves_the_mirror(self, tmp_path: Path) -> None:
        from robopy.kinematics.synthetic_dual_arm import write_synthetic_dual_arm_urdf
        from robopy.viewer.model_bundle import ModelBundle
        from robopy.viewer.server import ViewerServer

        bundle = ModelBundle.load(write_synthetic_dual_arm_urdf(tmp_path / "m.urdf"))
        mirror, sim, _ = _mirror()
        sim.joint("torso_yaw").position_rad = 0.5
        mirror.read_once()
        for machine, expected in ((None, False), (mirror, True)):
            server = ViewerServer(bundle, port=0, machine=machine)
            thread = threading.Thread(target=server.serve_forever, daemon=True)
            thread.start()
            try:
                base = f"http://127.0.0.1:{server.server_address[1]}"
                with urllib.request.urlopen(f"{base}/api/machine") as res:
                    body = json.loads(res.read())
                with urllib.request.urlopen(f"{base}/api/model") as res:
                    model = json.loads(res.read())
            finally:
                server.shutdown()
                server.server_close()
            assert body["available"] is expected and model["machine_mirror"] is expected
            if expected:
                assert body["joints"]["torso_yaw_dof"] == pytest.approx(-0.5, abs=2e-3)
