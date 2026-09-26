"""robopy-rakuda-calibrate: measured values in, a loadable bilateral config out."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Dict, List

import pytest

from robopy.config.dotrobopy import apply_rakuda_dotconfig
from robopy.config.robot_config.rakuda_config import RakudaConfig
from robopy.motor.dynamixel_control_table import XControlTable
from robopy.robots.rakuda.calibrate import (
    ArmCalibrator,
    AutoConsole,
    JointResult,
    build_control_section,
    main,
    proposed_urdf_joints,
    self_check,
    simulated_buses,
    write_config,
)

MOTORS = ("torso_yaw", "r_arm_sh_pitch1", "l_arm_sh_pitch1")


class ScriptedConsole:
    """Answers from a list; moves a simulated joint when asked to."""

    def __init__(self, bus: Any, answers: List[str]) -> None:
        self.bus = bus
        self.answers = list(answers)
        self.lines: List[str] = []
        self.nudge = 0.0

    def say(self, text: str) -> None:
        self.lines.append(text)

    def ask(self, prompt: str, default: str = "") -> str:
        self.lines.append(prompt)
        answer = self.answers.pop(0) if self.answers else ""
        if answer.startswith("move:"):
            motor, rad = answer[5:].split("=")
            self.bus.joint(motor).position_rad = float(rad)
            return default
        return answer or default

    def sleep(self, seconds: float) -> None:
        for name in self.bus.motors:
            self.bus.joint(name).position_rad += self.nudge


class TestArmCalibrator:
    def test_registers_zero_direction_limits_and_torque_constant(self) -> None:
        bus = simulated_buses(MOTORS)["follower"]
        bus.write(XControlTable.VELOCITY_LIMIT, "torso_yaw", 100)  # 100 x 0.229 rpm
        bus.write(XControlTable.CURRENT_LIMIT, "torso_yaw", 1000)  # x 0.00269 A
        console = ScriptedConsole(
            bus,
            [
                "torso_yaw_dof",
                "",
                "",  # URDF joints (proposal kept for the arms)
                "",  # zero pose: Enter
                # torso: direction (polled; nudge set below), then two ends, then Kt
                "move:torso_yaw=-0.5",
                "move:torso_yaw=0.7",
                "y",
                "",
                "0.5",
                "0.2",
                "",
                # r_arm: ends, no Kt
                "move:r_arm_sh_pitch1=0.3",
                "move:r_arm_sh_pitch1=-0.3",
                "n",
                # l_arm
                "move:l_arm_sh_pitch1=0.2",
                "move:l_arm_sh_pitch1=-0.4",
                "n",
            ],
        )
        currents = iter([0.1] * 20 + [0.3] * 20 + [0.1] * 200)
        cal = ArmCalibrator(
            bus,
            "follower",
            MOTORS,
            console,
            current_fraction=0.5,
            current_reader=lambda m: next(currents),
        )
        cal.read_registers()
        cal.confirm_urdf_joints()
        cal.measure_zero("straight")
        console.nudge = -0.01  # the operator turns every joint the negative way
        for motor in MOTORS:
            cal.measure_direction(motor)
            cal.measure_limits(motor)
            cal.measure_torque_constant(motor)
        cal.finish()
        torso = cal.results["torso_yaw"]
        assert torso.model == "xm430-w350" and torso.urdf_joint == "torso_yaw_dof"
        assert torso.max_velocity_rad_s == pytest.approx(
            100 * 0.229 * 2 * 3.14159265 / 60, rel=1e-3
        )
        assert torso.current_limit_a == pytest.approx(1000 * 0.00269 * 0.5, rel=1e-3)
        assert torso.direction == -1  # the count fell when moved "positive"
        assert torso.lower_limit_rad is not None and torso.upper_limit_rad is not None
        assert torso.lower_limit_rad < 0 < torso.upper_limit_rad
        assert torso.torque_constant_nm_per_a == pytest.approx(0.5 * 9.80665 * 0.2 / 0.2, rel=1e-3)
        assert torso.validated and "Kt from 0.5 kg" in torso.notes
        right = cal.results["r_arm_sh_pitch1"]
        assert right.urdf_joint == "shoulder_pitch_right_dof"  # the proposal, confirmed by Enter
        assert right.torque_constant_nm_per_a is None and not right.validated
        assert bus.registers("torso_yaw").torque_enable == 0  # left limp
        assert bus.registers("torso_yaw").operating_mode == 3  # back in position mode

    def test_no_movement_leaves_direction_unmeasured(self) -> None:
        bus = simulated_buses(MOTORS)["leader"]
        console = ScriptedConsole(bus, [])
        cal = ArmCalibrator(bus, "leader", ["torso_yaw"], console, move_timeout_s=0.0)
        cal.measure_zero("x")
        assert cal.measure_direction("torso_yaw") is None
        assert "direction not measured" in cal.results["torso_yaw"].notes
        with pytest.raises(ValueError):
            ArmCalibrator(bus, "leader", ["ghost"], console)
        with pytest.raises(ValueError):
            ArmCalibrator(bus, "sideways", ["torso_yaw"], console)


def _result(motor: str, complete: bool) -> JointResult:
    r = JointResult(
        motor=motor,
        model="xm430-w350",
        urdf_joint=proposed_urdf_joints()[motor],
        zero_count=2048,
        lower_limit_rad=-1.0,
        upper_limit_rad=1.0,
        max_velocity_rad_s=3.0,
        current_limit_a=0.5,
        torque_constant_nm_per_a=1.2 if complete else None,
    )
    r.validated = r.complete_for_hardware
    return r


class TestConfigFile:
    def test_section_couples_only_complete_joints_and_keeps_existing_gains(self) -> None:
        leader = {m: _result(m, True) for m in MOTORS}
        follower = {m: _result(m, m != "l_arm_sh_pitch1") for m in MOTORS}
        existing = {"control_period_s": 0.004, "bilateral": {"stiffness_nm_per_rad": 2.5}}
        control, notes = build_control_section(
            leader, follower, existing=existing, allow_current=True
        )
        assert control["control_period_s"] == 0.004 and control["mode"] == "bilateral_joint"
        assert control["bilateral"]["coupled_motors"] == ["torso_yaw", "r_arm_sh_pitch1"]
        assert control["bilateral"]["stiffness_nm_per_rad"] == 2.5
        assert control["bilateral"]["leader_current_limit_a"] == {
            "torso_yaw": 0.5,
            "r_arm_sh_pitch1": 0.5,
        }
        assert control["allow_hardware_current_output"] is True
        assert any("l_arm_sh_pitch1" in n for n in notes)
        assert "motor" not in control["leader_joint_calibration"]["torso_yaw"]
        nothing, notes = build_control_section({m: _result(m, False) for m in MOTORS}, follower)
        assert nothing["bilateral"]["coupled_motors"] == []
        assert nothing["allow_hardware_current_output"] is False

    def test_written_file_loads_and_passes_the_hardware_check(self, tmp_path: Path) -> None:
        path = tmp_path / ".robopy" / "rakuda" / "config.yaml"
        path.parent.mkdir(parents=True)
        path.write_text(
            "leader:\n  torque_enabled: [l_arm_grip]\nfollower:\n  torque_enabled: null\n"
        )
        leader = {m: _result(m, True) for m in MOTORS}
        follower = {m: _result(m, True) for m in MOTORS}
        control, _ = build_control_section(leader, follower, allow_current=True)
        write_config(path, control, coupled=control["bilateral"]["coupled_motors"])
        assert list(tmp_path.joinpath(".robopy", "rakuda").glob("config.yaml.bak-*"))
        cfg = apply_rakuda_dotconfig(
            RakudaConfig(leader_port="a", follower_port="b"), base_dir=tmp_path
        )
        assert cfg.control is not None and cfg.control.mode == "bilateral_joint"
        assert cfg.control.bilateral.coupled_motors == list(MOTORS)
        assert cfg.leader_torque_enabled == ["l_arm_grip", *MOTORS]  # widened to the coupling
        assert cfg.control.follower_joint_calibration["torso_yaw"].torque_constant_nm_per_a == 1.2
        models: Dict[str, Dict[str, str]] = {
            s: {m: "xm430-w350" for m in MOTORS} for s in ("leader", "follower")
        }
        assert self_check(path, models) == []
        incomplete, _ = build_control_section(leader, {m: _result(m, False) for m in MOTORS})
        write_config(path, incomplete, coupled=[], keep_backup=False)
        problems = self_check(path, models)
        assert any("coupled_motors is empty" in p for p in problems)
        assert any("allow_hardware_current_output is false" in p for p in problems)


class TestCommand:
    def test_simulated_run_writes_a_complete_config(
        self, tmp_path: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        out = tmp_path / ".robopy" / "rakuda" / "config.yaml"
        assert (
            main(
                [
                    "--simulate",
                    "--output",
                    str(out),
                    "--allow-current",
                    "--motors",
                    ",".join(MOTORS),
                ]
            )
            == 0
        )
        assert "passes JointMap.require('hardware')" in capsys.readouterr().out
        cfg = apply_rakuda_dotconfig(
            RakudaConfig(leader_port="a", follower_port="b"), base_dir=tmp_path
        )
        assert cfg.control is not None and cfg.control.allow_hardware_current_output
        assert cfg.control.bilateral.coupled_motors == list(MOTORS)

    def test_head_and_grippers_are_refused_and_ports_are_required(
        self, capsys: pytest.CaptureFixture[str]
    ) -> None:
        with pytest.raises(SystemExit):
            main(["--simulate", "--motors", "head_yaw"])
        assert "never coupled" in capsys.readouterr().err
        with pytest.raises(SystemExit):
            main(["--leader-port", "/dev/x"])
        assert "--follower-port" in capsys.readouterr().err
        console = AutoConsole(simulated_buses(MOTORS), say=lambda _: None)
        assert (
            console.ask("  torso_yaw: measure the torque constant with a known weight? (y/N)", "n")
            == "y"
        )
        assert console.current_a("torso_yaw") == 0.10
        console.ask("  hang the mass, let it settle, then press Enter")
        assert console.current_a("torso_yaw") == 0.35
