"""``robopy-vr`` command line: TLS files are checked before anything heavy runs."""

from __future__ import annotations

import shutil
import ssl
import subprocess
from pathlib import Path

import pytest

from robopy.vr.__main__ import (
    _resolve_tls,
    build_parser,
    generate_self_signed_certificate,
)


def _resolve(argv: list[str]):  # type: ignore[no-untyped-def]
    parser = build_parser()
    return _resolve_tls(parser.parse_args(argv), parser)


class TestResolveTls:
    def test_plain_http_by_default(self) -> None:
        assert _resolve([]) is None

    def test_cert_and_key_go_together(self, capsys: pytest.CaptureFixture[str]) -> None:
        with pytest.raises(SystemExit) as exc:
            _resolve(["--cert", "cert.pem"])
        assert exc.value.code == 2
        assert "--cert and --key go together" in capsys.readouterr().err

    def test_missing_files_stop_the_command_with_the_way_out(
        self, tmp_path: Path, capsys: pytest.CaptureFixture[str]
    ) -> None:
        cert, key = tmp_path / "cert.pem", tmp_path / "key.pem"
        with pytest.raises(SystemExit) as exc:
            _resolve(["--cert", str(cert), "--key", str(key)])
        assert exc.value.code == 2
        err = capsys.readouterr().err
        assert str(cert) in err and str(key) in err
        assert "openssl req" in err and "--self-signed" in err

    def test_existing_files_are_returned(self, tmp_path: Path) -> None:
        cert, key = tmp_path / "c.pem", tmp_path / "k.pem"
        cert.write_text("x")
        key.write_text("y")
        assert _resolve(["--cert", str(cert), "--key", str(key)]) == (cert, key)


@pytest.mark.skipif(shutil.which("openssl") is None, reason="openssl not installed")
class TestSelfSigned:
    def test_generates_a_certificate_python_can_load(self, tmp_path: Path) -> None:
        cert, key = tmp_path / "tls" / "cert.pem", tmp_path / "tls" / "key.pem"
        generate_self_signed_certificate(cert, key, days=1)
        context = ssl.create_default_context(ssl.Purpose.CLIENT_AUTH)
        context.load_cert_chain(str(cert), str(key))  # raises if the PEMs are bad

    def test_flag_creates_default_files_in_cwd_then_reuses_them(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.chdir(tmp_path)
        assert _resolve(["--self-signed"]) == (Path("cert.pem"), Path("key.pem"))
        first = Path("cert.pem").read_bytes()
        assert _resolve(["--self-signed"]) == (Path("cert.pem"), Path("key.pem"))
        assert Path("cert.pem").read_bytes() == first, "existing files are reused, not replaced"

    def test_flag_honours_explicit_paths(self, tmp_path: Path) -> None:
        cert, key = tmp_path / "a.pem", tmp_path / "b.pem"
        assert _resolve(["--self-signed", "--cert", str(cert), "--key", str(key)]) == (cert, key)
        assert cert.is_file() and key.is_file()


class TestOpensslConfigFallback:
    """A fake ``openssl`` that cannot find its config until OPENSSL_CONF points at one."""

    def test_retries_with_a_distribution_config(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        conf = tmp_path / "openssl.cnf"
        conf.write_text("# a config")
        monkeypatch.setattr("robopy.vr.__main__.OPENSSL_CONF_CANDIDATES", (str(conf),))
        monkeypatch.setattr("robopy.vr.__main__.shutil.which", lambda _: "/usr/bin/openssl")
        monkeypatch.delenv("OPENSSL_CONF", raising=False)
        calls: list[dict[str, str] | None] = []

        def fake_run(cmd, capture_output, text, env=None):  # type: ignore[no-untyped-def]
            calls.append(env)
            if env is None or env.get("OPENSSL_CONF") != str(conf):
                return subprocess.CompletedProcess(
                    cmd, 1, "", 'Can\'t open "/usr/local/ssl/openssl.cnf" for reading'
                )
            Path(cmd[cmd.index("-out") + 1]).write_text("cert")
            Path(cmd[cmd.index("-keyout") + 1]).write_text("key")
            return subprocess.CompletedProcess(cmd, 0, "", "")

        monkeypatch.setattr("robopy.vr.__main__.subprocess.run", fake_run)
        generate_self_signed_certificate(tmp_path / "c.pem", tmp_path / "k.pem")
        assert len(calls) == 2 and calls[0] is None
        assert (tmp_path / "c.pem").read_text() == "cert"

    def test_other_failures_are_reported_verbatim(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        monkeypatch.setattr("robopy.vr.__main__.shutil.which", lambda _: "/usr/bin/openssl")

        def fake_run(cmd, capture_output, text, env=None):  # type: ignore[no-untyped-def]
            return subprocess.CompletedProcess(cmd, 1, "", "unable to write key")

        monkeypatch.setattr("robopy.vr.__main__.subprocess.run", fake_run)
        with pytest.raises(RuntimeError, match="unable to write key"):
            generate_self_signed_certificate(tmp_path / "c.pem", tmp_path / "k.pem")

    def test_no_openssl_binary(self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
        monkeypatch.setattr("robopy.vr.__main__.shutil.which", lambda _: None)
        with pytest.raises(RuntimeError, match="openssl is not installed"):
            generate_self_signed_certificate(tmp_path / "c.pem", tmp_path / "k.pem")


class TestLabDefaults:
    def test_rotation_and_head_signs_default_to_the_measured_machine(self) -> None:
        args = build_parser().parse_args([])
        assert args.camera_rotate == 180 and args.head_signs == "1,-1"
        assert args.camera_mirror == "on"
        assert (
            build_parser().parse_args(["--camera-rotate", "0", "--head-signs", "auto"]).head_signs
            == "auto"
        )


class TestBilateralFlag:
    def test_bilateral_needs_hardware_head(self, capsys: pytest.CaptureFixture[str]) -> None:
        from robopy.vr.__main__ import main

        with pytest.raises(SystemExit) as exc:
            main(["--bilateral"])
        assert exc.value.code == 2
        assert "--bilateral goes with --hardware-head" in capsys.readouterr().err
        args = build_parser().parse_args(
            ["--hardware-head", "--bilateral", "--leader-port", "/dev/x"]
        )
        assert args.bilateral and args.leader_port == "/dev/x"


class TestLeaderGrippers:
    def test_default_switches_the_leader_grippers_off(self) -> None:
        from robopy.vr.__main__ import _leader_grippers

        class Bus:
            def __init__(self) -> None:
                self.disabled: list[str] = []
                self.written: dict[str, int] = {}

            def torque_disabled(self, names: list[str]) -> None:
                self.disabled += names

            def sync_write(self, item: object, values: dict[str, int]) -> None:
                self.written.update(values)

        class Leader:
            GRIPPER_MOTORS = ("l_arm_grip", "r_arm_grip")

            def __init__(self) -> None:
                self.motors = Bus()
                self.config = type("C", (), {"leader_torque_enabled": None})()

        off = Leader()
        assert "OFF" in _leader_grippers(off, hold=False)
        assert off.motors.disabled == ["l_arm_grip", "r_arm_grip"] and off.motors.written == {}
        held = Leader()
        assert "2400" in _leader_grippers(held, hold=True)
        assert held.motors.written == {"l_arm_grip": 2400, "r_arm_grip": 2400}
        assert held.motors.disabled == []
        assert build_parser().parse_args([]).leader_grip_hold is False
