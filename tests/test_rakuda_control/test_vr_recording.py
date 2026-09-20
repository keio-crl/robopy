"""Session recording and its rendering to video."""

from __future__ import annotations

import importlib.util
import json
import time
from pathlib import Path
from typing import Any, Dict, List

import numpy as np
import pytest

pytest.importorskip("pinocchio")

from robopy.viewer.model_bundle import ModelBundle  # noqa: E402
from robopy.vr.recording import RECORDING_FORMAT, SessionRecorder  # noqa: E402
from robopy.vr.render import RenderSettings, load_recording, resample  # noqa: E402
from robopy.vr.server import VRServerConfig  # noqa: E402

from .test_vr_server import HEAD0, controller, make_session  # noqa: E402

HAS_MUJOCO = importlib.util.find_spec("mujoco") is not None


@pytest.fixture(scope="module")
def bundle(synthetic_urdf: Path) -> ModelBundle:
    from .test_vr_server import SOFT_LIMITS

    return ModelBundle.load(synthetic_urdf, soft_limits=SOFT_LIMITS)


class TestSessionRecorder:
    def test_start_add_stop_writes_the_documented_file(self, tmp_path: Path) -> None:
        rec = SessionRecorder(tmp_path / "rec")
        assert not rec.active
        rec.add({"q": [1.0]}, 5.0)  # ignored: not recording
        rec.start({"joint_names": ["a"], "robot": "x"}, 10.0)
        assert rec.active and rec.describe()["frames"] == 0
        rec.add({"q": [0.1]}, 10.0)
        rec.add({"q": [0.2]}, 10.5)
        assert rec.describe()["seconds"] == pytest.approx(0.5)
        path = rec.stop(11.0)
        assert path is not None and path.parent == tmp_path / "rec"
        doc = json.loads(path.read_text())
        assert doc["format"] == RECORDING_FORMAT
        assert doc["joint_names"] == ["a"] and doc["robot"] == "x"
        assert doc["duration_s"] == pytest.approx(1.0)
        assert [f["t"] for f in doc["frames"]] == [0.0, 0.5]
        assert doc["frames"][1]["q"] == [0.2]
        assert not rec.active and rec.stop(12.0) is None
        assert rec.describe()["last_recording"] == str(path)
        assert rec.describe()["render"] is None

    def test_render_runs_in_the_background_and_reports(self, tmp_path: Path) -> None:
        seen: List[float] = []

        def render(path: Path, progress: Any) -> List[Path]:
            progress(0.5)
            seen.append(0.5)
            out = path.with_suffix(".mp4")
            out.write_bytes(b"")
            return [out]

        rec = SessionRecorder(tmp_path, render=render)
        rec.start({}, 0.0)
        rec.add({"q": []}, 0.0)
        path = rec.stop(1.0)
        assert path is not None and rec.wait_for_render(5.0)
        state = rec.describe()["render"]
        assert state["status"] == "done" and state["progress"] == 1.0
        assert state["outputs"] == [str(path.with_suffix(".mp4"))]
        assert seen == [0.5]

    def test_render_failure_is_reported_not_raised(self, tmp_path: Path) -> None:
        def render(path: Path, progress: Any) -> List[Path]:
            raise RuntimeError("no GL here")

        rec = SessionRecorder(tmp_path, render=render)
        rec.start({}, 0.0)
        rec.stop(1.0)
        assert rec.wait_for_render(5.0)
        state = rec.describe()["render"]
        assert state["status"] == "failed" and "no GL here" in state["error"]


class TestSessionRecording:
    def test_record_messages_capture_the_session(self, bundle: ModelBundle, tmp_path: Path) -> None:
        cfg = VRServerConfig(state_hz=1000.0, record_dir=tmp_path / "rec", render_videos=False)
        session, backend = make_session(bundle, mapping="absolute", config=cfg)
        rec = SessionRecorder(tmp_path / "rec")
        session.recorder = rec
        hello = session.handle({"type": "hello"}, 0.0)
        assert hello is not None and hello["recording"]["active"] is False
        assert hello["camera_available"] is True
        reply = session.handle({"type": "record", "action": "start"}, 0.0)
        assert reply is not None and reply["type"] == "recording" and reply["recording"]["active"]
        t = 0.0
        for _ in range(30):
            t += 1.0 / 60.0
            state = session.handle(
                {
                    "type": "pose",
                    "head": HEAD0,
                    "left": controller(-0.3, 1.2, -0.4, clutch=True),
                    "right": None,
                },
                t,
            )
        assert state is not None and state["recording"]["active"]
        assert state["recording"]["frames"] == 30
        reply = session.handle({"type": "record", "action": "stop"}, t)
        assert reply is not None and not reply["recording"]["active"]
        path = Path(reply["recording"]["last_recording"])
        doc = load_recording(path)
        assert doc["joint_names"] == list(backend.joint_positions())
        assert doc["head"]["yaw_joint"] == "head_yaw_dof"
        assert doc["anchor_m"] == pytest.approx(list(session.robot_anchor_m))
        assert doc["arms"] == {"mapping": "absolute", "position_scale": 1.0}
        frame = doc["frames"][-1]
        assert len(frame["q"]) == len(doc["joint_names"])
        assert frame["head"]["yaw_rad"] == pytest.approx(0.0, abs=1e-6)
        left = frame["controllers"]["left"]
        assert left["clutched"] is True
        # Controller at robot (0.4, 0.3, 1.2) - head (0, 0, 1.6) => anchor + (0.4, 0.3, -0.4)
        expected = np.asarray(session.robot_anchor_m) + [0.4, 0.3, -0.4]
        assert left["p_base"] == pytest.approx(list(expected), abs=1e-9)
        assert frame["controllers"]["right"] is None
        assert frame["targets"]["left"] is not None and frame["targets"]["right"] is None
        assert frame["ik"]["status"] and "left_position_m" in frame["ik"]

    def test_toggle_status_and_close(self, bundle: ModelBundle, tmp_path: Path) -> None:
        session, _ = make_session(bundle)
        assert session.handle({"type": "record", "action": "start"}, 0.0) == {
            "type": "error",
            "message": "recording is disabled on this server",
        }
        session.recorder = SessionRecorder(tmp_path)
        assert session.handle({"type": "record"}, 0.0)["recording"]["active"] is False  # type: ignore[index]
        assert session.handle({"type": "record", "action": "toggle"}, 0.0)["recording"]["active"]  # type: ignore[index]
        bad = session.handle({"type": "record", "action": "rewind"}, 0.0)
        assert bad is not None and bad["type"] == "error"
        session.handle({"type": "pose", "head": HEAD0, "left": None, "right": None}, 0.1)
        session.close()  # the operator left: the recording is finished, not lost
        assert not session.recorder.active
        assert session.recorder.describe()["last_recording"] is not None


class TestCameraTap:
    def test_camera_stream_becomes_the_first_person_video(self, tmp_path: Path) -> None:
        import cv2

        from robopy.vr.camera import FrameStreamer, JpegEncoder, SyntheticFrameSource
        from robopy.vr.recording import CameraTap
        from robopy.vr.render import load_recording, render_recording

        streamer = FrameStreamer(SyntheticFrameSource(64, 48), fps=60.0, encoder=JpegEncoder(80))
        streamer.start()
        try:
            rec = SessionRecorder(tmp_path, camera=CameraTap(streamer, fps=20.0))
            rec.start({"joint_names": ["a"]}, 0.0)
            for i in range(6):
                rec.add({"q": [0.1 * i]}, 0.05 * i)
                time.sleep(0.05)
            path = rec.stop(0.3)
        finally:
            streamer.stop()
        assert path is not None
        doc = load_recording(path)
        video = path.parent / doc["camera_video"]
        assert video.name == f"{path.stem}_first_person.mp4" and video.is_file()
        cap = cv2.VideoCapture(str(video))
        frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        ok, image = cap.read()
        cap.release()
        assert ok and image.shape == (48, 64, 3) and 3 <= frames <= 12
        assert rec.describe()["camera_frames"] == frames
        if HAS_MUJOCO:
            # The renderer keeps the camera video as the first-person view.
            outputs = render_recording(
                path,
                settings=RenderSettings(fps=10.0, width=160, height=120),
                views=("first_person",),
            )
            assert outputs == [video]
            assert cv2.VideoCapture(str(video)).get(cv2.CAP_PROP_FRAME_COUNT) == frames

    def test_a_failing_tap_does_not_break_the_recording(self, tmp_path: Path) -> None:
        from robopy.vr.recording import CameraTap

        class Dead:
            @property
            def latest(self) -> Any:
                raise RuntimeError("camera exploded")

        rec = SessionRecorder(tmp_path, camera=CameraTap(Dead(), fps=50.0))
        rec.start({}, 0.0)
        time.sleep(0.1)
        path = rec.stop(0.1)
        assert path is not None and json.loads(path.read_text())["camera_video"] is None
        with pytest.raises(ValueError):
            CameraTap(Dead(), fps=0.0)


class TestRenderHelpers:
    def test_gl_backend_defaults_to_egl_on_headless_linux(
        self, monkeypatch: pytest.MonkeyPatch
    ) -> None:
        from robopy.vr.render import select_gl_backend

        monkeypatch.delenv("MUJOCO_GL", raising=False)
        monkeypatch.setattr("robopy.vr.render.sys.platform", "linux")
        assert select_gl_backend() == "egl"
        monkeypatch.setenv("MUJOCO_GL", "osmesa")
        assert select_gl_backend() == "osmesa"  # an explicit choice is kept
        monkeypatch.delenv("MUJOCO_GL")
        monkeypatch.setattr("robopy.vr.render.sys.platform", "darwin")
        assert select_gl_backend() is None

    def test_resample_holds_the_last_frame(self) -> None:
        frames: List[Dict[str, Any]] = [{"t": 0.0, "q": 0}, {"t": 0.4, "q": 1}, {"t": 0.45, "q": 2}]
        out = resample(frames, 10.0)  # instants 0.0 .. 0.4: frame 1 takes over at 0.4
        assert [f["q"] for f in out] == [0, 0, 0, 0, 1]
        assert [f["q"] for f in resample(frames, 20.0)][-2:] == [1, 2]  # 0.45 -> frame 2
        assert resample([], 10.0) == []

    def test_settings_validation(self) -> None:
        with pytest.raises(ValueError):
            RenderSettings(fps=0)
        with pytest.raises(ValueError):
            load_recording(Path(__file__))  # not a recording


class TestVideoWriter:
    @staticmethod
    def _fourcc(path: Path) -> str:
        import cv2

        cap = cv2.VideoCapture(str(path))
        try:
            code = int(cap.get(cv2.CAP_PROP_FOURCC))
            frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        finally:
            cap.release()
        return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4)) + f":{frames}"

    def test_h264_through_ffmpeg_when_available(self, tmp_path: Path) -> None:
        from robopy.vr.render import find_ffmpeg, open_video_writer

        if find_ffmpeg() is None:
            pytest.skip("no ffmpeg on this machine")
        out = tmp_path / "clip.mp4"
        writer = open_video_writer(out, 30.0, 64, 48)
        if writer.codec != "h264":
            pytest.skip("ffmpeg without libx264")
        for i in range(5):
            writer.write(np.full((48, 64, 3), i * 40, dtype=np.uint8))
        writer.close()
        assert out.stat().st_size > 0
        assert self._fourcc(out) == "h264:5"  # OpenCV reports the codec, H.264

    def test_falls_back_to_opencv_without_ffmpeg(
        self, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, caplog: pytest.LogCaptureFixture
    ) -> None:
        from robopy.vr.render import open_video_writer

        monkeypatch.setattr("robopy.vr.render.find_ffmpeg", lambda: None)
        out = tmp_path / "clip.mp4"
        with caplog.at_level("WARNING", logger="robopy.vr.render"):
            writer = open_video_writer(out, 30.0, 64, 48)
        assert writer.codec == "mp4v" and "mp4v" in caplog.text
        writer.write(np.zeros((48, 64, 3), dtype=np.uint8))
        writer.close()
        assert self._fourcc(out) == "FMP4:1"  # OpenCV's name for MPEG-4 part 2

    def test_odd_sizes_are_rounded_up_to_even(self) -> None:
        s = RenderSettings(width=321, height=241)
        assert (s.width, s.height) == (322, 242)


@pytest.mark.skipif(not HAS_MUJOCO, reason="mujoco not installed")
class TestRenderWithMujoco:
    def test_recording_renders_to_two_mp4s(self, bundle: ModelBundle, tmp_path: Path) -> None:
        import cv2

        from robopy.vr.render import render_recording

        session, _ = make_session(bundle, mapping="absolute")
        session.recorder = SessionRecorder(tmp_path)
        session.handle({"type": "record", "action": "start"}, 0.0)
        t = 0.0
        for _ in range(12):
            t += 0.1
            session.handle(
                {
                    "type": "pose",
                    "head": HEAD0,
                    "left": controller(-0.2, 1.3, -0.3, clutch=True),
                    "right": None,
                },
                t,
            )
        path = session.recorder.stop(t)
        assert path is not None
        progress: List[float] = []
        outputs = render_recording(
            path, settings=RenderSettings(fps=10.0, width=320, height=240), progress=progress.append
        )
        assert [p.name for p in outputs] == [
            f"{path.stem}_third_person.mp4",
            f"{path.stem}_first_person.mp4",
        ]
        assert progress and progress[-1] == pytest.approx(1.0)
        for out in outputs:
            cap = cv2.VideoCapture(str(out))
            assert cap.isOpened() and int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) == 13
            ok, frame = cap.read()
            assert ok and frame.shape == (240, 320, 3) and frame.mean() > 5
            cap.release()
