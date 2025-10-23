import queue
import time
from logging import getLogger
from threading import Event, Lock, Thread
from typing import override

import librosa
import numpy as np
import pyaudio
from numpy.typing import NDArray

from robopy.config.sensor_config.params_config import AudioParams

from ..common.sensor import Sensor

logger = getLogger(__name__)


class AudioSensor(Sensor):
    def __init__(self, config: AudioParams):
        self.config = config
        self.name = config.name
        self.fps = config.fps
        self._is_connected = False

        # Threading for async read
        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.new_frame_event: Event = Event()
        self.latest_frame: NDArray | None = None
        self.frame_ready = False  # Flag to track if initial frame is ready

        # Audio parameters
        self.sample_rate = 44100
        self.chunk_size = 1100
        self.n_mels = 64
        self.n_fft = 2048
        self.hop_length = 690
        self.fmax = 15000

        self.pyaudio_instance = None
        self.stream = None

        # Audio buffer and thread management
        self.audio_buffer = queue.Queue(maxsize=2)
        self.buffer_data = np.zeros(self.sample_rate, dtype=np.float32)  # 1秒分のバッファ
        self.buffer_lock = Lock()

    def connect(self) -> None:
        if self.pyaudio_instance is None:
            raise RuntimeError(f"Failed to initialize Audio sensor: {self.name}")

        # Open audio stream
        self.stream = self.pyaudio_instance.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=self.sample_rate // self.fps,
            stream_callback=self._audio_callback,
        )

        self._is_connected = True

        if self.fps is not None:
            self.stream.set_sample_rate(self.sample_rate)

        # Start capture thread and wait for initial frame
        self._start_capture_thread()

        # Wait for initial audio frame
        logger.info(f"Waiting for initial audio frame from Audio sensor {self.name}...")
        initial_timeout = 2.0  # 2 seconds for initial frame
        if not self.new_frame_event.wait(timeout=initial_timeout):
            logger.warning(f"Initial frame timeout for Audio sensor {self.name}")
        else:
            logger.info(f"Audio sensor {self.name} connected successfully")

    def disconnect(self) -> None:
        # Stop capture thread if running
        self._stop_capture_thread()

        # Stop audio stream
        if self.stream is not None:
            self.stream.stop_stream()
            self.stream.close()
            self.stream = None

        self._is_connected = False
        self.frame_ready = False  # Reset frame ready flag

        # Terminate PyAudio instance
        if self.pyaudio_instance is not None:
            self.pyaudio_instance.terminate()
            self.pyaudio_instance = None

        logger.info(f"Audio sensor {self.name} disconnected")

    @override
    def read(self) -> NDArray[np.float32]:
        if not self.is_connected:
            raise RuntimeError(f"Audio sensor {self.name} is not connected")
        frame = self.latest_frame.copy()
        return frame

    @override
    def async_read(self, timeout_ms: float = 100) -> NDArray[np.float32]:
        """Read the latest frame asynchronously (non-blocking)."""
        if not self.is_connected:
            logger.warning(f"Audio sensor {self.name} is not connected")

        if self.thread is None or not self.thread.is_alive():
            self._start_capture_thread()
            # Wait a bit for thread to start capturing
            time.sleep(0.01)

        # Check if we already have a frame available
        with self.frame_lock:
            if self.latest_frame is not None and self.frame_ready:
                return self.latest_frame.copy()

        # Wait for new frame
        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            thread_alive = self.thread is not None and self.thread.is_alive()
            # Try to get the latest frame even if timeout occurred
            with self.frame_lock:
                if self.latest_frame is not None:
                    logger.debug(f"Using cached frame for {self.name} after timeout")
                    return self.latest_frame.copy()

            raise TimeoutError(
                f"Timeout waiting for new frame from Audio sensor {self.name}. "
                f"Thread alive: {thread_alive}"
            )

        with self.frame_lock:
            frame = self.latest_frame
            # Don't clear the event immediately - keep it for potential quick successive calls
            if frame is not None:
                frame_copy = frame.copy()
            else:
                frame_copy = None

        if frame_copy is None:
            raise RuntimeError(f"No frame available from Audio sensor {self.name}.")

        return frame_copy

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """Audio stream callback function"""
        if not self._is_connected:
            return (in_data, pyaudio.paComplete)

        try:
            audio_data = np.frombuffer(in_data, dtype=np.float32)
            self.audio_buffer.put(audio_data, timeout=0.1)
            return (in_data, pyaudio.paContinue)
        except queue.Full:
            # If buffer is full, discard old data
            try:
                self.audio_buffer.get_nowait()
                self.audio_buffer.put(audio_data, timeout=0.1)
            except queue.Empty:
                pass
            return (in_data, pyaudio.paContinue)
        except Exception as e:
            logger.error(f"Error in audio callback: {e}")
            return (in_data, pyaudio.paComplete)

    def _start_capture_thread(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            self._stop_capture_thread()

        self.stop_event = Event()
        self.thread = Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.info(f"Capture thread for Audio sensor {self.name} started.")

    def _stop_capture_thread(self) -> None:
        if self.stop_event:
            self.stop_event.set()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)

        self.thread = None
        self.stop_event = None
        logger.info(f"Capture thread for Audio sensor {self.name} stopped.")

    def _capture_loop(self) -> None:
        """Background thread loop for continuous frame capture."""
        if self.stop_event is None:
            raise RuntimeError("Stop event is not initialized.")

        logger.debug(f"Capture loop started for Audio sensor {self.name}")
        frame_count = 0

        while not self.stop_event.is_set():
            try:
                frame = self.audio_buffer.get(timeout=0.1)

                if frame is not None:
                    with self.frame_lock:
                        self.latest_frame = frame
                        if not self.frame_ready:
                            self.frame_ready = True

                self.new_frame_event.set()
                frame_count += 1

                if frame_count % 100 == 0:
                    logger.debug(f"Audio sensor {self.name} captured {frame_count} frames")
                else:
                    logger.warning(f"Audio sensor {self.name} returned None frame")

            except Exception as e:
                if self._is_connected:
                    logger.error(f"Error in capture loop for Audio sensor {self.name}: {e}")
                # Add small delay to prevent tight error loop
                time.sleep(0.01)

        logger.debug(
            f"Capture loop stopped for Audio sensor {self.name}, total frames: {frame_count}"
        )

    @property
    def is_connected(self) -> bool:
        return self._is_connected

    def __repr__(self) -> str:
        return f"AudioSensor(name={self.name}, fps={self.fps}, connected={self._is_connected})"
