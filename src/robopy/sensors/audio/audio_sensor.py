import queue
import time
from logging import getLogger
from threading import Event, Lock, Thread
from typing import Any, override

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

        # Audio parameters (from config)
        self.sample_rate = config.sample_rate
        self.chunk_size = 1100  # Calculated based on fps
        self.n_mels = config.n_mels
        self.n_fft = config.n_fft
        self.hop_length = config.hop_length
        self.fmax = config.fmax

        self.pyaudio_instance: Any = None
        self.stream: Any = None

        # Audio buffer and thread management
        self.audio_buffer: queue.Queue[NDArray | None] = queue.Queue(maxsize=2)
        self.buffer_data = np.zeros(self.sample_rate, dtype=np.float32)  # 1秒分のバッファ
        self.buffer_lock = Lock()
        self.callback_active = True  # Flag to control audio callback

    def connect(self) -> None:
        # Initialize PyAudio instance
        self.pyaudio_instance = pyaudio.PyAudio()

        # Calculate frames_per_buffer
        fps_val = self.fps if self.fps is not None else 30
        frames_per_buffer = self.sample_rate // fps_val
        logger.debug(
            f"Opening audio stream: sample_rate={self.sample_rate}, "
            f"fps={self.fps}, frames_per_buffer={frames_per_buffer}"
        )

        # Open audio stream
        self._is_connected = True
        self.callback_active = True

        logger.debug(f"Opening audio stream with callback for Audio sensor {self.name}")

        self.stream = self.pyaudio_instance.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.sample_rate,
            input=True,
            frames_per_buffer=frames_per_buffer,
            stream_callback=self._audio_callback,
        )
        assert self.stream is not None

        # PyAudioのstream_callbackモードでは自動的に開始されるが、
        # 念のため明示的にstartを呼ぶ
        try:
            if not self.stream.is_active():
                self.stream.start_stream()
                logger.debug(f"Audio stream started for Audio sensor {self.name}")
            else:
                logger.debug(f"Audio stream is already active for Audio sensor {self.name}")
        except Exception as e:
            logger.error(f"Failed to start audio stream: {e}")

        stream = self.stream
        logger.debug(f"Stream active: {stream.is_active()}, latency: {stream.get_input_latency()}")

        # Start capture thread and wait for initial frame
        self._start_capture_thread()

        # Wait for initial audio frame
        logger.debug(f"Waiting for initial audio frame from Audio sensor {self.name}...")
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
        self.callback_active = False  # Stop callback

        # Terminate PyAudio instance
        if self.pyaudio_instance is not None:
            self.pyaudio_instance.terminate()
            self.pyaudio_instance = None

        logger.info(f"Audio sensor {self.name} disconnected")

    def _find_input_device(self) -> int | None:
        """Find an available input device."""
        if self.pyaudio_instance is None:
            return None

        logger.debug(f"Searching for input devices for Audio sensor {self.name}...")

        # List available devices
        device_count = self.pyaudio_instance.get_device_count()
        logger.debug(f"Found {device_count} audio devices")

        for i in range(device_count):
            try:
                device_info = self.pyaudio_instance.get_device_info_by_index(i)
                device_name = device_info["name"]
                max_input_channels = device_info["maxInputChannels"]

                logger.debug(
                    f"Device {i}: {device_name} (max input channels: {max_input_channels})"
                )

                # Use the first device with input capability
                if max_input_channels > 0:
                    logger.info(f"Using input device {i}: {device_name}")
                    return i

            except Exception as e:
                logger.warning(f"Error getting info for device {i}: {e}")
                continue

        # Fallback to default input device
        try:
            default_device = self.pyaudio_instance.get_default_input_device_info()
            logger.info(f"Using default input device: {default_device['name']}")
            return default_device["index"]
        except Exception as e:
            logger.warning(f"Could not get default input device: {e}")
            return None

    @override
    def read(self) -> NDArray[np.float32]:
        if not self.is_connected:
            raise RuntimeError(f"Audio sensor {self.name} is not connected")

        with self.frame_lock:
            if self.latest_frame is not None and self.frame_ready:
                return self.latest_frame.copy()

            # If no frame is available, return a dummy frame
            logger.warning(f"No audio frame available for {self.name}, returning dummy frame")
        return np.zeros((self.n_mels, self.n_mels), dtype=np.float32)

    @override
    def async_read(self, timeout_ms: float = 100) -> NDArray[np.float32]:
        """Read the latest frame asynchronously (non-blocking)."""
        if not self.is_connected:
            logger.warning(f"Audio sensor {self.name} is not connected")
            # Return a dummy frame if not connected
            return np.zeros((self.n_mels, self.n_mels), dtype=np.float32)

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

            # If no frame is available and thread is alive, return a dummy frame
            # This handles cases where audio device is not available
            if thread_alive:
                logger.warning(f"No audio frame available for {self.name}, returning dummy frame")
                return np.zeros((self.n_mels, self.n_mels), dtype=np.float32)

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
            # Return a dummy frame instead of raising an error
            logger.warning(
                f"No frame available from Audio sensor {self.name}, returning dummy frame"
            )
            return np.zeros((self.n_mels, self.n_mels), dtype=np.float32)

        return frame_copy

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """Audio stream callback function"""
        # 最初に呼ばれた時の初期化
        if not hasattr(self, "_callback_count"):
            self._callback_count = 0

        if not self._is_connected or not self.callback_active:
            logger.debug(
                f"Callback returning early: "
                f"_is_connected={self._is_connected}, "
                f"callback_active={self.callback_active}"
            )
            return (in_data, pyaudio.paComplete)

        try:
            audio_data = np.frombuffer(in_data, dtype=np.float32)

            # デバッグ: 最初の数回のコールバックのみ
            self._callback_count += 1

            if self._callback_count <= 5:
                logger.debug(
                    f"[AUDIO CALLBACK #{self._callback_count}] "
                    f"data shape={audio_data.shape}, "
                    f"mean={np.mean(audio_data):.6f}, "
                    f"std={np.std(audio_data):.6f}, "
                    f"min={np.min(audio_data):.6f}, "
                    f"max={np.max(audio_data):.6f}"
                )

            # 音声レベルをチェック (RMS)
            rms = np.sqrt(np.mean(audio_data**2))
            if self._callback_count <= 5:
                logger.debug(f"[AUDIO CALLBACK #{self._callback_count}] RMS: {rms:.6f}")

            # バッファにデータを追加（非ブロッキング）
            try:
                self.audio_buffer.put_nowait(audio_data)
            except queue.Full:
                # バッファが満杯の場合、古いデータを破棄して新データを追加
                try:
                    self.audio_buffer.get_nowait()
                    self.audio_buffer.put_nowait(audio_data)
                    if self._callback_count <= 20:
                        logger.debug("Audio buffer full, discarding old data")
                except queue.Empty:
                    pass

            return (in_data, pyaudio.paContinue)
        except Exception as e:
            logger.error(f"Error in audio callback: {e}")
            import traceback

            traceback.print_exc()
            return (in_data, pyaudio.paComplete)

    def _start_capture_thread(self) -> None:
        if self.thread is not None and self.thread.is_alive():
            self._stop_capture_thread()

        self.stop_event = Event()
        self.thread = Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
        logger.debug(f"Capture thread for Audio sensor {self.name} started.")

    def _stop_capture_thread(self) -> None:
        if self.stop_event:
            self.stop_event.set()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)

        self.thread = None
        self.stop_event = None
        logger.debug(f"Capture thread for Audio sensor {self.name} stopped.")

    def _capture_loop(self) -> None:
        """Background thread loop for continuous frame capture."""
        if self.stop_event is None:
            raise RuntimeError("Stop event is not initialized.")

        logger.debug(f"[CAPTURE LOOP] Started for Audio sensor {self.name}")
        frame_count = 0
        empty_count = 0

        while not self.stop_event.is_set():
            try:
                # Get new audio data from buffer
                buffer_size_before = self.audio_buffer.qsize()
                new_data = self.audio_buffer.get(timeout=0.1)

                if new_data is not None:
                    # デバッグ: 最初の数回のフレームのみ
                    if frame_count < 5:
                        logger.debug(
                            f"[CAPTURE LOOP] Processing frame #{frame_count}: "
                            f"new_data shape={new_data.shape}, "
                            f"buffer_data shape={self.buffer_data.shape}, "
                            f"buffer size before: {buffer_size_before}"
                        )

                    # Update buffer data (sliding window approach like in reference/saver.py)
                    with self.buffer_lock:
                        # Shift buffer and add new data
                        self.buffer_data = np.concatenate(
                            (self.buffer_data[len(new_data) :], new_data)
                        )

                        # Generate mel spectrogram from buffer data
                        mel_spectrogram = librosa.feature.melspectrogram(
                            y=self.buffer_data,
                            sr=self.sample_rate,
                            n_fft=self.n_fft,
                            hop_length=self.hop_length,
                            n_mels=self.n_mels,
                            fmax=self.fmax,
                        )

                        # Convert to log scale using fixed reference (not per-frame max)
                        # This prevents background from gradually brightening
                        # Using ref=1.0 provides stable amplitude scaling across frames
                        log_mel_spectrogram = librosa.power_to_db(mel_spectrogram, ref=1.0)
                        processed_frame = np.flipud(log_mel_spectrogram)

                        # デバッグ: 最初の数回のフレームのみ
                        if frame_count < 3:
                            logger.debug(
                                f"Processed spectrogram: "
                                f"shape={processed_frame.shape}, "
                                f"min={np.min(processed_frame):.2f}, "
                                f"max={np.max(processed_frame):.2f}, "
                                f"mean={np.mean(processed_frame):.2f}"
                            )

                    with self.frame_lock:
                        self.latest_frame = processed_frame
                        if not self.frame_ready:
                            self.frame_ready = True

                    self.new_frame_event.set()
                    frame_count += 1

                    # フレーム処理の詳細はDEBUGのみ
                    logger.debug(
                        f"Audio sensor {self.name} captured frame {frame_count}, "
                        f"shape: {processed_frame.shape}"
                    )
                else:
                    logger.warning(f"Audio sensor {self.name} returned None frame")

            except queue.Empty:
                # No data available, continue
                empty_count += 1
                buffer_size = self.audio_buffer.qsize()
                if empty_count % 20 == 0:  # Every 2 seconds (20 * 0.1s)
                    logger.debug(
                        f"[CAPTURE LOOP] {self.name}: No data received for "
                        f"{empty_count * 0.1:.1f} seconds "
                        f"(buffer size: {buffer_size})"
                    )
                continue
            except Exception as e:
                if self._is_connected:
                    logger.error(f"Error in capture loop for Audio sensor {self.name}: {e}")
                    import traceback

                    traceback.print_exc()
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
