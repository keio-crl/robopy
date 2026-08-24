"""Rakuda teleoperation with the low-latency Dynamixel settings enabled.

See docs/performance/dynamixel.md for what each knob does and how much it buys.
Measure your own bus with:

    uv run python scripts/bench_dynamixel.py --port /dev/ttyUSB0 --ids 1,2,3
"""

import logging
import time

from robopy.config import RakudaConfig
from robopy.motor.dynamixel_transport import native_available
from robopy.robots.rakuda.rakuda_pair_sys import RakudaPairSys

logging.basicConfig(level=logging.INFO)


def main() -> None:
    config = RakudaConfig(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
        # "auto" uses the C++ transport when robopy_dxl is installed
        # (`uv pip install ./native/robopy_dxl`) and the Python SDK otherwise.
        motor_backend="auto",
        # Lower the FTDI USB latency timer from its 16ms default when opening.
        usb_latency_timer_ms=1,
        # Zero the per-motor Return Delay Time (factory default 250 = 500us).
        # Written to EEPROM once; motors already at 0 are left alone.
        return_delay_time=0,
    )

    system = RakudaPairSys(config)
    system.connect()

    if not native_available():
        print(
            "robopy_dxl is not installed: running on the pure-Python transport.\n"
            "Install it with `uv pip install ./native/robopy_dxl` for Fast Sync Read."
        )

    try:
        # Warm up so the group objects and the fast/plain decision are settled.
        for _ in range(10):
            system.get_observation()

        frames = 500
        start = time.perf_counter()
        for _ in range(frames):
            system.control_step()
        elapsed = time.perf_counter() - start

        print(
            f"backend={system.leader.motors.backend}  "
            f"{frames} control steps in {elapsed:.2f}s  "
            f"({frames / elapsed:.1f} Hz, {elapsed / frames * 1000:.2f} ms/step)"
        )
    finally:
        system.disconnect()


if __name__ == "__main__":
    main()
