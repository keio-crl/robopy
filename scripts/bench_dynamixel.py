"""Benchmark the Dynamixel transports.

Two modes:

* ``--simulated`` (default) runs against the pty based protocol simulator in
  ``tests/support/fake_dynamixel.py``.  No hardware needed, and it is the only
  way to compare the backends reproducibly in CI.  It measures **host side
  overhead only** -- packet building, parsing, syscalls, Python/C++ boundary --
  because a pty has neither wire time nor a USB latency timer.
* ``--port /dev/ttyUSB0 --ids 1,2,3`` runs against real motors, which is what
  actually matters.  There, the protocol level wins (Fast Sync Read, return
  delay, latency timer) dominate the host side ones.

Examples::

    uv run python scripts/bench_dynamixel.py
    uv run python scripts/bench_dynamixel.py --motors 17 --iterations 2000
    uv run python scripts/bench_dynamixel.py --port /dev/ttyUSB0 --ids 1,2,3,4,5
    uv run python scripts/bench_dynamixel.py --port /dev/ttyUSB0 --ids 1,2 \\
        --second-port /dev/ttyUSB1 --second-ids 1,2
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path
from typing import Callable, Dict, List, Sequence

REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO_ROOT / "tests"))

from robopy.motor.dynamixel_bus import (  # noqa: E402
    DynamixelBus,
    DynamixelMotor,
    native_available,
    sync_read_parallel,
)
from robopy.motor.dynamixel_control_table import XControlTable  # noqa: E402


def build_motors(ids: Sequence[int], model: str = "xm430-w350") -> Dict[str, DynamixelMotor]:
    return {f"m{i}": DynamixelMotor(i, f"m{i}", model) for i in ids}


def measure(label: str, fn: Callable[[], object], iterations: int, warmup: int = 20) -> None:
    for _ in range(warmup):
        fn()

    samples: List[float] = []
    for _ in range(iterations):
        start = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - start) * 1000.0)

    samples.sort()
    mean = statistics.fmean(samples)
    print(
        f"  {label:<34} mean {mean:7.3f} ms   p50 {samples[len(samples) // 2]:7.3f}   "
        f"p99 {samples[int(len(samples) * 0.99)]:7.3f}   ({1000.0 / mean:6.1f} Hz)"
    )


def bench_bus(bus: DynamixelBus, names: List[str], iterations: int) -> None:
    print(f"\n[{bus.backend}] {len(names)} motors on {bus.port}")
    goals = {name: 2048 for name in names}

    measure(
        "sync_read(PRESENT_POSITION)",
        lambda: bus.sync_read(XControlTable.PRESENT_POSITION, names),
        iterations,
    )
    measure(
        "sync_write(GOAL_POSITION)",
        lambda: bus.sync_write(XControlTable.GOAL_POSITION, goals),
        iterations,
    )

    def read_then_write() -> None:
        bus.sync_read(XControlTable.PRESENT_POSITION, names)
        bus.sync_write(XControlTable.GOAL_POSITION, goals)

    measure("read + write (control step)", read_then_write, iterations)

    if bus.backend == "native":
        fast = bus.uses_fast_sync_read(XControlTable.PRESENT_POSITION, names)
        print(f"  fast sync read: {'in use' if fast else 'unavailable, downgraded'}")


def bench_pair(
    leader: DynamixelBus, follower: DynamixelBus, names: List[str], iterations: int
) -> None:
    print(f"\n[{leader.backend}] two buses, {len(names)} motors each")

    def sequential() -> None:
        leader.sync_read(XControlTable.PRESENT_POSITION, names)
        follower.sync_read(XControlTable.PRESENT_POSITION, names)

    def parallel() -> None:
        sync_read_parallel(
            [
                (leader, XControlTable.PRESENT_POSITION, names),
                (follower, XControlTable.PRESENT_POSITION, names),
            ]
        )

    measure("sequential leader + follower", sequential, iterations)
    measure("parallel leader + follower", parallel, iterations)


def run_simulated(args: argparse.Namespace) -> None:
    from support.fake_dynamixel import FakeDynamixelBus

    ids = list(range(1, args.motors + 1))
    names = [f"m{i}" for i in ids]
    backends = ["python"] + (["native"] if native_available() else [])
    if not native_available():
        print("robopy_dxl is not installed: only the Python backend will be measured.\n")

    print(
        "Simulated bus (pty). This measures HOST SIDE overhead only -- there is no\n"
        "wire time and no USB latency timer, so the absolute numbers are far below\n"
        "what real motors cost. The protocol wins (fast sync read collapsing N status\n"
        "packets into one, and the return delay paid per motor) only show up on real\n"
        f"hardware. Simulated return delay: {args.return_delay_us} us per status packet."
    )

    for backend in backends:
        device = FakeDynamixelBus(ids, return_delay_us=args.return_delay_us).start()
        try:
            bus = DynamixelBus(device.port, build_motors(ids), backend=backend)
            bus.open(latency_timer_ms=None)
            try:
                bench_bus(bus, names, args.iterations)
            finally:
                bus.close()
        finally:
            device.stop()

    for backend in backends:
        first = FakeDynamixelBus(ids, return_delay_us=args.return_delay_us).start()
        second = FakeDynamixelBus(ids, return_delay_us=args.return_delay_us).start()
        try:
            leader = DynamixelBus(first.port, build_motors(ids), backend=backend)
            follower = DynamixelBus(second.port, build_motors(ids), backend=backend)
            leader.open(latency_timer_ms=None)
            follower.open(latency_timer_ms=None)
            try:
                bench_pair(leader, follower, names, args.iterations)
            finally:
                leader.close()
                follower.close()
        finally:
            first.stop()
            second.stop()


def run_hardware(args: argparse.Namespace) -> None:
    ids = [int(part) for part in args.ids.split(",") if part.strip()]
    names = [f"m{i}" for i in ids]
    backends = ["python"] + (["native"] if native_available() else [])

    for backend in backends:
        bus = DynamixelBus(args.port, build_motors(ids, args.model), backend=backend)
        bus.open(baudrate=args.baudrate, latency_timer_ms=args.latency_timer)
        try:
            if args.return_delay is not None:
                bus.set_return_delay_time(args.return_delay)
            bench_bus(bus, names, args.iterations)
        finally:
            bus.close()

    if not args.second_port:
        return

    second_ids = [int(part) for part in (args.second_ids or args.ids).split(",") if part.strip()]
    for backend in backends:
        leader = DynamixelBus(args.port, build_motors(ids, args.model), backend=backend)
        follower = DynamixelBus(
            args.second_port, build_motors(second_ids, args.model), backend=backend
        )
        leader.open(baudrate=args.baudrate, latency_timer_ms=args.latency_timer)
        follower.open(baudrate=args.baudrate, latency_timer_ms=args.latency_timer)
        try:
            bench_pair(leader, follower, names, args.iterations)
        finally:
            leader.close()
            follower.close()


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--port", help="Real serial port. Omit to run the simulated benchmark.")
    parser.add_argument("--ids", default="1,2,3", help="Comma separated motor ids on --port.")
    parser.add_argument("--second-port", help="Optional second bus, for the parallel benchmark.")
    parser.add_argument("--second-ids", help="Motor ids on --second-port (defaults to --ids).")
    parser.add_argument("--model", default="xm430-w350", help="Motor model name.")
    parser.add_argument("--baudrate", type=int, default=1_000_000)
    parser.add_argument(
        "--latency-timer",
        type=int,
        default=1,
        help="USB latency timer in ms to request before opening (hardware mode).",
    )
    parser.add_argument(
        "--return-delay",
        type=int,
        default=None,
        help="If set, write this RETURN_DELAY_TIME to every motor first (hardware mode).",
    )
    parser.add_argument("--motors", type=int, default=17, help="Motor count (simulated mode).")
    parser.add_argument(
        "--return-delay-us",
        type=int,
        default=0,
        help="Simulated per-status-packet delay in us (simulated mode).",
    )
    parser.add_argument("--iterations", type=int, default=500)
    args = parser.parse_args()

    if args.port:
        run_hardware(args)
    else:
        run_simulated(args)


if __name__ == "__main__":
    main()
