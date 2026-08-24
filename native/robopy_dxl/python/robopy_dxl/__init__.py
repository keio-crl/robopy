"""C++ Dynamixel transport for robopy.

This package is a thin pybind11 wrapper around the **C++** DynamixelSDK.  It is
optional: `robopy` runs on the pure-Python `dynamixel_sdk` when it is absent and
switches to this transport automatically when it is importable.

What the native transport actually buys (measured against the Python SDK on the
same bus, see `scripts/bench_dynamixel.py`):

* **Fast Sync Read** (protocol 2.0 instruction ``0x8A``).  A plain sync read of
  N motors costs N status packets, each preceded by that motor's return delay.
  Fast Sync Read answers with a single broadcast packet.  The Python SDK pinned
  by robopy (3.7.x) does not implement it.
* **Reused group objects.**  The Python bus allocates a new
  ``GroupSyncRead``/``GroupSyncWrite`` and re-serialises every parameter on each
  call; here the groups are built once at connect time.
* **GIL-free transfers**, so a leader and a follower arm (two independent USB
  devices) can be read concurrently instead of sequentially.
* **USB latency-timer tuning**, which is usually the single largest cost on
  Linux: the ftdi_sio default of 16 ms dwarfs the few milliseconds of wire time.

Example
-------
```python
from robopy_dxl import Bus, set_latency_timer

set_latency_timer("/dev/ttyUSB0", 1)
bus = Bus("/dev/ttyUSB0", baudrate=1_000_000)
bus.open()
group = bus.make_read_group(address=132, length=4, ids=[1, 2, 3], is_signed=True)
positions = bus.sync_read(group)  # -> np.ndarray[int32]
```
"""

from ._core import (
    Bus,
    DxlCommError,
    PortTuning,
    __version__,
    get_latency_timer,
    set_latency_timer,
    sync_read_parallel,
    sync_write_parallel,
)

__all__ = [
    "Bus",
    "DxlCommError",
    "PortTuning",
    "__version__",
    "get_latency_timer",
    "set_latency_timer",
    "sync_read_parallel",
    "sync_write_parallel",
]
