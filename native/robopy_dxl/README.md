# robopy-dxl

C++ transport for robopy's Dynamixel bus: a [pybind11](https://pybind11.readthedocs.io/)
extension around the **C++** [DynamixelSDK](https://github.com/ROBOTIS-GIT/DynamixelSDK).

It is an *optional* companion package. `robopy` keeps working on the pure-Python
`dynamixel_sdk`; when `robopy_dxl` is importable, `DynamixelBus` picks it up
automatically (`backend="auto"`).

## Why

Most of the latency in a Dynamixel control loop is protocol and USB latency, not
Python. This package attacks the parts that Python cannot reach:

| | Python `dynamixel_sdk` 3.7.x | `robopy_dxl` |
| --- | --- | --- |
| Fast Sync Read (`0x8A`) | not implemented | yes, with automatic fallback |
| Group objects | rebuilt on every call | built once at connect |
| Packet parsing | Python byte loops | C++ |
| GIL during transfer | held | released |
| USB latency timer | left at the 16 ms default | lowered via sysfs + `ASYNC_LOW_LATENCY` |
| Two buses (leader + follower) | sequential | `sync_read_parallel` |

Fast Sync Read is the structural win. A plain sync read of *N* motors costs *N*
status packets, each preceded by that motor's return delay; Fast Sync Read
answers the whole group with one broadcast packet.

## Install

```bash
# from a robopy checkout
uv pip install ./native/robopy_dxl
# or
pip install ./native/robopy_dxl
```

The build downloads DynamixelSDK (pinned tag, default `4.0.5`) and links it
statically, so the wheel is self contained. Requirements: a C++17 compiler and
CMake ≥ 3.20.

Offline or against a local checkout:

```bash
pip install ./native/robopy_dxl \
  -C cmake.define.ROBOPY_DXL_SDK_DIR=/path/to/DynamixelSDK
```

Pin a different SDK tag with `-C cmake.define.ROBOPY_DXL_SDK_TAG=4.0.4`.

## Usage

```python
from robopy_dxl import Bus, set_latency_timer, sync_read_parallel

set_latency_timer("/dev/ttyUSB0", 1)  # do this before opening

bus = Bus("/dev/ttyUSB0", baudrate=1_000_000)
bus.open()

# Groups are registered once and reused for every transfer.
present_position = bus.make_read_group(address=132, length=4, ids=[1, 2, 3], is_signed=True)
goal_position = bus.make_write_group(address=116, length=4, ids=[1, 2, 3])

positions = bus.sync_read(present_position)      # np.ndarray[int32], id order
bus.sync_write(goal_position, positions)

# Two arms on two USB devices, read concurrently:
leader_pos, follower_pos = sync_read_parallel([(bus_a, group_a), (bus_b, group_b)])
```

`bus.group_uses_fast(handle)` reports whether the group is still on Fast Sync
Read; it flips to `False` permanently the first time the motors fail to answer a
`0x8A` (older firmware), and the plain sync read is used from then on.

## Permissions

Lowering the latency timer through sysfs needs write access to
`/sys/bus/usb-serial/devices/ttyUSB0/latency_timer`. Without it the package falls
back to the `TIOCSSERIAL` / `ASYNC_LOW_LATENCY` ioctl, which needs no privileges
but only helps for `ftdi_sio`. To make it permanent:

```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \
  | sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Licence

This package is Apache-2.0, matching the vendored-at-build-time DynamixelSDK
(© ROBOTIS CO., LTD., Apache-2.0).
