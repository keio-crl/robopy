# Motor API

Dynamixel バスと、その下で動く transport のリファレンスです。
チューニングの考え方は [Performance / Dynamixel](../performance/dynamixel.md) を参照してください。

## :material-bus: DynamixelBus

::: robopy.motor.dynamixel_bus.DynamixelBus
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - open
        - close
        - backend
        - tune_port
        - set_return_delay_time
        - set_calibration
        - sync_read
        - sync_write
        - read
        - write
        - uses_fast_sync_read
        - torque_enabled
        - torque_disabled

### DynamixelMotor

::: robopy.motor.dynamixel_bus.DynamixelMotor
    options:
      show_root_heading: true
      show_source: false

### sync_read_parallel

::: robopy.motor.dynamixel_bus.sync_read_parallel
    options:
      show_root_heading: true
      show_source: false

## :material-swap-horizontal: Transport

::: robopy.motor.dynamixel_transport
    options:
      show_root_heading: true
      show_source: false
      members:
        - native_available
        - create_transport
        - DynamixelTransport
        - PythonTransport
        - NativeTransport
        - DynamixelCommError

## :material-usb: ポートチューニング

::: robopy.motor.port_tuning
    options:
      show_root_heading: true
      show_source: false
      members:
        - set_latency_timer
        - get_latency_timer
        - PortTuning
