# VR テレオペ API

`robopy.vr` は Meta Quest などの WebXR ヘッドセットから Rakuda を操作するためのモジュールです。
使い方は [Rakuda ページの VR テレオペ節](../robots/rakuda.md#vr) を参照してください。

## 座標変換

::: robopy.vr.xr_math
    options:
      show_root_heading: true
      show_source: false
      members:
        - XR_TO_ROBOT
        - xr_pose_to_robot
        - xr_rotation_to_robot
        - yaw_pitch_of_forward
        - OperatorFrame

## 頭部追従

::: robopy.vr.head_tracking
    options:
      show_root_heading: true
      show_source: false
      members:
        - HeadJointMapping
        - HeadTrackingConfig
        - HeadTracker
        - HeadCommand

## 腕のテレオペ

::: robopy.vr.arm_teleop
    options:
      show_root_heading: true
      show_source: false
      members:
        - ArmTeleopConfig
        - ControllerSample
        - ArmCommand
        - ArmTeleop
        - DualArmTeleop

## バックエンド

::: robopy.vr.backend
    options:
      show_root_heading: true
      show_source: false
      members:
        - TeleopCommand
        - BackendReport
        - TeleopBackend
        - SimulationBackend
        - ControlSystemBackend

## カメラ配信

::: robopy.vr.camera
    options:
      show_root_heading: true
      show_source: false
      members:
        - FrameSource
        - SyntheticFrameSource
        - OpenCVFrameSource
        - CallableFrameSource
        - JpegEncoder
        - FrameStreamer

## サーバ

::: robopy.vr.server
    options:
      show_root_heading: true
      show_source: false
      members:
        - VRServerConfig
        - TeleopSession
        - VRServer
        - serve_vr

## WebSocket

::: robopy.vr.websocket
    options:
      show_root_heading: true
      show_source: false
      members:
        - handshake
        - WebSocket
        - encode_frame
        - decode_frame
