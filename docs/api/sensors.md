# センサーAPI

このページでは、Robopyのセンサー制御に関連するAPIを説明します。

## :material-camera: カメラ

### RealsenseCamera

::: robopy.sensors.visual.realsense_camera.RealsenseCamera
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - connect
        - disconnect
        - capture
        - is_connected
        - find_cameras

### WebCamera

::: robopy.sensors.visual.web_camera.WebCamera
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - connect
        - disconnect
        - capture
        - is_connected

### カメラ設定

::: robopy.config.sensor_config.visual_config.RealsenseCameraConfig
    options:
      show_root_heading: true
      show_source: false

::: robopy.config.sensor_config.visual_config.WebCameraConfig
    options:
      show_root_heading: true
      show_source: false

## 触覚センサー

### DigitSensor

::: robopy.sensors.tactile.digit_sensor.DigitSensor
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - connect
        - disconnect
        - capture
        - is_connected

### タクタイル設定

::: robopy.config.sensor_config.TactileParams
    options:
      show_root_heading: true
      show_source: false

## :material-cog: センサー統合

### RakudaSensorParams

::: robopy.config.robot_config.rakuda_config.RakudaSensorParams
    options:
      show_root_heading: true
      show_source: false

### CameraParams

::: robopy.config.sensor_config.CameraParams
    options:
      show_root_heading: true
      show_source: false

## 使用例

### RealSenseカメラの使用

```python
from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# 設定の作成
config = RealsenseCameraConfig(
    fps=30,
    width=640,
    height=480,
    color_mode="rgb"
)

# カメラの作成と接続
camera = RealsenseCamera("main", config)
camera.connect()

try:
    # 画像キャプチャ
    image = camera.capture()
    print(f"キャプチャした画像サイズ: {image.shape}")

finally:
    camera.disconnect()
```

### タクタイルセンサーの使用

```python
from robopy.sensors.tactile.digit_sensor import DigitSensor

# センサーの作成（シリアル番号を指定）
sensor = DigitSensor("left", "D20542")
sensor.connect()

try:
    # タクタイルデータの取得
    tactile_data = sensor.capture()
    print(f"タクタイルデータサイズ: {tactile_data.shape}")

finally:
    sensor.disconnect()
```

### 複数センサーの統合

```python
from robopy import RakudaConfig, RakudaSensorParams, TactileParams
from robopy.config.sensor_config import CameraParams
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# 統合センサー設定
sensors = RakudaSensorParams(
    cameras=[
        CameraParams(
            name="main",
            config=RealsenseCameraConfig(
                fps=30,
                width=640,
                height=480,
                color_mode="rgb"
            )
        )
    ],
    tactile=[
        TactileParams(serial_num="D20542", name="left"),
        TactileParams(serial_num="D20537", name="right"),
    ]
)

# ロボット設定に統合
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=sensors
)
```

### カメラデバイスの検出

```python
from robopy.sensors.visual.realsense_camera import RealsenseCamera

# 利用可能なRealSenseカメラを検出
cameras = RealsenseCamera.find_cameras()
print(f"検出されたカメラ: {cameras}")

for camera_info in cameras:
    print(f"カメラ名: {camera_info['name']}")
    print(f"シリアル番号: {camera_info['serial']}")
```

### センサーの状態確認

```python
# カメラの接続状態確認
if camera.is_connected:
    print("✅ カメラは接続されています")
else:
    print("❌ カメラが接続されていません")

# タクタイルセンサーの接続状態確認
if sensor.is_connected:
    print("✅ タクタイルセンサーは接続されています")
else:
    print("❌ タクタイルセンサーが接続されていません")
```
