# 設定API

このページでは、Robopyの設定に関連するAPIを説明します。

## :material-cog: ロボット設定

### RakudaConfig

::: robopy.config.robot_config.rakuda_config.RakudaConfig
    options:
      show_root_heading: true
      show_source: false

### KochConfig

::: robopy.config.robot_config.koch_config.KochConfig
    options:
      show_root_heading: true
      show_source: false

### RakudaSensorParams

::: robopy.config.robot_config.rakuda_config.RakudaSensorParams
    options:
      show_root_heading: true
      show_source: false

## :material-camera-outline: センサー設定

### CameraParams

::: robopy.config.sensor_config.CameraParams
    options:
      show_root_heading: true
      show_source: false

### TactileParams

::: robopy.config.sensor_config.TactileParams
    options:
      show_root_heading: true
      show_source: false

## :material-video: ビジュアル設定

### RealsenseCameraConfig

::: robopy.config.sensor_config.visual_config.RealsenseCameraConfig
    options:
      show_root_heading: true
      show_source: false

### WebCameraConfig

::: robopy.config.sensor_config.visual_config.WebCameraConfig
    options:
      show_root_heading: true
      show_source: false

## :material-chart-line: データ型

### RakudaObs

::: robopy.config.robot_config.rakuda_config.RakudaObs
    options:
      show_root_heading: true
      show_source: false

### KochObservation

::: robopy.config.robot_config.koch_config.KochObservation
    options:
      show_root_heading: true
      show_source: false

## 使用例

### 基本的なRakuda設定

```python
from robopy import RakudaConfig

# 最小限の設定
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1"
)

print(f"Leader ポート: {config.leader_port}")
print(f"Follower ポート: {config.follower_port}")
print(f"スローモード: {config.slow_mode}")
```

### センサー付きの設定

```python
from robopy import RakudaConfig, RakudaSensorParams, TactileParams
from robopy.config.sensor_config import CameraParams
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# 完全なセンサー設定
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(
        cameras=[
            CameraParams(
                name="main",
                config=RealsenseCameraConfig(
                    fps=30,
                    width=640,
                    height=480,
                    color_mode="rgb",
                    depth_mode="z16"
                )
            ),
            CameraParams(
                name="side",
                config=RealsenseCameraConfig(
                    fps=15,
                    width=424,
                    height=240,
                    color_mode="rgb"
                )
            )
        ],
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ]
    ),
    slow_mode=False
)
```

### Koch設定

```python
from robopy.config.robot_config.koch_config import KochConfig

# Koch ロボットの設定
config = KochConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    camera_config=None  # カメラなし
)
```

### カメラ設定の詳細

```python
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# 高解像度設定
high_res_config = RealsenseCameraConfig(
    fps=15,
    width=1280,
    height=720,
    color_mode="rgb",
    depth_mode="z16",
    enable_depth=True
)

# 高速設定
fast_config = RealsenseCameraConfig(
    fps=60,
    width=424,
    height=240,
    color_mode="rgb",
    enable_depth=False
)

# Webカメラ設定
from robopy.config.sensor_config.visual_config import WebCameraConfig

webcam_config = WebCameraConfig(
    device_id=0,
    fps=30,
    width=640,
    height=480
)
```

### 設定の検証

```python
# 設定値の確認
if config.sensors:
    if config.sensors.cameras:
        print(f"カメラ数: {len(config.sensors.cameras)}")
        for cam in config.sensors.cameras:
            print(f"  {cam.name}: {cam.config}")

    if config.sensors.tactile:
        print(f"タクタイルセンサー数: {len(config.sensors.tactile)}")
        for tactile in config.sensors.tactile:
            print(f"  {tactile.name}: {tactile.serial_num}")
```

### 設定の保存と読み込み

```python
import json
from pathlib import Path

# 設定の保存（JSON形式）
config_dict = {
    "leader_port": config.leader_port,
    "follower_port": config.follower_port,
    "slow_mode": config.slow_mode,
}

if config.sensors:
    if config.sensors.cameras:
        config_dict["cameras"] = [
            {
                "name": cam.name,
                "fps": cam.config.fps,
                "width": cam.config.width,
                "height": cam.config.height,
            }
            for cam in config.sensors.cameras
        ]

    if config.sensors.tactile:
        config_dict["tactile"] = [
            {
                "name": tactile.name,
                "serial_num": tactile.serial_num,
            }
            for tactile in config.sensors.tactile
        ]

# ファイルに保存
config_path = Path("robot_config.json")
with open(config_path, "w") as f:
    json.dump(config_dict, f, indent=2)

print(f"設定を {config_path} に保存しました")
```

### 環境変数による設定

```python
import os
from robopy import RakudaConfig

# 環境変数から設定値を取得
config = RakudaConfig(
    leader_port=os.getenv("RAKUDA_LEADER_PORT", "/dev/ttyUSB0"),
    follower_port=os.getenv("RAKUDA_FOLLOWER_PORT", "/dev/ttyUSB1"),
    slow_mode=os.getenv("RAKUDA_SLOW_MODE", "false").lower() == "true"
)
```
