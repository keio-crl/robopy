# カメラセンサー

Robopyは複数のカメラタイプをサポートしています。統一されたインターフェースにより、異なるカメラを簡単に切り替えることができます。

## :material-camera: サポートしているカメラ

### Intel RealSense カメラ

高精度なRGB-Dカメラです。深度情報とカラー画像を同時に取得できます。

#### 特徴

- **RGB + 深度**: カラー画像と深度画像の同時取得
- **高解像度**: 最大1920x1080（RGB）
- **高フレームレート**: 最大90fps（解像度による）
- **Linux専用**: 現在はLinux環境でのみサポート

#### 設定例

```python
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# 高解像度設定
config = RealsenseCameraConfig(
    fps=30,
    width=1280,
    height=720,
    color_mode="rgb",
    depth_mode="z16",
    enable_depth=True
)

# 高速設定
fast_config = RealsenseCameraConfig(
    fps=60,
    width=640,
    height=480,
    color_mode="rgb",
    enable_depth=False
)
```

### Webカメラ

一般的なUSB Webカメラです。シンプルな画像取得に適しています。

#### 特徴

- **広い互換性**: USB UVC対応カメラ
- **シンプル**: RGB画像のみ
- **低コスト**: 安価で入手しやすい
- **クロスプラットフォーム**: Windows、Linux、macOS対応

#### 設定例

```python
from robopy.config.sensor_config.visual_config import WebCameraConfig

# 標準設定
config = WebCameraConfig(
    device_id=0,
    fps=30,
    width=640,
    height=480
)

# 高解像度設定
hd_config = WebCameraConfig(
    device_id=0,
    fps=15,
    width=1920,
    height=1080
)
```

## :material-cog: 基本的な使用方法

### カメラの作成と接続

```python
from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# 設定
config = RealsenseCameraConfig(
    fps=30,
    width=640,
    height=480,
    color_mode="rgb"
)

# カメラの作成
camera = RealsenseCamera("main", config)

# 接続
camera.connect()

try:
    # 画像取得
    image = camera.capture()
    print(f"画像サイズ: {image.shape}")

    # 接続状態確認
    if camera.is_connected:
        print("✅ カメラは接続されています")

finally:
    camera.disconnect()
```

### Webカメラの使用

```python
from robopy.sensors.visual.web_camera import WebCamera
from robopy.config.sensor_config.visual_config import WebCameraConfig

# 設定
config = WebCameraConfig(
    device_id=0,
    fps=30,
    width=640,
    height=480
)

# カメラの作成と接続
camera = WebCamera("webcam", config)
camera.connect()

try:
    # 画像取得
    image = camera.capture()
    print(f"画像サイズ: {image.shape}")

finally:
    camera.disconnect()
```

## :material-robot: ロボットとの統合

### Rakudaロボットでの使用

```python
from robopy import RakudaConfig, RakudaSensorParams
from robopy.config.sensor_config import CameraParams
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# カメラ付きRakuda設定
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
                    color_mode="rgb"
                )
            )
        ],
        tactile=[]
    )
)
```

### 複数カメラの使用

```python
# 複数カメラの設定
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
        ),
        CameraParams(
            name="side",
            config=WebCameraConfig(
                device_id=1,
                fps=15,
                width=424,
                height=240
            )
        )
    ],
    tactile=[]
)
```

## :material-search: カメラの検出と選択

### RealSenseカメラの検出

```python
from robopy.sensors.visual.realsense_camera import RealsenseCamera

# 利用可能なカメラを検出
cameras = RealsenseCamera.find_cameras()

for camera_info in cameras:
    print(f"カメラ名: {camera_info['name']}")
    print(f"シリアル番号: {camera_info['serial']}")
    print(f"製品ライン: {camera_info['product_line']}")
```

### Webカメラのテスト

```python
import cv2

def test_webcam(device_id=0):
    """Webカメラのテスト"""
    cap = cv2.VideoCapture(device_id)

    if not cap.isOpened():
        print(f"❌ カメラ {device_id} を開けませんでした")
        return False

    # テスト画像の取得
    ret, frame = cap.read()
    cap.release()

    if ret:
        print(f"✅ カメラ {device_id} は正常に動作します")
        print(f"   解像度: {frame.shape[:2]}")
        return True
    else:
        print(f"❌ カメラ {device_id} から画像を取得できませんでした")
        return False

# 複数のデバイスIDをテスト
for device_id in range(3):
    test_webcam(device_id)
```

## :material-tune: 設定の最適化

### フレームレートと解像度のバランス

```python
# 高フレームレート優先（低解像度）
high_fps_config = RealsenseCameraConfig(
    fps=90,
    width=424,
    height=240,
    color_mode="rgb",
    enable_depth=False
)

# 高解像度優先（低フレームレート）
high_res_config = RealsenseCameraConfig(
    fps=15,
    width=1920,
    height=1080,
    color_mode="rgb",
    enable_depth=True
)

# バランス設定
balanced_config = RealsenseCameraConfig(
    fps=30,
    width=640,
    height=480,
    color_mode="rgb",
    enable_depth=True
)
```

### データ収集での設定

```python
# データ収集専用設定（効率重視）
data_collection_config = RealsenseCameraConfig(
    fps=30,
    width=640,
    height=480,
    color_mode="rgb",
    depth_mode="z16",
    enable_depth=True
)

# リアルタイム表示用設定（応答性重視）
realtime_config = RealsenseCameraConfig(
    fps=60,
    width=424,
    height=240,
    color_mode="rgb",
    enable_depth=False
)
```

## :material-alert-circle: トラブルシューティング

### RealSenseカメラが認識されない

```bash
# udevルールの確認
ls /etc/udev/rules.d/ | grep realsense

# librealsense2のインストール確認
python -c "import pyrealsense2 as rs; print('RealSense OK')"
```

### Webカメラが開けない

```python
# 利用可能なデバイスの確認
import cv2

for i in range(5):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"デバイス {i}: 利用可能")
        cap.release()
    else:
        print(f"デバイス {i}: 利用不可")
```

### 権限エラー

```bash
# ユーザーをvideoグループに追加
sudo usermod -a -G video $USER
# ログアウト・ログインが必要

# デバイスファイルの権限確認
ls -la /dev/video*
```

## :material-link-variant: 関連API

- [**RealsenseCamera**](../api/sensors.md#robopy.sensors.visual.realsense_camera.RealsenseCamera) - RealSenseカメラクラス
- [**WebCamera**](../api/sensors.md#robopy.sensors.visual.web_camera.WebCamera) - Webカメラクラス
- [**RealsenseCameraConfig**](../api/sensors.md#robopy.config.sensor_config.visual_config.RealsenseCameraConfig) - RealSense設定
- [**WebCameraConfig**](../api/sensors.md#robopy.config.sensor_config.visual_config.WebCameraConfig) - Webカメラ設定
