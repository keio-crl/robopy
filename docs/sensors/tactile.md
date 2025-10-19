# タクタイルセンサー

Robopyは、DIGIT触覚センサーを統合したタクタイル情報の取得をサポートしています。

## DIGIT タクタイルセンサー

DIGITは、Meta Reality Labs（旧Facebook Reality Labs）が開発した視覚ベースの触覚センサーです。

### 特徴

- **高解像度触覚画像**: 240x320ピクセルのRGB画像
- **リアルタイム取得**: 高フレームレートでの連続取得
- **接触検知**: 物体との接触を視覚的に検出
- **変形検知**: センサー表面の変形を画像として記録

## :material-cog: 基本的な使用方法

### センサーの作成と接続

```python
from robopy.sensors.tactile.digit_sensor import DigitSensor

# センサーの作成（シリアル番号を指定）
sensor = DigitSensor("left", "D20542")

# 接続
sensor.connect()

try:
    # タクタイル画像の取得
    tactile_image = sensor.read()
    print(f"タクタイル画像サイズ: {tactile_image.shape}")
    
    # 接続状態確認
    if sensor.is_connected:
        print("✅ センサーは接続されています")
    
finally:
    sensor.disconnect()
```

### 連続データ取得

```python
import time

sensor = DigitSensor("right", "D20537")
sensor.connect()

try:
    # 10回の連続取得
    for i in range(10):
        tactile_data = sensor.capture()
        print(f"フレーム {i}: {tactile_data.shape}")
        time.sleep(0.1)  # 100ms間隔
        
finally:
    sensor.disconnect()
```

## :material-robot: ロボットとの統合

### Rakudaロボットでの使用

```python
from robopy import RakudaConfig, RakudaSensorParams, TactileParams

# タクタイルセンサー付きRakuda設定
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(
        cameras=[],
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ]
    )
)
```

### 複数センサーの同期取得

```python
from robopy import RakudaRobot

robot = RakudaRobot(config)
robot.connect()

try:
    # データ記録（タクタイルセンサー含む）
    obs = robot.record_parallel(max_frame=100, fps=20)
    
    # タクタイルデータの確認
    if obs['sensors']['tactile']:
        for name, data in obs['sensors']['tactile'].items():
            print(f"タクタイル {name}: {data.shape}")
            
finally:
    robot.disconnect()
```

## :material-chart-line: データ形式

### 画像データ

DIGITセンサーから取得される画像は以下の形式です：

```python
# 単一フレーム
tactile_image = sensor.read()
# 形状: (3, 240, 320) - C x H x W

# 複数フレーム（記録データ）
obs = robot.record_parallel(max_frame=100, fps=20)
tactile_data = obs['sensors']['tactile']['left']
# 形状: (100,3,240, 320) - Frames x C x H x W
```

### 色情報

- **RGB形式**: 各ピクセルが3チャンネル（赤、緑、青）
- **値範囲**: 0-255の8bit整数
- **データ型**: `numpy.float32`

## :material-eye: データの可視化

### 単一画像の表示

```python
import matplotlib.pyplot as plt

# タクタイル画像の取得
tactile_image = sensor.read()

# 表示
plt.figure(figsize=(8, 6))
plt.imshow(tactile_image)
plt.title("DIGIT タクタイル画像")
plt.axis('off')
plt.show()
```

### 時系列データの可視化

```python
from robopy.utils.animation_maker import visualize_rakuda_obs

# 記録データからアニメーション生成
obs = robot.record_parallel(max_frame=100, fps=20)

# タクタイルセンサーを含むアニメーション生成
visualize_rakuda_obs(
    obs=obs,
    save_dir="./tactile_animation",
    fps=20
)
```

## :material-tune: 設定とパラメータ

### センサー設定

```python
from robopy.config.sensor_config import TactileParams

# 基本設定
left_tactile = TactileParams(
    serial_num="D20542",
    name="left"
)

right_tactile = TactileParams(
    serial_num="D20537",
    name="right"
)
```

## :material-alert-circle: トラブルシューティング

### センサーが認識されない

```python
# 利用可能なDIGITセンサーの確認
from digit_interface.digit import Digit

# センサー一覧の取得
digit_devices = Digit.get_digit_devices()
print(f"検出されたDIGITセンサー: {digit_devices}")
```

### 接続エラー

```python
try:
    sensor.connect()
except Exception as e:
    print(f"接続エラー: {e}")
    print("以下を確認してください:")
    print("- センサーのUSB接続")
    print("- シリアル番号の正確性")
    print("- 他のプロセスでの使用状況")
```

### データ取得エラー

```python
try:
    tactile_data = sensor.capture()
except Exception as e:
    print(f"データ取得エラー: {e}")
    
    # 再接続試行
    sensor.disconnect()
    time.sleep(1)
    sensor.connect()
```

### パフォーマンス問題

```python
import time

# フレームレートの測定
sensor = DigitSensor("test", "D20542")
sensor.connect()

try:
    start_time = time.time()
    frame_count = 100
    
    for i in range(frame_count):
        tactile_data = sensor.capture()
    
    end_time = time.time()
    actual_fps = frame_count / (end_time - start_time)
    print(f"実際のフレームレート: {actual_fps:.1f} fps")
    
finally:
    sensor.disconnect()
```

## :material-link-variant: 関連API

- [**DigitSensor**](../api/sensors.md#robopy.sensors.tactile.digit_sensor.DigitSensor) - DIGITセンサークラス
- [**TactileParams**](../api/sensors.md#robopy.config.sensor_config.TactileParams) - タクタイル設定
- [**RakudaSensorParams**](../api/config.md#robopy.config.robot_config.rakuda_config.RakudaSensorParams) - 統合センサー設定