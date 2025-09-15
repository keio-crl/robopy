# タクタイルセンサー

Robopyは、DIGIT触覚センサーを統合したタクタイル情報の取得をサポートしています。

## :material-hand: DIGIT タクタイルセンサー

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
    tactile_image = sensor.capture()
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
    obs = robot.record_parallel(max_frame=100, fps=30)
    
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
tactile_image = sensor.capture()
# 形状: (240, 320, 3) - Height x Width x RGB

# 複数フレーム（記録データ）
obs = robot.record_parallel(max_frame=100, fps=30)
tactile_data = obs['sensors']['tactile']['left']
# 形状: (100, 240, 320, 3) - Frames x Height x Width x RGB
```

### 色情報

- **RGB形式**: 各ピクセルが3チャンネル（赤、緑、青）
- **値範囲**: 0-255の8bit整数
- **データ型**: `numpy.uint8`

## :material-eye: データの可視化

### 単一画像の表示

```python
import matplotlib.pyplot as plt

# タクタイル画像の取得
tactile_image = sensor.capture()

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
obs = robot.record_parallel(max_frame=100, fps=30)

# タクタイルセンサーを含むアニメーション生成
visualize_rakuda_obs(
    obs=obs,
    save_dir="./tactile_animation",
    fps=30
)
```

### カスタム可視化

```python
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def create_tactile_animation(tactile_data, save_path="tactile.gif"):
    """タクタイルデータのアニメーション作成"""
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    
    def animate(frame):
        ax1.clear()
        ax2.clear()
        
        # 左右のタクタイルデータを表示
        if 'left' in tactile_data:
            ax1.imshow(tactile_data['left'][frame])
            ax1.set_title(f"左手 - フレーム {frame}")
            ax1.axis('off')
        
        if 'right' in tactile_data:
            ax2.imshow(tactile_data['right'][frame])
            ax2.set_title(f"右手 - フレーム {frame}")
            ax2.axis('off')
    
    # アニメーション作成
    frames = len(tactile_data['left']) if 'left' in tactile_data else len(tactile_data['right'])
    anim = animation.FuncAnimation(fig, animate, frames=frames, interval=100)
    
    # 保存
    anim.save(save_path, writer='pillow', fps=10)
    plt.close()

# 使用例
tactile_data = obs['sensors']['tactile']
create_tactile_animation(tactile_data)
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

### センサーの較正

```python
def calibrate_sensor(sensor):
    """センサーの較正（基準画像の取得）"""
    
    print("センサーに何も触れずに5秒待機...")
    time.sleep(5)
    
    # 基準画像の取得
    baseline_images = []
    for i in range(10):
        image = sensor.capture()
        baseline_images.append(image)
        time.sleep(0.1)
    
    # 平均画像を計算
    baseline = np.mean(baseline_images, axis=0).astype(np.uint8)
    
    return baseline

# 使用例
sensor = DigitSensor("left", "D20542")
sensor.connect()

try:
    baseline = calibrate_sensor(sensor)
    print(f"基準画像を取得: {baseline.shape}")
    
finally:
    sensor.disconnect()
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