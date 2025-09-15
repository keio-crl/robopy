# クイックスタート

このガイドでは、Robopyを使用してロボットの基本操作を素早く開始する方法を説明します。

## :material-robot: Rakuda Robotの基本操作

### 1. 設定の作成

```python
from robopy import RakudaConfig, RakudaSensorParams, TactileParams

# 基本設定（タクタイルセンサーあり）
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",      # Leaderアームのポート
    follower_port="/dev/ttyUSB1",    # Followerアームのポート
    sensors=RakudaSensorParams(
        cameras=None,  # カメラなし
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ],
    ),
    slow_mode=False,  # 高速モード
)
```

!!! tip "ポート設定"
    USBポートは環境によって異なります。`ls /dev/ttyUSB*`で確認してください。

### 2. ロボットの接続と制御

```python
from robopy import RakudaRobot

# ロボットインスタンスの作成
robot = RakudaRobot(config)

try:
    # 接続
    robot.connect()
    print("✅ ロボットに接続しました")
    
    # テレオペレーション（5秒間）
    print("🎮 テレオペレーションを開始します...")
    robot.teleoperation(duration=5)
    
finally:
    # 切断
    robot.disconnect()
    print("🔌 ロボットから切断しました")
```

### 3. データ記録

#### 基本記録

```python
# データ記録（100フレーム、5Hz）
print("📊 データ記録を開始します...")
obs = robot.record(max_frame=100, fps=5)

# 記録されたデータの確認
print(f"📈 記録完了！")
print(f"  Leader アーム: {obs['arms']['leader'].shape}")
print(f"  Follower アーム: {obs['arms']['follower'].shape}")

if obs['sensors']:
    if obs['sensors']['tactile']:
        for name, data in obs['sensors']['tactile'].items():
            if data is not None:
                print(f"  タクタイル {name}: {data.shape}")
```

#### 高速並列記録

```python
# 並列記録（1000フレーム、30Hz）
print("⚡ 高速データ記録を開始します...")
obs = robot.record_parallel(
    max_frame=1000, 
    fps=30,
    max_processing_time_ms=25.0  # 処理時間制限
)

print(f"📈 高速記録完了！フレーム数: {obs['arms']['leader'].shape[0]}")
```

## :material-flask: 実験ハンドラーの使用

より簡単に実験を行うには、[`RakudaExpHandler`](../api/utils.md#robopy.utils.exp_interface.RakudaExpHandler)を使用します。

```python
from robopy.utils.exp_interface import RakudaExpHandler

# 実験ハンドラーの作成
handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    left_digit_serial="D20542",    # 左タクタイル
    right_digit_serial="D20537",   # 右タクタイル
    fps=30                         # 記録周波数
)

# インタラクティブな記録・保存
handler.recode_save(
    max_frames=1000,
    save_path="my_experiment",
    if_async=True  # 並列処理を使用
)
```

!!! success "インタラクティブ操作"
    この方法では、以下のような対話的な操作ができます：
    
    1. ✨ ウォームアップ指示に従ってロボットを準備
    2. ⏯️ Enterキーで記録開始
    3. 💾 記録完了後、データ保存の選択
    4. 🎬 アニメーション自動生成

## :material-chart-line: データの可視化

記録したデータをアニメーションで確認：
### 生成されるファイル

- `rakuda_obs_animation.gif` - カメラ+タクタイルセンサーのアニメーション
- `rakuda_arm_obs.png` - アーム関節角度のプロット

## :material-camera: カスタマイズ設定

RealSenseカメラを使用する場合の設定：

```python
from robopy.config.sensor_config import CameraParams
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# RealSenseカメラ付き設定
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
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ],
    ),
)
```

## :material-cog: 実践的な例

### 完全な記録セッション

```python
#!/usr/bin/env python3
"""
Rakuda Robot記録セッション例
"""

from robopy import RakudaConfig, RakudaSensorParams, TactileParams, RakudaRobot
from robopy.utils.animation_maker import visualize_rakuda_obs
import time

def main():
    # 設定
    config = RakudaConfig(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
        sensors=RakudaSensorParams(
            cameras=None,
            tactile=[
                TactileParams(serial_num="D20542", name="left"),
                TactileParams(serial_num="D20537", name="right"),
            ],
        ),
        slow_mode=False,
    )
    
    # ロボット作成
    robot = RakudaRobot(config)
    
    try:
        # 接続
        print("🔗 ロボットに接続中...")
        robot.connect()
        print("✅ 接続完了")
        
        # ウォームアップ
        print("🔥 5秒間のウォームアップを開始...")
        robot.teleoperation(duration=5)
        
        # 記録準備
        print("📝 記録準備中... Enterで開始")
        input()
        
        # データ記録
        print("🎬 記録開始（30秒間、30Hz）")
        start_time = time.time()
        obs = robot.record_parallel(
            max_frame=900,  # 30秒 × 30fps
            fps=30,
            max_processing_time_ms=25.0
        )
        end_time = time.time()
        
        print(f"✅ 記録完了（{end_time - start_time:.1f}秒）")
        print(f"📊 記録フレーム数: {obs['arms']['leader'].shape[0]}")
        
        # アニメーション生成
        print("🎨 アニメーション生成中...")
        visualize_rakuda_obs(
            obs=obs,
            save_dir="./experiment_output",
            fps=30
        )
        print("🎬 アニメーション生成完了")
        
    except KeyboardInterrupt:
        print("⚠️  ユーザーによって中断されました")
    except Exception as e:
        print(f"❌ エラーが発生しました: {e}")
    finally:
        robot.disconnect()
        print("🔌 ロボットから切断しました")

if __name__ == "__main__":
    main()
```

## :material-alert-circle: よくある問題と解決策

### ポート接続エラー

```python
try:
    robot.connect()
except RuntimeError as e:
    print(f"❌ 接続失敗: {e}")
    print("💡 以下を確認してください:")
    print("  - USBケーブルの接続")
    print("  - ポート権限（sudo chmod 666 /dev/ttyUSB*）")
    print("  - ポート番号（ls /dev/ttyUSB*）")
```

### センサー読み取りエラー

```python
# センサーの個別確認
if robot.sensors and robot.sensors.tactile:
    for sensor in robot.sensors.tactile:
        if sensor.is_connected:
            print(f"✅ {sensor.name}: 接続OK")
        else:
            print(f"❌ {sensor.name}: 接続NG")
```

### パフォーマンス問題

```python
# フレームスキップ確認
obs = robot.record_parallel(max_frame=100, fps=30)
actual_fps = obs['arms']['leader'].shape[0] / (100/30)
print(f"実際のFPS: {actual_fps:.1f}")

if actual_fps < 25:
    print("⚠️ フレームスキップが発生しています")
    print("💡 max_processing_time_msを増やすか、fpsを下げてください")
```

## :material-arrow-right: 次のステップ

基本操作を理解したら、以下のドキュメントでさらに詳しく学習しましょう：

- [**Rakudaロボットの詳細**](../robots/rakuda.md) - より高度な制御方法
- [**センサー設定**](../sensors/cameras.md) - カメラとタクタイルセンサーの設定
- [**実験インターフェース**](../experiments/handlers.md) - 大規模実験の実行方法
- [**API リファレンス**](../api/robots.md) - 全ての関数とクラスの詳細