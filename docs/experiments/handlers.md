# 実験ハンドラー

Robopyの実験ハンドラーは、ロボットの制御とデータ収集を統合し、効率的な実験の実行を支援します。

## :material-flask: RakudaExpHandler

[`RakudaExpHandler`](../api/utils.md#robopy.utils.exp_interface.rakuda_exp_handler.RakudaExpHandler)は、Rakudaロボットでの実験を簡単に実行するためのハイレベルインターフェースです。

### 特徴

- **統合制御**: ロボット、センサーの一括管理
- **インタラクティブ操作**: 対話的な実験実行
- **自動保存**: データとメタデータの自動保存
- **アニメーション生成**: 結果の自動可視化

## :material-cog: 基本的な使用方法

### ハンドラーの作成

```python
from robopy.utils.exp_interface import RakudaExpHandler

# 実験ハンドラーの作成
handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    left_digit_serial="D20542",
    right_digit_serial="D20537",
    fps=30
)
```

### カメラ付きハンドラー

```python
# RealSenseカメラ付きハンドラー
handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    left_digit_serial="D20542",
    right_digit_serial="D20537",
    camera_serial="D123456789",  # RealSenseのシリアル番号
    fps=30
)
```

### 実験の実行

```python
# インタラクティブな実験実行
handler.recode_save(
    max_frames=1000,
    save_path="experiment_001",
    if_async=True
)
```

## :material-play-circle: インタラクティブ実験の流れ

`recode_save()`メソッドは以下の流れで実験を実行します：

### 1. 初期化フェーズ

```
🔗 ロボットへの接続...
✅ Leader アーム接続完了
✅ Follower アーム接続完了
✅ タクタイルセンサー接続完了
✅ カメラ接続完了
```

### 2. ウォームアップフェーズ

```
🔥 ウォームアップを開始します（5秒間）
   ロボットを少し動かして準備してください...
```

### 3. 記録フェーズ

```
📝 記録準備完了
   Enterキーを押して記録を開始してください...
```

ユーザーがEnterキーを押すと記録開始：

```
🎬 記録開始（30秒間、30Hz）
📊 進行状況: ████████████████████ 100% (1000/1000 フレーム)
✅ 記録完了（29.8秒）
```

### 4. 保存フェーズ

```
💾 データを保存しますか？ [Y/n]: Y
🗂️  保存中...
✅ データ保存完了: experiment_001/
```

### 5. 可視化フェーズ

```
🎨 アニメーションを生成しますか？ [Y/n]: Y
🎬 アニメーション生成中...
✅ アニメーション生成完了
   - experiment_001/rakuda_obs_animation.gif
   - experiment_001/rakuda_arm_obs.png
```

## :material-cog-outline: 詳細設定

### パラメータの調整

```python
# 高速記録設定
handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    left_digit_serial="D20542",
    right_digit_serial="D20537",
    fps=60,  # 高フレームレート
    max_processing_time_ms=15.0  # 短い処理時間制限
)

# 長時間記録設定
handler.recode_save(
    max_frames=3000,  # 60秒間（50Hz）
    save_path="long_experiment",
    if_async=True
)
```

### カスタムセンサー設定

```python
from robopy.config.sensor_config.visual_config import RealsenseCameraConfig

# カスタムカメラ設定でハンドラー作成
camera_config = RealsenseCameraConfig(
    fps=30,
    width=1280,
    height=720,
    color_mode="rgb",
    enable_depth=True
)

handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    left_digit_serial="D20542",
    right_digit_serial="D20537",
    camera_config=camera_config,
    fps=30
)
```

## :material-script: スクリプト化された実験

### 非インタラクティブ実験

```python
#!/usr/bin/env python3
"""
自動化された実験スクリプト
"""

from robopy.utils.exp_interface import RakudaExpHandler
import time

def automated_experiment(experiment_name: str, duration_sec: float, fps: int = 30):
    """自動化された実験実行"""
    
    handler = RakudaExpHandler(
        leader_port="/dev/ttyUSB0",
        follower_port="/dev/ttyUSB1",
        left_digit_serial="D20542",
        right_digit_serial="D20537",
        fps=fps
    )
    
    try:
        # ロボット接続
        print(f"🔗 実験 '{experiment_name}' を開始...")
        robot = handler._create_robot()
        robot.connect()
        
        # ウォームアップ
        print("🔥 ウォームアップ中...")
        robot.teleoperation(duration=3)
        
        # データ記録
        max_frames = int(duration_sec * fps)
        print(f"📊 記録開始（{duration_sec}秒間、{fps}Hz）...")
        
        obs = robot.record_parallel(
            max_frame=max_frames,
            fps=fps,
            max_processing_time_ms=25.0
        )
        
        # データ保存
        print("💾 データ保存中...")
        handler._save_data(obs, experiment_name)
        
        # アニメーション生成
        print("🎬 アニメーション生成中...")
        handler._generate_animation(obs, experiment_name)
        
        print("✅ 実験完了")
        
    finally:
        robot.disconnect()

# 使用例
if __name__ == "__main__":
    automated_experiment("auto_exp_001", duration_sec=30.0, fps=30)
```

### バッチ実験

```python
def batch_experiments():
    """複数の実験を自動実行"""
    
    experiments = [
        {"name": "exp_slow", "fps": 10, "duration": 20},
        {"name": "exp_medium", "fps": 30, "duration": 15},
        {"name": "exp_fast", "fps": 60, "duration": 10},
    ]
    
    for exp in experiments:
        print(f"\n{'='*50}")
        print(f"実験: {exp['name']}")
        print(f"{'='*50}")
        
        automated_experiment(
            experiment_name=exp['name'],
            duration_sec=exp['duration'],
            fps=exp['fps']
        )
        
        # 実験間の休憩
        print("⏸️  5秒間休憩...")
        time.sleep(5)

# 実行
batch_experiments()
```

## :material-folder: データ構造

実験ハンドラーは以下の構造でデータを保存します：

```
experiment_001/
├── metadata.json              # 実験メタデータ
├── rakuda_obs_animation.gif   # 全体アニメーション
├── rakuda_arm_obs.png         # アーム動作プロット
├── arms_leader.blosc2         # Leaderアームデータ
├── arms_follower.blosc2       # Followerアームデータ
├── sensors_tactile_left.blosc2   # 左タクタイルデータ
├── sensors_tactile_right.blosc2  # 右タクタイルデータ
└── sensors_cameras_main.blosc2   # カメラデータ（存在する場合）
```

### メタデータ例

```json
{
  "experiment_name": "experiment_001",
  "timestamp": "2025-01-15T10:30:45",
  "duration_sec": 29.8,
  "fps": 30,
  "total_frames": 894,
  "robot_config": {
    "leader_port": "/dev/ttyUSB0",
    "follower_port": "/dev/ttyUSB1"
  },
  "sensors": {
    "tactile": ["left", "right"],
    "cameras": ["main"]
  }
}
```

## :material-alert-circle: 注意点と制限

### パフォーマンス考慮

- **高フレームレート**: 60Hz以上では処理能力に注意
- **センサー数**: 多数のセンサーは処理負荷を増加
- **記録時間**: 長時間記録はメモリ使用量に注意

### エラーハンドリング

```python
try:
    handler.recode_save(
        max_frames=1000,
        save_path="experiment_001",
        if_async=True
    )
except KeyboardInterrupt:
    print("⚠️  ユーザーによって中断されました")
except Exception as e:
    print(f"❌ 実験中にエラーが発生: {e}")
    # 部分的なデータの保存など
```

## :material-link-variant: 関連API

- [**RakudaExpHandler**](../api/utils.md#robopy.utils.exp_interface.rakuda_exp_handler.RakudaExpHandler) - 実験ハンドラークラス
- [**RakudaRobot**](../api/robots.md#robopy.robots.rakuda.rakuda_robot.RakudaRobot) - ベースロボットクラス
- [**BlocsHandler**](../api/utils.md#robopy.utils.blocs_handler.BlocsHandler) - データ保存クラス