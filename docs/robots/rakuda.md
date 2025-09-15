# Rakuda Robot

[`RakudaRobot`](../api/robots.md#robopy.robots.rakuda.rakuda_robot.RakudaRobot)は、二腕ロボットシステムでLeader/Followerアーム構成を持つロボットです。テレオペレーションとデータ収集機能を提供します。

## :material-robot-outline: 構成要素

### ロボットアーム

- **Leader Arm** ([`RakudaLeader`](../api/robots.md#robopy.robots.rakuda.rakuda_leader.RakudaLeader)): 操作者が制御するアーム
- **Follower Arm** ([`RakudaFollower`](../api/robots.md#robopy.robots.rakuda.rakuda_follower.RakudaFollower)): Leaderの動きを追従するアーム
- **Pair System** ([`RakudaPairSys`](../api/robots.md#robopy.robots.rakuda.rakuda_pair_sys.RakudaPairSys)): 両アームの協調制御

### センサー統合

- **カメラ**: Intel RealSenseカメラ対応
- **タクタイルセンサー**: DIGIT触覚センサー対応

## :material-cog: 基本的な使用方法

### 設定の作成

```python
from robopy import RakudaConfig, RakudaSensorParams, TactileParams

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
    slow_mode=False,  # 高速モード
)
```

### ロボットの操作

```python
from robopy import RakudaRobot

robot = RakudaRobot(config)

try:
    # 接続
    robot.connect()
    
    # テレオペレーション（10秒間）
    robot.teleoperation(duration=10)
    
    # データ記録
    obs = robot.record_parallel(max_frame=500, fps=30)
    
finally:
    robot.disconnect()
```

## :material-database: データ記録機能

### 標準記録

```python
# シーケンシャル記録（低fps推奨）
obs = robot.record(max_frame=100, fps=5)
```

### 並列記録

```python
# 並列記録（高fps対応）
obs = robot.record_parallel(
    max_frame=1000, 
    fps=30,
    max_processing_time_ms=25.0
)
```

### 記録データ構造

記録されるデータは[`RakudaObs`](../api/config.md#robopy.config.robot_config.rakuda_config.RakudaObs)型で、以下の構造を持ちます：

```python
{
    "arms": {
        "leader": np.ndarray,    # (frames, 17) - 関節角度など
        "follower": np.ndarray,  # (frames, 17) - 関節角度など
    },
    "sensors": {
        "cameras": {
            "main": np.ndarray,  # (frames, H, W, C) - RGB画像
        },
        "tactile": {
            "left": np.ndarray,  # (frames, H, W, C) - 触覚データ
            "right": np.ndarray, # (frames, H, W, C) - 触覚データ
        }
    }
}
```

## :material-flask: 実験ハンドラー

[`RakudaExpHandler`](../api/utils.md#robopy.utils.exp_interface.rakuda_exp_handler.RakudaExpHandler)を使用すると、より簡単に実験を実行できます：

```python
from robopy.utils.exp_interface import RakudaExpHandler

handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    left_digit_serial="D20542",
    right_digit_serial="D20537",
    fps=30
)

# インタラクティブな記録・保存
handler.recode_save(
    max_frames=1000,
    save_path="experiment_001",
    if_async=True
)
```

## :material-speedometer: パフォーマンス最適化

### 並列処理

`record_parallel()`メソッドでは、アーム制御とセンサー読み取りを並列化して高速化しています：

```python
# 最大25msの処理時間制限で30Hz記録
obs = robot.record_parallel(
    max_frame=1000,
    fps=30,
    max_processing_time_ms=25.0
)
```

### メモリ効率

- フレームバッファリングなし
- 最新フレームのみ保持
- 効率的な numpy 配列変換

## :material-alert-circle: トラブルシューティング

### 接続エラー

```python
try:
    robot.connect()
except RuntimeError as e:
    print(f"接続失敗: {e}")
    # ポートやボーレート設定を確認
```

### センサー読み取りエラー

```python
# センサーの個別確認
if robot.sensors.tactile:
    for sensor in robot.sensors.tactile:
        if sensor.is_connected:
            print(f"{sensor.name}: 接続OK")
```

### パフォーマンス問題

```python
# フレームスキップ確認
obs = robot.record_parallel(max_frame=100, fps=30)
# ログでスキップされたフレーム数を確認
```

## :material-link-variant: 関連クラス

- [`RakudaConfig`](../api/config.md#robopy.config.robot_config.rakuda_config.RakudaConfig) - ロボット設定
- [`RakudaPairSys`](../api/robots.md#robopy.robots.rakuda.rakuda_pair_sys.RakudaPairSys) - アーム協調制御
- [`RakudaLeader`](../api/robots.md#robopy.robots.rakuda.rakuda_leader.RakudaLeader) - リーダーアーム
- [`RakudaFollower`](../api/robots.md#robopy.robots.rakuda.rakuda_follower.RakudaFollower) - フォロワーアーム