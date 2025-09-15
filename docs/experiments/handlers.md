# 実験ハンドラー

Robopyの実験ハンドラーは、ロボットの制御とデータ収集を統合し、データ収集プロセスを簡素化するための高レベルインターフェースです。

## :material-flask: RakudaExpHandler

[`RakudaExpHandler`](../api/utils.md#robopy.utils.exp_interface.rakuda_exp_handler.RakudaExpHandler)は、Rakudaロボットでの実験を簡単に実行するためのハイレベルインターフェースです。

### 特徴

- **統合制御**: ロボット、センサーの一括管理
- **インタラクティブ操作**: 対話的な実験実行
- **自動保存**: データとメタデータの自動保存
- **アニメーション生成**: 結果の自動可視化

## :material-cog: 基本的な使用方法

### ハンドラーの作成
Rakudaロボット用の基本的な実験ハンドラーの作成方法：

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
ここで、cameraを宣言しない場合、デフォルトのRealsenseカメラ設定が使用されます。


### 実験の実行

```python
# インタラクティブな実験実行
handler.recode_save(
    max_frames=1000,
    save_path="experiment_001",
    if_async=True
)
```
### データの保存とアニメーション生成

## :material-alert-circle: 注意点と制限
上記のコードで収集したデータは、指定された`save_path`に保存されます。保存されるデータには、ロボットの状態、センサーの読み取り値、カメラ画像などが含まれます。

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