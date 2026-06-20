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
=== "Default"

    ```python
        from robopy.config import RakudaConfig, RakudaSensorParams, TactileParams
        from robopy.utils import RakudaExpHandler


        # RealSenseカメラ設定（オプション）, 設定しない場合は自動的に name = "main"として1つのカメラが使用されます
        # tactileセンサーはデフォルトでは使用されない
        config=RakudaConfig(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyUSB1",
        )

        handler = RakudaExpHandler(
            rakuda_config=config,
            fps=10 # データを収集するフレームレート (max 20)
        )

        # データ記録と保存
        handler.record_save(
            max_frames=150, # 収集するフレーム数
            save_path="experiment_001", # 保存先ディレクトリ: data/experiment_001/...
        )
    ```
=== "Custom"
    ```python
        from robopy.config import RakudaConfig, RakudaSensorParams, TactileParams
        from robopy.utils import RakudaExpHandler

        config=RakudaConfig(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyUSB1",
            sensors=RakudaSensorParams(
                camera=[
                    CameraParams(name="main",width=640,height=480,fps=30),
                    ...
                ],
                tactile=[
                    TactileParams(serial_num="D20542", name="left_digit", fps=30),
                    TactileParams(serial_num="D20537", name="right_digit", fps=30),
                ],
            ),
        )

        handler = RakudaExpHandler(
            rakuda_config=config,
            fps=10 # データを収集するフレームレート (max 20)
        )

        # データ記録と保存
        handler.record_save(
            max_frames=150, # 収集するフレーム数
            save_path="experiment_001", # 保存先ディレクトリ: data/experiment_001/...
        )
    ```
ここで、cameraを宣言しない場合、デフォルトのRealsenseカメラ設定が使用されます。


### 実験の実行

```python
# インタラクティブな実験実行
handler.record_save(
    max_frames=1000,
    save_path="experiment_001",
    if_async=True
)
```

### 推論actionを1フレーム送信する

`send_action()` は、Rakuda followerのモーター順に並んだ17次元actionを1フレーム送信し、
送信後のarm・camera・tactile・audioを含む `RakudaObs` を返します。`fps` はpolicyの
フレームレート、`control_hz` はフレーム間を線形補間して送信する制御周波数です。

```python
handler = RakudaExpHandler(
    rakuda_config=config,
    metadata_config=metadata,
    fps=10,
    control_hz=100,
)

obs = handler.send_action(follower_action)  # follower_action.shape == (17,)
```

### データの保存とアニメーション生成

## :material-alert-circle: 注意点と制限
上記のコードで収集したデータは、指定された`save_path`に保存されます。保存されるデータには、ロボットの状態、センサーの読み取り値、カメラ画像などが含まれます。

### パフォーマンス考慮

- **高フレームレート**: 30Hz以上では処理能力に注意
- **センサー数**: 多数のセンサーは処理負荷を増加
- **記録時間**: 長時間記録はメモリ使用量に注意

### エラーハンドリング

```python
try:
    handler.record_save(
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
