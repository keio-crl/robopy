# クイックスタート

このガイドでは、Robopyを使用してロボットを制御し、実験ハンドラーを利用してデータを収集する方法を説明します。


## :material-robot: Rakuda Robotの基本操作

## :material-rocket-launch: Quickstart

### インストール

=== "uv（推奨）"

    ```bash
    # 基本パッケージのインストール
    uv add git+https://github.com/keio-crl/robopy.git --tag v0.3.4

    # extras（project.optional-dependencies）: RealSenseサポート（現状Linuxのみ）
    uv add git+https://github.com/keio-crl/robopy.git --tag v0.3.4 --extra realsense

    # 複数extrasを同時に（--extra を複数回指定）
    uv add git+https://github.com/keio-crl/robopy.git --tag v0.3.2 --extra realsense --extra audio
    ```

=== "pip"

    ```bash
    # 基本インストール
    git clone https://github.com/keio-crl/robopy.git
    cd robopy
    pip install -e .

    # RealSenseサポート（オプション / 現状Linuxのみ）
    pip install -e ".[realsense]"  # または: pip install pyrealsense2

    # 複数extrasを同時に（カンマ区切り）
    pip install -e ".[realsense,audio]"
    ```

### 1. データ収集の基本
!!! tip "ポート設定"
    USBポートは環境によって異なります。`ls /dev/ttyUSB*`で確認してください。

=== "視覚のみ"

    ```python
        from robopy.config import RakudaConfig, RakudaSensorParams, TactileParams
        from robopy.utils import RakudaExpHandler



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
=== "触覚センサー付き"
    ```python
        from robopy.config import RakudaConfig, RakudaSensorParams, TactileParams
        from robopy.utils import RakudaExpHandler

        config=RakudaConfig(
            leader_port="/dev/ttyUSB0",
            follower_port="/dev/ttyUSB1",
            sensors=RakudaSensorParams(
                tactile=[
                    TactileParams(serial_num="D20542", name="left_digit", fps=30),
                    TactileParams(serial_num="D20537", name="right_digit", fps=30),
                ],
            ),
        )
        handler = RakudaExpHandler(
            rakuda_config=config,
            fps=10
        )
        # データ記録と保存
        handler.record_save(
            max_frames=150, # 収集するフレーム数
            save_path="experiment_001", # 保存先ディレクトリ: data/experiment_001/...
        )
    ```
詳しい使い方は[実験ハンドラーのドキュメント](../experiments/handlers.md)を参照してください。


### 2. ロボットの接続と制御

```python
from robopy import RakudaRobot

# ロボットインスタンスの作成
robot = RakudaRobot(config)

try:
    # 接続
    robot.connect()
    print("✅ ロボットに接続しました")

    print("🎮 テレオペレーションを開始します...")
    robot.teleoperation()

finally:
    # 切断
    robot.disconnect()
    print("🔌 ロボットから切断しました")
```


## :material-arrow-right: 次のステップ

- [**Rakudaロボットの詳細**](../robots/rakuda.md) - より高度な制御方法
- [**センサー設定**](../sensors/cameras.md) - カメラとタクタイルセンサーの設定
- [**API リファレンス**](../api/robots.md) - 全ての関数とクラスの詳細
