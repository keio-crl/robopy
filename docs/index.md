# Robopy Documentation

**Robopy**は、ロボット制御のためのPython interfaceです。rakuda と koch robotに対応し、カメラと触覚センサーを統合したデータ収集をサポートします。

## :material-robot: 主な特徴

- **🤖 多種ロボット対応**: RakudaRobot、KochRobotをサポート
- **📷 センサー統合**: カメラ、タクタイルセンサーの統一インターフェース 
- **🛠 シンプルな依存関係**: ROSなどのC/C++ベースのライブラリ必要なし  
- **⚡ 高性能データ収集**: 並列処理による高速データキャプチャ
- **🎬 可視化機能**: データの収集、整形機能

## :material-rocket-launch: クイックスタート

### インストール
<!---TODO --->

=== "uv（推奨）"

    ```bash
    # 基本パッケージのインストール
    uv add git+https://github.com/keio-crl/robopy.git --tag v0.1.0
    # RealSenseサポート（Linux）
    uv add pyrealsense2
    ```

=== "pip"

    ```bash
    # 基本インストール
    git clone https://github.com/keio-crl/robopy.git
    cd robopy
    pip install -e .

    # RealSenseサポート（Linux）
    pip install pyrealsense2
    ```

### 基本的な使用例

```python
from robopy.utils.exp_interface import RakudaExpHandler

# 実験ハンドラーの作成
handler = RakudaExpHandler(
    leader_port="/dev/ttyUSB0", # leader armのポート
    follower_port="/dev/ttyUSB1", # follower armのポート
    left_digit_serial="D20542", # left digit sensorのシリアル
    right_digit_serial="D20537",   # right digit sensorのシリアル
    fps=10 # データを収集するフレームレート (max 30)
)

# データ記録と保存
handler.recode_save(
    max_frames=150, # 収集するフレーム数
    save_path="experiment_001", # 保存先ディレクトリ: data/experiment_001/...
)
```

## :material-book-open: ドキュメント構成

<div class="grid cards" markdown>

-   :material-download: **始め方**

    ---

    インストールとセットアップガイド

    [:octicons-arrow-right-24: インストール](getting-started/installation.md)
    [:octicons-arrow-right-24: クイックスタート](getting-started/quickstart.md)

-   :material-robot: **ロボット**

    ---

    各ロボットシステムの詳細な使用方法

    [:octicons-arrow-right-24: Rakuda](robots/rakuda.md)
    [:octicons-arrow-right-24: Koch](robots/koch.md)

-   :material-camera: **センサー**

    ---

    センサーの設定と使用方法

    [:octicons-arrow-right-24: カメラ](sensors/cameras.md)
    [:octicons-arrow-right-24: タクタイル](sensors/tactile.md)

-   :material-flask: **実験**

    ---

    データ収集の実行方法

    [:octicons-arrow-right-24: 実験ハンドラー](experiments/handlers.md)
    [:octicons-arrow-right-24: データ収集](experiments/data-collection.md)

-   :material-api: **API リファレンス**

    ---

    詳細なAPI仕様

    [:octicons-arrow-right-24: ロボット](api/robots.md)
    [:octicons-arrow-right-24: センサー](api/sensors.md)

-   :material-devices: **開発者向け**

    ---

    開発ガイドと貢献方法

    [:octicons-arrow-right-24: 開発ガイド](developer/development.md)
</div>

## :material-help-circle: サポート

質問や問題がある場合は、[GitHub Issues](https://github.com/keio-crl/robopy/issues)でお知らせください。