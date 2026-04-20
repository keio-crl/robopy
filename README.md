# Robopy

[ENG](README_EN.md) | 日本語

**Robopy**は、ロボット制御のためのPython interfaceです。Rakuda と Koch robot（実装中）に対応し、カメラ・触覚・音声センサーを統合したデータ収集をサポートします。

## 🚀 Quick Start

### インストール

```bash
uv add git+https://github.com/keio-crl/robopy.git --tag v0.3.3
# RealSenseサポート（Linux）
uv add pyrealsense2
# Audioサポート
uv add "pyaudio>=0.2.14"
```

### 基本的な使用例
#### データ収集

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
#### データの読み込み

```python
from robopy.utils.h5_handler import H5Handler

if __name__ == "__main__":
    # H5ファイルのパス
    h5_file_path = "path/to/your/hierarchical_data.h5"

    # H5ファイルの情報を取得して表示
    file_info = H5Handler.get_info(h5_file_path)
    print("H5 File Information:")

    for key, value in file_info.items():
        print(f"  {key}: shape={value['shape']}, dtype={value['dtype']}")

    # 階層的なデータを読み込む
    hierarchical_data = H5Handler.load_hierarchical(h5_file_path)
    print("\nLoaded Hierarchical Data:")
    for category, datasets in hierarchical_data.items():
        for name, data in datasets.items():
            print(f"  {category}/{name}: shape={data.shape}, dtype={data.dtype}")

```

## 🤖 主な特徴

- **🔄 マルチロボット対応**: [`RakudaRobot`](src/robopy/robots/rakuda/rakuda_robot.py)、KochRobotをサポート
- **📷 センサー統合**: RealSenseカメラ、DIGIT触覚センサー、音声センサーの統一インターフェース
- **⚡ 高性能データ収集**: 並列処理による30Hz高速データキャプチャ
- **🎬 可視化機能**: データのアニメーション生成機能
- **🛠 シンプルな依存関係**: ROSなどのC/C++ベースのライブラリ不要

## 📋 基本的な使い方

### 1. 設定の作成

```python
from robopy import RakudaConfig, RakudaSensorParams, TactileParams

config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    # RealSenseカメラ設定（オプション）, 設定しない場合は自動的に name = "main"として1つのカメラが使用されます
    cameras=[
        CameraParams(name="main",width=640,height=480,fps=30),
        ...
    ],
    sensors=RakudaSensorParams(
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ],
    ),
)
```

### 2. ロボットの制御

```python
from robopy import RakudaRobot

robot = RakudaRobot(config)

try:
    robot.connect()

    # テレオペレーション（5秒間）
    robot.teleoperation(duration=5)

finally:
    robot.disconnect()
```

## 📊 記録データ構造

記録されるデータは[`RakudaObs`](src/robopy/config/robot_config/rakuda_config.py)型で以下の構造を持ちます：

```python
{
    "arms": {
        "leader": np.ndarray,    # (frames, 17) - 関節角度
        "follower": np.ndarray,  # (frames, 17) - 関節角度
    },
    "sensors": {
        "cameras": {
            "main": np.ndarray,  # (frames, C, H, W) - RGB画像
        },
        "tactile": {
            "left": np.ndarray,  # (frames, C, H, W) - 触覚データ
            "right": np.ndarray, # (frames, C, H, W) - 触覚データ
        },
        "audio": {
            "mic": np.ndarray,   # (frames, C, H, W) - 音声データ
        }
    }
}
```

## 📁 プロジェクト構成

```
robopy/
├── src/robopy/
│   ├── robots/          # ロボット制御クラス
│   │   ├── rakuda/      # Rakudaロボット
│   │   └── koch/        # Kochロボット
│   ├── sensors/         # センサー制御
│   │   ├── visual/      # カメラ
│   │   └── tactile/     # タクタイルセンサー
│   ├── config/          # 設定クラス
│   └── utils/           # ユーティリティ
│       ├── exp_interface/  # 実験インターフェース
│       └── worker/         # データ保存・処理
├── docs/                # ドキュメント
├── examples/            # サンプルコード
└── tests/              # テストコード
```

## 📚 ドキュメント

詳細なドキュメントは以下をご参照ください：

- [Documentation](https://keio-crl.github.io/robopy/)
