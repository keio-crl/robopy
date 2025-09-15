# インストール

## :material-check-circle: 前提条件

- Python 3.12以上
- Linux環境（RealSenseカメラ使用時）
- UV パッケージマネージャー（推奨）

## :material-download: インストール方法

### uv （推奨）

!!! tip "UV パッケージマネージャー"
    UVは高速なPythonパッケージマネージャーです。まだインストールしていない場合は、[公式ドキュメント](https://docs.astral.sh/uv/)を参照してください。

```bash
# 基本パッケージのインストール
uv add robopy

```

### pip環境

```bash
# 基本インストール
pip install robopy

# 開発環境
pip install robopy[dev]

# RealSenseサポート（Linux）
pip install pyrealsense2
```

## :material-package-variant: 依存関係

### 必須依存関係

| パッケージ | バージョン | 用途 |
|-----------|-----------|------|
| `blosc2` | >=3.8.0 | データ圧縮 |
| `digit-interface` | >=0.2.1 | タクタイルセンサー |
| `dynamixel-sdk` | >=3.7.31 | Dynamixelモーター制御 |
| `matplotlib` | >=3.10.6 | データ可視化 |
| `numpy` | >=2.2.6 | 数値計算 |
| `opencv-python` | >=4.12.0.88 | 画像処理 |
| `rich` | >=14.1.0 | コンソール出力 |

### オプション依存関係

| パッケージ | バージョン | 用途 | 対応OS |
|-----------|-----------|------|--------|
| `pyrealsense2` | >=2.54.2 | Intel RealSenseカメラ | Linux専用 |

### 開発依存関係

| パッケージ | バージョン | 用途 |
|-----------|-----------|------|
| `mypy` | >=1.17.1 | 型チェック |
| `pytest` | >=8.4.1 | テストフレームワーク |
| `ruff` | >=0.12.11 | リンター/フォーマッター |

### ドキュメント依存関係

| パッケージ | バージョン | 用途 |
|-----------|-----------|------|
| `mkdocs` | >=1.6.1 | ドキュメント生成 |
| `mkdocs-material` | >=9.5.0 | Material テーマ |
| `mkdocstrings[python]` | >=0.24.0 | API ドキュメント自動生成 |

### 依存関係の確認

=== "UV環境"

    ```bash
    # インストール済みパッケージの確認
    uv tree

    # 特定のパッケージの確認
    uv list | grep robopy
    ```

=== "pip環境"

    ```bash
    # インストール済みパッケージの確認
    pip list | grep robopy

    # 依存関係の確認
    pip show robopy
    ```

## :material-alert-circle: トラブルシューティング

### RealSenseカメラが認識されない

!!! warning "Linux専用"
    RealSenseカメラは現在Linux環境でのみサポートされています。

```bash
# udevルールの設定
sudo apt install librealsense2-utils

# デバイスの確認
python -c "from robopy.sensors.visual.realsense_camera import RealsenseCamera; RealsenseCamera.find_cameras()"
```

### Dynamixelモーターに接続できない

#### 1. ポート権限の確認

```bash
# ポート権限の設定
sudo chmod 666 /dev/ttyUSB*

# または、ユーザーをdialoutグループに追加
sudo usermod -a -G dialout $USER
# ※ログアウト・ログインが必要
```

#### 2. ポート設定の確認

```python
from robopy.motor.dynamixel_bus import DynamixelBus

# バス接続テスト
bus = DynamixelBus("/dev/ttyUSB0", 1000000)
motors = bus.scan_motors()
print(f"検出されたモーター: {motors}")
```

#### 3. 一般的なポート問題

```bash
# USBポートの確認
ls -la /dev/ttyUSB*

# dmesgでUSBデバイスのログ確認
dmesg | tail -20
```

### パッケージインストールエラー

#### UV環境での問題

```bash
# キャッシュクリア
uv cache clean

# 依存関係の再解決
uv lock --upgrade
uv sync
```

#### pip環境での問題

```bash
# キャッシュクリア
pip cache purge

# 強制再インストール
pip install --force-reinstall robopy
```

### Python バージョンの問題

```bash
# Pythonバージョンの確認
python --version

# 3.12以上が必要
python -c "import sys; print(sys.version_info >= (3, 12))"
```

## :material-lightbulb: 推奨セットアップ

### 開発環境

```bash
# 1. リポジトリクローン
git clone https://github.com/keio-crl/robopy.git
cd robopy

# 2. 全依存関係のインストール
uv sync --group dev --group docs --group linux

# 3. 環境確認
uv run python -c "import robopy; print('OK')"

# 4. テスト実行
uv run pytest

# 5. ドキュメント確認
uv run mkdocs serve
```

### 本番環境

```bash
# 最小限のインストール
uv add robopy

# 必要に応じてセンサー対応
uv sync --group linux  # RealSenseが必要な場合
```

## :material-arrow-right: 次のステップ

インストールが完了したら、[クイックスタート](quickstart.md)でRobopyの基本的な使用方法を学びましょう。