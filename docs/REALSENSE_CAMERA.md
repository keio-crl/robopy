# RealSense Camera Implementation for Robopy

この実装は、Intel RealSenseカメラをrobopyフレームワークで使用するためのクラスです。LeRobotの実装をベースに、robopyの既存アーキテクチャに合わせて簡素化し、**threadingによるメインスレッドをブロックしない処理**を重視して実装されています。

## 主な特徴

- **非ブロッキング処理**: バックグラウンドスレッドで継続的にフレームをキャプチャ
- **同期・非同期読み取り**: `get_observation()`（同期）と`async_read()`（非同期）の両方をサポート
- **深度データ対応**: カラーフレームと深度フレームの同時キャプチャ
- **robopy統合**: 既存のSensorインターフェースと完全互換
- **エラーハンドリング**: 接続エラーやタイムアウトの適切な処理

## インストール

Linux環境でpyrealsense2をインストール：

```bash
# UV環境の場合
uv add --group linux pyrealsense2

# または pip の場合
pip install pyrealsense2
```

## 基本的な使用方法

### 1. カメラの検出

```python
from robopy.sensors.visual.realsense_camera import RealsenseCamera
from robopy.config.visual_config.camera_config import RealsenseCameraConfig

# 利用可能なカメラを検出（find_realsense_cameras.py を使用）
```

### 2. カメラの初期化と接続

```python
# 設定を作成
config = RealsenseCameraConfig(
    fps=30,
    width=640,
    height=480,
    color_mode="rgb",  # または "bgr"
    is_depth_camera=True  # 深度データが必要な場合
)

# カメラインスタンスを作成
camera = RealsenseCamera(index=0, name="main_camera", config=config)

# 接続（バックグラウンドスレッドが開始される）
camera.connect()
```

### 3. フレームの読み取り

#### 同期読み取り（ブロッキング）
```python
# メインスレッドがブロックされる
frame = camera.get_observation(specific_color="rgb")
print(f"Frame shape: {frame.shape}")  # (C, H, W) 形式
```

#### 非同期読み取り（非ブロッキング）
```python
# メインスレッドはブロックされない
frame = camera.async_read(timeout_ms=200)
print(f"Async frame shape: {frame.shape}")  # (C, H, W) 形式
```

#### 深度データの読み取り
```python
# 同期
depth = camera.read_depth(timeout_ms=1000)

# 非同期
depth = camera.async_read_depth(timeout_ms=200)
print(f"Depth shape: {depth.shape}")  # (H, W) 形式、uint16（ミリメートル単位）
```

### 4. クリーンアップ

```python
# 必ずリソースを解放
camera.disconnect()
```

## 設定オプション

`RealsenseCameraConfig`で利用可能な設定：

```python
config = RealsenseCameraConfig(
    fps=30,                     # フレームレート
    width=640,                  # 画像幅
    height=480,                 # 画像高さ
    color_mode="rgb",           # カラーモード ("rgb" または "bgr")
    is_depth_camera=True,       # 深度データを有効にする
    auto_exposure=False,        # 自動露出
    exposure=190.0,             # 手動露出値
    auto_white_balance=False,   # 自動ホワイトバランス
    white_balance=3300.0,       # 手動ホワイトバランス値
    min_depth=100.0,           # 最小深度（mm）
    max_depth=2000.0           # 最大深度（mm）
)
```

## Threading設計

### バックグラウンドキャプチャ
- `connect()`時にバックグラウンドスレッドが開始
- 継続的にフレームをキャプチャし、最新フレームを保持
- メインスレッドをブロックしない

### Thread安全性
- `Lock`を使用してフレームデータの競合状態を防止
- `Event`を使用してフレーム到着を通知

### エラーハンドリング
- タイムアウト処理
- カメラ切断時の自動停止
- 例外の適切なログ出力

## 使用例

### デモ実行

```bash
python demo_realsense.py
```

### テスト実行

```bash
python test_realsense.py
```

### カメラ検出

```bash
python find_realsense_cameras.py
```

## 対応カメラ

Intel RealSenseシリーズ：
- D435/D435i
- D455
- D415
- L515
- その他のpyrealsense2対応デバイス

## パフォーマンス

- バックグラウンドキャプチャにより、メインスレッドの処理が中断されない
- `async_read()`は通常数ミリ秒で完了
- 100フレームごとにキャプチャ時間をログ出力（デバッグレベル）

## トラブルシューティング

### カメラが見つからない場合
```bash
python find_realsense_cameras.py
```
でデバイスを確認

### 権限エラー
```bash
# udevルールの設定が必要な場合があります
sudo apt install librealsense2-utils
```

### タイムアウトエラー
- USBケーブルの確認
- USB 3.0ポートの使用
- 他のアプリケーションでカメラが使用されていないか確認

## 実装詳細

### LeRobotとの違い
- robopyのSensorインターフェースに準拠
- 設定システムの統合
- CHW形式での画像出力
- エラーメッセージの簡素化

### 最適化ポイント
- フレームバッファリングなし（最新フレームのみ保持）
- 効率的なカラー変換
- 最小限のメモリコピー
