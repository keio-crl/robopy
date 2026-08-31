# Changelog

## [Unreleased]

### Changed
- **カメラフレームが uint8 になりました（記録データの形式変更）**。センサ出力が
  8 bit なので float32 に広げるのをやめました。640x480 の 1 フレームあたり後処理の
  CPU が 2.74ms → 0.32ms、バイト数が 3.7MB → 0.9MB になります。深度は uint16（mm）。
  触覚・音声・関節角は従来どおり float32 です。0-1 正規化が必要な場合は利用側で
  `frames.astype(np.float32) / 255.0` を行ってください。
- `H5Handler` が dtype を保持するようになりました（以前は保存・読み込みの両方で
  すべて float32 に変換していました）。この変更以前の記録は float32 のまま読めます。
- Dynamixel 通信を transport 層に分離し、`DynamixelBus(backend="auto"|"python"|"native")`
  で切り替えられるようにしました。sync read/write のグループは connect 時に 1 回だけ
  構築して再利用します。
- `DynamixelBus.open()` が既定で USB latency timer を 1ms に下げます
  （`latency_timer_ms=None` で無効化）。
- sync read で一部のモータが応答しなかった場合、短い配列を返さず例外を投げるように
  なりました。

### Added
- 任意の C++ transport `robopy_dxl`（`native/robopy_dxl`、pybind11 + C++ 版
  DynamixelSDK）。Fast Sync Read (`0x8A`)、GIL 解放、グループ再利用に対応。
  インストールされていれば自動的に使われます。
- `RakudaConfig` に `motor_backend` / `usb_latency_timer_ms` / `return_delay_time`。
- `sync_read_parallel()`: leader / follower を同時に読み取り。
- `DynamixelBus.set_return_delay_time()`（EEPROM を無駄に書かないよう、既に目標値の
  モータはスキップ）。
- `robopy.motor.port_tuning`: USB latency timer の調整。
- `scripts/bench_dynamixel.py`: 実機なし（pty 上の protocol 2.0 シミュレータ）でも
  動くベンチマーク。
- ハードウェア不要のテスト: `tests/support/fake_dynamixel.py`（Dynamixel デバイス
  シミュレータ）、`tests/test_dynamixel_bus.py`、`tests/test_rakuda_dynamixel_integration.py`、
  `tests/test_camera_frames.py`。


## [0.3.2] - 2026-01-30

### Added
- 音声センサー統合の追加


## [0.2.0] - 2025-09-30

### Added
- Robopyの初期リリース
- Rakudaロボット対応
- Kochロボット対応（開発中）
- Intel RealSenseカメラサポート
- DIGITタクタイルセンサーサポート
- H5形式データ保存機能
- 実験ハンドラー機能

### Features
- テレオペレーション機能
- 並列データ記録
- マルチセンサー統合


## [0.1.0] - 2025-09-18

### Initial Release
- 基本的なロボット制御機能
