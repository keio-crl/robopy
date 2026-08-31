# 制御ループとカメラの高速化

Dynamixel のみで動く robot（Rakuda / Koch など）の制御ループと、
カメラのフレーム経路を速くするための、robopy 側の仕組みと設定のまとめです。

## :material-timer-outline: どこに時間がかかっているのか

まず「Python が遅いから遅い」わけでは**ない**、という点を押さえておくと判断を誤りません。
17 軸を 1 Mbps で sync read した場合の内訳はおおよそ次のようになります。

| 要因 | 実測/理論値 | 支配的か |
| --- | --- | --- |
| USB latency timer（FTDI, Linux 既定 16 ms） | 最大 16 ms / 往復 | **最大** |
| Return Delay Time（既定 250 = 500 µs × モータ数） | 17 軸で約 8.5 ms | **大** |
| ステータスパケットの実転送時間（15 byte × 17） | 約 2.6 ms | 中 |
| Python 側のパケット組み立て・解析 | 約 1.2 ms | 小 |

つまり効くのは、C++ 化そのものよりも **プロトコルと USB の使い方**です。
robopy はその順番どおりに手当てしています。

## :material-numeric-1-circle: USB latency timer を下げる（設定不要・既定で有効）

`DynamixelBus.open()` は既定で latency timer を 1 ms に下げます。
sysfs への書き込み権限がなければ、権限不要の `TIOCSSERIAL` / `ASYNC_LOW_LATENCY`
にフォールバックします。

```python
bus.open()                      # 既定: latency_timer_ms=1
bus.open(latency_timer_ms=None) # 触らない
```

恒久化する場合は udev ルールを入れてください。

```bash
echo 'ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"' \
  | sudo tee /etc/udev/rules.d/99-dynamixel-latency.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## :material-numeric-2-circle: Return Delay Time を 0 にする（opt-in）

工場出荷値の 250（= 500 µs）は、通常の sync read では**モータ台数ぶん直列に加算**されます。
17 軸なら約 8.5 ms。0 にしても USB ホストでは問題ありません。

```python
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    return_delay_time=0,   # 既定は None（書き込まない）
)
```

RETURN_DELAY_TIME は EEPROM 領域なので既定では書き込みません。
`return_delay_time` を指定した場合も、**既に目標値になっているモータは書き換えません**
（2 回目以降の connect は read だけで終わります）。

## :material-numeric-3-circle: C++ transport（`robopy_dxl`）を入れる

`native/robopy_dxl` は **C++ 版 DynamixelSDK** を pybind11 で包んだ optional package です。
インストールされていれば `DynamixelBus` が自動的に使います（`backend="auto"`）。

```bash
uv pip install ./native/robopy_dxl
```

Python の `dynamixel_sdk` に対する違いは次のとおりです。

| | Python `dynamixel_sdk` 3.7.x | `robopy_dxl` |
| --- | --- | --- |
| Fast Sync Read (`0x8A`) | 未実装 | 対応（非対応 firmware は自動フォールバック） |
| group オブジェクト | 呼び出しごとに再生成 | connect 時に 1 回だけ生成 |
| パケット解析 | Python の byte ループ | C++ |
| 転送中の GIL | 保持 | 解放 |

一番効くのは **Fast Sync Read** です。通常の sync read は N 台なら N 個のステータスパケットが
返り、その一つ一つの前に Return Delay が入りますが、Fast Sync Read は
**1 個のブロードキャストパケット**にまとまります。Python SDK（robopy が pin している 3.7.x）
にはこの命令の実装がないため、これは C++ 側でしか得られません。

!!! note "Fast Sync Read の安全性"
    `robopy_dxl` は SDK の `GroupFastSyncRead` をそのまま使わず、応答パケット内の
    ID 列が要求順どおりかを検証します。SDK 実装は検証しないため、応答しないモータが
    1 台あると以降の値が 1 つずつずれたまま「成功」として返ってしまいます。

## :material-numeric-4-circle: leader / follower を同時に読む

2 本のアームは別々の USB デバイスなので、直列に待つ理由がありません。
`RakudaPairSys.get_observation()` は内部で
[`sync_read_parallel`](../api/motor.md#robopy.motor.dynamixel_bus.sync_read_parallel) を使い、両方を同時に読みます。

```python
from robopy.motor.dynamixel_bus import sync_read_parallel
from robopy.motor.dynamixel_control_table import XControlTable

leader_pos, follower_pos = sync_read_parallel([
    (leader.motors, XControlTable.PRESENT_POSITION, leader_names),
    (follower.motors, XControlTable.PRESENT_POSITION, follower_names),
])
```

これが効くのは native transport のときだけです。Python SDK は `readPort` を
Python バイトコードで回すため転送中もほぼ GIL を握っており、スレッド化すると
むしろ遅くなります（実測で約 1.5 倍遅い）。そのため python backend では
`sync_read_parallel` は逐次実行にフォールバックします。

## :material-speedometer: ベンチマーク

`scripts/bench_dynamixel.py` は実機なしでも動きます（pty 上の protocol 2.0 シミュレータ）。

```bash
# シミュレータ: 17 軸、Return Delay 500 µs 相当
uv run python scripts/bench_dynamixel.py --motors 17 --return-delay-us 500

# 実機
uv run python scripts/bench_dynamixel.py --port /dev/ttyUSB0 --ids 1,2,3,4,5 \
    --second-port /dev/ttyUSB1
```

シミュレータでの実測（17 軸、Return Delay 500 µs 相当、ホスト側の負荷のみ。
実際の伝送時間と USB latency timer は含まれません）:

| | python backend | native backend |
| --- | --- | --- |
| `sync_read(PRESENT_POSITION)` | 15.3 ms | **0.97 ms** |
| read + write（1 制御ステップ） | 14.7 ms | **1.02 ms** |
| 2 バス逐次 | 27.1 ms | 1.81 ms |
| 2 バス並列 | 27.2 ms | **1.03 ms** |

!!! warning "この数字の読み方"
    シミュレータは実際の伝送時間も USB latency timer も再現しません。
    実機での改善幅は latency timer と Return Delay の現状値に強く依存します。
    自分の環境の数字は必ず `--port` 付きで測ってください。

## :material-cog-outline: RakudaConfig のまとめ

```python
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    motor_backend="auto",      # "auto" | "python" | "native"
    usb_latency_timer_ms=1,    # None で無効化
    return_delay_time=0,       # None で書き込まない（既定）
)
```

## :material-camera: RealSense / カメラ

**C++ で書き直しても速くなりません。** `pyrealsense2` は既に librealsense2（C++）の
pybind11 バインディングで、キャプチャ経路は最初から C++ です。Python 側にあるのは
`wait_for_frames()` と `get_data()` の薄い呼び出しだけです。

robopy のカメラ経路で実際にコストになっていたのは、C++ / Python の境界ではなく
**フレームの dtype** でした。センサが 8 bit で出したものを float32 に広げていたため、
情報は 1 bit も増えないのに CPU も容量も 4 倍かかっていました。

現在は **uint8 のまま** CHW で保持します（深度は `z16` のまま uint16 ミリメートル）。

| 640x480 の 1 フレーム | 変更前（float32） | 現在（uint8） |
| --- | --- | --- |
| 後処理の CPU | 2.74 ms | **0.32 ms** |
| バイト数 | 3,686,400 | **921,600** |

30 fps × カメラ 2 台なら、後処理の CPU が 164 ms/s から 19 ms/s になります。

### データフォーマットへの影響

これは**記録データの形式変更**です。

- `RakudaObs.sensors.cameras` などカメラ観測は `NDArray[np.uint8]`、値域 0-255。
- `RealsenseCamera.read_depth()` / `async_read_depth()` は `NDArray[np.uint16]`（mm）。
- `H5Handler` は dtype をそのまま保存・復元します（以前は何でも float32 に丸めていました）。
  BLOSC 側は元から dtype を保持しています。
- 触覚（DIGIT）と音声は今までどおり float32 です。

学習側で 0-1 正規化が必要なら、読み込んだ後に明示的に行ってください。
robopy 側でやらないのは、正規化の流儀（0-1 / ImageNet 統計 / …）が用途ごとに違ううえ、
記録して保存するだけの経路にまで一番重い表現を強制することになるからです。

```python
frames = H5Handler.load_hierarchical(path)["camera"]["main"]  # uint8 (N, C, H, W)
normalized = frames.astype(np.float32) / 255.0
```

!!! warning "既存の記録データについて"
    この変更以前に取った HDF5 / BLOSC は float32 のままです。ファイル側は
    そのまま読めますが、新旧を混ぜて学習する場合は dtype を見て分岐してください。

    ```python
    if frames.dtype == np.uint8:
        frames = frames.astype(np.float32) / 255.0
    ```

### 残っている改善余地

- 深度を使わないカメラでも `rs.align` を毎フレーム呼んでいる
- `wait_for_frames` のポーリングではなくフレームコールバックを使う
- DIGIT 触覚センサーも uint8 のフレームを float32 に広げている（同じ 4 倍の無駄。
  ただしこれも記録フォーマットの変更になるため、今回は変更していません）
