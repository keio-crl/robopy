# Utils API

このページでは、Robopyのユーティリティ機能に関連するAPIを説明します。

## :material-flask: 実験ハンドラー

### RakudaExpHandler

::: robopy.utils.exp_interface.rakuda_exp_handler.RakudaExpHandler
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - record_save
        - connect
        - disconnect

## :material-database: データ処理

### H5Handler

::: robopy.utils.h5_handler.H5Handler
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - load
        - save
        - get_info
        - load_hierarchical

### BlocsHandler

::: robopy.utils.blosc_handler.BLOSCHandler
    options:
      show_root_heading: true
      show_source: false

## :material-wrench: ユーティリティ関数

### find_usb_port

::: robopy.utils.find_usb_port
    options:
      show_root_heading: true
      show_source: false

## 使用例

### 実験データの記録と保存

```python
from robopy.config import RakudaConfig, RakudaSensorParams, TactileParams
from robopy.utils.exp_interface import RakudaExpHandler

# 設定
config = RakudaConfig(
    leader_port="/dev/ttyUSB0",
    follower_port="/dev/ttyUSB1",
    sensors=RakudaSensorParams(
        tactile=[
            TactileParams(serial_num="D20542", name="left"),
            TactileParams(serial_num="D20537", name="right"),
        ],
    ),
)

# ハンドラー作成
handler = RakudaExpHandler(
    rakuda_config=config,
    fps=20
)

# データ記録と保存
handler.record_save(
    max_frames=1000,
    save_path="experiment_001",
    if_async=True
)
```

### H5ファイルの読み込みと保存

```python
from robopy.utils import H5Handler

# H5ファイルの情報取得
file_info = H5Handler.get_info("path/to/data.h5")

# 階層的データの読み込み
hierarchical_data = H5Handler.load_hierarchical("path/to/data.h5")

# データの保存
import h5py
with h5py.File("output.h5", "w") as f:
    f.create_dataset("array_data", data=your_data)
```
