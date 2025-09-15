# ユーティリティAPI

このページでは、Robopyのユーティリティ機能に関連するAPIを説明します。

## :material-flask: 実験インターフェース

### RakudaExpHandler

::: robopy.utils.exp_interface.rakuda_exp_handler.RakudaExpHandler
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - recode_save

## :material-animation: アニメーション作成

### visualize_rakuda_obs

::: robopy.utils.animation_maker.visualize_rakuda_obs
    options:
      show_root_heading: true
      show_source: false

### visualize_arm_obs

::: robopy.utils.animation_maker.visualize_rakuda_obs
    options:
      show_root_heading: true
      show_source: false

## :material-database: データハンドリング

### BlocsHandler

::: robopy.utils.blosc_handler.BLOSCHandler
    options:
      show_root_heading: true
      show_source: false
      members:
        - __init__
        - save
        - load
        - compress_data
        - decompress_data

## :material-usb: USB ポート検出

### find_usb_port

::: robopy.utils.find_usb_port.find_port
    options:
      show_root_heading: true
      show_source: false


## 使用例

### 実験ハンドラーの使用

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

# インタラクティブな記録セッション
handler.recode_save(
    max_frames=1000,
    save_path="experiment_001",
    if_async=True
)
```

### アニメーション生成

```python
from robopy.utils.animation_maker import visualize_rakuda_obs, visualize_arm_obs
import numpy as np

# 記録データからアニメーション生成
obs = {
    "arms": {
        "leader": np.random.random((100, 17)),
        "follower": np.random.random((100, 17))
    },
    "sensors": {
        "cameras": {
            "main": np.random.randint(0, 255, (100, 480, 640, 3), dtype=np.uint8)
        },
        "tactile": {
            "left": np.random.randint(0, 255, (100, 240, 320, 3), dtype=np.uint8),
            "right": np.random.randint(0, 255, (100, 240, 320, 3), dtype=np.uint8)
        }
    }
}

# 全体のアニメーション
visualize_rakuda_obs(
    obs=obs,
    save_dir="./animations",
    fps=30
)

# アーム動作のみのプロット
visualize_arm_obs(
    arm_obs=obs["arms"],
    save_path="./arm_motion.png"
)
```

### データの保存と読み込み

```python
from robopy.utils.blocs_handler import BlocsHandler

# データハンドラーの作成
handler = BlocsHandler("experiment_data")

# データの保存
data = {
    "arms": obs["arms"],
    "sensors": obs["sensors"],
    "metadata": {
        "fps": 30,
        "duration": 33.3,
        "robot_type": "rakuda"
    }
}

handler.save(data)
print(f"データを {handler.save_path} に保存しました")

# データの読み込み
loaded_data = handler.load()
print(f"アームデータサイズ: {loaded_data['arms']['leader'].shape}")
```

### USB ポートの検出

```python
from robopy.utils.find_usb_port import find_usb_port, get_available_ports

# 利用可能なシリアルポートを取得
ports = get_available_ports()
print(f"利用可能なポート: {ports}")

# 特定のデバイスを検索
usb_port = find_usb_port(
    vendor_id=0x0403,  # FTDI
    product_id=0x6014  # FT232H
)

if usb_port:
    print(f"対象デバイスが {usb_port} で見つかりました")
else:
    print("対象デバイスが見つかりませんでした")
```

### データ圧縮

```python
from robopy.utils.blocs_handler import BlocsHandler
import numpy as np

# 大きなデータの作成
large_data = np.random.random((1000, 1000, 3))

# 圧縮保存
handler = BlocsHandler("large_experiment")
compressed_data = handler.compress_data(large_data)
print(f"圧縮率: {len(compressed_data) / large_data.nbytes:.2%}")

# 展開
decompressed_data = handler.decompress_data(compressed_data)
print(f"データが正確に復元されました: {np.allclose(large_data, decompressed_data)}")
```

### カスタムアニメーション

```python
import matplotlib.pyplot as plt
import matplotlib.animation as animation

def create_custom_animation(arm_data, save_path="custom_animation.gif"):
    """カスタムアニメーションの作成例"""
    
    fig, ax = plt.subplots(figsize=(10, 6))
    
    def animate(frame):
        ax.clear()
        ax.plot(arm_data["leader"][frame], label="Leader", marker='o')
        ax.plot(arm_data["follower"][frame], label="Follower", marker='s')
        ax.set_title(f"フレーム {frame}")
        ax.set_ylabel("関節角度 [rad]")
        ax.set_xlabel("関節番号")
        ax.legend()
        ax.grid(True)
    
    # アニメーションの作成
    anim = animation.FuncAnimation(
        fig, animate, frames=len(arm_data["leader"]), 
        interval=100, blit=False
    )
    
    # 保存
    anim.save(save_path, writer='pillow', fps=10)
    plt.close()
    print(f"カスタムアニメーションを {save_path} に保存しました")

# 使用例
create_custom_animation(obs["arms"])
```

### 実験データの管理

```python
from pathlib import Path
import json
from datetime import datetime

class ExperimentManager:
    """実験データの管理クラス"""
    
    def __init__(self, base_dir="experiments"):
        self.base_dir = Path(base_dir)
        self.base_dir.mkdir(exist_ok=True)
    
    def create_experiment(self, name=None):
        """新しい実験フォルダを作成"""
        if name is None:
            name = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        exp_dir = self.base_dir / name
        exp_dir.mkdir(exist_ok=True)
        
        # メタデータファイルの作成
        metadata = {
            "name": name,
            "created_at": datetime.now().isoformat(),
            "status": "created"
        }
        
        with open(exp_dir / "metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)
        
        return exp_dir
    
    def list_experiments(self):
        """実験一覧を取得"""
        experiments = []
        for exp_dir in self.base_dir.iterdir():
            if exp_dir.is_dir():
                metadata_file = exp_dir / "metadata.json"
                if metadata_file.exists():
                    with open(metadata_file, "r") as f:
                        metadata = json.load(f)
                    experiments.append(metadata)
        return experiments

# 使用例
manager = ExperimentManager()
exp_dir = manager.create_experiment("rakuda_test_001")
print(f"実験フォルダを作成: {exp_dir}")

# 実験一覧の表示
experiments = manager.list_experiments()
for exp in experiments:
    print(f"実験: {exp['name']} (作成日時: {exp['created_at']})")
```