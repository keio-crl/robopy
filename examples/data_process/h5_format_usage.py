"""
サンプル：H5形式でのデータ保存と読み込みの使用例

このスクリプトは、RakudaSaveWorkerを使用してH5形式で階層的な
データを保存・読み込みする方法を示しています。
"""

import numpy as np

from robopy.utils import H5Handler


def example_hierarchical_save() -> None:
    """階層的なデータ構造をH5形式で保存する例"""

    # 階層的なデータ構造を作成
    hierarchical_data = {
        "camera": {
            "main": np.random.rand(100, 3, 480, 640).astype(np.float32),
            "side": np.random.rand(100, 3, 480, 640).astype(np.float32),
        },
        "tactile": {
            "left": np.random.rand(100, 3, 224, 224).astype(np.float32),
            "right": np.random.rand(100, 3, 224, 224).astype(np.float32),
        },
        "arm": {
            "leader": np.random.rand(100, 7).astype(np.float32),
            "follower": np.random.rand(100, 7).astype(np.float32),
        },
    }

    # H5形式で保存
    output_path = "output/rakuda_observations.h5"
    H5Handler.save_hierarchical(hierarchical_data, output_path, compress=True)
    print(f"✓ Saved hierarchical data to {output_path}")

    # 保存されたファイル情報を確認
    info = H5Handler.get_info(output_path)
    print("\nFile structure:")
    for key, value in info.items():
        print(f"  {key}: shape={value['shape']}, dtype={value['dtype']}")

    # データを読み込む
    loaded_data = H5Handler.load_hierarchical(output_path)
    print("✓ Loaded hierarchical data")
    camera_shapes = [data.shape for data in loaded_data["camera"].values()]
    print(f"  Camera shapes: {camera_shapes}")
    tactile_shapes = [data.shape for data in loaded_data["tactile"].values()]
    print(f"  Tactile shapes: {tactile_shapes}")
    leader_shape = loaded_data["arm"]["leader"].shape
    follower_shape = loaded_data["arm"]["follower"].shape
    print(f"  Arm shapes: {leader_shape}, {follower_shape}")


def example_single_array() -> None:
    """単一の配列をH5形式で保存・読み込みする例"""

    # 配列を作成
    data = np.random.rand(100, 7).astype(np.float32)

    # 保存
    output_path = "output/arm_data.h5"
    H5Handler.save_single_array(data, output_path, dataset_name="leader_arm")
    print(f"✓ Saved single array to {output_path}")

    # 読み込み
    loaded_array = H5Handler.load_single_array(output_path, dataset_name="leader_arm")
    print(f"✓ Loaded array with shape: {loaded_array.shape}")
    matches = np.allclose(data, loaded_array)
    print(f"  Data matches: {matches}")


# ========================================
# 例3: RakudaSaveWorkerでの使用
# ========================================


def example_rakuda_save_worker() -> None:
    """RakudaSaveWorkerでH5形式のデータ保存を使用する例"""

    # RakudaConfigを設定（実際の使用時）
    # config = RakudaConfig(...)
    # worker = RakudaSaveWorker(config, worker_num=4, fps=20)

    # obs = robot.record(max_frame=100, fps=20)
    # worker.save_all_obs(obs, save_path="data/experiment_01", save_gif=True)

    # 実行後、以下のファイルが生成される:
    # - data/experiment_01/rakuda_observations.h5  # 主要データ
    # - data/experiment_01/arm_obs.png             # 腕観測の可視化
    # - data/experiment_01/rakuda_obs_animation.gif # アニメーション（オプション）

    print("✓ Example RakudaSaveWorker usage (commented for demo)")
    print("  Main output: rakuda_observations.h5")
    print("  Structure:")
    print("    ├── camera/")
    print("    │   ├── main")
    print("    │   └── side")
    print("    ├── tactile/")
    print("    │   ├── left")
    print("    │   └── right")
    print("    └── arm/")
    print("        ├── leader")
    print("        └── follower")


# ========================================
# メイン実行
# ========================================


if __name__ == "__main__":
    import os

    # 出力ディレクトリを作成
    os.makedirs("output", exist_ok=True)

    print("=" * 60)
    print("H5Handler の使用例")
    print("=" * 60)

    print("\n【例1】階層的データ保存")
    print("-" * 60)
    example_hierarchical_save()

    print("\n【例2】単一配列の保存")
    print("-" * 60)
    example_single_array()

    print("\n【例3】RakudaSaveWorker統合")
    print("-" * 60)
    example_rakuda_save_worker()

    print("\n" + "=" * 60)
    print("✓ すべての例が完了しました")
    print("=" * 60)
