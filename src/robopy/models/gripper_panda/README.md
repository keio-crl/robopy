# Panda "longer finger" gripper meshes

The two finger meshes from the Franka Emika Panda gripper, as modified by the
CALVIN environment, used here to give the simulated Rakuda a hand it can close.

| パス | 内容 |
| --- | --- |
| `visual/longer_finger_v2.obj` | 視覚メッシュ（279 頂点、40 KB） |
| `collision/longer_finger_v2.obj` | 衝突メッシュ（96 頂点） |
| `*/longer_finger_v2.mtl` | OBJ が参照する材質ファイル。MuJoCo は読まないが、欠けていると他のツールが警告する |
| `LICENSE.txt` | Apache License 2.0（上流のまま） |

## 出典

`calvin_env/data/franka_panda/meshes/` の `longer_finger_v2.obj`
（[CALVIN](https://github.com/mees/calvin) の `calvin_env`、Apache License 2.0）。
`panda_longer_finger.urdf` が標準の Panda フィンガの代わりに使っているもので、
指を伸ばして細い物体を掴みやすくした改造版です。

**変更点**: ファイルは 1 バイトも変更していません。robopy 側は MJCF を生成するときに
参照するだけです（`robopy.sim.panda_gripper`）。

## これは Rakuda の実機グリッパでは**ありません**

Rakuda-2 の CAD エクスポートは、グリッパを `gripper_left_dof` / `gripper_right_dof` という
**形状を一切持たない fixed フレーム**として出力しています。実機には XM430-W350 で駆動される
グリッパが 1 腕 1 個ありますが、その形状データはこのリポジトリにありません。

そのため、ここで組み合わせているのは「Rakuda の腕 + Panda の指」という**実在しないロボット**です。

- シミュレータ内で把持タスクを組むには十分です。
- **実機転移は期待できません。** 指の形・ストローク・把持力のどれも実機と一致しません。
- さらに `gripper_*_dof` から実際の把持点までの TCP オフセットは
  `examples/config/rakuda_control.example.yaml` で明示的に**未測定**（`validated: false`）です。
  つまり「指をどこに付けるか」自体が測定値ではなく仮定です。

実機に合わせたいなら、まず実機グリッパの CAD と TCP オフセットを用意してください。
そのときは `robopy.sim.panda_gripper` を置き換える形になります。

## 寸法の注意

| | 寸法 | 質量 |
| --- | --- | --- |
| `longer_finger_v2` 1 本 | 21 × 26 × **96 mm** | 上流 URDF では 0.1 kg |
| Rakuda の手首リンク（比較） | 65 × 62 × **83 mm** | **0.126 kg** |

指 1 本が最終リンクより長く、上流の質量をそのまま使うと手が前腕より重くなります。
実際に使う値は `robopy.sim.panda_gripper` に書いてあります。
