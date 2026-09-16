# Rakuda-2 モデル（`assembly_2`、パッケージデータ）

`Rakuda-2_simulation_ready.zip`（Onshape エクスポート + 凸包後処理）を、アーカイブ内のレイアウトのまま
robopy のパッケージデータとして置いています。`package://assembly_2/...` は、このディレクトリ
（`robopy/models/rakuda`）を package dir として解決します。場所は `robopy.models.find_rakuda_model()` が返します。
元アーカイブの説明は `assembly_2/README_SIMULATION.md` を参照してください。

| パス | wheel | 内容 |
| --- | --- | --- |
| `assembly_2/urdf/assembly_2.urdf` | 同梱 | Onshape の元エクスポート（視覚メッシュのみ） |
| `assembly_2/urdf/assembly_2_convex_collision.urdf` | 同梱 | `<visual>`=視覚メッシュ、`<collision>`=凸包。**robopy の既定** |
| `assembly_2/urdf/assembly_2_mesh_collision.urdf` | 同梱 | 視覚メッシュを衝突形状にも流用（重い） |
| `assembly_2/collision_meshes/*.stl` | 同梱 | 凸包 137 個、2.9 MB |
| `assembly_2/meshes/*.stl` | **含まない**（Git LFS） | 視覚メッシュ 137 個、53 MB。`robopy-models fetch` で取得 |
| `assembly_2/package.xml`, `CMakeLists.txt`, `launch/` | 同梱 | ROS 1 パッケージのメタデータ（robopy は使わない） |

## 視覚メッシュ

wheel には入っていません（`pyproject.toml` の `wheel-exclude`）。インストール済みの環境では

```bash
robopy-models fetch        # ~/.cache/robopy/models/rakuda/assembly_2/meshes/ に取得
```

で GitHub の LFS 配信エンドポイントから取得します（非公開リポジトリなら `GITHUB_TOKEN`）。
チェックアウトでは `git lfs install && git lfs pull` でも同じです。取得しなくても、運動学・IK・衝突判定・
ビューア・VR はすべて動きます（ビューアは凸包を描画）。

## ライセンス

`assembly_2/package.xml` の `<license>` はアーカイブのまま `Proprietary` です。公開リポジトリで配布する
場合は、CAD データの権利者に合わせて修正してください。
