# Rakuda-2 モデル（`assembly_2`）

`Rakuda-2_simulation_ready.zip`（Onshape エクスポート + 凸包後処理）を、アーカイブ内のレイアウトのまま
置いています。`package://assembly_2/...` は、このディレクトリ（`models/rakuda`）を package dir として解決します。
元アーカイブの説明は `assembly_2/README_SIMULATION.md` を参照してください。

| パス | 管理 | 内容 |
| --- | --- | --- |
| `assembly_2/urdf/assembly_2.urdf` | git | Onshape の元エクスポート（視覚メッシュのみ） |
| `assembly_2/urdf/assembly_2_convex_collision.urdf` | git | `<visual>`=視覚メッシュ、`<collision>`=凸包。**robopy の既定** |
| `assembly_2/urdf/assembly_2_mesh_collision.urdf` | git | 視覚メッシュを衝突形状にも流用（重い） |
| `assembly_2/collision_meshes/*.stl` | git | 凸包 137 個、2.9 MB |
| `assembly_2/meshes/*.stl` | **Git LFS** | 視覚メッシュ 137 個、53 MB。**未追加**（下記） |
| `assembly_2/package.xml`, `CMakeLists.txt`, `launch/` | git | ROS 1 パッケージのメタデータ（robopy は使わない） |

## 視覚メッシュの追加（一度だけ、LFS を使えるマシンで）

リポジトリ直下の `.gitattributes` に `models/rakuda/assembly_2/meshes/*.stl` を LFS 管理にする規則が
あるので、ファイルを置いて `git add` すれば自動的に LFS ポインタとしてコミットされます。

```bash
git lfs install
unzip -j Rakuda-2_simulation_ready.zip 'assembly_2/meshes/*.stl' -d models/rakuda/assembly_2/meshes/
git add models/rakuda/assembly_2/meshes
git lfs ls-files | wc -l          # 137
git commit -m "models(rakuda): add visual meshes via Git LFS"
git push
```

追加後、他のクローンでは `git lfs install && git lfs pull` で取得できます。取得しない／未追加の
チェックアウトでも、運動学・IK・衝突判定・ビューアはすべて動きます（ビューアは凸包を描画）。

## ライセンス

`assembly_2/package.xml` の `<license>` はアーカイブのまま `Proprietary` です。公開リポジトリで配布する
場合は、CAD データの権利者に合わせて修正してください。
