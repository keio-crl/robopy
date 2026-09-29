# robopy — プロジェクト固有の注意（Rakuda 実機まわり）

コードから読み取れない、実機で痛い目を見て学んだことだけを書く。一般的な構成は `docs/robots/rakuda.md`。

## 校正値・可動域は「一本の経路」しかない

- モータの count → 関節角は `JointCalibration.count_to_rad`（`control.follower_joint_calibration` の
  `zero_count` / `direction` / `urdf_joint`）**だけ**。viewer の `--mirror-follower`、`robopy-vr --hardware`、
  `ServoLoop` は全部これを通る。関節の可動域も `ModelBundle` / `rakuda_control` の
  `limit_profile`（URDF ∩ `control.model.soft_limits_rad`、`joint_limit_overrides` は広げる）で共通。
- したがって「viewer では合うのに VR / 実機制御では違う」ときに、校正の読み方を疑って別実装を足さない。
  差が出るのは **判定の厳しさ**（viewer は表示するだけ、IK・サーボは範囲外を拒否する）か、
  経路の外にあるもの（ヘッドの neutral、開始姿勢、操作座標系）である。まずログの数値と
  config の数値を突き合わせて、どの層が拒否しているかを特定する。

## 可動域は「止まっている実機」が必ず満たす値にする

- `soft_limits_rad` は「測った travel − 2° マージン」で書かれる。実機はストッパに当たって休むとき
  travel をさらに 1° ほど超えて読める（例：右肘が伸び切りで 0.2439、travel 上限 0.227、soft 0.1921）。
  つまり **soft limit は静止した実機に破られる前提**で扱うこと。
- IK が「保持中（動かしていない）関節が範囲外」を理由に**全体を infeasible** にすると、片腕のせいで
  両腕とも一切動かなくなる。保持関節の範囲外は「限界に置いて解く」か「内向きにだけ動かす」で処理し、
  範囲外の警告は出しても解は返す。`HELD_JOINT_LIMIT_TOLERANCE_RAD` のような固定の許容値で
  ごまかすと、値が数 mrad 超えただけで再発する（今回 0.0518 > 0.05）。
- 校正を変えたら（zero, direction, travel, URDF 幅）、実機を脱力／静止させた状態で **制御に使う
  limit profile で全関節が範囲内か**を確かめる。viewer で見た目が合うことは確認にならない。

## direction / zero を触るときの整合

- direction を反転したら、`lower_limit_rad` / `upper_limit_rad`（測った travel）と対応する
  `soft_limits_rad` を**同時に**符号反転して入れ替える。zero を動かしたら両方を同じ量ずらす。
  URDF を広げた関節は URDF と MJCF も一緒に直す。config の `note` に何をしたか残す。
- 片方だけ直すと、URDF ∩ soft が空になったり、実機が届く範囲が禁止域になったりして、
  症状は「追従しない」「ゼロがずれて見える」として現れる（校正自体は正しいことが多い）。

## 名前の対応

- モータ名（`r_arm_el_yaw`, `r_arm_wr_roll`, `r_arm_sh_pitch2` …）と URDF 関節名
  （`elbow_pitch_right_dof`, `wrist_yaw_right_dof`, `elbow_yaw_right_dof` …）は**名前で対応しない**。
  対応は config の `urdf_joint` が唯一の正。ログ・報告では必ず両方を併記する。

## 実機の運用メモ

- FTDI の `latency_timer` が 16 ms のままだと 17 モータの一括読み出しが 50 ms 予算を超えて
  サーボがフォールトする。起動前に
  `echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer`。
- 変更後の確認は `robopy-vr --hardware --hardware-check`（トルクを入れずに構成だけ検証）。
