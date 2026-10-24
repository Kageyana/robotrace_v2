---
type: codex-failure
date: 2026-10-12
task: "横残差候補のIMU位置・座標確認"
status: resolved
severity: low
tags: [codex/failure, geometry, imu]
---

# 文書の旧STEP参照とKiCad Value検索の仮定を修正した

## 確認した結果

AGENTS.mdの参照`Machine/robotrace_v2 v86.step`が存在せず、読込に失敗した。`rg --files Machine`で存在する`robotrace_v3.STEP`を確認した。続いてBMI088をKiCadのValue文字列で検索したが一致しなかった。U8のライブラリは`robocon:BMI088`、Valueは`~`だった。

## 原因と対処

文書の参照が現在のチェックアウトと一致していなかった。部品のValueにセンサー型番が入るという仮定も誤っていた。実在するモデルを明示して解析し、KiCadのReference/ライブラリID/回転と照合した。CADでは後方31.675 mm、右0.099 mm。コード位置は変更していない。CADと実機の一致は未確認であり、調査手順の修復と実機一致の確認は別事項。

## 再発防止と確認

モデル読込前に`rg --files Machine`で存在確認し、欠落時は代替ファイル名・版・実機一致の未確認を明記する。部品検索はValueだけに依存せずReferenceとライブラリIDも使用する。AGENTS.mdとhardware-review Skillを同時更新した。出典SHA256と抽出値を`analysis/slip-lateral-candidate/geometry-check.json`へ保存し、U8座標と回転を照合済み。手順の修復はresolved、実機の位置・生X方向は別途未確認として継続する。
