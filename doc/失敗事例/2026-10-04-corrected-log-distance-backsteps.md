---
type: codex-failure
date: 2026-10-04
task: 全ログコース分類
status: resolved
severity: high
tags: [codex/failure, log-analysis, distance]
---

# 補正距離の戻りと過去ゴール回数を一律に異常扱いした

## 結果

全件分類の途中確認で12555本中7740本に距離逆行フラグが付いた。既存course_005の11373はoptimalTrace=2、emcStop=0で、encTotalOptimalが7回戻り、最大1段の戻りは5572 pulse。5000にも11回の戻りが記録されている。この段階の判定は最終結果として出力・適用していない。

また12363、12378はemcStop=0、sgMarkerAtLogEnd=2だが、最初のコードは3未満を異常としていた。

## 原因

encTotalOptimalはDISTANCE走行のマーカー位置補正を含む座標であり、生の走行積算値と同じ単調性を仮定できない。過去の規定ゴール回数は現在のコード定数から推測できない。

## 対処と再発防止

一次走行の逆行は異常扱いを維持する。一次走行以外のencTotalOptimalは総距離の10%以内の戻りのみ単調包絡で特徴量化し、戻り回数・最大補正量を出力する。この上限は自動分類の保守的な解析条件で、正常補正の物理上限を証明するものではない。モード欠落は完走保留を維持する。

sgMarkerAtLogEndへ固定の3回条件を適用しない。2未満は過去ゴール回数不明として保留にする。実際に存在する停止・ゴール情報で判定する。

今後は正常な既存一次/二次ログを使ったコース別終了状態の内訳を、全件結果出力前に確認する。test_classify_logs.pyに一次逆行と二次補正の区別を追加した。

## 確認方法

`python analysis/script/test_classify_logs.py`

全12555本の再走査後、11373、12363、12378のNORMAL_CANDIDATEを確認した。course_005の既存824本はすべて正常候補、分類照合も824/824一致。修正の回帰テストも通過した。
