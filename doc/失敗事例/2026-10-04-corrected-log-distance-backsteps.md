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

## 再発：2026-10-12の13089～13093確認

新規check_primary_05_13089_13093.pyで一次・二次へ一律にencTotalOptimalの単調性assertを適用し、13092のマーカー距離補正で停止した。既存対策は分類スクリプト内に留まり、新しい解析コードへ引き継がれなかった。

13092/13093は各5段、最大83.54/72.29mm戻るが、全てcourseMarker=3の行でありraw左右積算平均は単調。control.cのControl_ApplyMarkerCorrection_p()とcourseAnalysis.cの呼出しで、二次走行の補正距離が戻り得ることを再確認した。単調性・5mm記録間隔の健全性はraw左右積算平均へ切り替え、encTotalOptimalの単調性assertは一次に限定した。二次補正の戻り回数・最大量をmetrics.csvへ明示し、Yaw/XY比較にもraw距離を使用する。

再発防止を追加：新規解析のload境界で、列名だけでなく「raw積算／補正座標」「適用走行モード」を明示する。一次とDISTANCEを含む代表セットでloadを確認してから図を作成する。固定時間上限も流用しない。今回のスクリプトでは全8本（旧一次3本＋新5本）の読込・距離検証・全図生成・入力ハッシュ不変が完了し、validation.jsonのpassed=trueを確認したため、この再発への対処はresolved。

再実行：python analysis/script/check_primary_05_13089_13093.py。成果物：analysis/primary_05_13089_13093/。
