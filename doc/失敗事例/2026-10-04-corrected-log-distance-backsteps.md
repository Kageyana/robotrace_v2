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

## 再発：2026-10-17の13444～13448の記録間隔検査

対象タスクは横滑りなしのDISTANCE4走（13444・13445・13447・13448）の動画照合。既存ユーティリティ `analysis/script/check_fast_only_logs_13257_13297.py` のmetrics()は、encTotalOptimalの負差分を除外した残りで5mm記録間隔を検査していた。13444の小さな補正では差分が正のまま短縮され、初回prepareがassertで停止した。最終結果への出力前に停止しており、ファームウェアや原本の破損ではない。

過去対策は「負差分を正常補正として区別する」だけでは不十分だった。マーカー補正は正の差分も変える。4走のraw間隔は約2.53～7.56mmで、速度と1ms量子化に基づく既存許容幅内。補正座標の最大負差分は約35.64～52.62mmで、補正とrawの差の跳びはcourseMarkerの0→2/3遷移と一致した。

記録間隔の入力を `diff((encTotalL+encTotalR)/(2*PULSE_MILLIMETER))` に変更し、目標間隔は実ログのlogDistanceTargetMmから取得した。速度から導いた既存の許容幅は広げていない。補正座標の戻りは別指標として残した。解析Skillの入力境界で、正差分だけを選んだ補正距離も使用禁止と明示した。

確認は `analysis/script/finalize_video_slip_13444_13448.py`。13323・13326・13327・13438～13442・今回4本の計12本で読込・metrics・raw記録間隔検査が成功し、今回4本の原本CSV/動画ハッシュ不変とbin全CRC一致を確認した。結果は `analysis/video_slip_13444_13448/spacing_regression.json` とvalidation.json。解析入力の誤りへの対処はresolved。実機の横判定の不要ON原因はこの修正では確定していない。
