---
type: codex-failure
date: 2026-09-13
task: "距離カルマン融合後の一次走行ログ確認"
status: resolved
severity: high
tags:
  - codex/failure
  - robotrace
  - distance-kalman
  - covariance
---

# ログ12407で距離融合の無効更新が909回発生した

## 要約

一次走行ログ12407で`distanceKalman.invalidUpdateCount=909`となった。共分散の通常更新で、更新前共分散を全要素コピーする前に`priorCovariance[1][j]`を参照しており、未初期化配列の値が候補共分散へ混入する実装不具合が確認された。ログ12407は経路生成・性能比較へ使用しない。ログ12408もローカライゼーション喪失ログのため採用しない。

## 対処

- 速度観測の共分散更新をJoseph形式へ置き換えた。
- 更新前共分散を全要素退避し、Joseph形式の新共分散を別行列で算出する。
- 新共分散の対称性、全要素の有限性、対角要素の非負性、融合距離差分上限を検証してから状態を確定する。
- 融合出力のfloat値を整数変換前に検証し、非有限値または±535 pulse超過時は制限した生エンコーダ値へ戻す。
- `distanceKalman.outputGuardCount`を走行単位で保持し、終了ヘッダへ保存する。

## 再発防止

- 共分散更新は入力行列を完全に退避し、計算途中の同じ配列を更新前値として参照しない。
- Python合成モデルへJoseph形式、40秒連続更新、非有限入力、負の共分散、最大加速度、float→整数境界の回帰テストを追加する。
- 実機一次走行で`invalidUpdateCount=0`、`outputGuardCount=0`、ログ欠落なしを確認してから共分散パラメータを評価・調整する。

## ステータス

2026-09-14に修正後の一次走行ログ12439、12441、12444を確認した。3本とも
`emcStop=0`、`cntlog`欠落なし、`logSchemaVersion=6`、`logRecordSizeBytes=48`、
`distanceKalman.invalidUpdateCount=0`、`distanceKalman.outputGuardCount=0`、
`dbgOverflowFinal=0`、`logOverflowFinal=0`だった。合成回帰テスト15件とDebugビルドも
成功したため、Joseph形式更新の再発防止策は検証済みとして`resolved`とする。

ログヘッダの`gitCommit=f34d347`は修正をコミットする前のビルド情報だが、修正後に追加した
Version 6列と診断値が記録されている。今後は実機ログとソースを一意に対応させるため、
走行用ビルドを作る前に対象変更をコミットし、dirty状態もヘッダへ記録する。
