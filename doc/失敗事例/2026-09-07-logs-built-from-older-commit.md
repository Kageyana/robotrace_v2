---
type: codex-failure
date: 2026-09-07
task: "Version 12 Level 1走行ログと引き継ぎHEADの照合"
status: open
severity: medium
tags:
  - codex/failure
  - logging
  - firmware
  - flash
---

# 実機ログのビルドコミットが引き継ぎHEADと一致しない

## 要約

引き継ぎ時のHEADは `cf86014` だが、実機ログ12358～12362のヘッダには `gitCommit=5838f7d` が記録されていた。実機が引き継ぎHEADのELFで走行したことをログだけでは確認できないため、Version 12の採用判定に使用するログを保留する。

## 確認できた事実

- リポジトリHEAD: `cf86014`
- 対象ログ: 12358～12362
- 対象ログの `branch`: `codex/path-association-guard`
- 対象ログの `gitCommit`: `5838f7d`
- 12358～12362は `emcStop=0` だが、ビルドコミットが不一致である。

## 原因または未確定事項

5838f7dのELFが書き込まれたまま走行した、または別PC・別ビルドのログが同じフォルダへコピーされた可能性がある。書き込み対象ELF、ST-Link書き込み結果、ログコピー元は未確認である。

## 対処と次の調査

- 12358～12362はVersion 12の最終採用判定から一旦除外する。
- 現在のHEADからDebug/Releaseをビルドし、ELFの埋め込みコミットを確認する。
- ST-Link書き込み後、次のログヘッダが `gitCommit=cf86014` またはその後の採用コミットになることを確認する。
- ログ取得前に、実機表示またはログヘッダの `gitCommit`、`branch`、`buildDate` を確認する。

## 再発防止策

実機ログ解析の開始時に、対象ログの `gitCommit` と現在のHEADを比較し、不一致なら性能比較・採用判定を行わず、書き込み済みELFとログ取得元を確認する。

## 確認方法

`git rev-parse --short HEAD` とログヘッダの `gitCommit=` を照合し、書き込み後に新規ログで一致することを確認する。
