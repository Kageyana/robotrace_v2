---
type: codex-failure
date: 2026-08-11
task: "ログ診断列を使った制御調整"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace/logging
  - scope-control
---

# 調整中に診断ログ列とprofileを増やし続けて解析条件を複雑化した

## 要約

横スリップ、PWM飽和、角速度cap、emcStop診断を切り分ける過程で、profile 1〜8相当のログ列入れ替えを繰り返した。各profileは目的を持っていたが、比較対象のスキーマが混在し、常用ログと一時診断ログの境界が複雑になった。

## 発生状況

- 36B内で targetSpeedBase、targetSpeedScale、targetAngularvelo、speedPwmReqL/R、speedPwmOutL/R、センサー内訳、停止診断列などを順番に入れ替えた。
- 調整完了後も不要な診断getter、ラッチ値、ログ用変数が残り、ユーザーから削除を指示された。

## 原因

診断目的の列を追加する判断はできていたが、開始時に「タスク終了時に戻す列」「残す列」「一時profileの終了条件」を明示していなかった。

## 解決

- 現行の通常ログを36B標準構成へ戻した。
- 一時診断コード、不要なログgetter、停止切り分け用APIを削除した。
- AGENTS.md に、2次走行までに必要なログを残し、その他の列はタスク終了後に戻す運用を追記した。

## 再発防止

診断を開始する前に、目的、必要列、同サイズでの置換対象、終了条件、復帰条件を記録する。採用パラメータに不要な診断コードをコミットしない。

## 関連

- AGENTS.md: D:/robotrace/robotrace_v2/AGENTS.md

