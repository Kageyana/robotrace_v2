---
type: codex-failure
date: 2026-09-26
task: "オートスタート方式設定のPCテスト"
status: resolved
severity: medium
tags:
  - codex/failure
  - robotrace
  - test
  - auto-run
---

# SLIP設定の前走条件で2走目を検証していなかった

## 要約

方式パーサーの初回テストで、2走目に `SLIP` を指定した設定が受理された。一次走行は `DISTANCE` ではないため、この設定は不正として修復対象にする必要がある。

## 発生した状況

- タスク: `auto_run.txt` の方式割当て・依存関係テスト
- 実行環境: Windows PC、GCCホストテスト
- 前提条件: `SLIP` は直前の設定方式が `DISTANCE` の場合だけ有効

## 何を試したか

1. `2,SLIP` を含む設定がパーサーで拒否されることを確認した。

## 結果・エラー

```text
FAIL: !autoRunParseConfig("2,SLIP\n3,DISTANCE\n4,PATH\n5,SHORTCUT", modes)
```

## 原因

依存条件のループ開始位置が配列スロット1で、2走目（スロット0）の直前方式である一次走行を検証していなかった。

## 解決・回避策

全スロットを検査し、スロット0の `SLIP` は無条件で拒否するよう修正した。ホストテストを再実行し、設定パーサーとCSV行解析テストが通ることを確認した。

## 今後の予防策

- 依存関係を先行スロット参照で検証する設定パーサーでは、先頭スロットに先行項目が存在しない場合を明示的に検査する。
- 受入テストに「2走目SLIP拒否」と「DISTANCE直後のSLIP受理」の両方を残す。

## 関連

- 関連ノート: なし
- 参考リンク: `robotrace_v2/tests/auto_run_test.c`
