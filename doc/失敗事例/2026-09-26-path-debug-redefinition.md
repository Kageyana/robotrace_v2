---
type: codex-failure
date: 2026-09-26
task: "PATH REPLAY / SHORTCUT main integration"
status: resolved
severity: low
tags:
  - codex/failure
---

# PATH実装のDebugビルドでローカル変数を再定義

## 要約

経路終端検証を追加したとき、同じ関数スコープで `previousPulse` を二度宣言し、Debugビルドが失敗した。

## 発生した状況

- タスク: PATH REPLAY / SHORTCUTの選択移植
- 実行環境: Windows、ARM GCC、CMake Debug preset
- 前提条件: 既存ログ終端処理へ一次経路閉路・距離検証処理を追加

## 何を試したか

1. `cmake --build --preset Debug` を実行した。
2. 同一スコープ内の2つ目の変数名を処理用途に合わせて変更し、Debug/Releaseを再ビルドした。

## 結果・エラー

```text
初回Debugビルド: previousPulse の再定義でコンパイル失敗
修正後Debugビルド: 成功
修正後Releaseビルド: 成功
```

## 原因

既存のパルス位置変数と、追加した二回目の走査用パルス位置変数に同じ名前を付けた。

## 解決・回避策

二回目の走査で使う変数を別名にし、両ビルドで再確認した。

## 今後の予防策

- 同一関数へ走査パスや検証段階を追加した後は、ビルド前に追加変数名が既存スコープと重ならないことを確認する。
- firmware変更の最終確認ではDebugとReleaseの両方をビルドする。

## 関連

- 関連ノート: なし
- 参考リンク: なし
