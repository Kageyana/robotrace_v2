---
type: codex-failure
date: 2026-08-23
task: "robotrace_v2 旧ショートカットコメント除去"
status: resolved
severity: high
tags:
  - codex/failure
  - c
  - regex
---

# 複数行正規表現が C 関数群まで削除した

## 要約

文字化けした6行のコメントだけを消す正規表現に dot-all と貪欲な `.*` を併用し、離れた区切り線から対象コメントまでの関数群も削除した。Debugリンクの未定義参照で検出し、直前の差分を逆適用して復旧した。

## 発生した状況

- 対象: `robotrace_v2/Core/Src/courseAnalysis.c`
- 意図: `setShortCutTarget` を含む古いコメント6行だけを削除
- 使用したパターン: `(?ms)` と `.*setShortCutTarget` を含む複数行正規表現

## 結果・エラー

Debugリンクで `getLogNumber`, `clearXYcie`, `calcROC`, `calcXYcie`, `calcXYcies`, `readLogDistance`, `asignVelocity` などが未定義になった。

## 原因

`(?s)` によって `.` が改行にも一致し、貪欲な `.*` が手前の区切り線から対象コメントまでを横断した。

## 解決・回避策

1. 適用直前に保存した unified diff を `git apply --check -R` 後に逆適用した。
2. 欠落関数が復元されたことを `rg` で確認した。
3. 対象行の出現数、前後の区切り線、次コメント名を検証し、行配列から厳密な6行だけを削除した。
4. Debug/Releaseビルドを再実行する。

## 今後の予防策

- ソース編集で dot-all と貪欲な `.*` を併用しない。
- 複数行コメントの機械削除は、対象出現数と直前・直後行を検証してから行番号ベースで行う。
- 小変更でも差分行数を確認し、リンクまで実行する。
- 編集スクリプトの検索APIは「全体で一意」「指定位置以後の最初」「行先頭一致」を使い分ける。今回、`f_close()` の全体一意検索と1行関数の行全体一致は差分作成前に停止したため、範囲検索と先頭一致へ修正した。

## 関連

- 関連ノート: [[2026-08-23-exec-helper-setup-refresh-error]]