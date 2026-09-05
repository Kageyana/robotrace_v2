---
type: codex-failure
date: 2026-08-22
task: "最新main書き込み後の実機ログ確認"
status: open
severity: medium
tags:
  - codex/failure
  - robotrace
  - firmware
---

# 最新main書き込み後もログのgitCommitが旧値

## 要約

最新mainを書き込んだと報告された実機ログで、ログヘッダの`gitCommit`が現行mainの最新コミットではなく、親コミットの`fe24705`のままだった。

## 発生した状況

- 対象ログ: `12118`〜`12127`
- ログヘッダ: `gitCommit=fe24705`, `branch=main`
- 現行main: `102a5f7`
- 実装を含むコミット: `635aa15`以降

## 何を試したか

1. ログのヘッダを読み取り、走行有効条件とgitCommitを確認した。
2. ログ作成時刻とソース側の現在HEADを比較した。

## 結果・エラー

走行ログは`emcStop=0`で有効だったが、現在mainの変更が書き込まれたことをgitCommitで確認できなかった。

## 原因

旧ビルド成果物を再利用した、別チェックアウトでビルドした、またはビルド後にmainが更新された可能性がある。確定していない。

## 解決・回避策

未解決。main HEAD確認後に`cmake --preset Debug`と`cmake --build --preset Debug`を実行し、生成ELFのコミット識別値と書き込み対象ファイルを確認してから再書き込みする。

## 今後の予防策

書き込み前に、ビルド出力の生成時刻、`gitCommit`、`branch`を確認し、実機ログのヘッダが期待コミットと一致しない場合は採用評価を開始しない。

## 関連

- 関連ノート:
- 参考リンク:
