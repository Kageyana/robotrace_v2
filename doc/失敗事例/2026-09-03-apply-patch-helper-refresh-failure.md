---
type: codex-failure
date: 2026-09-03
task: "マーカー誤検出調査用ログ追加"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - tooling
---

# apply_patchのWindowsサンドボックス更新失敗

## 要約

対象ソースの編集時にapply_patchが複数回、Windowsサンドボックスのhelper_unknown_errorで失敗した。

## 発生した状況

- タスク: マーカー誤検出調査用のログ列追加
- 実行環境: Windows Codex desktop、D:\\robotrace\\robotrace_v2
- 前提条件: ワークスペース書き込み権限あり

## 何を試したか

1. apply_patchを絶対パス・相対パスで再実行した。
2. exec_commandのrequire_escalatedで読み取りが成功することを確認した。
3. 固定アンカーを検証するPowerShell置換へ切り替えた。

## 結果・エラー

```text
apply_patch verification failed: fs sandbox helper failed with status exit code: 1
windows sandbox failed: helper_unknown_error: setup refresh had errors
```

## 原因

apply_patch内部のWindowsサンドボックス更新処理が一時的に失敗した。対象ファイルやアンカーの内容が原因ではなかった。

## 解決・回避策

固定アンカーの出現回数を1回と検証したうえで、require_escalatedのPowerShellからUTF-8（BOMなし）で置換した。変更後にgit diffとDebugビルドで確認した。

## 今後の予防策

apply_patchが同じhelper_unknown_errorを2回以上返した場合は、対象アンカーを読み取り確認してから、範囲を限定した置換へ切り替える。置換後は必ずgit diffとビルドを実行する。

## 関連

- 関連ノート:
- 参考リンク:
