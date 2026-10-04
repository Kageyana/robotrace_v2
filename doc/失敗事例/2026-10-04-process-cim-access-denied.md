---
type: codex-failure
date: 2026-10-04
task: 長時間分類プロセスの更新
status: resolved
severity: low
tags: [codex/failure, permission]
---

# プロセス照会用CIMがアクセス拒否になった

Get-CimInstance Win32_Processによる分類プロセスのPID確認がアクセス拒否となった。CIMの具体的な拒否設定は未調査。

exec_commandが返したsession_idにwrite_stdinでCtrl-Cを送信し、対象セッションだけが終了したことを確認した。スクリプトは入力を変更せず、保存済み解析キャッシュから再開できた。

今後、Codexから開始した長時間処理の中断には保存したsession_idを使う。全Pythonプロセスを名前だけで停止しない。CIM取得や追加権限を前提にした手順を組まない。
