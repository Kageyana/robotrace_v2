---
type: codex-failure
date: 2026-08-11
task: "バッテリー最大電圧 8.4 V の設定"
status: resolved
severity: low
tags:
  - codex/failure
  - robotrace
  - implementation-assumption
---

# バッテリー最大電圧指定をコード上限クリップとして過剰実装した

## 要約

ユーザーの「バッテリーの最大電圧を8.4ボルトに設定」という依頼に対し、AGENTS.md では既に LiPo 2S の満充電最大電圧 `8.4 V` が明記されていたにもかかわらず、コードへ `BATTERY_MAX_VOLTAGE_V` と `limitBatteryVoltage()` を追加して `batteryVoltage_V` を上限クリップする実装まで行った。ユーザーから「変更をもどしてください」と指示され、該当変更を戻した。

## 発生した状況

- タスク: バッテリー最大電圧 8.4 V の設定
- 実行環境: `D:\robotrace\robotrace_v2`, PowerShell, workspace-write sandbox
- 前提条件: AGENTS.md には「LiPo 2S 満充電時の最大電圧は `8.4 V`」が既に記載されていた

## 何を試したか

1. `battery.h` に `BATTERY_MAX_VOLTAGE_V 8.4f` を追加した。
2. `battery.c` に `limitBatteryVoltage()` を追加した。
3. `control.c` と `SDcard.c` の `batteryVoltage_V = AD2VOLTAGE(batteryAD)` を上限クリップ付きに変更した。
4. Debug ビルドを実行し、ビルド成功を確認した。
5. ユーザー指示に従って上記の 8.4 V クリップ関連変更を戻した。

## 結果・エラー

```
ユーザー: 変更をもどしてください
```

戻し後、`BATTERY_MAX_VOLTAGE_V` と `limitBatteryVoltage` が残っていないことを `rg` で確認した。

## 原因

依頼文の「設定」を、ドキュメント上の基準値確認ではなく、実行時の電圧値を 8.4 V でクリップするコード変更として解釈した。すでに AGENTS.md に 8.4 V が書かれている場合は、まず「コード上の上限処理が必要か」「AGENTS.md の明示で十分か」を確認すべきだった。

## 解決・回避策

該当変更を戻し、既存の `AD2VOLTAGE(batteryAD)` による電圧算出へ戻した。今後、既にドキュメントに存在する運用値を「設定」する依頼では、実装に入る前に現在の定義箇所と未定義箇所を報告し、コード挙動まで変更する必要があるか確認する。

## 今後の予防策

- 「最大値」「下限」「基準値」などの依頼では、まず AGENTS.md、関連 Skill、コード定数、ログヘッダのどこに既に定義があるかを確認する。
- 既に方針として明記済みの値をコードへ反映する場合は、上限クリップ、表示しきい値、ログ比較条件、走行禁止条件のどれを変えるのかを明示してから編集する。
- 物理量の最大値をランタイムでクリップすると、速度FFやログ比較の意味が変わるため、ユーザーが明示しない限り勝手に制御値へ適用しない。

## 関連

- 関連ノート:
- 参考リンク:
