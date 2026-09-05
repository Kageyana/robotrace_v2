---
type: codex-failure
date: 2026-08-11
task: "mainからPIDライン速度調整ブランチへ電圧指令化をマージ"
status: resolved
severity: medium
tags:
  - codex/failure
  - git
  - integration
  - robotrace
---

# 電圧指令化のマージで制御・ログ・モーター実装が競合した

## 要約
`main`の電圧指令化を`codex/pid-line-speed-tuning`へマージした際、`log_schema.h`、`PIDcontrol.c`、`motor.c`で競合が発生した。旧PWM出力経路と電圧指令経路を機械的に選択すると、制御調整側の変更またはログ列を失う可能性があった。

## 発生した状況
- マージ対象: `main`のモーター電圧指令化。
- 受け側: `codex/pid-line-speed-tuning`。
- 競合ファイル: `robotrace_v2/Core/Inc/log_schema.h`、`robotrace_v2/Core/Src/PIDcontrol.c`、`robotrace_v2/Core/Src/motor.c`。

## 何を試したか
1. `main`を受け側ブランチへマージした。
2. ログ列、PID/FF経路、モーター出力経路をそれぞれ確認して競合を解消した。
3. DebugとReleaseのCMake configure/buildを実行した。

## 結果・エラー

```text
CONFLICT: log_schema.h
CONFLICT: PIDcontrol.c
CONFLICT: motor.c
```

解消後、Debug/Releaseともビルド成功。

## 原因
電圧指令化とPIDライン速度調整が同じ制御・ログ境界を変更しており、ブランチ間で出力API、ログスキーマ、PID経路が分岐していた。

## 解決・回避策
ログスキーマは新しい電圧列と既存デバッグ列を統合し、PIDは電圧FF経路を維持し、モーター出力は電圧指令APIを採用した。マージ後に両ビルドを確認した。

## 今後の予防策
- 制御API、ログスキーマ、設定ファイル形式を変更するブランチは、長期間分岐させず早めに統合する。
- 競合解消後は`motorPwmOut*`の残存、ログ先頭6列、`motorpwm`と指令電圧の意味を確認する。
- マージ後はDebugとReleaseを両方ビルドし、実ログのヘッダと列数を確認する。

## 関連

- マージコミット: `3d0e4d6`
- 電圧指令化コミット: `fe24705`
