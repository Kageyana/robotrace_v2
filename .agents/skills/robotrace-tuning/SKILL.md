---
name: robotrace-tuning
description: ロボトレースのPID、速度フィードフォワード、スリップ判定、速度・加速度パラメータを調整するときに使う。走行ログ比較、再現性判断、速度向上、変更後10本ログ確認を扱う。
---

# Robotrace Tuning

## Overview

Use this skill for parameter tuning and control-improvement decisions. Keep project-wide limits, units, parameter locations, and safety constraints in `AGENTS.md` as the source of truth.

## Priority

Optimize in this order:

1. Reproducibility
2. Finish rate
3. Lap time reduction

Do not adopt a change that improves finish rate or lap time but worsens reproducibility.

## Workflow

1. Identify the target mode and compare only compatible logs:
   - first runs with first runs
   - secondary runs with the same secondary mode
   - autoStart only against runs where all 5 runs completed
2. Validate logs using the log-analysis skill.
3. Change at most two parameters per run.
4. Tune in this order:
   - PID
   - speed feedforward
   - slip detection
   - speed
5. When increasing speed, start from sections with smaller curvature radius.
6. After a control change, collect at least 10 real-run logs.
7. Compare against logs from before the code or parameter change.
8. Adopt a change only when the goal time improves stably without reducing reproducibility.

## Allowed Risks

Accept only these risks for time reduction:

- mild running slip
- high current
- high voltage

If high-speed behavior gets worse:

- adjust PID parameters first
- lower speed parameters if needed

## Parameter Handling

- Do not rely on the log header to record parameter changes.
- Save comparison results under `analysis/`.
- Include comparison logs, changed values, adoption decision, and remaining issues.
- Confirm relevant parameter locations in `AGENTS.md` before editing code.

## Review Before Editing

For firmware or parameter changes, check:

- whether the 1 ms control period is affected
- whether log format remains compatible
- whether real-machine safety constraints remain satisfied
- whether generated logs can still support the comparison workflow
