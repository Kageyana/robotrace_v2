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

1. Before starting a new trial, review reusable failures in `D:\ドキュメント\codex\失敗事例`. Check whether the candidate, voltage condition, log selection, or failure mode has already been rejected.
2. Identify the target mode and compare only compatible logs:
   - first runs with first runs
   - secondary runs with the same secondary mode
   - autoStart only against runs where all 5 runs completed
   - compare autoStart run numbers separately (`2` with `2`, `3` with `3`, `4` with `4`, `5` with `5`)
   - treat autoStart 1 as the first-run baseline, not as an equivalent secondary run
   - exclude an entire autoStart set when any of its five runs is incomplete or has `emcStop != 0`
   - do not use a lap-time average that mixes autoStart run numbers for adoption decisions
   - autoStart target speed is adaptive: later runs may be slower or faster because the previous run's slip is fed back into the next speed plan
3. Validate logs using the log-analysis skill.
4. Use only runs with a start/header `batteryVoltage_V >= 7.3 V` for tuning comparison and adoption decisions. This project-specific threshold overrides the generic 7.6 V default. A running `batteryVoltage_mV` drop below 7.3 V does not invalidate the run, but remains useful for diagnosing speed tracking, acceleration, PWM saturation, and slip.
5. Change at most two parameters per run.
6. Tune in this order:
   - PID
   - speed feedforward
   - slip detection
   - speed
7. When increasing speed, start from sections with smaller curvature radius.
8. After a control change, collect at least 10 real-run logs.
9. If a tested parameter or tuning constant is rejected and the only change is reverting it to the previous known-good value, do not require a new real-run confirmation just for the revert. Confirm the restored value by SD readback or code inspection.
10. Compare against logs from before the code or parameter change.
11. Adopt a change only when the goal time improves stably without reducing reproducibility.
12. When a trial is rejected or fails, record the reusable failure in Obsidian using the `record-failures` Skill. Include the changed values, compatible log range, objective facts, cause or uncertainty, workaround, and prevention step before starting the next trial.

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
