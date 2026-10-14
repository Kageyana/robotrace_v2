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

## Offline SLIP speed-plan diagnosis

Regenerate the primary DISTANCE plan before every trial; do not reuse a previously SLIP-adjusted cache. Match saved speed/lead parameters. Separate base-plan lead, risk scaling, the SLIP-stage lead, forward/backward speed-difference limits and integer target conversion. A 5% increase before postprocessing does not imply the final plan is exactly 1.05 times the base. Label counterfactual lead/risk removal as offline sensitivity, not a proven lap-time gain. Corrected distance can step backward at markers; compare by plan index or validated event alignment.

## Slip detector SHADOW evaluation

For the 2026-10-12 v2 parallel evaluation, keep the legacy detector connected to control, next-run speed planning and PATH distance correction. Inspect the new validity, interval peaks, ON time and invalid reasons before interpreting flag frequency. The user restored the 72-column normal profile on 2026-10-14, so both normal72 and detailed87 record the diagnostics. Temporary normal43 logs retain legacy flags but do not record new detector output. Do not interpret missing diagnostics as zero. Use doc/slip-detector-v2.md and the log-analysis skill for units and schema compatibility. Do not tune the detector and speed profile in the same comparison. DWT measurements are explicitly excluded from this implementation request. Real-machine adoption still requires the first new-format log and at least 10 compatible runs; host tests or sparse-log replay do not establish physical slip accuracy.

For the lateral tangential candidate, use `doc/slip-lateral-candidate.md`. Keep thresholds and speed plans fixed while comparing the initial SHADOW lane and candidate in at least 10 same-mode, similar-voltage runs. Verify physical X direction before adoption. Separate steady turns/transients, CW/CCW and speed bands; compare residuals, interval peaks, ON time, validity, angular-acceleration dependence, detection position, completion, speed tracking and lap time. A lower ON rate alone does not establish more accurate slip detection. Existing IMU compensation and control remain connected to the initial paths; adoption is a separate change. Account for remaining angular-acceleration LPF lag.
