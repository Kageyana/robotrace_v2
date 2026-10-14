# 失敗事例とSkill再発防止の対応（2026-10-15）

既存142件（ファームウェア118、ログビューワー13、共通保存先11）の本文・追記を確認した。失敗記録のstatusや過去の定数は変更していない。これは手順の反映確認であり、未解決の実機原因の解消を示さない。

7つの既存Skillを更新し、ログビューワー用Skillを1つ追加した。既存のユーザー差分を保持し、パス/権限/依存/完了判定を実行前に確認する形へ反映した。

## 反映先

- C: [共通事前確認](skill-execution-preflight.md)。全ロボトレースSkillの冒頭から参照。
- A: [ログ解析](../.agents/skills/robotrace-log-analysis/references/failure-prevention.md)。入力契約・単位・比較・集計。
- T: [チューニング](../.agents/skills/robotrace-tuning/references/failure-prevention.md)。試行設計・既知不採用・呼出状態。
- S: [SD設定](../.agents/skills/robotrace-sd-settings/references/failure-prevention.md)。設定・復旧・形式・RAM。
- H: [ハード確認](../.agents/skills/robotrace-hardware-review/references/failure-prevention.md)。CAD・軸・初期化・周期欠測。
- F: [書込み・デバッグ](../.agents/skills/robotrace-flash-debug/references/failure-prevention.md)。成果物・ハーネス・接続順序。
- K: [skill-creator](C:/Users/Natsuki/.codex/skills/.system/skill-creator/references/windows-skill-maintenance.md)。保護パス・ドル記号・PyYAML・UTF-8。
- R: [record-failures](C:/Users/Natsuki/.codex/skills/record-failures/SKILL.md)。保存先・再発記録・Skill同時更新。
- V: [ログビューワー](F:/Dropbox/Document/robotrace/Log/log/.agents/skills/robotrace-log-viewer/SKILL.md)。13件を専用手順へ統合。
- N: M5Stack/Core2固有の7件。対象となる既存Skillがなく、今回のロボトレースSkill実行の失敗ではないため読了のみ。DTR/RTS等の未検証仮説を無関係なSkillへ一般化せず、記録を維持。

Gitのネットワーク制限とOAuth workflow権限不足は異なる原因として共通手順へ記載した。今回push・認証変更・Git履歴更新は行っていない。

## 全件対応表

| 保存先 | 事例 | 反映先 |
|---|---|---|
| firmware | [2026-08-11-ambiguous-commit-message-amend-target.md](失敗事例/2026-08-11-ambiguous-commit-message-amend-target.md) | C |
| firmware | [2026-08-11-battery-max-voltage-overimplementation.md](失敗事例/2026-08-11-battery-max-voltage-overimplementation.md) | T |
| firmware | [2026-08-11-diagnostic-log-schema-churn.md](失敗事例/2026-08-11-diagnostic-log-schema-churn.md) | T, S |
| firmware | [2026-08-11-fixed-column-second-log-parser.md](失敗事例/2026-08-11-fixed-column-second-log-parser.md) | A, S |
| firmware | [2026-08-11-git-index-lock-sandbox-permission.md](失敗事例/2026-08-11-git-index-lock-sandbox-permission.md) | C |
| firmware | [2026-08-11-index-local-tuning-overfit.md](失敗事例/2026-08-11-index-local-tuning-overfit.md) | T |
| firmware | [2026-08-11-line-trace-rms-tolerance-misaligned.md](失敗事例/2026-08-11-line-trace-rms-tolerance-misaligned.md) | T |
| firmware | [2026-08-11-log-column-order-course-parser.md](失敗事例/2026-08-11-log-column-order-course-parser.md) | A, S |
| firmware | [2026-08-11-log-comparison-mismatched-speed-profile.md](失敗事例/2026-08-11-log-comparison-mismatched-speed-profile.md) | A |
| firmware | [2026-08-11-log-range-selection-regex-omitted-files.md](失敗事例/2026-08-11-log-range-selection-regex-omitted-files.md) | A |
| firmware | [2026-08-11-log-record-size-affected-control.md](失敗事例/2026-08-11-log-record-size-affected-control.md) | T, S |
| firmware | [2026-08-11-low-voltage-logs-mixed-with-adoption-comparison.md](失敗事例/2026-08-11-low-voltage-logs-mixed-with-adoption-comparison.md) | A |
| firmware | [2026-08-11-mixed-controller-gains-confounded-log-comparison.md](失敗事例/2026-08-11-mixed-controller-gains-confounded-log-comparison.md) | A |
| firmware | [2026-08-11-motor-voltage-merge-conflicts.md](失敗事例/2026-08-11-motor-voltage-merge-conflicts.md) | C |
| firmware | [2026-08-11-powershell-dollar-skill-prompt-expansion.md](失敗事例/2026-08-11-powershell-dollar-skill-prompt-expansion.md) | C, K |
| firmware | [2026-08-11-pwm-saturation-correlation-misinterpretation.md](失敗事例/2026-08-11-pwm-saturation-correlation-misinterpretation.md) | A |
| firmware | [2026-08-11-revert-confirmation-overtested.md](失敗事例/2026-08-11-revert-confirmation-overtested.md) | T |
| firmware | [2026-08-11-scope-creep-naming-normalization.md](失敗事例/2026-08-11-scope-creep-naming-normalization.md) | C |
| firmware | [2026-08-11-skill-validator-missing-pyyaml.md](失敗事例/2026-08-11-skill-validator-missing-pyyaml.md) | K |
| firmware | [2026-08-11-unnecessary-emergency-stop-debug-api.md](失敗事例/2026-08-11-unnecessary-emergency-stop-debug-api.md) | T |
| firmware | [2026-08-11-windows-rg-glob-not-expanded.md](失敗事例/2026-08-11-windows-rg-glob-not-expanded.md) | C |
| firmware | [2026-08-16-autostart-11966-11970-emcstop3.md](失敗事例/2026-08-16-autostart-11966-11970-emcstop3.md) | T |
| firmware | [2026-08-16-powershell-python-quote-error.md](失敗事例/2026-08-16-powershell-python-quote-error.md) | C |
| firmware | [2026-08-16-slip-expand1-040-no-stable-improvement.md](失敗事例/2026-08-16-slip-expand1-040-no-stable-improvement.md) | T |
| firmware | [2026-08-16-slip-expand2-020-no-stable-improvement.md](失敗事例/2026-08-16-slip-expand2-020-no-stable-improvement.md) | T |
| firmware | [2026-08-16-slip-lat-extra025-no-stable-improvement.md](失敗事例/2026-08-16-slip-lat-extra025-no-stable-improvement.md) | T |
| firmware | [2026-08-16-slip-lat-risk015-no-stable-improvement.md](失敗事例/2026-08-16-slip-lat-risk015-no-stable-improvement.md) | T |
| firmware | [2026-08-16-slip-long-risk030-no-stable-improvement.md](失敗事例/2026-08-16-slip-long-risk030-no-stable-improvement.md) | T |
| firmware | [2026-08-16-speed-ff160-no-stable-improvement.md](失敗事例/2026-08-16-speed-ff160-no-stable-improvement.md) | T |
| firmware | [2026-08-16-theoretical-lap-test-missing-math-import.md](失敗事例/2026-08-16-theoretical-lap-test-missing-math-import.md) | F |
| firmware | [2026-08-19-git-index-lock-permission-denied.md](失敗事例/2026-08-19-git-index-lock-permission-denied.md) | C |
| firmware | [2026-08-22-main-flash-old-gitcommit.md](失敗事例/2026-08-22-main-flash-old-gitcommit.md) | F |
| firmware | [2026-08-22-shell-helper-setup-refresh-error.md](失敗事例/2026-08-22-shell-helper-setup-refresh-error.md) | C |
| firmware | [2026-08-23-exec-helper-setup-refresh-error.md](失敗事例/2026-08-23-exec-helper-setup-refresh-error.md) | C |
| firmware | [2026-08-23-multiline-regex-deleted-course-functions.md](失敗事例/2026-08-23-multiline-regex-deleted-course-functions.md) | C |
| firmware | [2026-08-23-path-replay-autostart5-encoder-stop.md](失敗事例/2026-08-23-path-replay-autostart5-encoder-stop.md) | T |
| firmware | [2026-08-23-path-replay-kheading400-worse.md](失敗事例/2026-08-23-path-replay-kheading400-worse.md) | T |
| firmware | [2026-08-23-path-replay-localization-false-stop.md](失敗事例/2026-08-23-path-replay-localization-false-stop.md) | T |
| firmware | [2026-08-23-path-replay-log-11-with-valid-csv.md](失敗事例/2026-08-23-path-replay-log-11-with-valid-csv.md) | S |
| firmware | [2026-08-23-path-replay-yaw-oscillation-hairpin.md](失敗事例/2026-08-23-path-replay-yaw-oscillation-hairpin.md) | T |
| firmware | [2026-08-23-shortcut-gate-requires-explicit-approval.md](失敗事例/2026-08-23-shortcut-gate-requires-explicit-approval.md) | T |
| firmware | [2026-08-23-stlink-erase-needs-under-reset-100khz.md](失敗事例/2026-08-23-stlink-erase-needs-under-reset-100khz.md) | F |
| firmware | [2026-09-02-path-gain-analysis-matplotlib-missing.md](失敗事例/2026-09-02-path-gain-analysis-matplotlib-missing.md) | A |
| firmware | [2026-09-03-apply-patch-helper-refresh-failure.md](失敗事例/2026-09-03-apply-patch-helper-refresh-failure.md) | C |
| firmware | [2026-09-03-path-csv-line-truncation-error-7.md](失敗事例/2026-09-03-path-csv-line-truncation-error-7.md) | A, S |
| firmware | [2026-09-03-path-replay-false-goal-localization.md](失敗事例/2026-09-03-path-replay-false-goal-localization.md) | S |
| firmware | [2026-09-03-stlink-target-not-found-marker-debug.md](失敗事例/2026-09-03-stlink-target-not-found-marker-debug.md) | F |
| firmware | [2026-09-06-git-merge-orig-head-lock-permission.md](失敗事例/2026-09-06-git-merge-orig-head-lock-permission.md) | C |
| firmware | [2026-09-06-powershell-foreach-pipeline-parser.md](失敗事例/2026-09-06-powershell-foreach-pipeline-parser.md) | C |
| firmware | [2026-09-26-auto-run-slip-first-slot-validation.md](失敗事例/2026-09-26-auto-run-slip-first-slot-validation.md) | S |
| firmware | [2026-09-26-autostart-primary-route-endpoint.md](失敗事例/2026-09-26-autostart-primary-route-endpoint.md) | S |
| firmware | [2026-09-26-autostart-slip-analysis-stall.md](失敗事例/2026-09-26-autostart-slip-analysis-stall.md) | S |
| firmware | [2026-09-26-git-index-permission-recurrence.md](失敗事例/2026-09-26-git-index-permission-recurrence.md) | C |
| firmware | [2026-09-26-path-debug-redefinition.md](失敗事例/2026-09-26-path-debug-redefinition.md) | F |
| firmware | [2026-09-26-path-pc-test-toolchain.md](失敗事例/2026-09-26-path-pc-test-toolchain.md) | F |
| firmware | [2026-09-28-debug-bench-fixture-generation-working-directory.md](失敗事例/2026-09-28-debug-bench-fixture-generation-working-directory.md) | C, F |
| firmware | [2026-09-28-stlink-target-not-found-status-check.md](失敗事例/2026-09-28-stlink-target-not-found-status-check.md) | F |
| firmware | [2026-09-29-debug-bench-csv-validation-false-negative.md](失敗事例/2026-09-29-debug-bench-csv-validation-false-negative.md) | S, F |
| firmware | [2026-09-29-debug-bench-gdb-server-and-breakpoints.md](失敗事例/2026-09-29-debug-bench-gdb-server-and-breakpoints.md) | F |
| firmware | [2026-09-29-debug-bench-hardware-breakpoint-limit.md](失敗事例/2026-09-29-debug-bench-hardware-breakpoint-limit.md) | F |
| firmware | [2026-09-29-debug-bench-iwdg-initialization-order.md](失敗事例/2026-09-29-debug-bench-iwdg-initialization-order.md) | F |
| firmware | [2026-09-29-debug-bench-no-completion-usb-error.md](失敗事例/2026-09-29-debug-bench-no-completion-usb-error.md) | S, F |
| firmware | [2026-09-29-debug-bench-path-fixture-route-offset.md](失敗事例/2026-09-29-debug-bench-path-fixture-route-offset.md) | F |
| firmware | [2026-09-29-debug-bench-pause-reset.md](失敗事例/2026-09-29-debug-bench-pause-reset.md) | F |
| firmware | [2026-09-29-debug-bench-reset-cause-before-hal-init.md](失敗事例/2026-09-29-debug-bench-reset-cause-before-hal-init.md) | F |
| firmware | [2026-09-29-gdb-function-call-sd-init-hang.md](失敗事例/2026-09-29-gdb-function-call-sd-init-hang.md) | F |
| firmware | [2026-09-29-git-index-stage-permission-denied.md](失敗事例/2026-09-29-git-index-stage-permission-denied.md) | C |
| firmware | [2026-09-29-log-padding-bench-sd-init-failure.md](失敗事例/2026-09-29-log-padding-bench-sd-init-failure.md) | S, F |
| firmware | [2026-09-29-skill-validator-missing-pyyaml.md](失敗事例/2026-09-29-skill-validator-missing-pyyaml.md) | K |
| firmware | [2026-09-30-apply-patch-mojibake-context.md](失敗事例/2026-09-30-apply-patch-mojibake-context.md) | C |
| firmware | [2026-10-03-analysis-mm-ms-speed-conversion.md](失敗事例/2026-10-03-analysis-mm-ms-speed-conversion.md) | A |
| firmware | [2026-10-03-bmi088-initialization-spec-mismatch.md](失敗事例/2026-10-03-bmi088-initialization-spec-mismatch.md) | H |
| firmware | [2026-10-03-cross-fit-insufficient-channels.md](失敗事例/2026-10-03-cross-fit-insufficient-channels.md) | A |
| firmware | [2026-10-03-distance-log-fixed-time-gap-assumption.md](失敗事例/2026-10-03-distance-log-fixed-time-gap-assumption.md) | A |
| firmware | [2026-10-03-git-metadata-readonly-sandbox.md](失敗事例/2026-10-03-git-metadata-readonly-sandbox.md) | C |
| firmware | [2026-10-03-log-batch-normal-end-precondition.md](失敗事例/2026-10-03-log-batch-normal-end-precondition.md) | A |
| firmware | [2026-10-03-log-time-u16-wrap-distance-validation.md](失敗事例/2026-10-03-log-time-u16-wrap-distance-validation.md) | A |
| firmware | [2026-10-03-protected-skill-file-write.md](失敗事例/2026-10-03-protected-skill-file-write.md) | C, K |
| firmware | [2026-10-03-sparse-yaw-course-distortion.md](失敗事例/2026-10-03-sparse-yaw-course-distortion.md) | A |
| firmware | [2026-10-03-yi-analysis-geometry-render-numerics.md](失敗事例/2026-10-03-yi-analysis-geometry-render-numerics.md) | A, H |
| firmware | [2026-10-03-yi-cross-anchor-phase-confounding.md](失敗事例/2026-10-03-yi-cross-anchor-phase-confounding.md) | A, H |
| firmware | [2026-10-04-classification-corrupt-csv.md](失敗事例/2026-10-04-classification-corrupt-csv.md) | A |
| firmware | [2026-10-04-corrected-log-distance-backsteps.md](失敗事例/2026-10-04-corrected-log-distance-backsteps.md) | A |
| firmware | [2026-10-04-cubeclt-gdb-python-unavailable.md](失敗事例/2026-10-04-cubeclt-gdb-python-unavailable.md) | F |
| firmware | [2026-10-04-deferred-csv-stack-reservation.md](失敗事例/2026-10-04-deferred-csv-stack-reservation.md) | S, F |
| firmware | [2026-10-04-fatfs-log-reader-header-cycle.md](失敗事例/2026-10-04-fatfs-log-reader-header-cycle.md) | S, F |
| firmware | [2026-10-04-float-timing-readback-stlink-usb-error.md](失敗事例/2026-10-04-float-timing-readback-stlink-usb-error.md) | F |
| firmware | [2026-10-04-fpu-audit-log-encoding.md](失敗事例/2026-10-04-fpu-audit-log-encoding.md) | C, F |
| firmware | [2026-10-04-git-index-sandbox-write.md](失敗事例/2026-10-04-git-index-sandbox-write.md) | C |
| firmware | [2026-10-04-isr-timing-flash-stlink-usb-error.md](失敗事例/2026-10-04-isr-timing-flash-stlink-usb-error.md) | F |
| firmware | [2026-10-04-no-sd-line-calibration-start-block.md](失敗事例/2026-10-04-no-sd-line-calibration-start-block.md) | S |
| firmware | [2026-10-04-noise-analysis-input-preflight.md](失敗事例/2026-10-04-noise-analysis-input-preflight.md) | A |
| firmware | [2026-10-04-process-cim-access-denied.md](失敗事例/2026-10-04-process-cim-access-denied.md) | C |
| firmware | [2026-10-04-shared-ram-host-harness-dependencies.md](失敗事例/2026-10-04-shared-ram-host-harness-dependencies.md) | S, F |
| firmware | [2026-10-04-slip-parity-cached-plan-precondition.md](失敗事例/2026-10-04-slip-parity-cached-plan-precondition.md) | S |
| firmware | [2026-10-04-start-calibration-yaw-pid-not-updated.md](失敗事例/2026-10-04-start-calibration-yaw-pid-not-updated.md) | T |
| firmware | [2026-10-04-stlink-compare-sections-false-mismatch.md](失敗事例/2026-10-04-stlink-compare-sections-false-mismatch.md) | F |
| firmware | [2026-10-11-bmi088-forward-y-sign.md](失敗事例/2026-10-11-bmi088-forward-y-sign.md) | H |
| firmware | [2026-10-11-sd-pending-csv-recovery.md](失敗事例/2026-10-11-sd-pending-csv-recovery.md) | S |
| firmware | [2026-10-11-yaw-fusion-reference-and-convergence.md](失敗事例/2026-10-11-yaw-fusion-reference-and-convergence.md) | A |
| firmware | [2026-10-12-csv-long-column-name-truncation.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-12-csv-long-column-name-truncation.md) | A, S |
| firmware | [2026-10-12-fatfs-exfat-uninitialized-fragment.md](失敗事例/2026-10-12-fatfs-exfat-uninitialized-fragment.md) | S |
| firmware | [2026-10-12-geometry-source-preflight.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-12-geometry-source-preflight.md) | H |
| firmware | [2026-10-12-host-float-snapshot-comparison.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-12-host-float-snapshot-comparison.md) | F |
| firmware | [2026-10-12-sd-csv-recovery-disk-full.md](失敗事例/2026-10-12-sd-csv-recovery-disk-full.md) | S |
| firmware | [2026-10-12-slip-current-readiness-assumption.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-12-slip-current-readiness-assumption.md) | T |
| firmware | [2026-10-12-slip-lateral-tangential-contamination.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-12-slip-lateral-tangential-contamination.md) | T, H |
| firmware | [2026-10-13-adc-host-fixture-contract.md](失敗事例/2026-10-13-adc-host-fixture-contract.md) | H, F |
| firmware | [2026-10-13-line-sensor-periodic-skip-starvation.md](失敗事例/2026-10-13-line-sensor-periodic-skip-starvation.md) | H |
| firmware | [2026-10-13-stlink-adc-debug-observation.md](失敗事例/2026-10-13-stlink-adc-debug-observation.md) | F |
| firmware | [2026-10-14-empty-analysis-stratum.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-14-empty-analysis-stratum.md) | A |
| firmware | [2026-10-14-line-omega-low-gain-curve-understeer.md](失敗事例/2026-10-14-line-omega-low-gain-curve-understeer.md) | T |
| firmware | [2026-10-14-log-corrected-distance-monotonic-assumption.md](失敗事例/2026-10-14-log-corrected-distance-monotonic-assumption.md) | A |
| firmware | [2026-10-14-log-discovery-nonnumeric-csv.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-14-log-discovery-nonnumeric-csv.md) | A |
| firmware | [2026-10-14-log-metadata-integer-decimal-format.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-14-log-metadata-integer-decimal-format.md) | A |
| firmware | [2026-10-14-right-marker-goal-assumption.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-14-right-marker-goal-assumption.md) | A |
| firmware | [2026-10-14-slip-postprocessing-gain-assumption.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-14-slip-postprocessing-gain-assumption.md) | T |
| firmware | [2026-10-14-straight-wobble-selection-bias.md](https://github.com/Kageyana/robotrace_v2/blob/e9903ca/doc/%E5%A4%B1%E6%95%97%E4%BA%8B%E4%BE%8B/2026-10-14-straight-wobble-selection-bias.md) | A |
| viewer | [2026-10-01-log-viewer-git-dubious-ownership.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-01-log-viewer-git-dubious-ownership.md) | V |
| viewer | [2026-10-01-matlab-uiaxes-currentpoint-readonly.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-01-matlab-uiaxes-currentpoint-readonly.md) | V |
| viewer | [2026-10-01-matlab-viewer-validation-types.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-01-matlab-viewer-validation-types.md) | V |
| viewer | [2026-10-02-matlab-empty-header-textscan.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-02-matlab-empty-header-textscan.md) | V |
| viewer | [2026-10-02-matlab-equation-fixture-and-import-options.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-02-matlab-equation-fixture-and-import-options.md) | V |
| viewer | [2026-10-02-matlab-settings-test-ui-and-fixture.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-02-matlab-settings-test-ui-and-fixture.md) | V |
| viewer | [2026-10-02-matlab-viewer-test-state-isolation.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-02-matlab-viewer-test-state-isolation.md) | V |
| viewer | [2026-10-02-python-viewer-source-encoding.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-02-python-viewer-source-encoding.md) | V |
| viewer | [2026-10-03-mlapp-code-newline-normalization.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-03-mlapp-code-newline-normalization.md) | V |
| viewer | [2026-10-03-mlapp-new-axes-design-properties.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-03-mlapp-new-axes-design-properties.md) | V |
| viewer | [2026-10-03-mlapp-test-shadowed-by-m-file.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-03-mlapp-test-shadowed-by-m-file.md) | V |
| viewer | [2026-10-03-viewer-sparse-yaw-distortion.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-03-viewer-sparse-yaw-distortion.md) | V |
| viewer | [2026-10-04-viewer-marker-correction-distance.md](F:/Dropbox/Document/robotrace/Log/log/doc/失敗事例/2026-10-04-viewer-marker-correction-distance.md) | V |
| global | [2026-09-17-bridge-keeps-stale-serial-handle.md](D:/ドキュメント/codex/失敗事例/2026-09-17-bridge-keeps-stale-serial-handle.md) | N |
| global | [2026-09-17-core2-dtr-serial-reconnect.md](D:/ドキュメント/codex/失敗事例/2026-09-17-core2-dtr-serial-reconnect.md) | N |
| global | [2026-09-17-core2-requires-manual-reset-after-usb-reconnect.md](D:/ドキュメント/codex/失敗事例/2026-09-17-core2-requires-manual-reset-after-usb-reconnect.md) | N |
| global | [2026-09-17-m5stack-com-port-auto-detection.md](D:/ドキュメント/codex/失敗事例/2026-09-17-m5stack-com-port-auto-detection.md) | N |
| global | [2026-09-17-m5stack-display-stays-blank-after-reconnect.md](D:/ドキュメント/codex/失敗事例/2026-09-17-m5stack-display-stays-blank-after-reconnect.md) | N |
| global | [2026-09-17-m5stack-offline-indicator-not-cleared.md](D:/ドキュメント/codex/失敗事例/2026-09-17-m5stack-offline-indicator-not-cleared.md) | N |
| global | [2026-09-17-platformio-command-unavailable.md](D:/ドキュメント/codex/失敗事例/2026-09-17-platformio-command-unavailable.md) | N |
| global | [2026-09-19-git-push-sandbox-network.md](D:/ドキュメント/codex/失敗事例/2026-09-19-git-push-sandbox-network.md) | R, C |
| global | [2026-09-19-github-push-workflow-scope.md](D:/ドキュメント/codex/失敗事例/2026-09-19-github-push-workflow-scope.md) | R, C |
| global | [2026-09-22-git-index-lock-permission.md](D:/ドキュメント/codex/失敗事例/2026-09-22-git-index-lock-permission.md) | R, C |
| global | [2026-09-22-version4-pc-reader-closed-file.md](D:/ドキュメント/codex/失敗事例/2026-09-22-version4-pc-reader-closed-file.md) | A, R |

## 確認範囲

- 更新前SHA256を全対象で照合し、変更後に反映内容・リンク先・frontmatter・フェンス・差分を検査する。
- 通常Pythonと同梱PythonにPyYAMLがないことを事前確認したため、quick_validate.pyは起動せず代替構造検査を使う。標準バリデーターの成功とは報告しない。
- 保護領域への書込みは対象19ファイルに限定した必要権限付き処理で実施し、全19ファイルの反映後SHA256一致を確認した。
- 検証資料・変更前コピーはanalysis/skill-hardening-20261015に保存する。全142原本のSHA256を反映後にも照合する。
- ファームウェアコード・SD実設定・実機挙動の変更はないため、書込み・走行・ファームビルドは行わない。

8Skillのfrontmatter・UTF-8・フェンス・参照17箇所、全19ファイルの内容ハッシュ、既存142件の原本SHA256の検査がステージ/反映後とも成功した。対象限定のGit差分検査と、日本語パターン＋globを指定した失敗検索補助の実行も成功した。

今回新たに確認した2件も記録し、再発防止へ反映した。既存142件の件数には含めていない。

- [exec間のTEMP共有前提](失敗事例/2026-10-15-sandbox-temp-not-shared-between-exec.md)
- [例示リンクと既存空白の検証誤判定](失敗事例/2026-10-15-skill-link-validation-code-fence-example.md)

## mainへの選択反映

ユーザー指定に従い、今回のSkill関連4コミットだけをmainへ反映した。先行の制御変更5件は含めない。mainに未収録の事例原本は調査元e9903caのGit参照とした。この節はmain上の内容であり、上記の元ブランチでの全142件の読了・ハッシュ検証とは区別する。

- [管理worktreeのGit読取権限](失敗事例/2026-10-15-managed-worktree-git-read-permission.md)
- [選択反映時のSkill競合](失敗事例/2026-10-15-selective-skill-integration-context.md)
