---
title: "CA_SLIP_DOWN_LAT_EXTRA=0.25 no stable improvement"
date: 2026-08-16
status: open
project: robotrace_v2
---

## Task
Evaluate the non-local lateral-slip direct-deceleration candidate `CA_SLIP_DOWN_LAT_EXTRA=0.25` against the known baseline `0.30` after the 7.1 V motor-command conversion.

## Observed result
- Candidate logs `12041-12060`: 4 complete autoStart sets, all 16 secondary runs valid.
- All runs had `emcStop=0`, minimum rows `904`, maximum `cntlog` gap `11 ms`, and start voltage `7.75-8.20 V`, above the project threshold `7.3 V`.
- Compared by autoStart run number against baseline `11921-11935`:
  - run 2: lap `-7.9 ms`, speed-error abs-p95 `+0.0312 m/s`, line RMS `+3.22`, gyro RMS `+6.43`, lateral slip `+1.14 points`
  - run 3: lap `-12.9 ms`, speed-error abs-p95 `+0.0172 m/s`, line RMS approximately unchanged, gyro RMS `-1.01`, lateral slip `-0.40 points`
  - run 4: lap `-31.2 ms`, speed-error abs-p95 `+0.0203 m/s`, line RMS `+2.52`, gyro RMS `+8.79`, lateral slip `+2.38 points`
  - run 5: lap `+14.1 ms`, speed-error abs-p95 unchanged, line RMS `-7.93`, gyro RMS `-4.83`, lateral slip `-0.89 points`
- The candidate improved lap in runs 2-4, but speed-error p95 was worse in runs 2-4 and lateral slip was worse in runs 2 and 4. No stable multi-metric improvement was reproduced.

## Cause or uncertainty
- Weakening direct lateral-slip deceleration may shorten some later laps, but it also leaves higher speed error and increases lateral slip in selected run numbers.
- Candidate voltage was generally higher than the baseline, so voltage is a remaining confound; this does not establish that `0.25` is better.

## Working remedy / next investigation
- Do not adopt `CA_SLIP_DOWN_LAT_EXTRA=0.25` without another controlled comparison. Restore the known baseline `0.30` before the next independent trial.
- If the speed-plan adjustment is resumed, change only one non-local parameter and compare the same autoStart run numbers with similar start voltage.

## Prevention
- Do not adopt based on lap improvement alone. Require run-number-specific speed-error and lateral-slip behavior to remain non-worse across at least two complete sets.
- Keep candidate and baseline start-voltage bands close, preferably within approximately `0.2 V`.
