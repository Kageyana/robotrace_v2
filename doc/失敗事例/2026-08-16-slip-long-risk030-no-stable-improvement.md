---
title: "CA_SLIP_DOWN_LONG_RISK=0.03 no stable improvement"
date: 2026-08-16
status: open
project: robotrace_v2
---

## Task
Evaluate the non-local longitudinal-slip risk deceleration candidate `CA_SLIP_DOWN_LONG_RISK=0.03` against the baseline `0.04` after the 7.1 V motor-command conversion.

## Observed result
- Logs `12076-12097` contained 3 complete valid autoStart sets: `12077-12081`, `12082-12086`, and `12088-12092`.
- Set `12093-12097` was excluded as a whole because `12094` was incomplete (`rows=570`, `last_cnt_ms=3154 ms`, `optimalIndex_max=120`). Logs `12076` and `12087` were isolated first-run records and were not used as secondary-run sets.
- Valid candidate start voltage was `7.65-7.82 V`, above the `7.3 V` threshold; valid runs had `emcStop=0` and maximum `cntlog` gap `12 ms`.
- Per autoStart run-number comparison against baseline `11921-11935`:
  - run 2: lap `+23.0 ms`, speed-error abs-p95 `+0.0499 m/s`, line RMS `+15.98`, gyro RMS `+7.35`, lateral slip `+2.33 points`
  - run 3: lap `+26.0 ms`, speed-error abs-p95 `+0.0125 m/s`, line RMS `-1.48`, lateral slip `-1.43 points`
  - run 4: lap `-76.7 ms`, speed-error abs-p95 `+0.0374 m/s`, gyro RMS `+11.11`, lateral slip `+1.46 points`
  - run 5: lap `+15.7 ms`, speed-error abs-p95 `+0.0062 m/s`, line RMS `-10.35`, lateral slip `-0.91 points`
- Speed-error p95 was worse for all run numbers and lap improvement appeared only in run 4. No stable multi-run improvement was reproduced.

## Cause or uncertainty
- Weakening longitudinal-slip risk deceleration can shorten one adaptive run but did not improve speed tracking consistently and increased line/gyro load in run 2 and gyro load in run 4.
- The excluded incomplete set is a separate run-start/hardware event and should not be attributed to this parameter without further evidence.

## Working remedy / next investigation
- Do not adopt `CA_SLIP_DOWN_LONG_RISK=0.03`. Restore the baseline `0.04` before the next independent trial.
- Avoid further isolated slip-risk attenuation without a new measurement; the recent `EXPAND`, lateral, and longitudinal attenuation candidates have not produced stable speed-tracking gains.

## Prevention
- Exclude an entire autoStart set when any run is incomplete, even if other files in that set are individually valid.
- Require speed-error p95 non-worsening for each autoStart run number before adopting a speed-plan change.
