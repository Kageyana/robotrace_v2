---
title: "CA_SLIP_DOWN_LAT_RISK=0.15 no stable improvement"
date: 2026-08-16
status: open
project: robotrace_v2
---

## Task
Evaluate the non-local lateral-slip risk deceleration candidate `CA_SLIP_DOWN_LAT_RISK=0.15` against the baseline `0.20` after the 7.1 V motor-command conversion.

## Observed result
- Candidate logs `12061-12075`: 3 complete autoStart sets, all 12 secondary runs valid.
- All runs had `emcStop=0`, and start voltage was `7.69-8.00 V`, above the `7.3 V` threshold.
- Per autoStart run-number comparison against baseline `11921-11935`:
  - run 2: lap `+39.3 ms`, speed-error abs-p95 `+0.0374 m/s`, line RMS `+15.63`, gyro RMS `+5.06`, lateral slip `+1.59 points`
  - run 3: lap `+38.0 ms`, speed-error abs-p95 unchanged, line RMS approximately unchanged, lateral slip `-0.29 points`
  - run 4: lap `+11.3 ms`, speed-error abs-p95 `+0.0187 m/s`, gyro RMS `+13.00`, lateral slip `+1.09 points`
  - run 5: lap `+53.7 ms`, speed-error abs-p95 `+0.0125 m/s`, lateral slip `-0.30 points`
- Lap was slower for every run number and speed-error p95 was worse or unchanged for every run number.

## Cause or uncertainty
- Reducing the risk-based lateral deceleration did not improve the target/current speed relationship; it increased lap time and sometimes increased lateral slip and line load.
- The comparison voltage band was acceptable but not identical to the baseline, so voltage remains a minor confound.

## Working remedy / next investigation
- Do not adopt `CA_SLIP_DOWN_LAT_RISK=0.15`. Restore the known baseline `0.20` before the next independent trial.
- Treat the lateral-slip deceleration parameter branch (`LAT_RISK` and `LAT_EXTRA`) as not productive unless a new hypothesis and a controlled comparison are defined.

## Prevention
- Require speed-error p95 and lap to be non-worse across the same autoStart run numbers; do not adopt from lateral-slip reduction alone.
- Keep at least three complete sets and comparable start-voltage bands for a speed-plan change.
