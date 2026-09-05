---
title: "CA_SLIP_EXPAND_2=0.20 no stable improvement"
date: 2026-08-16
status: open
project: robotrace_v2
---

## Task
Evaluate the non-local slip expansion candidate `CA_SLIP_EXPAND_2=0.20` against the `speedFeedForwardGain=150` baseline after the 7.1 V motor-command conversion.

## Observed result
- Candidate logs `12006-12040`: 7 autoStart sets, all 28 secondary runs valid.
- Start voltage was `7.54-7.84 V`, all above the `7.3 V` eligibility threshold; maximum `cntlog` gap was `13 ms`; all `emcStop=0`; minimum rows `898`; minimum final time `5063 ms`.
- Per autoStart run-number comparison against baseline `11921-11935` was mixed for lap and line metrics, but speed error abs-p95 was worse or unchanged for every run number:
  - run 2: `+0.0446 m/s`
  - run 3: `+0.0125 m/s`
  - run 4: `+0.0303 m/s`
  - run 5: approximately unchanged
- Lap change by run number was `+36.2, +14.2, -13.8, +22.2 ms`; no stable improvement was reproduced.
- The later low-voltage sets showed larger variation, including run 2 at `7.56 V` with lap `5416 ms`, line RMS `88.0`, and motor duty saturation `13.69%`.

## Cause or uncertainty
- The candidate did not provide stable speed tracking improvement. The comparison is partly confounded by the candidate voltage declining from `7.84` to `7.54 V`, although the start-voltage eligibility threshold was met.
- Do not attribute the run-number lap variation to the slip expansion alone; autoStart adapts later target speeds from earlier runs.

## Working remedy / next investigation
- Treat `CA_SLIP_EXPAND_2=0.20` as not adopted and compare future candidates against the known-good value `0.25` using the same run-number grouping.
- Keep the voltage band as close as practical, preferably `7.7-7.9 V` at the start, for the next candidate.

## Prevention
- Always report run-number-specific lap and speed metrics, not a pooled lap average.
- Before adopting a parameter, require stable speed-error improvement across run numbers and at least two complete autoStart sets with similar start voltage.
