# IMU dataset — trips between two points (A ↔ B)

> **The distance question is answered in [`DISTANCE_REPORT.md`](DISTANCE_REPORT.md):**
> **66.3 m of walking between points 54.2 m apart.** This file documents the first three
> recordings; later recordings live in `../point-to-point-back/` and `../foot/`.

iPhone 14 · Sensor Logger 1.65.1 · 2026-09-17 · Europe/Stockholm (UTC+2)
Single device (`f24d06db-f205-45df-9fef-e162378af3f2`), 9 sensor streams, nominal 100 Hz.

| Trip | Leg | Start (local) | Duration | Trimmed | Source zip |
|---|---|---|---|---|---|
| `trip1_outbound` | A → B | 09:21:07 | 112.3 s | no | `2026-09-17_07-21-07.zip` |
| `trip2_return`   | B → A | 09:24:16 |  77.4 s | no | `2026-09-17_07-24-16.zip` |
| `trip3_outbound` | A → B | 09:32:00 |  75.5 s | **yes — see below** | `2026-09-17_07-31-11.zip` |

Idle gap trip1→trip2: 76 s. Trip2→trip3: 338 s (~5.6 min).

## Layout

```
organized/
├── README.md                  ← this file
├── data_dictionary.csv        ← every column, unit, meaning
├── trips.csv                  ← one row per trip: timing, trim, device, motion summary
├── qc_summary.csv             ← one row per trip×sensor: rate, gaps, dropouts, NaNs
├── figures/overview.png       ← accel / gyro / attitude / compass for all 3 trips
├── trip1_outbound/            ← per-sensor CSVs, values UNCHANGED from the export
│   ├── accelerometer.csv, accelerometer_uncalibrated.csv, gravity.csv,
│   │   gyroscope.csv, gyroscope_uncalibrated.csv, magnetometer.csv,
│   │   magnetometer_uncalibrated.csv, orientation.csv, compass.csv
│   └── metadata.csv
├── trip2_return/ , trip3_outbound/
└── merged/                    ← analysis-ready, all sensors on one 100 Hz grid
    ├── trip1_outbound_100hz.csv / .parquet
    ├── trip2_return_100hz.csv   / .parquet
    ├── trip3_outbound_100hz.csv / .parquet        ← trimmed
    ├── trip3_outbound_untrimmed_100hz.parquet     ← full 132 s, kept for reference
    └── all_trips_100hz.parquet  ← all 3 trips stacked (26,512 rows), use `trip` column
```

**Per-sensor files** are the raw export, only reorganised: columns reordered to
`timestamp_utc, time_ns, seconds_elapsed, x, y, z` (the app exports `z,y,x` — easy to
mis-read), files renamed to snake_case, and an absolute UTC timestamp added. No values touched.
For trip 3 they contain only the trimmed window; `seconds_elapsed` still counts from the
original recording start, so it begins at 49.0 s rather than 0.

**Merged files** are linearly interpolated onto an exact 100 Hz grid spanning the window
covered by *all* sensors. Angles (yaw, compass bearing) are interpolated on the unit circle,
not numerically, so the 0/360 wrap is handled correctly; quaternions are renormalised.
`t_s` restarts at 0 for every trip so the legs line up; `t_s_recording` keeps the original
recording clock (identical to `t_s` for trips 1–2, offset by +49.0 s for trip 3).

## Quick start

```python
import pandas as pd
df = pd.read_parquet("organized/merged/all_trips_100hz.parquet")
t1 = df[df.trip == "trip1_outbound"]
```

## Trip 3 was trimmed to its moving window

The original trip-3 recording ran 132.0 s but the phone was only in motion for part of it:
~49 s of stationary lead-in at the start and ~8 s of stationary tail at the end.

Motion onset/offset were detected from a 0.25 s rolling accelerometer standard deviation and
mean angular rate, against the trip's own stationary baseline (acc σ = 0.035 m/s², gyro =
0.005 rad/s), requiring motion to be sustained for ≥1 s. Sustained motion runs
**t = 49.19 → 124.28 s**; the cut was made slightly outside that, at **[49.0, 124.5] s**, to
keep the full onset and settle transients.

| | before trim | after trim |
|---|---|---|
| duration | 132.0 s | **75.5 s** |
| stationary | 31.6 % | **1.6 %** |
| accel RMS | 1.57 m/s² | **2.03 m/s²** |

An isolated 0.43 s acceleration spike at t ≈ 19.5 s (phone being set down or adjusted) is
correctly excluded — it is a transient, not travel. The short quiet stretches that remain
*inside* the trimmed window are genuine mid-trip stops and were deliberately kept.

The full untrimmed recording is preserved at `merged/trip3_outbound_untrimmed_100hz.parquet`,
and the original zips are untouched, so the trim is fully reversible.

Trip 3 is now 75.5 s against trip 2's 77.4 s — close enough to compare directly.

## Data quality — all clean

Checked every stream for gaps, dropouts, duplicate timestamps, NaNs and clock monotonicity.

- **0 missing values, 0 duplicate timestamps, 0 gaps >50 ms, 0 clock reversals** across all 27 trip×sensor streams (trim included — the cut landed inside a continuous run, so no gap was introduced).
- Sampling is metronomic: dt = 9.96 ms (100.40 Hz) with min = max to 0.01 ms.
- `magnetometer_uncalibrated` runs on a different clock: 10.32 ms (96.87 Hz). Expected, not a fault — it is why the merged grid is needed.
- Sensor start times within a trip differ by 15–45 ms; the merged grid starts after the last sensor is live, so no edge extrapolation.
- Physical sanity: |gravity| = 9.8067 constant, |quaternion| = 1.000000, gyro bias (cal vs uncal) ≈ 3e-3 rad/s.

## Units — read this before using the accelerometer

`accelerometer.csv` is **user acceleration with gravity already removed**, in m/s².
`accelerometer_uncalibrated.csv` is the **raw sensor in g** (not m/s²). Verified relation:

```
acc_raw * 9.80665 - gravity = acc      (residual ≈ 0.1 m/s², interpolation only)
```

Don't mix the two, and don't subtract gravity twice.

## ⚠️ Trip 3 is still a different kind of recording from trips 1 and 2

Trimming fixed the duration mismatch. It did **not** make the carrying mode comparable.

| | trip1 | trip2 | **trip3 (trimmed)** |
|---|---|---|---|
| mean pitch / roll | +0.13 / −0.03 rad | +0.13 / +0.01 rad | **+0.02 / −0.01 rad (flat)** |
| attitude variation | ±0.25 rad | ±0.25 rad | **±0.04 rad** |
| gyro RMS | 0.33 rad/s | 0.46 rad/s | **0.26 rad/s** |
| accel RMS | 0.88 m/s² | 1.25 m/s² | **2.03 m/s² (peaks 21)** |
| dominant frequency | 1.22 Hz | 3.35 Hz (1.71 Hz fundamental) | **0.30 Hz** |

- **Trips 1 and 2 are hand-carried walking.** Continuous motion, tilted phone, pitch/roll
  swinging with gait, clear step-frequency peaks. Trip 2 was walked noticeably faster than
  trip 1 (1.71 Hz vs 1.22 Hz cadence, 77 s vs 112 s).
- **Trip 3 is a phone lying flat on a wheeled cart** (confirmed by the recordist). Pitch/roll stay
  pinned near zero and barely move even while vertical acceleration spikes to ±8 m/s² (peak 21).
  That combination — violent translation, almost no rotation — rules out walking. Mean speed works
  out at ~1.25 m/s, i.e. the cart was pushed at walking pace.

**Treat trip 3 as a different modality, not as a third walking leg.** If the experiment needs
three like-for-like legs, it should be re-recorded the way trips 1 and 2 were.

## ⚠️ No GPS / location data in *these three* recordings

`Location` was not enabled for trips 1–3. It **was** enabled for the later foot-mounted pair in
`foot/`, which provides the only external positional reference in the dataset — see
`DISTANCE_REPORT.md`.

## ⚠️ Magnetometer: unusable for absolute heading, but a good spatial fingerprint

Calibrated field magnitude is 33 µT median and swings 14–60 µT within a single trip. **The sensor
is healthy** — measured at rest it reads 45.76 µT ± 0.23 µT, which is normal for this latitude. The
swings are *route-correlated spatial disturbance* from the building structure, not sensor noise.

Two consequences, pulling in opposite directions:

- `compass_bearing_deg` and `ori_yaw` are unreliable as **absolute heading**. Relative turn
  detection from the gyro is sound; absolute magnetic heading is not.
- The disturbance pattern is repeatable along the route, so |B| works as a **spatial fingerprint**.
  Correlation between trip 1 and reversed trip 2 is 0.88, against 0.12 for the forward-direction
  control — independent confirmation that they traverse the same corridor in opposite directions.

## ⚠️ Correction: the route is essentially straight

An earlier version of this file described the route as turning ~140°. That was wrong. The large
yaw excursions are the **device being handled at the endpoints**, not route geometry: in trip 3
(rigidly mounted, so device yaw tracks platform yaw) heading changes only 2° while the cart covers
57.8 m, then swings +175° in 25 s while moving just 1.6 m.

The foot-mounted recordings resolve the true geometry: three straight legs of 6.2 m, 50.7 m and
10.4 m joined by two corners of −49° and +104°.

## Notes

- `Annotation.csv` was empty in all three exports (no event markers were placed during recording).
  Using it would have resolved the trip-3 arrival ambiguity — see `DISTANCE_REPORT.md`.
- Leg labels (`outbound` / `return`) follow the stated going → back → going order; they are not
  derived from the data, since there is no position fix.
