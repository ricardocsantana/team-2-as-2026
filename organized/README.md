# IMU dataset — 3 trips between two points (A ↔ B)

iPhone 14 · Sensor Logger 1.65.1 · 2026-09-17 · Europe/Stockholm (UTC+2)
Single device (`f24d06db-f205-45df-9fef-e162378af3f2`), 9 sensor streams, nominal 100 Hz.

| Trip | Leg | Start (local) | Duration | Source zip |
|---|---|---|---|---|
| `trip1_outbound` | A → B | 09:21:07 | 112.3 s | `2026-09-17_07-21-07.zip` |
| `trip2_return`   | B → A | 09:24:16 |  77.4 s | `2026-09-17_07-24-16.zip` |
| `trip3_outbound` | A → B | 09:31:11 | 132.0 s | `2026-09-17_07-31-11.zip` |

Idle gap trip1→trip2: 76 s. Trip2→trip3: 338 s (~5.6 min).

## Layout

```
organized/
├── README.md                  ← this file
├── data_dictionary.csv        ← every column, unit, meaning
├── trips.csv                  ← one row per trip: timing, device, motion summary
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
    ├── trip3_outbound_100hz.csv / .parquet
    └── all_trips_100hz.parquet  ← all 3 trips stacked (32,167 rows), use `trip` column
```

**Per-sensor files** are the raw export, only reorganised: columns reordered to
`timestamp_utc, time_ns, seconds_elapsed, x, y, z` (the app exports `z,y,x` — easy to
mis-read), files renamed to snake_case, and an absolute UTC timestamp added. No values touched.

**Merged files** are linearly interpolated onto an exact 100 Hz grid spanning the window
covered by *all* sensors. Angles (yaw, compass bearing) are interpolated on the unit circle,
not numerically, so the 0/360 wrap is handled correctly; quaternions are renormalised.

## Quick start

```python
import pandas as pd
df = pd.read_parquet("organized/merged/all_trips_100hz.parquet")
t1 = df[df.trip == "trip1_outbound"]
```

## Data quality — all clean

Checked every stream for gaps, dropouts, duplicate timestamps, NaNs and clock monotonicity.

- **0 missing values, 0 duplicate timestamps, 0 gaps >50 ms, 0 clock reversals** across all 27 trip×sensor streams.
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

## ⚠️ Trip 3 is not the same kind of recording as trips 1 and 2

This is the main thing to know before anyone compares legs.

| | trip1 | trip2 | **trip3** |
|---|---|---|---|
| mean pitch / roll | +0.13 / −0.03 rad | +0.13 / +0.01 rad | **+0.02 / −0.01 rad (flat)** |
| attitude variation | ±0.25 rad | ±0.25 rad | **±0.04 rad** |
| gyro RMS | 0.33 rad/s | 0.46 rad/s | **0.20 rad/s** |
| accel RMS | 0.88 m/s² | 1.25 m/s² | **1.57 m/s² (peaks 21)** |
| dominant frequency | 1.22 Hz | 3.35 Hz (1.71 Hz fundamental) | **0.40 Hz** |
| stationary | 0 % | 0 % | **32 %** |

- **Trips 1 and 2 are hand-carried walking.** Continuous motion for the whole recording,
  tilted phone, pitch/roll swinging with gait, clear step-frequency peaks. Trip 2 was walked
  noticeably faster than trip 1 (1.71 Hz vs 1.22 Hz cadence, 77 s vs 112 s).
- **Trip 3 is a flat, rigidly-supported phone** — lying on a surface, not in a hand.
  Pitch/roll are pinned near zero and barely move even while vertical acceleration spikes to
  ±8 m/s² (peak 21). That combination — violent translation, no rotation — rules out walking.
  It reads as a vehicle, cart or mount.
- **Trip 3 contains ~50 s of stationary lead-in** (0–50 s: accel RMS 0.10, gyro 0.004 rad/s)
  and ~12 s of stationary tail (120–132 s). Only **t ≈ 50–120 s is actual travel**, so the
  comparable moving duration is ~70 s, not 132 s.

**If the goal is three comparable A→B→A→B legs, trip 3 should be trimmed to t ∈ [50, 120] s
and the carrying mode difference noted — or the leg re-recorded the same way as trips 1 and 2.**

## ⚠️ No GPS / location data

`Location` was not enabled in any of the three recordings. There is no position, speed or
altitude ground truth, and no way to confirm the endpoints or path length from this data alone.
Anything positional must come from dead reckoning on the IMU (which will drift) or from an
external source.

## ⚠️ Magnetometer is disturbed — treat compass headings with caution

Calibrated field magnitude is 33 µT median and swings 14–60 µT within a single trip.
Stockholm's true field is ~50 µT and should be near-constant outdoors. The variation means
significant local magnetic disturbance (buildings, vehicle, rebar), so `compass_bearing_deg`
and `ori_yaw` are unreliable as absolute heading — the trip-mean bearings do not line up with
a clean 180° reversal between outbound and return. Relative turn detection from the gyro is
sound; absolute magnetic heading is not.

## Notes

- `Annotation.csv` was empty in all three exports (no event markers were placed during recording).
- Leg labels (`outbound` / `return`) follow the stated going → back → going order; they are not
  derived from the data, since there is no position fix.
