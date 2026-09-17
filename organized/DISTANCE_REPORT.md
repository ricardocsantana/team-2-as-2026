# Distance between points A and B

Indoor route, Linköping University · iPhone 14 · Sensor Logger 1.65.1 · 2026-09-17

## Result

**The route is 66.6 m of walking between two points that are 54.8 m apart in a straight line.
The 5–95% interval is 65–68 m on path length and 54–55 m on the straight line.**

| Quantity | Estimate | Spread across settings |
|---|---|---|
| **Route path length** | **66.6 m** | 65.2 – 68.0 m (±2.1%) |
| **Straight-line A→B** | **54.8 m** | 54.3 – 55.4 m (±1.0%) |
| Step length (this walker) | 0.68 m | stride 1.33 – 1.37 m |

The route is three straight corridor legs joined by two corners:

| Leg | Outbound A→B | Return B→A (reversed) | Agreement |
|---|---|---|---|
| 1 | 6.2 m | 6.2 m | 0% |
| 2 | 50.7 m | 49.8 m | 2% |
| 3 | 10.4 m | 9.2 m | 13% |
| Corners | −49°, +104° | −51°, +94° | — |

Leg 2 carries three-quarters of the distance and is the best-measured of the three.

## How it was measured

Foot-mounted zero-velocity-update (ZUPT) inertial navigation, from two recordings with the phone
strapped to the foot.

The foot is genuinely stationary during the stance phase of every stride. Detecting those stances
gives a velocity reset roughly twice per second, so integration drift never accumulates for more
than one swing. **No step-length model is used** — the distance falls out of the mechanisation
itself, which removes the largest source of uncertainty in conventional phone-based estimates.

Raw integrated trajectories were then snapped to a polyline: corners detected from sustained
heading change, straight legs fitted through the footprints by total least squares, waypoints
placed at the intersections. This removes stride-to-stride heading wander, which otherwise inflates
path length and shortens the straight-line chord.

Reproduce with `analysis/results.py` and `analysis/plots.py`; per-setting numbers are in
`distance_results.csv`.

## Evidence

| Check | Result | Why it counts |
|---|---|---|
| Two independent recordings | path 67.4 vs 65.2 m — **3% apart**; chord 54.3 vs 54.9 m — **1% apart** | separate walks, separate integrations |
| Per-leg agreement | 6.2/6.2, 50.7/49.8, 10.4/9.2 m | each leg measured twice, independently |
| Stance-threshold sensitivity | ±2.1% across a 1.6× threshold range | the answer is not a tuning artifact |
| Vertical drift | ≤1.15 m over 79–91 s | truth is ≈0 on a level route — bounds attitude and bias error |
| **GPS endpoint separation** | **56.0 m** vs 54.8 m inertial — **2% apart** | **entirely different sensor system** |
| Step length vs anthropometry | 0.68 m, tight IQR | expected ≈0.74 m at 179 cm; consistent with the reported slow pace |
| Keypoint-leg step count | 107 steps → 0.64 m/step | independent recording, same step length |

The GPS and vertical-drift checks are the load-bearing ones: both have external truth, so neither
can be satisfied by a self-consistent but wrong pipeline.

## Limits

**Absolute scale is confirmed to roughly ±4%, not better.** GPS agreement at 2% is reassuring, but
the indoor fixes carry 15–55 m reported accuracy, so that agreement is weaker evidence than the
number suggests. A tape-measured 20 m calibration walk, or counting floor tiles along leg 2, would
take scale to ~1%. That is the only measurement still worth making.

**Heading frames differ between recordings.** Loop closure fails with a 31 m residual and a 33.6°
discrepancy between outbound and reversed-return headings — indoor magnetic disturbance (|B| swings
14–60 µT against a 45.8 µT true field) pulls CoreMotion's yaw differently in each file. This does
not affect distance: both recordings independently report the same start-to-end separation, they
just disagree on its compass bearing. It does mean the absolute orientation of the route map is
unreliable, and the return trajectory is rotated 34° to overlay the outbound in the figures.

**Leg 3 differs by 13% between directions** (10.4 vs 9.2 m). It is short and sits next to the
endpoint where the walker turns on the spot, so its start point is the least well defined. Leg 2 is
the number to trust.

## Excluded

One stance threshold (gyro < 0.35 rad/s, accel < 1.2 m/s²) is reported in `distance_results.csv`
but excluded from the headline: it detects only 25 strides in 79 s, a 0.32 Hz stride rate that is
physically impossible for walking. Missed stances merge adjacent strides, inflating the median
stride to 2.01 m. This is a detector failure, not a competing estimate.

## Checks that prove nothing — deliberately not used as evidence

- **"76 m implies 0.55 m/step, which is plausible."** The step length was derived *from* the assumed
  distance. Any distance between roughly 60 and 95 m yields a "plausible" step length, so the check
  cannot fail and carries no information.
- **"Both walking trips agree once calibrated on the cart trip."** True by construction — it is one
  measurement printed twice. Only the residual disagreement is informative.
- **"Gyro-integrated yaw agrees with CoreMotion yaw."** CoreMotion's yaw is itself largely gyro
  integration; this tests arithmetic, not accuracy.
- **"Path/displacement ratio is consistent with a curved route."** Both come from one integration, so
  a common scale error cancels in the ratio.
- **"Velocity returns to zero at every stance."** The de-trend imposes that.

## Earlier estimates, superseded

| Method | Estimate | Why superseded |
|---|---|---|
| Cart ZUPT, full window | 76.4 m | includes a 9.8 m post-arrival backtrack |
| Cart ZUPT, to arrival | 64.8 m | one 36 s unconstrained segment carried 80% of it; ±10% from thresholds alone |
| Step count × step length | ~84 m | depends on cadence², and step counts were never threshold-stable (137–235 for one trip) |

The foot-mounted result supersedes all three: it is threshold-insensitive, needs no step-length
model, and is the only one with an external cross-check.

## Figures

| File | Shows |
|---|---|
| `figures/route_geometry.png` | both trajectories, fitted polyline, leg lengths, turn angles |
| `figures/waypoint_detection.png` | heading vs distance with detected corners — the corner picks are auditable |
| `figures/leg_comparison.png` | per-leg length, each direction |
| `figures/stance_diagnostics.png` | gyro with stance phases shaded; stride-length distribution |
| `figures/method_comparison.png` | every estimate with its interval |
| `figures/gps_overlay.png` | GPS fixes with accuracy discs against the endpoints |
