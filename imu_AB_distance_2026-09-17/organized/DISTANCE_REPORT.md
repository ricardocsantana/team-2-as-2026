# Distance between points A and B

Indoor route, Linköping University · iPhone 14 · Sensor Logger 1.65.1 · 2026-09-17

## Result

**The route is 66.6 m of walking between two points that are 54.6 m apart in a straight line.
The interval is 65–68 m on path length and 53–56 m on the straight line.**

| Quantity | Estimate | Spread across 4 traversals × settings |
|---|---|---|
| **Route path length** | **66.6 m** | 64.9 – 68.1 m (±2.4%) |
| **Straight-line A→B** | **54.6 m** | 53.2 – 55.8 m (±2.4%) |
| Step length (this walker) | 0.66 m | stride 1.30 – 1.37 m |

**The two numbers mean different things — don't conflate them.**

- **66.6 m — path length.** The sum of the three waypoint-to-waypoint legs
  (6.2 + 50.7 + 10.4). Each leg *is* a straight segment, but they chain through two corners, so
  this is how far you actually walk.
- **54.6 m — straight line A→B.** The direct distance between the endpoints, ignoring the corners.
  This is the dashed line in `figures/reconstructed_paths.png`.

The route bends, so path length exceeds the straight line by about 22%.

Four independent one-way traversals contribute: two separate foot recordings, plus the two halves
of a single continuous round trip. They are analysed by the same pipeline but share no data.

| Traversal | Path | Straight line |
|---|---|---|
| Separate recording, A→B | 67.4 m | 54.3 m |
| Separate recording, B→A | 65.2 m | 54.9 m |
| Round trip, A→B half | 66.1 m | 53.2 m |
| Round trip, B→A half | 64.9 m | 55.4 m |

The route is three straight corridor legs joined by two corners. All four traversals are shown in
A→B order, so returns are reversed:

| Leg | Separate A→B | Separate B→A | Round trip A→B | Round trip B→A | Spread |
|---|---|---|---|---|---|
| 1 | 6.2 m | 6.2 m | 9.4 m | 7.2 m | 6.2 – 9.4 |
| **2 (long)** | **50.7 m** | **49.8 m** | **47.7 m** | **48.9 m** | **47.7 – 50.7 (±3%)** |
| 3 | 10.4 m | 9.2 m | 9.0 m | 8.9 m | 8.9 – 10.4 |

Leg 2 carries three-quarters of the distance, is consistent to ±3% across all four traversals, and
is the number to trust. Legs 1 and 3 are short and sit next to the endpoints where the walker turns
on the spot, so their outer ends are the least well defined — that is where the spread lives.

## How it was measured

Foot-mounted zero-velocity-update (ZUPT) inertial navigation, from three recordings with the phone
strapped to the foot: two one-way walks and one continuous A→B→A round trip.

The foot is genuinely stationary during the stance phase of every stride. Detecting those stances
gives a velocity reset roughly twice per second, so integration drift never accumulates for more
than one swing. **No step-length model is used** — the distance falls out of the mechanisation
itself, which removes the largest source of uncertainty in conventional phone-based estimates.

Raw integrated trajectories were then snapped to a polyline: corners detected from sustained
heading change, straight legs fitted through the footprints by total least squares, waypoints
placed at the intersections. This removes stride-to-stride heading wander, which otherwise inflates
path length and shortens the straight-line chord.

Corners are detected at a 40° heading-change threshold, the centre of a 36–44° plateau over which
all four traversals agree on 3 legs with the same lengths — chosen for that stability, not tuned to
a target distance.

Reproduce with `analysis/results.py`, `analysis/roundtrip.py` and `analysis/plots.py`; per-setting
numbers are in `distance_results.csv` and `roundtrip_results.csv`.

## Evidence

| Check | Result | Why it counts |
|---|---|---|
| Four independent traversals | path 64.9–67.4 m; chord 53.2–55.4 m | separate walks, separate integrations |
| **Round-trip loop closure** | 12.9 m → **2.7 m (2.1%)** after fitting one drift rate | start and end are the *same physical point* — external truth |
| Per-leg agreement | 6.2/6.2, 50.7/49.8, 10.4/9.2 m | each leg measured twice, independently |
| Stance-threshold sensitivity | ±2.4% across a 1.6× threshold range | the answer is not a tuning artifact |
| Corner-threshold plateau | 36–44° all give 3 legs, identical lengths | segmentation is not tuned to a target |
| Vertical drift | ≤1.15 m over 79–91 s | truth is ≈0 on a level route — bounds attitude and bias error |
| **GPS endpoint separation** | **56.0 m** vs 54.6 m inertial — **2.5% apart** | **entirely different sensor system** |
| Step length vs anthropometry | 0.66 m, tight IQR | expected ≈0.74 m at 179 cm; consistent with the reported slow pace |
| Keypoint-leg step count | 107 steps → 0.64 m/step | independent recording, same step length |

The GPS and vertical-drift checks are the load-bearing ones: both have external truth, so neither
can be satisfied by a self-consistent but wrong pipeline.

## Limits

**Absolute scale is confirmed to roughly ±4%, not better.** GPS agreement at 2% is reassuring, but
the indoor fixes carry 15–55 m reported accuracy, so that agreement is weaker evidence than the
number suggests. A tape-measured 20 m calibration walk, or counting floor tiles along leg 2, would
take scale to ~1%. That is the only measurement still worth making.

**Heading drifts at about −8.7°/min, and that is the dominant error.** The round trip measures it
directly: it starts and ends at the same physical point in one continuous heading frame, so the
12.9 m gap between reconstructed start and end is pure drift. Fitting a single constant drift rate
closes it to **2.7 m, or 2.1% of the 133 m round trip** — a real test, since one unknown against two
equations (x and y) leaves a residual degree of freedom.

Consequences:
- **Path length is unaffected.** It is invariant under this rotation — the per-stride distances do
  not change, only where they point. Distance is far better determined than direction.
- The 33.6° heading discrepancy seen between the two *separate* recordings is explained
  quantitatively by this drift rate acting over their durations plus the gap between them.
- Absolute orientation of the route map is unreliable. The return trajectory is rotated 34° to
  overlay the outbound in `route_geometry.png`; that rotation is cosmetic and never fitted to scale.

**The short end legs carry most of the spread** (leg 1 ranges 6.2–9.4 m across traversals). They
abut the endpoints where the walker pivots on the spot, so where the route "starts" is genuinely
ambiguous by a stride or two. The long middle leg is stable to ±3%.

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
| `figures/reconstructed_paths.png` | **all four reconstructions overlaid + one panel each** — start here |
| `figures/route_geometry.png` | the two separate trajectories, fitted polyline, leg lengths, turn angles |
| `figures/waypoint_detection.png` | heading vs distance with detected corners — the corner picks are auditable |
| `figures/leg_comparison.png` | per-leg length, each direction |
| `figures/stance_diagnostics.png` | gyro with stance phases shaded; stride-length distribution |
| `figures/method_comparison.png` | every estimate with its interval |
| `figures/gps_overlay.png` | GPS fixes with accuracy discs against the endpoints |
| `figures/roundtrip_closure.png` | round trip before/after heading-drift correction |
