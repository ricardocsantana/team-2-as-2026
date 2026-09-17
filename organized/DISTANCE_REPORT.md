# Distance between points A and B

Indoor route, Linköping University · iPhone 14 · Sensor Logger 1.65.1 · 2026-09-17

## Result

**The route is 66.8 ± 0.8 m of walking between two points that are 54.1 ± 0.6 m apart in a
straight line.**

| Quantity | Optimal estimate | Precision | With method accuracy |
|---|---|---|---|
| **Route path length** | **66.8 m** | ±0.4 m (0.6%) | **±0.8 m (1.2%)** |
| **Straight-line A→B** | **54.1 m** | ±0.2 m (0.4%) | **±0.6 m (1.1%)** |
| Step length (this walker) | 0.66 m | stride 1.30 – 1.37 m | |

Computed by `analysis/optimal.py`. See *How the optimal estimate is formed* below — a plain mean
of the four traversals would be wrong.

**The two numbers mean different things — don't conflate them.**

- **66.8 m — path length.** How far you actually walk: four straight waypoint-to-waypoint legs
  (≈6.2 + 50.2 + 9.7 + 1.6) chained through three corners.
- **54.1 m — straight line A→B.** The direct distance between the endpoints, ignoring the corners.
  This is the dashed line in `figures/reconstructed_paths.png`.

The route bends, so path length exceeds the straight line by about 23%.

## How the optimal estimate is formed

A plain average of the four traversals would be wrong three times over.

**1. The four traversals are not four independent samples.** The two halves of the round trip come
from one recording and share one heading-drift fit, so they count as a single block. The estimator
averages within blocks (`sep-A`, `sep-B`, `rt`) first, then across blocks — three independent
measurements, not four.

**2. The polyline estimator is fragile; the raw path is not.** Across traversals the raw path
scatters by 1.05 m and moves 0.48 m with the stance threshold, while the polyline scatters by
3.44 m and moves 2.32 m. One configuration (`rt-ret` at the loose threshold) collapsed to 57.3 m
against a 68.1 m raw path — a corner-detection failure that a plain mean would have silently
absorbed. It is excluded by an explicit rule: reject any polyline below 95% of its own raw path.

**3. The two path estimators bracket the truth.** Raw integration counts stride-level foot wander
and so reads slightly high; the polyline straightens genuine bends and so reads slightly low. Their
midpoint is the estimate and half their difference enters the budget as model uncertainty — here
only ±0.04 m, because with clean segmentation they agree to within 0.7 m.

**GPS is fused by inverse variance** and contributes 0.0% of the weight: at ±15 m it corroborates
the answer but cannot move it. That is the honest outcome, not a reason to drop it.

### Precision is not accuracy

| | Path | Straight line |
|---|---|---|
| **Precision** — repeatability the data proves | 66.8 ± 0.4 m | 54.1 ± 0.2 m |
| **Accuracy** — adds 1% assumed method bias | 66.8 ± 0.8 m | 54.1 ± 0.6 m |

The accelerometer's static gain is verified at **+0.009% (spread ±0.04%)** from the stationary
lead-ins, so sensor scale is not the limit. But that validates the accelerometer, not the *dynamic
integration*: stance timing, foot roll during "stance", and the linear de-trend standing in for a
full EKF all bias distance without showing up in repeatability. Published ZUPT foot-INS
implementations land at 1–2%; **1% is carried here as a stated assumption, not a measurement.**

The four traversals agreeing to ±0.4 m proves the pipeline is *repeatable*. Nothing in the data
proves it is *correct* — the only external check is GPS at ±15 m, far too weak to constrain a 1%
bias. **Tape-measuring leg 2** (a single straight ~50 m corridor) would convert that assumption into
a measurement and collapse the total uncertainty to about ±0.4 m. It is the one measurement left
worth making.

Four independent one-way traversals contribute: two separate foot recordings, plus the two halves
of a single continuous round trip. They are analysed by the same pipeline but share no data.

| Traversal | Path | Straight line |
|---|---|---|
| Separate recording, A→B | 67.7 m | 53.4 m |
| Separate recording, B→A | 66.2 m | 54.0 m |
| Round trip, A→B half | 66.1 m | 53.2 m |
| Round trip, B→A half | 65.3 m | 55.4 m |

The route is **four** straight corridor legs joined by three corners, including a short final
segment of about 1.5 m just before B. All four traversals shown in A→B order:

| Leg | Separate A→B | Separate B→A | Round trip A→B | Round trip B→A | Spread |
|---|---|---|---|---|---|
| 1 | 6.2 m | 6.3 m | 9.4 m | 7.6 m | 6.2 – 9.4 |
| **2 (long)** | **50.2 m** | **49.7 m** | **47.7 m** | **48.8 m** | **47.7 – 50.2 (±2.6%)** |
| 3 | 9.7 m | 8.7 m | 9.0 m | 8.9 m | 8.7 – 9.7 |
| **4 (short, into B)** | **1.6 m** | **1.5 m** | not resolved | not resolved | 1.5 – 1.6 |
| **path** | **67.7 m** | **66.2 m** | **66.1 m** | **65.3 m** | 65.3 – 67.7 |

Leg 2 carries three-quarters of the distance, is consistent to ±2.6% across all four traversals,
and is the number to trust. Legs 1 and 3 abut the endpoints where the walker pivots on the spot, so
where the route "starts" is ambiguous by a stride or two — that is where the spread lives.

**The short final leg into B** is resolved only in the two separate recordings, which both approach
B cleanly: **1.62 m at +69°** and **1.51 m at +71°** — agreeing to 7% on length and 2° on angle.
It is not resolved in either round-trip half: the A→B half walks straight in, and the B→A half's
B-end is the turnaround pivot rather than an approach. A 1–2 m leg is only one or two strides, which
is too few footprints for the main corner detector to fit a heading, so it needs the dedicated tail
test described below.

Including it moves the totals slightly: path length up (the polyline no longer straightens the
bend) and the A→B chord down by about 0.5 m.

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
all four traversals agree on the main leg structure — chosen for that stability, not tuned to a
target distance.

**Short final segment.** The main detector cannot see a leg of one or two strides: it has too few
footprints after the turn to establish a new heading, and it falls below the 3 m minimum-leg floor.
A dedicated tail test looks for the largest heading change within 4 m of the end, measured by
*straight-line displacement* rather than path distance — a walker pivoting on the spot accumulates
path while going nowhere and would otherwise masquerade as a short segment. The candidate is kept
only if the leg survives the line fit at ≥1 m. This correctly admits the real 1.5–1.6 m segment in
both clean approaches and rejects the 0.4 m pivot artifact in the round trip.

Reproduce with `analysis/results.py`, `analysis/roundtrip.py` and `analysis/plots.py`; per-setting
numbers are in `distance_results.csv` and `roundtrip_results.csv`.

## Evidence

| Check | Result | Why it counts |
|---|---|---|
| Four independent traversals | path 65.3–67.7 m; chord 53.2–55.4 m | separate walks, separate integrations |
| **Round-trip loop closure** | 12.9 m → **2.7 m (2.1%)** after fitting one drift rate | start and end are the *same physical point* — external truth |
| Per-leg agreement | long leg 47.7–50.2 m across all four | each leg measured four times, independently |
| Short final segment | 1.62 m @ +69° vs 1.51 m @ +71° | two clean approaches agree to 7% and 2° |
| Stance-threshold sensitivity | ±2.4% across a 1.6× threshold range | the answer is not a tuning artifact |
| Corner-threshold plateau | 36–44° all give 3 legs, identical lengths | segmentation is not tuned to a target |
| Vertical drift | ≤1.15 m over 79–91 s | truth is ≈0 on a level route — bounds attitude and bias error |
| **GPS endpoint separation** | **56.0 m** vs 54.2 m inertial — **3% apart** | **entirely different sensor system** |
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
| `figures/final_approach.png` | zoom on the last 14 m into B, showing the short final segment |
| `figures/route_geometry.png` | the two separate trajectories, fitted polyline, leg lengths, turn angles |
| `figures/waypoint_detection.png` | heading vs distance with detected corners — the corner picks are auditable |
| `figures/leg_comparison.png` | per-leg length, each direction |
| `figures/stance_diagnostics.png` | gyro with stance phases shaded; stride-length distribution |
| `figures/method_comparison.png` | every estimate with its interval |
| `figures/gps_overlay.png` | GPS fixes with accuracy discs against the endpoints |
| `figures/roundtrip_closure.png` | round trip before/after heading-drift correction |
