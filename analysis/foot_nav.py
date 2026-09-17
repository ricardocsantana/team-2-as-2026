"""Foot-mounted ZUPT inertial navigation + waypoint/polyline fitting.

The foot is stationary during the stance phase of every stride, which gives a
zero-velocity update roughly twice a second.  That removes the need for any
step-length model: distance comes straight out of the mechanisation.

Route geometry is recovered by fitting straight legs between detected corners,
which strips the stride-to-stride heading wander out of the trajectory.
"""

import io
import os
import zipfile

import numpy as np
import pandas as pd

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FOOT_DIR = os.path.join(ROOT, "foot")
DT = 0.01

# Stance thresholds: (max |gyro| rad/s, max |user accel| m/s^2).
# Reported across all three so the spread becomes the error bar.
STANCE_SETS = [(0.35, 1.2), (0.5, 2.0), (0.8, 3.0)]
STANCE_DEFAULT = (0.8, 3.0)
MIN_STANCE_SAMPLES = 8  # 80 ms


def foot_files():
    return sorted(f for f in os.listdir(FOOT_DIR) if f.endswith(".zip"))


def load(fname, folder=None):
    """Read one Sensor Logger zip into aligned arrays."""
    z = zipfile.ZipFile(os.path.join(folder or FOOT_DIR, fname))

    def csv(name):
        return pd.read_csv(io.BytesIO(z.read(name)))

    acc, gyr, ori = csv("Accelerometer.csv"), csv("Gyroscope.csv"), csv("Orientation.csv")
    n = min(len(acc), len(gyr), len(ori))
    out = {
        "n": n,
        "t": acc["seconds_elapsed"].to_numpy()[:n],
        "acc": acc[["x", "y", "z"]].to_numpy()[:n],
        "gyr": gyr[["x", "y", "z"]].to_numpy()[:n],
        "quat": ori[["qw", "qx", "qy", "qz"]].to_numpy()[:n],
        "yaw": ori["yaw"].to_numpy()[:n],
    }
    try:
        out["loc"] = csv("Location.csv")
    except KeyError:
        out["loc"] = None
    return out


def quat_rotate(q, v):
    """Rotate body-frame vectors into the world frame."""
    w, x, y, z = q.T
    R = np.stack(
        [
            np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
            np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
            np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1),
        ],
        -2,
    )
    return np.einsum("nij,nj->ni", R, v)


def detect_stance(d, thresholds=STANCE_DEFAULT):
    """Return stance runs as (start, end) index pairs.

    Stance is low angular rate *and* low specific force sustained for >=80 ms;
    the erosion suppresses single-sample dropouts inside the swing phase.
    """
    gyr_thr, acc_thr = thresholds
    gn = np.linalg.norm(d["gyr"], axis=1)
    an = np.linalg.norm(d["acc"], axis=1)
    still = (gn < gyr_thr) & (an < acc_thr)
    still = pd.Series(still).rolling(5, center=True, min_periods=1).min().astype(bool).to_numpy()

    edge = np.diff(still.astype(int))
    starts = np.where(edge == 1)[0] + 1
    ends = np.where(edge == -1)[0] + 1
    if still[0]:
        starts = np.r_[0, starts]
    if still[-1]:
        ends = np.r_[ends, len(still)]
    return [(s, e) for s, e in zip(starts, ends) if e - s >= MIN_STANCE_SAMPLES]


def integrate(d, runs):
    """ZUPT-aided mechanisation: velocity is forced to zero at every stance.

    Each swing between two stances is integrated and then linearly de-trended so
    velocity starts and ends at zero, which cancels any constant bias exactly.
    """
    aw = quat_rotate(d["quat"], d["acc"])
    v = np.zeros((d["n"], 3))
    for k in range(len(runs) - 1):
        a0, a1 = runs[k][1], runs[k + 1][0]
        if a1 <= a0:
            continue
        seg = np.cumsum(aw[a0:a1], axis=0) * DT
        m = len(seg)
        v[a0:a1] = seg - np.outer(np.arange(m) / max(m - 1, 1), seg[-1])
    return np.cumsum(v, axis=0) * DT, v


def stance_positions(pos, runs):
    """One representative position (and sample index) per stance."""
    idx = np.array([(a + b) // 2 for a, b in runs])
    return pos[idx, :2], idx


# --------------------------------------------------------------------------
# Waypoint / polyline fitting
# --------------------------------------------------------------------------

def _course(pts, smooth=3):
    """Travel heading between consecutive footprints, unwrapped and smoothed.

    Derived from the trajectory rather than device yaw: the foot rotates through
    ~5 rad/s during swing, so integrating gyro yaw directly is meaningless here.
    """
    dxy = np.diff(pts, axis=0)
    ang = np.unwrap(np.arctan2(dxy[:, 1], dxy[:, 0]))
    if smooth > 1:
        ang = pd.Series(ang).rolling(smooth, center=True, min_periods=1).mean().to_numpy()
    return ang


def _fit_line(pts):
    """Total-least-squares line fit (PCA); both coordinates carry error."""
    c = pts.mean(axis=0)
    u = np.linalg.svd(pts - c)[2][0]
    return c, u / np.linalg.norm(u)


def _intersect(c1, u1, c2, u2, fallback):
    cross = u1[0] * u2[1] - u1[1] * u2[0]
    if abs(cross) < 1e-6:           # near-parallel: no stable intersection
        return fallback
    d = c2 - c1
    t = (d[0] * u2[1] - d[1] * u2[0]) / cross
    p = c1 + t * u1
    if np.linalg.norm(p - fallback) > 15.0:   # implausible corner, keep the join
        return fallback
    return p


def find_corners(pts, turn_thr_deg=40.0, persist=2, min_leg_m=3.0):
    """Indices into `pts` where the route turns.

    The 40 deg default is the centre of a 36-44 deg plateau over which all four
    independent traversals (two separate recordings, two halves of the round
    trip) agree on 3 legs with the same lengths -- chosen for that stability,
    not tuned to a target distance.

    A corner must be a sustained heading change, and both legs it joins must
    cover real ground -- that rejects the end-of-recording pivot where the
    walker turns on the spot after arriving.
    """
    if len(pts) < 6:
        return []
    ang = _course(pts)
    thr = np.deg2rad(turn_thr_deg)

    # Heading change across a window centred on each footprint.
    cand = []
    for i in range(persist, len(ang) - persist):
        before = ang[max(0, i - 4):i].mean()
        after = ang[i + 1:min(len(ang), i + 5)].mean()
        if abs(after - before) > thr:
            cand.append((i + 1, abs(after - before)))
    if not cand:
        return []

    # Collapse adjacent candidates, keeping the sharpest of each cluster.
    corners, group = [], [cand[0]]
    for c in cand[1:]:
        if c[0] - group[-1][0] <= 3:
            group.append(c)
        else:
            corners.append(max(group, key=lambda g: g[1])[0])
            group = [c]
    corners.append(max(group, key=lambda g: g[1])[0])

    # Drop corners that would create a leg with no real travel.
    kept = []
    bounds = [0] + corners + [len(pts) - 1]
    for j, c in enumerate(corners):
        prev_len = np.linalg.norm(pts[c] - pts[bounds[j]])
        next_len = np.linalg.norm(pts[bounds[j + 2]] - pts[c])
        if prev_len >= min_leg_m and next_len >= min_leg_m:
            kept.append(c)
    return kept


def fit_polyline(pts, corners):
    """Straight legs between corners, joined at their intersections."""
    bounds = [0] + list(corners) + [len(pts) - 1]
    segs = []
    for a, b in zip(bounds[:-1], bounds[1:]):
        block = pts[a:b + 1]
        if len(block) >= 2:
            segs.append(_fit_line(block))
        else:
            segs.append((block.mean(axis=0), np.array([1.0, 0.0])))

    # Endpoints are the start/end footprints projected onto their own leg.
    first = segs[0]
    way = [first[0] + np.dot(pts[0] - first[0], first[1]) * first[1]]
    for k in range(len(segs) - 1):
        way.append(_intersect(*segs[k], *segs[k + 1], fallback=pts[bounds[k + 1]]))
    last = segs[-1]
    way.append(last[0] + np.dot(pts[-1] - last[0], last[1]) * last[1])

    way = np.array(way)
    legs = np.linalg.norm(np.diff(way, axis=0), axis=1)
    turns = []
    for k in range(len(way) - 2):
        v1 = way[k + 1] - way[k]
        v2 = way[k + 2] - way[k + 1]
        turns.append(np.degrees(np.arctan2(
            v1[0] * v2[1] - v1[1] * v2[0], v1 @ v2)))
    return way, legs, np.array(turns), segs


def solve(fname, thresholds=STANCE_DEFAULT, folder=None, **corner_kw):
    """Full pipeline for one recording."""
    d = load(fname, folder)
    runs = detect_stance(d, thresholds)
    pos, vel = integrate(d, runs)
    pts, idx = stance_positions(pos, runs)
    corners = find_corners(pts, **corner_kw)
    way, legs, turns, segs = fit_polyline(pts, corners)

    strides = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    strides = strides[strides > 0.2]
    return {
        "name": fname,
        "data": d,
        "runs": runs,
        "pos": pos,
        "pts": pts,
        "stance_idx": idx,
        "corners": corners,
        "waypoints": way,
        "legs": legs,
        "turns": turns,
        "segments": segs,
        "raw_path": float(np.sum(np.linalg.norm(np.diff(pos[:, :2], axis=0), axis=1))),
        "raw_chord": float(np.linalg.norm(pos[-1, :2])),
        "poly_path": float(legs.sum()),
        "poly_chord": float(np.linalg.norm(way[-1] - way[0])),
        "vertical": float(pos[-1, 2]),
        "strides": strides,
        "duration": float(d["t"][-1]),
    }
