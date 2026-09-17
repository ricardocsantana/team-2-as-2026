"""Round-trip A->B->A in a single recording.

One continuous heading frame and the same physical start/end point, so the
closure residual is a direct measurement of drift rather than a frame mismatch.
The turnaround at B is the longest mid-recording stance.
"""

import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_nav as fn

RT_DIR = os.path.join(fn.ROOT, "foot_roundtrip")
RT_FILE = sorted(f for f in os.listdir(RT_DIR) if f.endswith(".zip"))[0]


def split_at_turnaround(d, runs, edge_guard=8.0):
    """Index of the stance where the walker turned around at B.

    The longest stance that is not the lead-in or the tail: standing still to
    pivot takes seconds, whereas a walking stance lasts a few hundred ms.
    """
    best, best_len = None, 0
    for k, (a, b) in enumerate(runs):
        t0, t1 = d["t"][a], d["t"][min(b, d["n"] - 1)]
        if t0 < edge_guard or t1 > d["t"][-1] - edge_guard:
            continue
        if (b - a) > best_len:
            best, best_len = k, b - a
    return best, best_len / 100


def fit_heading_drift(pts, t_stance, limit=0.01):
    """Constant heading-drift rate that best closes the loop.

    Gyro heading drift is the dominant error in foot-mounted INS and the ZUPT
    de-trend does nothing about it.  A round trip returns to its own start, so
    that known constraint identifies the rate: one unknown against two equations
    (x and y), which leaves a residual degree of freedom -- the fit can still
    fail, so the residual is worth reporting.

    Path length is invariant under this rotation; only geometry changes.
    """
    d = np.diff(pts, axis=0)
    tm = t_stance[:-1]

    def closure(w):
        c, s = np.cos(-w * tm), np.sin(-w * tm)
        return np.hypot((c * d[:, 0] - s * d[:, 1]).sum(),
                        (s * d[:, 0] + c * d[:, 1]).sum())

    grid = np.linspace(-limit, limit, 4001)
    w = grid[np.argmin([closure(g) for g in grid])]
    c, s = np.cos(-w * tm), np.sin(-w * tm)
    dc = np.c_[c * d[:, 0] - s * d[:, 1], s * d[:, 0] + c * d[:, 1]]
    return w, np.vstack([[0, 0], np.cumsum(dc, axis=0)]), closure(w), closure(0.0)


def analyse(thresholds=fn.STANCE_DEFAULT):
    d = fn.load(RT_FILE, RT_DIR)
    runs = fn.detect_stance(d, thresholds)
    pos, _ = fn.integrate(d, runs)
    pts_raw, idx = fn.stance_positions(pos, runs)

    t_stance = np.array([d["t"][a] for a, _ in runs])
    w, pts, closed, uncorrected = fit_heading_drift(pts_raw, t_stance)

    k, pause = split_at_turnaround(d, runs)
    out_pts, ret_pts = pts[:k + 1], pts[k:]

    res = {"file": RT_FILE, "duration": float(d["t"][-1]), "strides": len(runs),
           "turn_idx": k, "turn_time": float(d["t"][runs[k][0]]), "pause_s": pause,
           "pos": pos, "pts": pts, "pts_raw": pts_raw, "runs": runs, "data": d,
           "drift_deg_min": float(np.degrees(w) * 60),
           "closure_corrected": float(closed), "closure_uncorrected": float(uncorrected)}

    for tag, P in (("out", out_pts), ("ret", ret_pts)):
        corners = fn.find_corners(P)
        way, legs, turns, _ = fn.fit_polyline(P, corners)
        res[tag] = dict(pts=P, waypoints=way, legs=legs, turns=turns,
                        poly_path=float(legs.sum()),
                        poly_chord=float(np.linalg.norm(way[-1] - way[0])),
                        raw_path=float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1))),
                        raw_chord=float(np.linalg.norm(P[-1] - P[0])))

    res["raw_path_out"] = float(np.sum(np.linalg.norm(np.diff(out_pts, axis=0), axis=1)))
    res["raw_path_ret"] = float(np.sum(np.linalg.norm(np.diff(ret_pts, axis=0), axis=1)))
    res["closure_m"] = float(np.linalg.norm(pts[-1] - pts[0]))
    res["closure_pct"] = res["closure_m"] / (res["out"]["poly_path"] + res["ret"]["poly_path"]) * 100
    res["vertical"] = float(pos[-1, 2])
    strides = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    res["stride_med"] = float(np.median(strides[strides > 0.2]))
    return res


if __name__ == "__main__":
    print("### ROUND TRIP A->B->A, single recording ###\n")
    rows = []
    for thr in fn.STANCE_SETS:
        r = analyse(thr)
        rate = r["strides"] / r["duration"]
        valid = 0.50 <= rate <= 1.20
        rows.append(dict(gyro_thr=thr[0], acc_thr=thr[1], strides=r["strides"],
                         stride_hz=round(rate, 2), valid=valid,
                         turnaround_s=round(r["turn_time"], 1), pause_s=round(r["pause_s"], 1),
                         out_path=round(r["out"]["poly_path"], 1),
                         ret_path=round(r["ret"]["poly_path"], 1),
                         out_chord=round(r["out"]["poly_chord"], 1),
                         ret_chord=round(r["ret"]["poly_chord"], 1),
                         raw_out=round(r["raw_path_out"], 1),
                         raw_ret=round(r["raw_path_ret"], 1),
                         drift_deg_min=round(r["drift_deg_min"], 2),
                         closure_m=round(r["closure_corrected"], 2),
                         closure_uncorr_m=round(r["closure_uncorrected"], 2),
                         closure_pct=round(r["closure_corrected"] /
                                           (r["raw_path_out"] + r["raw_path_ret"]) * 100, 2),
                         vertical_m=round(r["vertical"], 2),
                         stride_m=round(r["stride_med"], 3)))
    df = pd.DataFrame(rows)
    df.to_csv(os.path.join(fn.ROOT, "organized", "roundtrip_results.csv"), index=False)
    print(df.to_string(index=False))

    ok = df[df.valid]
    r = analyse()
    print(f"\n   turnaround at B: t = {r['turn_time']:.1f}s, stood still {r['pause_s']:.1f}s")
    print(f"   outbound legs : {np.round(r['out']['legs'], 1)}  turns {np.round(r['out']['turns'], 0)}")
    print(f"   return legs   : {np.round(r['ret']['legs'], 1)}  turns {np.round(r['ret']['turns'], 0)}")
    print("\n=== LOOP CLOSURE (same physical point, one continuous heading frame) ===")
    print(f"   heading drift fitted : {ok.drift_deg_min.mean():+.2f} deg/min")
    print(f"   closure before/after : {ok.closure_uncorr_m.mean():.2f} m -> {ok.closure_m.mean():.2f} m")
    print(f"   residual as % of round trip: {ok.closure_pct.min():.2f}-{ok.closure_pct.max():.2f}%")
    print("\n=== DISTANCE ===")
    paths = np.r_[ok.raw_out.values, ok.raw_ret.values]
    chords = np.r_[ok.out_chord.values, ok.ret_chord.values]
    print(f"   path length   {paths.mean():5.1f} m  (range {paths.min():.1f}-{paths.max():.1f})")
    print(f"   straight line {chords.mean():5.1f} m  (range {chords.min():.1f}-{chords.max():.1f})")
    print(f"   vertical drift {ok.vertical_m.abs().max():.2f} m over {r['duration']:.0f}s (truth ~0)")
    print(f"   stride {ok.stride_m.mean():.2f} m -> step {ok.stride_m.mean()/2:.2f} m")
