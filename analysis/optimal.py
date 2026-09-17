"""Optimal combination of all distance measurements, with an error budget.

Averaging the four traversals equally is wrong for two reasons: the two halves
of the round trip share a recording and a heading-drift fit, so they are not
independent samples; and the polyline estimator occasionally collapses when
corner detection fails, which a plain mean would quietly absorb.

This picks the robust estimator, fuses the genuinely independent measurements,
and separates error that averages down from error that does not.
"""

import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_nav as fn
import roundtrip as rt

# Measured systematics
# Accelerometer static gain, measured on each foot recording's stationary lead-in:
# +0.035%, +0.028%, -0.038% -> mean +0.009%, spread +-0.04%.  Negligible, and it
# is NOT the 0.445% seen in the earlier cart session (different session/window).
SCALE_RESIDUAL = 0.0004

# Method-level accuracy of ZUPT foot-INS that has never been checked against a
# measured length.  The static gain above validates the accelerometer, not the
# dynamic integration: stance-timing, foot roll during "stance", and the linear
# de-trend standing in for a full EKF all bias distance without showing up in
# repeatability.  Published implementations land at 1-2%; 1% is taken here as a
# stated ASSUMPTION, not a measurement.  A tape-measured leg would remove it.
METHOD_ACCURACY = 0.01
GPS_CHORD, GPS_SIGMA = 56.0, 15.0   # indoor fixes, 14-55 m reported accuracy


def collect():
    rows = []
    for thr in [(0.5, 2.0), (0.8, 3.0)]:
        r = rt.analyse(thr)
        pts, k = r["pts"], r["turn_idx"]
        trav = [("sep-out", fn.solve(fn.foot_files()[0], thresholds=thr)["pts"], "sep-A"),
                ("sep-ret", fn.solve(fn.foot_files()[1], thresholds=thr)["pts"][::-1], "sep-B"),
                ("rt-out", pts[:k + 1], "rt"),
                ("rt-ret", pts[k:][::-1], "rt")]
        for nm, P, block in trav:
            _, way, legs, _, _ = fn.fit_route(P)
            raw_path = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
            poly_path = float(legs.sum())
            rows.append(dict(
                thr=thr[0], traversal=nm, block=block, n_legs=len(legs),
                raw_path=raw_path, poly_path=poly_path,
                raw_chord=float(np.linalg.norm(P[-1] - P[0])),
                poly_chord=float(np.linalg.norm(way[-1] - way[0])),
                # A polyline far below the raw path means the fit collapsed.
                seg_ok=bool(poly_path > 0.95 * raw_path)))
    return pd.DataFrame(rows)


def fuse(df, col, label, drop_blocks=()):
    """Block-average first (correlated measurements), then combine blocks."""
    ok = df[df.seg_ok] if col.startswith("poly") else df
    if drop_blocks:
        ok = ok[~ok.block.isin(drop_blocks)]
    block_means = ok.groupby("block")[col].mean()
    n_blocks = len(block_means)
    est = block_means.mean()

    # Random: scatter between independent blocks, not between all 8 rows.
    sd_between = block_means.std(ddof=1) if n_blocks > 1 else 0.0
    sem = sd_between / np.sqrt(n_blocks)

    # Systematic: threshold sensitivity within a traversal, plus scale factor.
    swing = ok.groupby("traversal")[col].apply(lambda x: x.max() - x.min()).mean() / 2
    scale = est * SCALE_RESIDUAL
    syst = np.hypot(swing, scale)

    total = np.hypot(sem, syst)
    return dict(quantity=label, estimate=est, n_blocks=n_blocks,
                sd_between=sd_between, random=sem, thr_syst=swing,
                scale_syst=scale, systematic=syst, total=total)


if __name__ == "__main__":
    df = collect()
    print("### All measurements ###")
    print(df.round(2).to_string(index=False))

    bad = df[~df.seg_ok]
    if len(bad):
        print("\nSegmentation failures excluded from polyline estimates "
              "(polyline < 95% of raw path — the fit collapsed):")
        print(bad[["thr", "traversal", "n_legs", "raw_path", "poly_path"]].round(2).to_string(index=False))

    print("\n### Estimator robustness ###")
    for col in ["raw_path", "poly_path", "raw_chord", "poly_chord"]:
        v = df[col]
        swing = df.groupby("traversal")[col].apply(lambda x: x.max() - x.min()).mean()
        print(f"   {col:11s} sd across traversals {v.std(ddof=1):5.2f} m | "
              f"mean threshold swing {swing:5.2f} m")
    print("   -> raw path is the robust choice; the polyline depends on corner detection")

    DROP = tuple(sys.argv[1:])           # e.g. `python optimal.py rt`
    if DROP:
        print(f"\n*** EXCLUDING BLOCK(S): {', '.join(DROP)} ***")
    res = pd.DataFrame([
        fuse(df, "raw_path", "path length (raw, robust)", DROP),
        fuse(df, "poly_path", "path length (polyline)", DROP),
        fuse(df, "raw_chord", "straight-line A-B (raw)", DROP),
        fuse(df, "poly_chord", "straight-line A-B (polyline)", DROP),
    ])
    print("\n### Optimal estimates — blocks averaged first, then combined ###")
    print("(blocks: sep-A, sep-B, rt — the two round-trip halves share one recording")
    print(" and one drift fit, so they count once, not twice)\n")
    for _, r in res.iterrows():
        print(f"   {r['quantity']:30s} {r['estimate']:6.2f} ± {r['total']:.2f} m"
              f"   (random {r['random']:.2f}, systematic {r['systematic']:.2f})")

    # Fuse the two path estimators: they bracket the truth (raw includes stride
    # wander, polyline straightens real bends), so take the midpoint.
    rp = res.iloc[0]
    pp = res.iloc[1]
    path = (rp.estimate + pp.estimate) / 2
    model = abs(rp.estimate - pp.estimate) / 2
    path_err = np.hypot(np.hypot(rp.total, pp.total) / 2, model)

    rc, pc = res.iloc[2], res.iloc[3]
    chord_imu = (rc.estimate + pc.estimate) / 2
    chord_imu_err = np.hypot(np.hypot(rc.total, pc.total) / 2,
                             abs(rc.estimate - pc.estimate) / 2)

    # Inverse-variance fusion with GPS.
    w_i, w_g = 1 / chord_imu_err**2, 1 / GPS_SIGMA**2
    chord = (chord_imu * w_i + GPS_CHORD * w_g) / (w_i + w_g)
    chord_err = np.sqrt(1 / (w_i + w_g))

    print("\n" + "=" * 68)
    print("  OPTIMAL ESTIMATE")
    print("=" * 68)
    print(f"  Route path length     {path:5.1f} ± {path_err:.1f} m   "
          f"({path_err/path*100:.1f}%)")
    print(f"  Straight-line A->B    {chord:5.1f} ± {chord_err:.1f} m   "
          f"({chord_err/chord*100:.1f}%)")
    print(f"\n  path: raw {rp.estimate:.1f} and polyline {pp.estimate:.1f} bracket the truth")
    print(f"        (raw counts stride wander, polyline straightens real bends)")
    print(f"        model spread contributes ±{model:.2f} m")
    print(f"  chord: IMU {chord_imu:.1f} ± {chord_imu_err:.1f}, GPS {GPS_CHORD:.1f} ± {GPS_SIGMA:.0f}")
    print(f"         GPS weight = {w_g/(w_i+w_g)*100:.1f}% — it corroborates but barely moves the estimate")
    print("\n" + "-" * 68)
    print("  PRECISION vs ACCURACY — the distinction that matters here")
    print("-" * 68)
    m_path, m_chord = path * METHOD_ACCURACY, chord * METHOD_ACCURACY
    print(f"  Precision (repeatability the data proves):")
    print(f"     path  {path:5.1f} +- {path_err:.1f} m      chord {chord:5.1f} +- {chord_err:.1f} m")
    print(f"  Accuracy (adds {METHOD_ACCURACY*100:.0f}% unvalidated method bias — an ASSUMPTION):")
    print(f"     path  {path:5.1f} +- {np.hypot(path_err, m_path):.1f} m      "
          f"chord {chord:5.1f} +- {np.hypot(chord_err, m_chord):.1f} m")
    print(f"\n  The four traversals agree to +-0.4 m, so the pipeline is repeatable.")
    print(f"  Nothing in the data proves it is CORRECT: the only external check is")
    print(f"  GPS at +-15 m, far too weak to constrain a 1% bias.")
    print(f"\n  To make the accuracy term measured rather than assumed: tape-measure")
    print(f"  leg 2 (a single straight ~50 m corridor). That one number would collapse")
    print(f"  the total uncertainty to roughly +-0.4 m.")
    res.to_csv(os.path.join(fn.ROOT, "organized", "optimal_estimate.csv"), index=False)
