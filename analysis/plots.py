"""Figures for the A<->B distance analysis."""

import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_nav as fn

OUT = os.path.join(fn.ROOT, "organized", "figures")
os.makedirs(OUT, exist_ok=True)

# Validated categorical slots 1-3 (all-pairs safe in light mode).
C_OUT, C_RET, C_GPS = "#2a78d6", "#eb6834", "#1baf7a"
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#b8b7b2"

plt.rcParams.update({
    "figure.facecolor": "#fcfcfb", "axes.facecolor": "#fcfcfb",
    "axes.edgecolor": MUTED, "axes.labelcolor": INK2, "text.color": INK,
    "xtick.color": INK2, "ytick.color": INK2,
    "axes.spines.top": False, "axes.spines.right": False,
    "grid.color": MUTED, "grid.linewidth": 0.4, "grid.alpha": 0.5,
    "font.size": 9, "axes.titlesize": 10, "figure.dpi": 140,
})


def kabsch_2d(A, B):
    """Rotation (no scale) taking A onto B, both n x 2 and pre-centred."""
    H = A.T @ B
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    return Vt.T @ np.diag([1, d]) @ U.T


def aligned(out, ret):
    """Put the return recording into the outbound's frame for comparison.

    Rotation only - scale is a measurement here and must never be fitted away.
    """
    a = out["waypoints"] - out["waypoints"][0]
    b = ret["waypoints"][::-1] - ret["waypoints"][::-1][0]
    m = min(len(a), len(b))       # leg counts can differ by one (short tail segment)
    R = kabsch_2d(b[:m], a[:m])
    ang = np.degrees(np.arctan2(R[1, 0], R[0, 0]))
    pts = (ret["pts"][::-1] - ret["waypoints"][::-1][0]) @ R.T
    way = b @ R.T
    return pts, way, ang


def fig_route(out, ret):
    pts_r, way_r, ang = aligned(out, ret)
    pts_o = out["pts"] - out["waypoints"][0]
    way_o = out["waypoints"] - out["waypoints"][0]

    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    ax.plot(pts_o[:, 0], pts_o[:, 1], ".", ms=3.5, color=C_OUT, alpha=0.45, zorder=2)
    ax.plot(pts_r[:, 0], pts_r[:, 1], ".", ms=3.5, color=C_RET, alpha=0.45, zorder=2)
    ax.plot(way_o[:, 0], way_o[:, 1], "-", lw=2, color=C_OUT, zorder=4,
            label=f"outbound A→B  ({out['poly_path']:.1f} m)")
    ax.plot(way_r[:, 0], way_r[:, 1], "-", lw=2, color=C_RET, zorder=4,
            label=f"return B→A  ({ret['poly_path']:.1f} m)")
    for w, c in ((way_o, C_OUT), (way_r, C_RET)):
        ax.plot(w[:, 0], w[:, 1], "o", ms=8, mfc="#fcfcfb", mec=c, mew=2, zorder=5)

    ax.plot([way_o[0, 0], way_o[-1, 0]], [way_o[0, 1], way_o[-1, 1]],
            "--", lw=1.4, color=INK2, zorder=3,
            label=f"straight line A–B  ({out['poly_chord']:.1f} m)")

    for w, lab, dy in ((way_o[0], "A", -2.2), (way_o[-1], "B", 1.6)):
        ax.annotate(lab, w, xytext=(0, dy * 6), textcoords="offset points",
                    ha="center", fontsize=12, fontweight="bold", color=INK)
    # Offset each label perpendicular to its own leg so nothing sits on a line.
    for i, L in enumerate(out["legs"]):
        m = (way_o[i] + way_o[i + 1]) / 2
        d = way_o[i + 1] - way_o[i]
        perp = np.array([d[1], -d[0]])
        perp = perp / np.linalg.norm(perp) * 26
        ax.annotate(f"{L:.1f} m", m, xytext=perp, textcoords="offset points",
                    ha="center", va="center", fontsize=9.5, color=C_OUT,
                    fontweight="bold",
                    bbox=dict(boxstyle="round,pad=0.22", fc="#fcfcfb", ec="none", alpha=0.85))
    for i, t in enumerate(out["turns"]):
        v1 = way_o[i + 1] - way_o[i]
        v2 = way_o[i + 2] - way_o[i + 1]
        bis = v1 / np.linalg.norm(v1) - v2 / np.linalg.norm(v2)
        bis = bis / np.linalg.norm(bis) * 34
        ax.annotate(f"{t:+.0f}°", way_o[i + 1], xytext=bis, textcoords="offset points",
                    ha="center", va="center", fontsize=9, color=INK2,
                    bbox=dict(boxstyle="round,pad=0.2", fc="#fcfcfb", ec="none", alpha=0.85))

    ax.set_xlabel("east–west (m)")
    ax.set_ylabel("north–south (m)")
    ax.set_title("Route geometry from foot-mounted ZUPT navigation\n"
                 f"dots = individual footprints · return rotated {ang:+.0f}° into the outbound frame (no scaling)",
                 loc="left")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend(frameon=False, loc="upper left", fontsize=8.5)
    fig.tight_layout()
    fig.savefig(f"{OUT}/route_geometry.png")
    plt.close(fig)
    return ang


def fig_waypoints(out, ret):
    fig, axes = plt.subplots(2, 1, figsize=(8.4, 5.6), sharex=False)
    for ax, r, c, name in ((axes[0], out, C_OUT, "outbound A→B"),
                           (axes[1], ret, C_RET, "return B→A")):
        d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(r["pts"], axis=0), axis=1))]
        dxy = np.diff(r["pts"], axis=0)
        head = np.degrees(np.unwrap(np.arctan2(dxy[:, 1], dxy[:, 0])))
        head = pd.Series(head).rolling(3, center=True, min_periods=1).mean().to_numpy()
        head = head - head[0]
        ax.plot(d[1:], head, "-", lw=1.6, color=c)
        for k, ci in enumerate(r["corners"]):
            ax.axvline(d[ci], color=INK2, ls="--", lw=1.2)
            ax.annotate(f"corner {k+1}\n{r['turns'][k]:+.0f}°", (d[ci], head.max()),
                        xytext=(6, -12), textcoords="offset points",
                        fontsize=8.5, color=INK2, va="top")
        ax.set_ylabel("heading (°)")
        ax.set_title(f"{name} — {len(r['corners'])} corners, legs "
                     + " / ".join(f"{L:.1f} m" for L in r["legs"]), loc="left")
        ax.grid(True)
    axes[1].set_xlabel("distance travelled (m)")
    fig.suptitle("Corner detection: heading vs distance along the route", x=0.01,
                 ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(f"{OUT}/waypoint_detection.png")
    plt.close(fig)


def fig_legs(out, ret):
    legs_o = out["legs"]
    legs_r = ret["legs"][::-1]
    n = min(len(legs_o), len(legs_r))
    x = np.arange(n)
    fig, ax = plt.subplots(figsize=(6.6, 4.0))
    w, gap = 0.38, 0.012          # surface gap between adjacent fills
    ax.bar(x - w / 2 - gap, legs_o[:n], w, color=C_OUT, label="outbound A→B", zorder=3)
    ax.bar(x + w / 2 + gap, legs_r[:n], w, color=C_RET, label="return B→A (reversed)", zorder=3)
    for i in range(n):
        ax.annotate(f"{legs_o[i]:.1f}", (i - w / 2 - gap, legs_o[i]), xytext=(0, 3),
                    textcoords="offset points", ha="center", fontsize=8.5, color=INK)
        ax.annotate(f"{legs_r[i]:.1f}", (i + w / 2 + gap, legs_r[i]), xytext=(0, 3),
                    textcoords="offset points", ha="center", fontsize=8.5, color=INK)
        diff = abs(legs_o[i] - legs_r[i]) / np.mean([legs_o[i], legs_r[i]]) * 100
        ax.annotate(f"Δ{diff:.0f}%", (i, max(legs_o[i], legs_r[i])), xytext=(0, 18),
                    textcoords="offset points", ha="center", fontsize=8, color=INK2)
    ax.set_xticks(x, [f"leg {i+1}" for i in range(n)])
    ax.set_ylabel("length (m)")
    ax.set_ylim(0, max(legs_o.max(), legs_r.max()) * 1.22)
    ax.set_title("Per-leg length, measured independently in each direction", loc="left")
    ax.grid(True, axis="y")
    ax.legend(frameon=False, fontsize=8.5)
    fig.tight_layout()
    fig.savefig(f"{OUT}/leg_comparison.png")
    plt.close(fig)


def fig_stance(out, ret):
    fig, axes = plt.subplots(2, 2, figsize=(9.2, 5.4),
                             gridspec_kw={"width_ratios": [2.3, 1]})
    for row, (r, c, name) in enumerate(((out, C_OUT, "outbound A→B"),
                                        (ret, C_RET, "return B→A"))):
        ax = axes[row, 0]
        d = r["data"]
        gn = np.linalg.norm(d["gyr"], axis=1)
        lo, hi = 0, min(len(gn), 2000)          # first 20 s detail
        ax.plot(d["t"][lo:hi], gn[lo:hi], lw=0.7, color=c)
        for a, b in r["runs"]:
            if a < hi:
                ax.axvspan(d["t"][a], d["t"][min(b, hi - 1)], color=MUTED, alpha=0.55, lw=0)
        ax.set_ylabel("|gyro| (rad/s)")
        ax.set_title(f"{name} — shaded = detected stance ({len(r['runs'])} strides)",
                     loc="left")
        ax.grid(True)

        ax = axes[row, 1]
        ax.hist(r["strides"], bins=18, color=c, zorder=3)
        med = np.median(r["strides"])
        ax.axvline(med, color=INK, lw=1.4, ls="--")
        ax.annotate(f"median {med:.2f} m\nstep {med/2:.2f} m", (0.04, 0.95),
                    xycoords="axes fraction", fontsize=8.5, color=INK, va="top")
        ax.set_ylabel("strides")
        ax.set_title("stride length", loc="left")
        ax.grid(True, axis="y")
    axes[1, 0].set_xlabel("time (s)")
    axes[1, 1].set_xlabel("stride length (m)")
    fig.suptitle("Stance detection and stride-length distribution", x=0.01, ha="left",
                 fontsize=11, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(f"{OUT}/stance_diagnostics.png")
    plt.close(fig)


def fig_methods(out, ret):
    """Every estimate on one axis, with its interval."""
    res = pd.read_csv(os.path.join(fn.ROOT, "organized", "distance_results.csv"))
    ok = res[res.valid]

    def mid_half(col):
        v = ok[col]
        return float((v.max() + v.min()) / 2), float((v.max() - v.min()) / 2)

    pp, ppe = mid_half("polyline_path_m")
    rp, rpe = mid_half("raw_path_m")
    pc, pce = mid_half("polyline_chord_m")
    rc, rce = mid_half("raw_chord_m")
    rows = [
        ("Foot ZUPT — polyline\n(this analysis)", pp, max(ppe, 1.0), C_OUT, "path"),
        ("Foot ZUPT — raw integration", rp, max(rpe, 1.0), C_OUT, "path"),
        ("Cart ZUPT — to arrival", 64.8, 7.0, C_RET, "path"),
        ("Cart ZUPT — full window", 76.4, 8.0, C_RET, "path"),
        ("Step count × step length", 84.0, 11.0, MUTED, "path"),
        ("Foot ZUPT — polyline chord", pc, max(pce, 1.0), C_OUT, "chord"),
        ("Foot ZUPT — raw chord", rc, max(rce, 1.0), C_OUT, "chord"),
        ("GPS endpoint separation", 56.0, 15.0, C_GPS, "chord"),
    ]
    fig, ax = plt.subplots(figsize=(8.0, 5.0))
    y = np.arange(len(rows))[::-1]
    for yi, (lab, v, e, c, kind) in zip(y, rows):
        ax.errorbar(v, yi, xerr=e, fmt="o", ms=8, color=c, ecolor=c,
                    elinewidth=2, capsize=4, zorder=3, alpha=0.95)
        ax.annotate(f"{v:.1f} m", (v, yi), xytext=(0, 11), textcoords="offset points",
                    ha="center", fontsize=8.5, color=INK, fontweight="bold")
    ax.set_yticks(y, [r[0] for r in rows], fontsize=8.5)
    ax.axhspan(2.5, 8.5, color=MUTED, alpha=0.18, lw=0, zorder=0)
    ax.annotate("path length", (95, 6.2), fontsize=9, color=INK2, fontweight="bold")
    ax.annotate("straight line A–B", (95, 1.4), fontsize=9, color=INK2, fontweight="bold")
    ax.set_xlabel("distance (m)")
    ax.set_xlim(35, 110)
    ax.set_title("All estimates with intervals — foot ZUPT and GPS are independent sensor systems",
                 loc="left")
    ax.grid(True, axis="x")
    fig.tight_layout()
    fig.savefig(f"{OUT}/method_comparison.png")
    plt.close(fig)


def fig_gps(out, ret):
    import io
    import zipfile
    R = 6371000.0
    fig, ax = plt.subplots(figsize=(6.8, 5.6))
    files = fn.foot_files()
    L0 = pd.read_csv(io.BytesIO(zipfile.ZipFile(
        os.path.join(fn.FOOT_DIR, files[0])).read("Location.csv")))
    lat0, lon0 = L0.latitude.iloc[0], L0.longitude.iloc[0]
    ends = []
    for f, c, name in ((files[0], C_OUT, "outbound"), (files[1], C_RET, "return")):
        L = pd.read_csv(io.BytesIO(zipfile.ZipFile(
            os.path.join(fn.FOOT_DIR, f)).read("Location.csv")))
        L = L[(L.seconds_elapsed >= 0) & (L.latitude != 0) & (L.horizontalAccuracy < 40)]
        x = np.deg2rad(L.longitude.values - lon0) * R * np.cos(np.deg2rad(lat0))
        y = np.deg2rad(L.latitude.values - lat0) * R
        for xi, yi, acc in zip(x, y, L.horizontalAccuracy.values):
            ax.add_patch(plt.Circle((xi, yi), acc, color=c, alpha=0.06, lw=0))
        ax.plot(x, y, "o-", ms=5, lw=1, color=c, alpha=0.85, label=f"{name} GPS fixes")
        ends.append((x[0], y[0]))
        ends.append((x[-1], y[-1]))
    A = np.array([ends[0], ends[3]]).mean(axis=0)
    B = np.array([ends[1], ends[2]]).mean(axis=0)
    ax.plot([A[0], B[0]], [A[1], B[1]], "--", lw=2, color=INK,
            label=f"A–B separation  {np.linalg.norm(B-A):.1f} m")
    for p, lab in ((A, "A"), (B, "B")):
        ax.plot(*p, "o", ms=11, mfc="#fcfcfb", mec=INK, mew=2, zorder=5)
        ax.annotate(lab, p, xytext=(0, 12), textcoords="offset points", ha="center",
                    fontsize=12, fontweight="bold")
    ax.set_xlabel("east (m)")
    ax.set_ylabel("north (m)")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend(frameon=False, fontsize=8.5, loc="upper left")
    ax.set_title("GPS fixes — independent check on the endpoints\n"
                 "shaded discs = reported horizontal accuracy (indoor fixes are weak)",
                 loc="left")
    fig.tight_layout()
    fig.savefig(f"{OUT}/gps_overlay.png")
    plt.close(fig)


def fig_roundtrip():
    """Round trip A->B->A: the only recording where loop closure is meaningful."""
    import roundtrip as rt
    r = rt.analyse()
    k = r["turn_idx"]
    fig, axes = plt.subplots(1, 2, figsize=(10.4, 5.4), sharex=True, sharey=True)
    for ax, P, title, resid in (
            (axes[0], r["pts_raw"], "as integrated", r["closure_uncorrected"]),
            (axes[1], r["pts"], "after heading-drift correction", r["closure_corrected"])):
        out, ret = P[:k + 1], P[k:]
        ax.plot(out[:, 0], out[:, 1], "-o", ms=3, lw=1.2, color=C_OUT, label="A→B")
        ax.plot(ret[:, 0], ret[:, 1], "-o", ms=3, lw=1.2, color=C_RET, label="B→A")
        ax.plot(*P[0], "o", ms=12, mfc="#fcfcfb", mec=INK, mew=2, zorder=5)
        ax.plot(*P[-1], "s", ms=11, mfc="#fcfcfb", mec=INK, mew=2, zorder=5)
        ax.annotate("start A", P[0], xytext=(12, 8), textcoords="offset points",
                    fontsize=9, fontweight="bold")
        ax.annotate("end", P[-1], xytext=(12, -14), textcoords="offset points", fontsize=9)
        ax.plot(*P[k], "^", ms=11, mfc="#fcfcfb", mec=INK2, mew=2, zorder=5)
        ax.annotate("turnaround B", P[k], xytext=(10, 10), textcoords="offset points",
                    fontsize=9, ha="left", color=INK2)
        ax.plot([P[0, 0], P[-1, 0]], [P[0, 1], P[-1, 1]], "--", lw=1.6, color=INK)
        ax.set_title(f"{title}\nclosure error {resid:.1f} m", loc="left")
        ax.set_aspect("equal")
        ax.grid(True)
        ax.set_xlabel("east–west (m)")
    axes[0].set_ylabel("north–south (m)")
    axes[0].legend(frameon=False, fontsize=8.5, loc="upper left")
    fig.suptitle("Round trip A→B→A — start and end are the same physical point, so the gap is pure drift",
                 x=0.01, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(f"{OUT}/roundtrip_closure.png")
    plt.close(fig)
    return r


def _traversals():
    """All four one-way traversals, footprints and polyline, in A->B order."""
    import roundtrip as rt
    files = fn.foot_files()
    o, r = fn.solve(files[0]), fn.solve(files[1])
    rtr = rt.analyse()
    k = rtr["turn_idx"]

    def rev(d):
        return dict(pts=d["pts"][::-1], way=d["waypoints"][::-1],
                    legs=d["legs"][::-1], path=d["poly_path"],
                    chord=d["poly_chord"])

    fwd = lambda d: dict(pts=d["pts"], way=d["waypoints"], legs=d["legs"],
                         path=d["poly_path"], chord=d["poly_chord"])
    return [
        ("separate recording · A→B", fwd(o), C_OUT),
        ("separate recording · B→A", rev(r), C_RET),
        ("round trip · A→B half", fwd(rtr["out"]), C_OUT),
        ("round trip · B→A half", rev(rtr["ret"]), C_RET),
    ]


def _align(t, ref_way):
    """Rotate+translate a traversal onto the reference. Rotation only — never scale."""
    a = t["way"] - t["way"][0]
    b = ref_way - ref_way[0]
    m = min(len(a), len(b))
    R = kabsch_2d(a[:m], b[:m])
    return (t["pts"] - t["way"][0]) @ R.T, (t["way"] - t["way"][0]) @ R.T


def fig_reconstructed_paths():
    T = _traversals()
    ref = T[0][1]["way"]
    fig = plt.figure(figsize=(12.6, 7.4))
    gs = fig.add_gridspec(2, 4, width_ratios=[1.45, 1.45, 1, 1], wspace=0.28, hspace=0.32)
    big = fig.add_subplot(gs[:, :2])

    for name, t, c in T:
        pts, way = _align(t, ref)
        big.plot(pts[:, 0], pts[:, 1], ".", ms=3, color=c, alpha=0.3, zorder=2)
        big.plot(way[:, 0], way[:, 1], "-", lw=2, color=c, alpha=0.9, zorder=4,
                 label=f"{name} — {t['path']:.1f} m")
        big.plot(way[:, 0], way[:, 1], "o", ms=7, mfc="#fcfcfb", mec=c, mew=1.8, zorder=5)

    _, w0 = _align(T[0][1], ref)
    big.plot([w0[0, 0], w0[-1, 0]], [w0[0, 1], w0[-1, 1]], "--", lw=1.6, color=INK, zorder=3,
             label=f"straight line A–B — {np.mean([t['chord'] for _, t, _ in T]):.1f} m")
    for p, lab, off in ((w0[0], "A", (-14, -18)), (w0[-1], "B", (-16, 10))):
        big.annotate(lab, p, xytext=off, textcoords="offset points", ha="center",
                     fontsize=14, fontweight="bold", color=INK)
    for i, L in enumerate(T[0][1]["legs"]):
        m = (w0[i] + w0[i + 1]) / 2
        d = w0[i + 1] - w0[i]
        perp = np.array([d[1], -d[0]])
        # Short legs get pushed further out, and the last one is nudged down so
        # it clears the panel subtitle.
        off = 34 if L < 3 else 30
        perp = perp / np.linalg.norm(perp) * off
        if i == len(T[0][1]["legs"]) - 1:
            perp = perp + np.array([26, -18])
        big.annotate(f"leg {i+1}\n{L:.1f} m", m, xytext=perp, textcoords="offset points",
                     ha="center", va="center", fontsize=9, color=INK, fontweight="bold",
                     bbox=dict(boxstyle="round,pad=0.25", fc="#fcfcfb", ec=MUTED, lw=0.6))
    big.set_aspect("equal")
    big.grid(True)
    big.set_xlabel("east–west (m)")
    big.set_ylabel("north–south (m)")
    big.legend(frameon=False, fontsize=8.5, loc="upper left")
    big.set_title("All four reconstructions overlaid\n"
                  "each rotated into a common frame (rotation only — scale never fitted)",
                  loc="left")

    for j, (name, t, c) in enumerate(T):
        ax = fig.add_subplot(gs[j // 2, 2 + j % 2])
        pts, way = _align(t, ref)
        ax.plot(pts[:, 0], pts[:, 1], ".", ms=2.5, color=c, alpha=0.35)
        ax.plot(way[:, 0], way[:, 1], "-o", lw=1.8, ms=5, color=c,
                mfc="#fcfcfb", mec=c, mew=1.5)
        ax.set_title(f"{name}\npath {t['path']:.1f} m · A–B {t['chord']:.1f} m",
                     loc="left", fontsize=8.5)
        ax.set_aspect("equal")
        ax.grid(True)
        ax.tick_params(labelsize=7)
        for i, L in enumerate(t["legs"]):
            m = (way[i] + way[i + 1]) / 2
            ax.annotate(f"{L:.1f}", m, xytext=(4, 4), textcoords="offset points",
                        fontsize=7.5, color=INK)

    n_legs = len(T[0][1]["legs"])
    paths = [t["path"] for _, t, _ in T]
    chords = [t["chord"] for _, t, _ in T]
    fig.suptitle(f"Reconstructed paths — {n_legs} straight legs totalling "
                 f"{np.mean(paths):.1f} m; A and B are {np.mean(chords):.1f} m apart",
                 x=0.01, ha="left", fontsize=12, fontweight="bold")
    fig.subplots_adjust(left=0.06, right=0.985, top=0.90, bottom=0.08)
    fig.savefig(f"{OUT}/reconstructed_paths.png")
    plt.close(fig)


def fig_final_approach():
    """Zoom on the last ~14 m into B, where a short segment may be unresolved."""
    T = _traversals()
    ref = T[0][1]["way"]
    fig, axes = plt.subplots(1, 4, figsize=(14.0, 5.2))
    for ax, (name, t, c) in zip(axes, T):
        pts, way = _align(t, ref)
        d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))]
        P = pts[d > d[-1] - 14]
        dd = d[d > d[-1] - 14]
        ax.plot(P[:, 0], P[:, 1], "-o", ms=5, lw=1, color=c, alpha=0.85, zorder=3)
        ax.plot(way[-2:, 0], way[-2:, 1], "--", lw=1.5, color=INK2, zorder=2,
                label=f"fitted leg {len(t['legs'])}: {t['legs'][-1]:.1f} m")
        ax.plot(*P[-1], "*", ms=18, mfc="#fcfcfb", mec=INK, mew=1.8, zorder=6)
        ax.annotate("B", P[-1], xytext=(9, 2), textcoords="offset points",
                    fontsize=13, fontweight="bold")

        v = np.diff(P, axis=0)
        ang = np.degrees(np.unwrap(np.arctan2(v[:, 1], v[:, 0])))
        turns = [(i, ang[i] - ang[i - 1]) for i in range(1, len(ang))
                 if abs(ang[i] - ang[i - 1]) > 30]
        # main corner = largest; short-segment candidate = last one inside 4 m of B
        main = max(turns, key=lambda x: abs(x[1])) if turns else None
        late = [x for x in turns if dd[-1] - dd[x[0]] < 4.0]
        late = late[0] if late else None
        for tag, item, col, dy in (("corner", main, INK2, 16), ("short seg", late, "#0f7d57", -26)):
            if item is None:
                continue
            i, turn = item
            ax.plot(*P[i], "o", ms=14, mfc="none", mec=col, mew=2.2, zorder=5)
            ax.annotate(f"{tag} {turn:+.0f}°\n{dd[-1]-dd[i]:.1f} m to B", P[i],
                        xytext=(10, dy), textcoords="offset points",
                        fontsize=8, color=col, fontweight="bold",
                        bbox=dict(boxstyle="round,pad=0.2", fc="#fcfcfb", ec="none", alpha=0.9))
        ax.set_aspect("equal")
        ax.grid(True)
        ax.tick_params(labelsize=7)
        ax.margins(0.22)
        ax.set_title(f"{name}", loc="left", fontsize=9, fontweight="bold")
        ax.set_xlabel("east–west (m)")
        ax.legend(frameon=False, fontsize=7.5, loc="lower left")
    axes[0].set_ylabel("north–south (m)")
    fig.suptitle("Final approach into B — last 14 m, individual footprints\n"
                 "green = a heading change within 4 m of B, i.e. the short segment the polyline merges away",
                 x=0.01, ha="left", fontsize=11, fontweight="bold")
    fig.tight_layout(rect=[0, 0, 1, 0.88])
    fig.savefig(f"{OUT}/final_approach.png")
    plt.close(fig)


if __name__ == "__main__":
    files = fn.foot_files()
    out, ret = fn.solve(files[0]), fn.solve(files[1])
    ang = fig_route(out, ret)
    fig_waypoints(out, ret)
    fig_legs(out, ret)
    fig_stance(out, ret)
    fig_methods(out, ret)
    fig_gps(out, ret)
    fig_roundtrip()
    fig_reconstructed_paths()
    fig_final_approach()
    print(f"wrote 9 figures to {OUT}")
    print(f"  return rotated {ang:+.1f}deg onto outbound frame")
