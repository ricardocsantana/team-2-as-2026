"""Consolidated distance results across stance thresholds -> organized/distance_results.csv"""

import os
import sys

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_nav as fn

rows = []
files = fn.foot_files()
label = {files[0]: "outbound_A_to_B", files[1]: "return_B_to_A"}

PLAUSIBLE_STRIDE_HZ = (0.50, 1.20)   # 1.0-2.4 Hz step rate; anything outside is a detector failure

for f in files:
    for thr in fn.STANCE_SETS:
        r = fn.solve(f, thresholds=thr)
        rate = len(r["runs"]) / r["duration"]
        valid = PLAUSIBLE_STRIDE_HZ[0] <= rate <= PLAUSIBLE_STRIDE_HZ[1]
        rows.append(dict(
            recording=label[f], gyro_thr=thr[0], acc_thr=thr[1],
            strides=len(r["runs"]), stride_hz=round(rate, 2), valid=valid,
            corners=len(r["corners"]),
            raw_path_m=round(r["raw_path"], 2),
            polyline_path_m=round(r["poly_path"], 2),
            raw_chord_m=round(r["raw_chord"], 2),
            polyline_chord_m=round(r["poly_chord"], 2),
            legs_m=" / ".join(f"{L:.1f}" for L in r["legs"]),
            turns_deg=" / ".join(f"{t:+.0f}" for t in r["turns"]),
            vertical_drift_m=round(r["vertical"], 2),
            median_stride_m=round(float(np.median(r["strides"])), 3),
            duration_s=round(r["duration"], 1),
        ))

df = pd.DataFrame(rows)
out = os.path.join(fn.ROOT, "organized", "distance_results.csv")
df.to_csv(out, index=False)
print(df.to_string(index=False))

bad = df[~df.valid]
if len(bad):
    print("\nEXCLUDED — stance detector failed the stride-rate plausibility gate:")
    print(bad[["recording", "gyro_thr", "acc_thr", "strides", "stride_hz",
               "median_stride_m"]].to_string(index=False))
    print("   (missed stances merge adjacent strides, inflating median stride length)")

ok = df[df.valid]
print("\n=== CONSOLIDATED (valid configurations only) ===")
for col, name in [("polyline_path_m", "polyline path length"),
                  ("raw_path_m", "raw integrated path"),
                  ("polyline_chord_m", "polyline straight-line A-B"),
                  ("raw_chord_m", "raw straight-line A-B")]:
    v = ok[col]
    print(f"   {name:28s} mean {v.mean():5.1f} m   range {v.min():5.1f}-{v.max():5.1f} m "
          f"(+-{(v.max()-v.min())/2/v.mean()*100:4.1f}%)")
s = ok["median_stride_m"]
print(f"   {'stride length':28s} mean {s.mean():5.2f} m   -> step {s.mean()/2:.2f} m")
print(f"   {"vertical drift":28s} max  {ok.vertical_drift_m.abs().max():5.2f} m  (truth ~0)")
