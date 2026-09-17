"""Fold the foot-mounted and keypoint-leg recordings into organized/ with the
same layout, naming and QC pass used for trips 1-3."""

import io
import os
import sys
import zipfile

import numpy as np
import pandas as pd

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import foot_nav as fn

OUT = os.path.join(fn.ROOT, "organized")

SENSORS = {
    "Accelerometer": ("accelerometer", "m/s^2"),
    "AccelerometerUncalibrated": ("accelerometer_uncalibrated", "g"),
    "Gravity": ("gravity", "m/s^2"),
    "Gyroscope": ("gyroscope", "rad/s"),
    "GyroscopeUncalibrated": ("gyroscope_uncalibrated", "rad/s"),
    "Magnetometer": ("magnetometer", "uT"),
    "MagnetometerUncalibrated": ("magnetometer_uncalibrated", "uT"),
    "Orientation": ("orientation", "rad / unit quaternion"),
    "Compass": ("compass", "deg"),
    "Location": ("location", "deg / m"),
}

SOURCES = [
    ("foot", "Link_ping_University-2026-09-17_08-24-55.zip", "foot1_outbound", "A->B"),
    ("foot", "Link_ping_University-2026-09-17_08-27-06.zip", "foot2_return", "B->A"),
    ("point-to-point-back", "2026-09-17_08-00-41.zip", "keyleg1", "B->A leg 1"),
    ("point-to-point-back", "2026-09-17_08-00-56.zip", "keyleg2", "B->A leg 2"),
    ("point-to-point-back", "2026-09-17_08-01-38.zip", "keyleg3", "B->A leg 3"),
    ("foot_roundtrip", "Link_ping_University-2026-09-17_08-54-13.zip",
     "foot3_roundtrip", "A->B->A"),
]

qc, manifest = [], []

for folder, zname, out_name, direction in SOURCES:
    z = zipfile.ZipFile(os.path.join(fn.ROOT, folder, zname))
    names = set(z.namelist())
    dst = os.path.join(OUT, out_name)
    os.makedirs(dst, exist_ok=True)

    meta = pd.read_csv(io.BytesIO(z.read("Metadata.csv"))).iloc[0]
    dur = None
    for sname, (oname, unit) in SENSORS.items():
        fn_csv = f"{sname}.csv"
        if fn_csv not in names or z.getinfo(fn_csv).file_size == 0:
            continue
        df = pd.read_csv(io.BytesIO(z.read(fn_csv)))
        if df.empty:
            continue
        t_ns = df["time"].to_numpy(dtype="int64")
        keep = [c for c in ["x", "y", "z", "qw", "qx", "qy", "qz", "roll", "pitch", "yaw",
                            "magneticBearing", "latitude", "longitude", "altitude",
                            "horizontalAccuracy", "verticalAccuracy"] if c in df.columns]
        out = pd.DataFrame({
            "timestamp_utc": pd.to_datetime(t_ns, unit="ns", utc=True).strftime("%Y-%m-%dT%H:%M:%S.%f"),
            "time_ns": t_ns,
            "seconds_elapsed": df["seconds_elapsed"].to_numpy(),
        })
        for c in keep:
            out[c] = df[c].to_numpy()
        out.to_csv(f"{dst}/{oname}.csv", index=False)

        dt = np.diff(t_ns) / 1e9
        if sname == "Accelerometer":
            dur = (t_ns[-1] - t_ns[0]) / 1e9
        qc.append(dict(
            recording=out_name, sensor=oname, unit=unit, n_samples=len(df),
            duration_s=round((t_ns[-1] - t_ns[0]) / 1e9, 3),
            rate_hz=round(1 / np.median(dt), 3) if len(dt) and np.median(dt) > 0 else np.nan,
            dt_min_ms=round(dt.min() * 1e3, 3) if len(dt) else np.nan,
            dt_max_ms=round(dt.max() * 1e3, 3) if len(dt) else np.nan,
            gaps_gt_50ms=int((dt > 0.05).sum()),
            duplicate_timestamps=int((dt == 0).sum()),
            n_missing=int(df.isna().sum().sum()),
            monotonic_time=bool((dt >= 0).all()),
        ))

    with open(f"{dst}/metadata.csv", "wb") as fh:
        fh.write(z.read("Metadata.csv"))

    manifest.append(dict(
        recording=out_name, direction=direction,
        mount=("foot-mounted" if folder.startswith("foot") else "hand-carried"),
        source=f"{folder}/{zname}", duration_s=round(dur, 2) if dur else None,
        device=meta["device name"], app_version=meta["appVersion"],
        has_location="Location.csv" in names and z.getinfo("Location.csv").file_size > 0,
    ))

pd.DataFrame(qc).to_csv(f"{OUT}/qc_summary_new_recordings.csv", index=False)
pd.DataFrame(manifest).to_csv(f"{OUT}/recordings_manifest.csv", index=False)
print(pd.DataFrame(manifest).to_string(index=False))
q = pd.DataFrame(qc)
print(f"\nQC over {len(q)} streams: gaps>50ms={q.gaps_gt_50ms.sum()}, "
      f"dupes={q.duplicate_timestamps.sum()}, NaNs={q.n_missing.sum()}, "
      f"all monotonic={q.monotonic_time.all()}")
