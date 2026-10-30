"""Fit the Livox Mid-360's scan pattern (config/sensors.json lidar.pattern) to a recording of a real one.

The recording: outdoor_kidnap_a of the Hard Point Cloud Localization Dataset (Zenodo record 10122133,
CC BY 4.0), a ROS 2 bag of livox_ros_driver2 PointCloud2 frames (x, y, z, t: ns after the frame's stamp,
line). Unzipped into <dir>, in the ROS 2 container:

    docker run --rm -v <dir>/outdoor_kidnap_a:/data:ro -v "$PWD":/project:ro ironlark-ros:humble \
        bash -c "source /opt/ros/humble/setup.bash && python3 /project/tools/fit_mid360_pattern.py \
        /data/rosbag2_2023_09_05-23_00_55_0.db3"

Only returns carry a direction (no-return slots are zeros with unusable times), so the pattern is fitted to
returns farther than 3 m (the driver's millimetres make nearer directions coarse): a slot every slot_ns,
lines 0-3 in turn; the head spins at spin_hz, from consecutive slots of one line (packets' times jitter
against each other, so not from the phase across a frame); the beams' elevation nods, a Fourier series
of nod_period_s (in windows: the motor's speed drifts); line a sits a times a fan from line 0 (elevation e0 + e1 cos az + e2 sin az, azimuth
z1 sin az + z2 cos az, degrees), measured on four consecutive slots. Prints the pattern and its residuals.
"""
import json
import sqlite3
import sys

import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

POINT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("t", "<u4"), ("i", "<f4"), ("tag", "u1"), ("line", "u1")])
HARMONICS = 4
WINDOW_S = 5.0


def returns(path):
    """Absolute time (s from the first frame), line, azimuth (rad), elevation (deg) of every return past 3 m."""
    db = sqlite3.connect(f"file:{path}?mode=ro", uri=True)
    topic = db.execute("select id from topics where name='/livox/points'").fetchone()[0]
    t, line, az, el = [], [], [], []
    for (data,) in db.execute("select data from messages where topic_id=? order by timestamp", (topic,)):
        msg = deserialize_message(data, PointCloud2)
        p = np.frombuffer(bytes(msg.data), POINT)
        r = np.sqrt(p["x"].astype(float) ** 2 + p["y"] ** 2 + p["z"] ** 2)
        p = p[r > 3.0]
        t.append(msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec + p["t"].astype(np.int64))
        line.append(p["line"])
        az.append(np.arctan2(p["y"], p["x"]).astype(float))
        el.append(np.degrees(np.arctan2(p["z"], np.hypot(p["x"], p["y"]))).astype(float))
    order = np.argsort(np.concatenate(t), kind="stable")
    t, line, az, el = (np.concatenate(v)[order] for v in (t, line, az, el))
    return t, (t - t[0]) * 1e-9, line.astype(int), az, el


def main():
    t_ns, t, line, az, el = returns(sys.argv[1])
    gaps = np.diff(t_ns)
    values, counts = np.unique(gaps[(gaps > 1000) & (gaps < 10000)], return_counts=True)
    slot_ns = int(values[np.argmax(counts)])
    wrap = lambda a: np.angle(np.exp(1j * a))  # noqa: E731

    # Spin: one line, four slots on.
    i = np.flatnonzero((t_ns[4:] - t_ns[:-4] == 4 * slot_ns) & (line[4:] == line[:-4]))
    steps = wrap(az[i + 4] - az[i])
    step = np.median(steps)
    spin_hz = step / (2 * np.pi * 4 * slot_ns * 1e-9)

    # Fan: lines 0-3 on four consecutive slots, each line's offset from line 0 over the line's number.
    i = np.flatnonzero((t_ns[3:] - t_ns[:-3] == 3 * slot_ns) & (line[:-3] == 0) & (line[3:] == 3))
    rows, d_el, d_az = [], [], []
    for a in (1, 2, 3):
        rows.append(np.column_stack([np.ones(len(i)), np.cos(az[i]), np.sin(az[i])]))
        d_el.append((el[i + a] - el[i]) / a)
        d_az.append((np.degrees(wrap(az[i + a] - az[i])) - np.degrees(step) * a / 4) / a)
    A = np.concatenate(rows)
    fan_el, *_ = np.linalg.lstsq(A, np.concatenate(d_el), rcond=None)
    fan_az, *_ = np.linalg.lstsq(A[:, [2, 1]], np.concatenate(d_az), rcond=None)
    fan_el_residual = np.concatenate(d_el) - A @ fan_el
    fan_az_residual = np.concatenate(d_az) - A[:, [2, 1]] @ fan_az

    # Nod: line 0's elevation, every return brought back to it by the fan.
    base = el - line * (fan_el[0] + fan_el[1] * np.cos(az) + fan_el[2] * np.sin(az))
    # The motor's speed drifts over minutes, so the nod's period and phase are fitted in windows of WINDOW_S,
    # each window's harmonics turned to the phase of its fundamental, and the shape is their median.
    def nod_fit(t_, el_, period):
        phase = 2 * np.pi * t_ / period
        A = np.column_stack([np.ones_like(phase)] + [f(m * phase) for m in range(1, HARMONICS + 1) for f in (np.cos, np.sin)])
        c, *_ = np.linalg.lstsq(A, el_, rcond=None)
        return c, el_ - A @ c

    shapes, periods, residuals = [], [], []
    for w in range(int(t[-1] // WINDOW_S)):
        k = (t >= w * WINDOW_S) & (t < (w + 1) * WINDOW_S)
        if k.sum() < 10000:
            continue
        t_, el_ = t[k] - w * WINDOW_S, base[k]
        period = min(np.arange(0.0995, 0.1035, 0.00001), key=lambda p: np.mean(nod_fit(t_[::20], el_[::20], p)[1] ** 2))
        c, residual = nod_fit(t_, el_, period)
        harmonics = c[1:].reshape(-1, 2) @ [1, -1j]  # a cos + b sin = Re((a - ib) e^{i phase})
        harmonics *= np.exp(-1j * np.arange(1, HARMONICS + 1) * np.angle(harmonics[0]))
        shapes.append(np.r_[c[0], np.column_stack([harmonics.real, -harmonics.imag]).ravel()])
        periods.append(period)
        residuals.append(residual)
    nod, period, residual = np.median(shapes, axis=0), np.median(periods), np.concatenate(residuals)

    pattern = {"slot_ns": slot_ns, "lines": 4, "spin_hz": round(float(spin_hz), 3), "nod_period_s": round(float(period), 6),
               "nod_deg": [round(float(c), 4) for c in nod],
               "fan_elevation_deg": [round(float(c), 4) for c in fan_el],
               "fan_azimuth_deg": [round(float(c), 4) for c in fan_az]}
    print(json.dumps(pattern))
    print(f"{len(t)} returns over {t[-1]:.1f} s; spin step IQR {np.degrees(np.subtract(*np.percentile(steps, [75, 25]))):.3f} deg; nod period {np.min(periods) * 1e3:.2f}-{np.max(periods) * 1e3:.2f} ms over {len(periods)} windows; "
          f"fan residual RMS elevation {np.sqrt(np.mean(fan_el_residual ** 2)):.2f}, azimuth {np.sqrt(np.mean(fan_az_residual ** 2)):.2f} deg; "
          f"nod residual median {np.median(np.abs(residual)):.2f}, 90% {np.percentile(np.abs(residual), 90):.2f} deg; "
          f"elevation {np.percentile(el, 0.1):.1f} to {np.percentile(el, 99.9):.1f} deg (0.1-99.9%)")


if __name__ == "__main__":
    main()
