"""Turn two recorded flights into the presentation's data: src/generated/*.js and public/img/cam/*.webp.

    .venv/bin/python presentation/scripts/build_data.py FOREST_RUN EMPTY_RUN EXTRACTED_DIR [FAILED_RUN ...]

FAILED_RUN: recorded empty-world runs whose check-recording failed, reported as they are.

EXTRACTED_DIR/<run>/ holds extract_bag.py's output for each run. Nothing here is synthesised: every number and
point comes from the run folders (result.json, telemetry.jsonl, ground_truth.jsonl) or their bags.
"""
import base64
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parent
forest_run, empty_run, extracted = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
GEN = HERE / "src/generated"


def b64(array):
    return base64.b64encode(np.ascontiguousarray(array).tobytes()).decode()


def rotate(q, v):
    """Rotate row vectors v (N, 3) by quaternions q (N, 4) or (4,), (x, y, z, w)."""
    q = np.broadcast_to(q, (len(v), 4))
    u, w = q[:, :3], q[:, 3:4]
    t = 2 * np.cross(u, v)
    return v + w * t + np.cross(u, t)


def scans_in_world(bag):
    truth = np.load(bag / "truth.npy")
    lidar_t = np.array(json.loads((bag / "static.json").read_text())["lidar_link"][0])
    for path in sorted((bag / "scans").glob("*.npy"), key=lambda p: int(p.stem)):
        stamp = int(path.stem) / 1e9
        i = min(np.searchsorted(truth[:, 0], stamp), len(truth) - 1)
        pose = truth[i]
        scan = np.load(path)
        finite = np.isfinite(scan["x"])
        local = np.stack([scan["x"], scan["y"], scan["z"]], 1)[finite].astype(np.float64) + lidar_t
        world = rotate(pose[4:8], local) + pose[1:4]
        yield stamp, pose, scan, finite, world


def voxel(points, size):
    keys = np.floor(points / size).astype(np.int64)
    _, first = np.unique(keys, axis=0, return_index=True)
    return points[np.sort(first)]


def telemetry(run):
    """10 Hz MAVLink as the check logged it, on the autopilot's clock (time_boot_ms; messages without one take the
    last seen)."""
    now, series = 0.0, {k: [] for k in ("alt", "servo", "ekf", "gps", "armed", "mode", "text", "ack", "att")}
    for line in (run / "telemetry.jsonl").read_text().splitlines():
        m = json.loads(line)
        kind = m["mavpackettype"]
        if "time_boot_ms" in m:
            now = m["time_boot_ms"] / 1000
        if kind == "GLOBAL_POSITION_INT":
            series["alt"].append((now, m["relative_alt"] / 1000, m["vz"] / -100))
        elif kind == "SERVO_OUTPUT_RAW":
            series["servo"].append((m["time_usec"] / 1e6, m["servo1_raw"], m["servo2_raw"], m["servo3_raw"],
                                    m["servo4_raw"]))
        elif kind == "EKF_STATUS_REPORT":
            series["ekf"].append((now, m["flags"]))
        elif kind == "GPS_RAW_INT":
            series["gps"].append((now, m["fix_type"], m["satellites_visible"]))
        elif kind == "HEARTBEAT" and m["autopilot"] == 3:
            series["armed"].append((now, bool(m["base_mode"] & 128)))
            series["mode"].append((now, m["custom_mode"]))
        elif kind == "STATUSTEXT":
            series["text"].append((now, m["text"]))
        elif kind == "COMMAND_ACK":
            series["ack"].append((now, m["command"], m["result"]))
        elif kind == "ATTITUDE":
            series["att"].append((now, m["roll"], m["pitch"], m["yaw"]))

    def changes(rows):
        out = []
        for row in rows:
            if not out or out[-1][1:] != row[1:]:
                out.append(row)
        return out

    for key in ("ekf", "gps", "armed", "mode"):
        series[key] = changes(series[key])
    return series


def flight(run, bag):
    result = json.loads((run / "result.json").read_text())
    truth = [json.loads(line) for line in (run / "ground_truth.jsonl").read_text().splitlines()]
    t = telemetry(run)
    return {
        "run": run.name,
        "manifest": json.loads((run / "manifest.json").read_text()),
        "result": result,
        "truth": [(round(s["time"], 2), *[round(v, 4) for v in s["position"]],
                   *[round(v, 4) for v in s["velocity"][:3]]) for s in truth],
        "telemetry": {k: [[round(v, 4) if isinstance(v, float) else v for v in row] for row in rows]
                      for k, rows in t.items()},
        "sensorCheck": json.loads((run / "sensor_check.json").read_text()) if (run / "sensor_check.json").exists()
        else None,
        "replayCheck": json.loads((run / "replay_check.json").read_text()) if (run / "replay_check.json").exists()
        else None,
        "bagMessages": {"truth": len(np.load(bag / "truth.npy")), "imu": len(np.load(bag / "imu.npy"))},
    }


forest_bag, empty_bag = extracted / forest_run.name, extracted / empty_run.name
data = {"forest": flight(forest_run, forest_bag), "empty": flight(empty_run, empty_bag)}
data["failedRecordings"] = [{"run": Path(r).name, **json.loads((Path(r) / "sensor_check.json").read_text()),
                             "acknowledged": json.loads((Path(r) / "result.json").read_text())["sensing"]["acknowledged"]}
                            for r in sys.argv[4:]]

# IMU around takeoff (forest): 250 Hz, kept at 50 Hz.
imu = np.load(forest_bag / "imu.npy")[::5]
data["forest"]["imu"] = [[round(float(v), 4) for v in row] for row in imu]

# Forest map: every scan placed by ground truth, deduplicated on a 20 cm grid.
cloud, path = [], []
hover_scan = None
for stamp, pose, scan, finite, world in scans_in_world(forest_bag):
    cloud.append(world)
    path.append(pose[1:4])
    if hover_scan is None and pose[3] > 2.9:
        hover_scan = (stamp, pose, scan, finite)
points = voxel(np.concatenate(cloud), 0.2)
centre = np.array([0.0, 0.0, 0.0])
quantized = np.round((points - centre) / 0.004).astype(np.int16)  # 4 mm, +-131 m
stamp, pose, scan, finite = hover_scan
single = np.stack([scan["x"], scan["y"], scan["z"]], 1)[finite]
data["forest"]["hoverScan"] = {"time": round(stamp, 3), "pose": [round(float(v), 4) for v in pose[1:8]],
                               "returns": int(finite.sum()), "of": len(scan)}

# Empty world: the hovering scans the wall check uses (above 2 m), in the world frame.
wall = []
for stamp, pose, scan, finite, world in scans_in_world(empty_bag):
    if pose[3] > 2.0 and len(wall) < 20:
        wall.append(world)
wall_points = voxel(np.concatenate(wall), 0.03)

clouds = {
    "forest": {"count": len(quantized), "scale": 0.004, "data": b64(quantized)},
    "scan": {"count": len(single), "scale": 0.004, "data": b64(np.round(single / 0.004).astype(np.int16)),
             "ring": b64(scan["ring"][finite].astype(np.uint8))},
    "wall": {"count": len(wall_points), "scale": 0.004, "data": b64(np.round(wall_points / 0.004).astype(np.int16))},
}
(GEN / "flight-data.js").write_text("export default " + json.dumps(data, separators=(",", ":")) + ";\n")
(GEN / "clouds.js").write_text("export default " + json.dumps(clouds, separators=(",", ":")) + ";\n")

# The forest flight's camera at 10 Hz (the bag's every second frame), as WebP.
out = HERE / "public/img/cam"
frames = []
for path in sorted((forest_bag / "frames").glob("*.npy"), key=lambda p: int(p.stem)):
    stamp = int(path.stem) / 1e9
    bgra = np.load(path)
    name = f"{int(path.stem) // 1_000_000:06d}.webp"
    Image.fromarray(bgra[:, :, 2::-1]).save(out / name, quality=72, method=6)
    frames.append([round(stamp, 2), name])
(GEN / "camera.js").write_text("export default " + json.dumps(frames) + ";\n")
print(len(quantized), "map points;", len(single), "scan returns;", len(wall_points), "wall points;", len(frames),
      "frames")
