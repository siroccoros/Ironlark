"""Independent post-flight mission evaluator.

Reads ground_truth.jsonl, lio_trajectory.jsonl, telemetry.jsonl, autonomy_report.json, and
sim-meta.json AFTER landing and writes the final verdict to result.json.
"""
import argparse
import json
import math
import sys
from pathlib import Path


def _in_box(pos, box):
    return (box["x"][0] <= pos[0] <= box["x"][1]
            and box["y"][0] <= pos[1] <= box["y"][1]
            and box["z"][0] <= pos[2] <= box["z"][1])


def lio_error_against_truth(truth, lio_trace):
    """Align odometry samples to nearest ground-truth timestamps (within 0.15 s) and compute 3D error."""
    if not truth or not lio_trace:
        return None, None
    t_times = [s["time"] for s in truth]
    errors = []
    j = 0
    for sample in lio_trace:
        t = sample["time"]
        while j + 1 < len(t_times) and abs(t_times[j + 1] - t) <= abs(t_times[j] - t):
            j += 1
        if abs(t_times[j] - t) <= 0.15:
            errors.append(math.dist(truth[j]["position"], sample["position"]))
    if not errors:
        return None, None
    rmse = math.sqrt(sum(e * e for e in errors) / len(errors))
    return round(rmse, 3), round(max(errors), 3)


def judge_mission(truth, lio_trace, autonomy_report, sim_meta, status_texts=()):
    """Independently judge the outdoor-to-interior GPS-denied mission from ground truth."""
    if not truth:
        raise RuntimeError("No Webots ground truth recorded")
    times = [s["time"] for s in truth]
    gaps = [(a, b) for a, b in zip(times, times[1:]) if b - a > 0.15]
    if times[0] > 0.3 or gaps:
        raise RuntimeError(f"Ground-truth trajectory incomplete: start {times[0]:.2f}s, {len(gaps)} gaps")

    for text in status_texts:
        if "EKF Failsafe" in text or "Crash:" in text:
            raise RuntimeError(f"Autopilot reported critical fault: {text}")

    positions = [s["position"] for s in truth]
    min_upright = min((s["orientation"][8] for s in truth if "orientation" in s), default=1.0)
    if min_upright < 0.35:
        raise RuntimeError(f"Drone flipped or crashed (min upright cosine {min_upright:.2f})")

    interior = sim_meta["interior_box"]
    inside_pts = [p for p in positions if _in_box(p, interior)]
    if len(inside_pts) < 50:
        raise RuntimeError(f"Drone did not spend enough time inside the interior ({len(inside_pts)} samples)")

    interior_travel = sum(math.dist(a, b) for a, b in zip(inside_pts, inside_pts[1:]))
    deepest_x = max(p[0] for p in inside_pts)
    y_span = max(p[1] for p in inside_pts) - min(p[1] for p in inside_pts)
    if interior_travel < 15.0 or deepest_x < interior["x"][0] + 5.0:
        raise RuntimeError(f"Insufficient interior exploration: travel {interior_travel:.1f} m, max x {deepest_x:.1f} m")

    frontiers_visited = int(autonomy_report.get("frontiers_visited", 0))
    free_vol = float(autonomy_report.get("interior_free_volume_m3", 0.0))
    if frontiers_visited < 1 or free_vol < 150.0:
        raise RuntimeError(f"Insufficient mapping/frontier progress: {frontiers_visited} frontiers, {free_vol} m^3")

    gps_denied_samples = sum(1 for s in truth if s.get("gps_denied"))
    if gps_denied_samples < 50:
        raise RuntimeError("Drone did not spend enough time in the GPS-denied zone")
    if not autonomy_report.get("gps_lost_observed"):
        raise RuntimeError("Autonomy never observed GPS loss (SIM_GPS1_ENABLE=0 did not cut GPS)")
    switches = autonomy_report.get("ekf_source_switches", [])
    if not any(sw.get("source") == 2 for sw in switches):
        raise RuntimeError("EKF3 never switched to ExternalNav (Source Set 2) during GPS denial")

    start_xy = positions[0][:2]
    final_pos = positions[-1]
    return_error = math.hypot(final_pos[0] - start_xy[0], final_pos[1] - start_xy[1])
    if return_error > 1.2:
        raise RuntimeError(f"Did not return to home pad: {return_error:.2f} m off")
    if not (-0.2 <= final_pos[2] <= 0.25):
        raise RuntimeError(f"Did not land cleanly: final height {final_pos[2]:.2f} m")

    lio_rmse, lio_max = lio_error_against_truth(truth, lio_trace)
    if lio_rmse is None or lio_rmse > 0.65 or lio_max > 2.5:
        raise RuntimeError(f"Odometry accuracy out of bounds: RMSE={lio_rmse} m (max={lio_max} m)")

    if not autonomy_report.get("passed"):
        raise RuntimeError(f"Autonomy reported failure: {autonomy_report.get('error', 'unknown')}")

    return {
        "duration_s": round(times[-1], 1),
        "deepest_interior_x_m": round(deepest_x, 2),
        "interior_travel_m": round(interior_travel, 2),
        "interior_y_span_m": round(y_span, 2),
        "frontiers_visited": frontiers_visited,
        "interior_free_volume_m3": free_vol,
        "gps_denied_seconds": round(gps_denied_samples * 0.1, 1),
        "ekf_source_switches": len(switches),
        "lio_rmse_m": lio_rmse,
        "lio_max_error_m": lio_max,
        "return_home_error_m": round(return_error, 3),
        "final_height_m": round(final_pos[2], 3),
        "min_upright_cos": round(min_upright, 3),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--judge", type=Path, required=True, help="Run folder to evaluate after landing")
    args = parser.parse_args()

    run = args.judge
    truth = [json.loads(line) for line in (run / "ground_truth.jsonl").read_text().splitlines() if line.strip()]
    lio_path = run / "lio_trajectory.jsonl"
    lio_trace = [json.loads(line) for line in lio_path.read_text().splitlines() if line.strip()] if lio_path.is_file() else []
    autonomy_report = json.loads((run / "autonomy_report.json").read_text())
    sim_meta = json.loads((run / "sim-meta.json").read_text())
    tel_path = run / "telemetry.jsonl"
    statuses = []
    if tel_path.is_file():
        for line in tel_path.read_text().splitlines():
            if '"STATUSTEXT"' in line:
                try:
                    statuses.append(json.loads(line).get("text", ""))
                except ValueError:
                    pass

    result = {"passed": False, "mission": "outdoor-to-cave-exploration", "estimator": "EKF3"}
    try:
        result["measurements"] = judge_mission(truth, lio_trace, autonomy_report, sim_meta, status_texts=statuses)
        result["passed"] = True
    except Exception as exc:
        result["error"] = str(exc)
    try:
        sys.path.insert(0, str(Path(__file__).resolve().parent))
        from motion_quality import measure
        result["motion"] = measure(run)
    except Exception as exc:  # measurements must never mask the verdict above
        result["motion_error"] = repr(exc)
    (run / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
