"""Judge a set of maze runs from evaluator artifacts, including unsuccessful escapes."""
import argparse
import json
import math
from pathlib import Path
from statistics import mean


def escape_index(samples, truth):
    side = truth["exit"]["side"]
    opening = truth["exit"]["opening"]
    bounds = truth["bounds"]
    for index, sample in enumerate(samples):
        x, y, z = sample["position"]
        if not 0.3 < z < truth["parameters"]["wall_height_m"]:
            continue
        if side in ("east", "west"):
            across = y
            outside = x > bounds["max"][0] + 0.33 if side == "east" else x < bounds["min"][0] - 0.33
            lower, upper = opening[0][1], opening[1][1]
        else:
            across = x
            outside = y > bounds["max"][1] + 0.33 if side == "north" else y < bounds["min"][1] - 0.33
            lower, upper = opening[0][0], opening[1][0]
        if outside and lower + 0.33 <= across <= upper - 0.33:
            return index
    return None


def minimum_clearance(samples, walls, obstacle=None, spawned_at=None):
    minimum = math.inf
    for sample in samples:
        x, y, z = sample["position"]
        active = walls + ([obstacle] if obstacle is not None and spawned_at is not None
                          and sample["time"] >= spawned_at else [])
        for wall in active:
            cx, cy, cz = wall["centre"]
            sx, sy, sz = wall["size"]
            if z > cz + sz / 2:
                continue
            distance = math.hypot(max(abs(x - cx) - sx / 2, 0),
                                  max(abs(y - cy) - sy / 2, 0))
            minimum = min(minimum, distance)
    return None if math.isinf(minimum) else round(minimum, 3)


def judge(run):
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest.get("world") != "maze":
        raise ValueError(f"{run} is not a maze run")
    result = json.loads((run / "result.json").read_text())
    truth = json.loads((run / "truth.json").read_text())
    samples = [json.loads(line) for line in (run / "ground_truth.jsonl").read_text().splitlines()]
    index = escape_index(samples, truth)
    obstacle_file = run / "obstacle.json"
    spawn_file = run / "obstacle-spawned.json"
    obstacle = json.loads(obstacle_file.read_text()) if obstacle_file.exists() else None
    spawned_at = json.loads(spawn_file.read_text())["time"] if spawn_file.exists() else None
    shortest = (obstacle["shortest_detour_m"] if obstacle is not None and spawned_at is not None
                else truth["shortest_path"]["length_m"])
    replan_time = None
    navigation_file = run / "navigation.jsonl"
    if spawned_at is not None and navigation_file.exists():
        entries = [json.loads(line) for line in navigation_file.read_text().splitlines()]
        before = [entry for entry in entries if entry["time"] < spawned_at]
        baseline = before[-1].get("blocked_replans", 0) if before else 0
        after = next((entry for entry in entries if entry["time"] >= spawned_at
                      and entry.get("blocked_replans", 0) > baseline), None)
        if after is not None:
            replan_time = round(after["time"] - spawned_at, 2)
    traveled = samples[:index + 1] if index is not None else samples
    distance = sum(math.dist(a["position"][:2], b["position"][:2])
                   for a, b in zip(traveled, traveled[1:]))
    delivery_lag = scan_gap = None
    odometry_file = run / "odometry.jsonl"
    if odometry_file.exists():
        entries = [json.loads(line) for line in odometry_file.read_text().splitlines()]
        lags = [entry["delivery_lag_s"] for entry in entries if entry.get("delivery_lag_s") is not None]
        delivery_lag = round(max(lags), 3) if lags else None
        # Wall seconds between consecutive scans reaching the odometer: ~0.125 s at 0.8x real
        # time; seconds-long gaps are delivery stalls.
        received = [entry["received_wall_s"] for entry in entries if entry.get("received_wall_s") is not None]
        scan_gap = round(max(b - a for a, b in zip(received, received[1:])), 3) if len(received) > 1 else None
    events = [event["state"] for event in result.get("events", [])]
    map_report = json.loads((run / "map_check.json").read_text()) if (run / "map_check.json").exists() else {}
    odom_report = json.loads((run / "odometry_check.json").read_text()) if (run / "odometry_check.json").exists() else {}
    return {"run": str(run), "seed": manifest["maze_seed"], "gps_denied": manifest.get("gps_denied", False),
            "flight_passed": result.get("passed", False), "escaped": index is not None,
            "error": result.get("error"),
            "failsafe_landed": any(state.endswith("FAILSAFE_LANDED") for state in events),
            "escape_time_s": round(samples[index]["time"] - samples[0]["time"], 2) if index is not None else None,
            "path_length_m": round(distance, 3),
            "shortest_path_m": shortest,
            "path_ratio": round(distance / shortest, 3) if index is not None else None,
            "ate_rmse_m": odom_report.get("ate_rmse_m"),
            "map_observed_accuracy": map_report.get("observed_accuracy"),
            "map_known_fraction": map_report.get("known_fraction"),
            "min_wall_clearance_m": minimum_clearance(samples, truth["walls"], obstacle, spawned_at),
            "dynamic_obstacle_spawned": spawned_at is not None,
            "replan_time_s": replan_time,
            "max_odometry_delivery_lag_s": delivery_lag,
            "max_scan_arrival_gap_s": scan_gap,
            "resources": result.get("resources")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("runs", nargs="+", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    rows = [judge(run.resolve()) for run in args.runs]
    numeric = ("escape_time_s", "path_ratio", "ate_rmse_m", "map_observed_accuracy",
               "map_known_fraction", "min_wall_clearance_m", "replan_time_s")
    summary = {"runs": len(rows), "gps_denied_runs": sum(row["gps_denied"] for row in rows),
               "flight_passed": sum(row["flight_passed"] for row in rows),
               "escaped": sum(row["escaped"] for row in rows),
               "escape_rate": mean(row["escaped"] for row in rows),
               "failures": {Path(row["run"]).name: row["error"] for row in rows if row["error"]},
               "mean": {key: (mean(values) if (values := [row[key] for row in rows if row[key] is not None])
                              else None) for key in numeric}}
    report = {"summary": summary, "runs": rows}
    rendered = json.dumps(report, indent=2) + "\n"
    if args.output:
        args.output.write_text(rendered)
    print(rendered, end="")


if __name__ == "__main__":
    main()
