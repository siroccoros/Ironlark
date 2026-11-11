"""Replay maze sensors through the odometer and score ATE/RPE against evaluator-only truth.

Usage: ./ironlark check-odometry runs/<recorded-maze-run>
The estimator sees only /imu/data and /lidar/points. Truth is opened after replay ends.
"""
import json
import math
import argparse
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2
from geometry_msgs.msg import PoseStamped

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sensing.pointcloud import cloud_xyzt, seconds  # noqa: E402
from sensing.odometry import ScanOdometry  # noqa: E402
from bag_reader import open_bag  # noqa: E402


def score(states, truth, initial_yaw=0.0, route=False):
    """Align the first evaluator pose once; report drift without fitting the trajectory."""
    good = [state for state in states if state["healthy"]]
    if len(good) < 10:
        return {"passed": False, "reason": f"only {len(good)} healthy scans", "healthy_scans": len(good)}
    t = np.array([sample["time"] for sample in truth])
    actual = np.array([sample["position"] for sample in truth])
    stamps = np.array([sample["time"] for sample in good])
    estimate = np.array([sample["position"] for sample in good])
    c, s = math.cos(initial_yaw), math.sin(initial_yaw)
    estimate[:, :2] = estimate[:, :2] @ np.array([[c, s], [-s, c]])
    reference = np.column_stack([np.interp(stamps, t, actual[:, i]) for i in range(3)])
    offset = reference[0] - estimate[0]
    error = np.linalg.norm(estimate + offset - reference, axis=1)
    # Relative position error over 1 s: each estimate compared with an estimate one second later.
    relative = []
    for i, stamp in enumerate(stamps):
        j = np.searchsorted(stamps, stamp + 1.0)
        if j < len(stamps) and abs(stamps[j] - stamp - 1.0) < 0.15:
            relative.append(np.linalg.norm((estimate[j] - estimate[i]) - (reference[j] - reference[i])))
    ate = float(np.sqrt(np.mean(error ** 2)))
    rpe = float(np.sqrt(np.mean(np.square(relative)))) if relative else None
    report = {"passed": bool(ate < 0.3 and rpe is not None and rpe < 0.2
                           and len(good) / len(states) >= 0.9),
            "limits": {"ate_rmse_m": 0.3, "rpe_1s_rmse_m": 0.2, "healthy_fraction": 0.9},
            "ate_rmse_m": ate,
            "ate_p95_m": float(np.percentile(error, 95)),
            "axis_rmse_m": {axis: float(np.sqrt(np.mean((estimate[:, i] + offset[i] - reference[:, i]) ** 2)))
                            for i, axis in enumerate("xyz")},
            "rpe_1s_rmse_m": rpe,
            "healthy_scans": len(good), "total_scans": len(states),
            "unhealthy_reasons": {reason: sum(s["reason"] == reason for s in states)
                                  for reason in set(s["reason"] for s in states) if reason != "ok"}}
    if route:
        truth_velocity = np.array([sample["velocity"][:2] for sample in truth])
        speed = np.linalg.norm(np.column_stack([
            np.interp(stamps, t, truth_velocity[:, i]) for i in range(2)]), axis=1)
        moving = speed > 0.2
        # Named evaluator case: the seed-7 route's 5.4 m eastbound straight section.
        corridor = (moving & (reference[:, 0] >= 3.5) & (reference[:, 0] <= 7.5)
                    & (np.abs(reference[:, 1] - 2.7) <= 0.6))
        all_stamps = np.array([state["time"] for state in states])
        all_xy = np.column_stack([np.interp(all_stamps, t, actual[:, i]) for i in range(2)])
        all_speed = np.linalg.norm(np.column_stack([
            np.interp(all_stamps, t, truth_velocity[:, i]) for i in range(2)]), axis=1)
        all_corridor = ((all_speed > 0.2) & (all_xy[:, 0] >= 3.5) & (all_xy[:, 0] <= 7.5)
                        & (np.abs(all_xy[:, 1] - 2.7) <= 0.6))
        healthy_corridor = sum(state["healthy"] for state, selected in zip(states, all_corridor) if selected)
        fraction = healthy_corridor / max(1, int(all_corridor.sum()))
        moving_ate = float(np.sqrt(np.mean(error[moving] ** 2))) if moving.any() else None
        corridor_ate = float(np.sqrt(np.mean(error[corridor] ** 2))) if corridor.any() else None
        report.update({"moving_scans": int(moving.sum()), "moving_ate_rmse_m": moving_ate,
                       "corridor_case": "seed7_eastbound_5.4m_straight",
                       "corridor_scans": int(corridor.sum()), "corridor_ate_rmse_m": corridor_ate,
                       "corridor_healthy_fraction": fraction})
        report["limits"].update({"moving_ate_rmse_m": 0.3, "corridor_ate_rmse_m": 0.3,
                                 "corridor_healthy_fraction": 0.9, "minimum_corridor_scans": 30})
        report["passed"] = bool(report["passed"] and moving.sum() >= 100 and corridor.sum() >= 30
                                and moving_ate < 0.3 and corridor_ate < 0.3 and fraction >= 0.9)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--message-stamp", action="store_true", help="diagnostic baseline using publication time")
    args = parser.parse_args()
    run = args.run
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest.get("world") != "maze":
        raise RuntimeError("odometry evaluation requires a recorded maze flight")
    config = json.loads((ROOT / "config/sensors.json").read_text())
    odom = ScanOdometry(config["lidar"]["translation"], use_acquisition_time=not args.message_stamp)
    reader = open_bag(run / "bag")
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/imu/data", "/lidar/points"]))
    states = []
    while reader.has_next():
        topic, raw, _ = reader.read_next()
        if topic == "/imu/data":
            msg = deserialize_message(raw, Imu)
            odom.imu(seconds(msg.header.stamp), msg.angular_velocity.z,
                     msg.angular_velocity.x, msg.angular_velocity.y,
                     (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        else:
            msg = deserialize_message(raw, PointCloud2)
            states.append(odom.scan(seconds(msg.header.stamp), cloud_xyzt(msg)))
    stem = "odometry_replay_message_stamp.jsonl" if args.message_stamp else "odometry_replay.jsonl"
    (run / stem).write_text("".join(json.dumps(state) + "\n" for state in states))
    # Evaluator boundary: only now is independent simulator truth opened.
    truth = [json.loads(line) for line in (run / "ground_truth.jsonl").read_text().splitlines()]
    truth_reader = open_bag(run / "bag")
    truth_reader.set_filter(rosbag2_py.StorageFilter(topics=["/ground_truth/pose"]))
    if not truth_reader.has_next():
        raise RuntimeError("no evaluator pose in the recording")
    _, raw, _ = truth_reader.read_next()
    orientation = deserialize_message(raw, PoseStamped).pose.orientation
    initial_yaw = math.atan2(2 * (orientation.w * orientation.z + orientation.x * orientation.y),
                             1 - 2 * (orientation.y ** 2 + orientation.z ** 2))
    report = score(states, truth, initial_yaw, route=bool(manifest.get("odometry_route_fixture")))
    report["sensor_only"] = True
    report["evaluator_initial_yaw_rad"] = initial_yaw
    report["scan_model"] = ("publication time diagnostic" if args.message_stamp else
                            "one Webots geometry snapshot at measured last-return time")
    report["timed_scans"] = sum(state["timing_valid"] for state in states)
    report["mean_scan_duration_s"] = (sum(state["scan_duration_s"] for state in states if state["timing_valid"])
                                       / max(1, report["timed_scans"]))
    report["mean_acquisition_delay_s"] = (sum(state["acquisition_delay_s"] for state in states if state["timing_valid"])
                                         / max(1, report["timed_scans"]))
    name = "odometry_check_message_stamp.json" if args.message_stamp else "odometry_check.json"
    (run / name).write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))
    if report.get("passed") is False:
        sys.exit(1)


if __name__ == "__main__":
    main()
