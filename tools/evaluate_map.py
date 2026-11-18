"""Build an occupancy map from recorded LiDAR and odometry, then score it against maze truth."""
import argparse
import json
import math
import sys
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from ros.lidar_odometry import cloud_xyzt, seconds  # noqa: E402
from sensing.mapping import OccupancyMap  # noqa: E402
from bag_reader import open_bag  # noqa: E402


def score(grid, truth):
    """Evaluate observed cells only; unobserved space is reported as unknown."""
    resolution = grid.resolution
    bounds = truth["bounds"]
    x0, y0 = (math.floor(v / resolution) for v in bounds["min"])
    x1, y1 = (math.ceil(v / resolution) for v in bounds["max"])
    cells = [(x, y) for x in range(x0, x1) for y in range(y0, y1)]

    def wall_at(x, y):
        px, py = (x + 0.5) * resolution, (y + 0.5) * resolution
        for wall in truth["walls"]:
            cx, cy = wall["centre"][:2]
            sx, sy = wall["size"][:2]
            if abs(px - cx) <= sx / 2 + resolution / 2 and abs(py - cy) <= sy / 2 + resolution / 2:
                return True
        return False

    known = [(cell, grid.state(cell), wall_at(*cell)) for cell in cells if grid.state(cell) != "unknown"]
    correct = sum((state == "occupied") == actual for _, state, actual in known)
    occupied = [(state, actual) for _, state, actual in known if state == "occupied"]
    free = [(state, actual) for _, state, actual in known if state == "free"]
    return {"observed_cells": len(known), "unknown_cells": len(cells) - len(known),
            "known_fraction": len(known) / len(cells),
            "observed_accuracy": correct / len(known) if known else 0,
            "occupied_precision": (sum(actual for _, actual in occupied) / len(occupied)
                                   if occupied else None),
            "free_precision": (sum(not actual for _, actual in free) / len(free) if free else None)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    args = parser.parse_args()
    run = args.run
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest.get("world") != "maze":
        raise RuntimeError("map evaluation needs a recorded maze flight")
    states = [json.loads(line) for line in (run / "odometry.jsonl").read_text().splitlines()]
    config = json.loads((ROOT / "config/sensors.json").read_text())
    grid = OccupancyMap(lidar_offset=config["lidar"]["translation"])
    reader = open_bag(run / "bag")
    reader.set_filter(rosbag2_py.StorageFilter(topics=["/lidar/points"]))
    scans = 0
    matched = 0
    state_index = 0
    while reader.has_next():
        _, raw, _ = reader.read_next()
        message = deserialize_message(raw, PointCloud2)
        scans += 1
        stamp = seconds(message.header.stamp)
        # A busy live ROS executor may skip a scan that the independent bag
        # recorder kept. Pair estimates with the scan that produced them.
        while (state_index < len(states)
               and states[state_index]["time"] + states[state_index]["acquisition_delay_s"] < stamp - 0.02):
            state_index += 1
        if state_index >= len(states):
            continue
        state = states[state_index]
        publication_time = state["time"] + state["acquisition_delay_s"]
        if abs(publication_time - stamp) > 0.02:
            continue
        state_index += 1
        matched += 1
        if state["healthy"] and matched % (manifest.get("map_stride") or 5) == 0:
            grid.update(state, cloud_xyzt(message))
    if matched < 0.9 * len(states):
        raise RuntimeError(f"Could match only {matched} of {len(states)} live odometry estimates to LiDAR scans")
    (run / "occupancy_map.json").write_text(json.dumps(grid.snapshot()) + "\n")
    # Evaluator boundary: open true walls only after the sensor-derived map is complete.
    truth = json.loads((run / "truth.json").read_text())
    report = {"sensor_only_map": True, "scans": scans, "matched_scans": matched,
              "live_estimates": len(states), "mapped_scans": grid.scans,
              **score(grid, truth)}
    (run / "map_check.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
