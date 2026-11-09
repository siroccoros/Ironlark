"""maze_lidar judges a scan against truth.json: it passes the scan the world would give, and names what is wrong with
one that is not. Scans are made here analytically, without maze_lidar's own ray casting: a LiDAR 1.5 m above the ground,
level, facing a wall whose near face is 3 m ahead, with the rig's layers (config/sensors.json) and its range noise."""
import json
import math
import random
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import maze_lidar  # noqa: E402

LIDAR = json.loads((ROOT / "config/sensors.json").read_text())["lidar"]
HEIGHT, FACE = 1.5, 3.0
TRUTH = {"walls": [{"centre": [FACE + 0.1, 0.0, 1.5], "size": [0.2, 100.0, 3.0]}],
         "ground": {"min": [-60.0, -60.0], "max": [60.0, 60.0], "top_z": 0.0, "thickness_m": 0.2}}
POSE = ((0.0, 0.0, HEIGHT), (0.0, 0.0, 0.0, 1.0))
LINK = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def scan(tilt_deg=0.0, face=FACE, drop_wall=False, seed=0, offset=math.pi, sense=-1, drop_below_top_deg=0.0):
    """One scan as a Webots LidarPoint array: each ray's range to the wall's face or the ground, whichever is nearer,
    with the rig's noise; infinite beyond the range or where it meets nothing."""
    rng = random.Random(seed)
    columns, layers = LIDAR["horizontal_resolution"], LIDAR["layers"]
    fov, rendered = LIDAR["vertical_field_of_view"], LIDAR["render_layers"]
    points = np.zeros(layers * columns, dtype=maze_lidar.POINT)
    for k in range(layers):
        el = fov / 2 - k * fov / (rendered - 1) + math.radians(tilt_deg)
        for j in range(columns):
            az = offset + sense * 2 * math.pi * j / columns
            d = (math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el))
            ranges = []
            if d[0] > 1e-9:
                t = face / d[0]
                top = math.degrees(math.atan2(3.0 - HEIGHT, t * math.cos(el))) - math.degrees(el)  # below the edge
                if 0.0 <= HEIGHT + t * d[2] <= 3.0 and not drop_wall and top >= drop_below_top_deg:
                    ranges.append(t)
            if d[2] < -1e-9:
                t = HEIGHT / -d[2]
                if t * d[0] < face or drop_wall:  # the ground before the wall (or everywhere, without it)
                    ranges.append(t)
            r = min(ranges) if ranges else math.inf
            if r > LIDAR["max_range"]:
                r = math.inf
            r = r + rng.gauss(0.0, LIDAR["noise"]) if math.isfinite(r) else r
            points[k * columns + j] = (d[0] * r, d[1] * r, d[2] * r, k, 0.0) if math.isfinite(r) else (
                math.inf, math.inf, math.inf, k, 0.0)
    return points


class MazeLidar(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.faithful = maze_lidar.check([(POSE, scan(seed=1)), (POSE, scan(seed=2))], LINK, TRUTH, LIDAR)

    def test_the_scan_the_world_gives_passes(self):
        report = self.faithful
        self.assertEqual(report["problems"], [])
        self.assertGreater(report["placed_share"], 0.999)
        self.assertEqual(report["missing"], 0)
        self.assertEqual(report["walls_seen"], 1)
        self.assertAlmostEqual(report["residual_over_noise"], 1.0, delta=0.15)

    def test_the_columns_azimuth_is_fitted_whatever_its_start_and_sense(self):
        self.assertEqual(self.faithful["azimuth"]["sense"], -1)
        self.assertAlmostEqual(abs(self.faithful["azimuth"]["offset_deg"]), 180.0, places=2)
        report = maze_lidar.check([(POSE, scan(offset=0.3, sense=1))], LINK, TRUTH, LIDAR)
        self.assertEqual((report["azimuth"]["sense"], report["problems"]), (1, []))
        self.assertAlmostEqual(report["azimuth"]["offset_deg"], math.degrees(0.3), places=2)

    def test_a_wall_where_truth_has_none_is_off(self):
        moved = {**TRUTH, "walls": [{"centre": [FACE + 0.4, 0.0, 1.5], "size": [0.2, 100.0, 3.0]}]}
        report = maze_lidar.check([(POSE, scan())], LINK, moved, LIDAR)
        self.assertTrue(any("of the returns lie on a surface" in p for p in report["problems"]), report["problems"])

    def test_a_wall_that_returns_nothing_is_missing(self):
        report = maze_lidar.check([(POSE, scan(drop_wall=True))], LINK, TRUTH, LIDAR)
        self.assertTrue(any("returned nothing" in p for p in report["problems"]), report["problems"])

    def test_layers_off_their_configured_angles_fail(self):
        report = maze_lidar.check([(POSE, scan(tilt_deg=1.0))], LINK, TRUTH, LIDAR)
        self.assertGreater(report["elevation_error_deg"], 0.9)
        self.assertTrue(any("configured angles" in p for p in report["problems"]), report["problems"])

    def test_rays_at_a_walls_top_edge_may_return_nothing(self):
        """Webots returns nothing for many rays just below a wall's top edge: within a layer, they are edge rays."""
        report = maze_lidar.check([(POSE, scan(drop_below_top_deg=1.5))], LINK, TRUTH, LIDAR)
        self.assertGreater(report["edge"], 0)
        self.assertEqual(report["problems"], [])

    def test_a_wall_cut_well_below_its_top_is_missing(self):
        report = maze_lidar.check([(POSE, scan(drop_below_top_deg=8.0))], LINK, TRUTH, LIDAR)
        self.assertTrue(any("returned nothing" in p for p in report["problems"]), report["problems"])

    def test_returns_from_the_drone_itself_are_counted_apart(self):
        points = scan()
        points[0] = (0.1, 0.0, 0.1, 0, 0.0)
        report = maze_lidar.check([(POSE, points)], LINK, TRUTH, LIDAR)
        self.assertEqual((report["self"], report["problems"]), (1, []))

    def test_no_scans_fail(self):
        self.assertEqual(maze_lidar.check([], LINK, TRUTH, LIDAR)["problems"], ["no hovering scans to check"])


if __name__ == "__main__":
    unittest.main()
