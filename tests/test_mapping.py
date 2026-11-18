"""Geometry checks for the LiDAR occupancy map."""
import unittest

import numpy as np

from sensing.mapping import OccupancyMap


class MappingTests(unittest.TestCase):
    def test_rays_leave_unseen_space_unknown(self):
        grid = OccupancyMap(resolution=0.2, lidar_offset=(0, 0, 0))
        state = {"position": [0, 0, 0], "yaw": 0}
        self.assertEqual(grid.update(state, np.array([[1.0, 0.0, 0.5]])), 1)
        self.assertEqual(grid.state((5, 0)), "occupied")
        self.assertEqual(grid.state((2, 0)), "free")
        self.assertEqual(grid.state((6, 0)), "unknown")
        self.assertEqual(grid.state((0, 3)), "unknown")

    def test_rotation_places_returns_in_world_coordinates(self):
        grid = OccupancyMap(resolution=0.2, lidar_offset=(0, 0, 0))
        grid.update({"position": [1, 1, 0], "yaw": np.pi / 2},
                    np.array([[1.0, 0.0, 0.5]]))
        self.assertEqual(grid.state((5, 10)), "occupied")
        self.assertEqual(grid.state((10, 5)), "unknown")

    def test_near_horizontal_no_return_marks_only_its_measured_ray_free(self):
        grid = OccupancyMap(resolution=0.2, lidar_offset=(0, 0, 0))
        cloud = np.full((32 * 900, 4), np.nan)
        cloud[225, :3] = (np.inf, np.inf, -np.inf)  # upper layer sees over walls
        cloud[30 * 900 + 225, :3] = (np.inf, np.inf, -np.inf)
        grid.update({"position": [0, 0, 1.5], "yaw": 0}, cloud)
        self.assertEqual(grid.state((0, 30)), "free")
        self.assertEqual(grid.state((0, 55)), "unknown")
        self.assertEqual(grid.state((30, 0)), "unknown")


if __name__ == "__main__":
    unittest.main()
