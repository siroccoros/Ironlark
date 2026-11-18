"""Planning decisions when a formerly open route becomes blocked."""
import math
import unittest

from sensing.mapping import OccupancyMap
from sensing.planning import nearest_frontier, obstacle_padding, route, widest_open_sector


class PlanningTests(unittest.TestCase):
    def test_open_sector_distinguishes_corridor_from_exterior(self):
        grid = OccupancyMap()
        for index in range(-10, 11):
            grid.log_odds[index, -10] = 5
            grid.log_odds[index, 10] = 5
            grid.log_odds[-10, index] = 5
            grid.log_odds[10, index] = 5
        self.assertLess(widest_open_sector(grid, (0.1, 0.1)), 100)
        for index in range(-10, 11):
            grid.log_odds[index, 10] = -5
            grid.log_odds[-10, index] = -5
            grid.log_odds[10, index] = -5
        width, heading = widest_open_sector(grid, (0.1, 0.1), with_heading=True)
        self.assertGreater(width, 170)
        self.assertGreater(math.sin(heading), 0.9)

    def test_new_obstacle_forces_detour(self):
        grid = OccupancyMap()
        for x in range(7):
            for y in range(3):
                grid.log_odds[x, y] = -3
        first = route(grid, (0, 1), (6, 1), clearance_cells=0)
        self.assertIn((3, 1), first)
        grid.log_odds[3, 1] = 5
        replanned = route(grid, (0, 1), (6, 1), clearance_cells=0)
        self.assertIsNotNone(replanned)
        self.assertNotIn((3, 1), replanned)
        self.assertGreater(len(replanned), len(first))

    def test_unknown_is_explicitly_unverified(self):
        grid = OccupancyMap()
        grid.log_odds[0, 0] = -3
        grid.log_odds[1, 0] = -3
        self.assertIsNone(route(grid, (0, 0), (3, 0), clearance_cells=0))
        path = route(grid, (0, 0), (3, 0), allow_unknown=True, clearance_cells=0)
        self.assertEqual(path[-1], (3, 0))
        self.assertEqual(grid.state((3, 0)), "unknown")
        self.assertEqual(nearest_frontier(grid, (0, 0), clearance_cells=0), [(0, 0), (1, 0)])

    def test_search_climbs_out_of_padding_without_nearing_walls(self):
        # Drift left the drone one cell from a wall it should keep three cells from
        # (runs/20261114-142338-31363); routes must still start, moving away from it.
        grid = OccupancyMap()
        for x in range(-2, 15):
            grid.log_odds[x, -1] = 5
            for y in range(0, 10):
                grid.log_odds[x, y] = -3
        start, goal = (0, 0), (12, 5)
        padding = obstacle_padding(grid, clearance_cells=3)
        self.assertIn(start, padding)
        for path in (route(grid, start, goal, clearance_cells=3),
                     nearest_frontier(grid, start, clearance_cells=3)):
            self.assertIsNotNone(path)
            inside = [cell for cell in path if cell in padding]
            self.assertEqual(inside, path[:len(inside)])
            distances = [padding[cell] for cell in inside]
            self.assertEqual(distances, sorted(set(distances)))
            self.assertNotIn(path[-1], padding)

    def test_padded_cells_stay_closed_away_from_the_start(self):
        grid = OccupancyMap()
        for x in range(0, 12):
            for y in range(-3, 4):
                grid.log_odds[x, y] = -3
        for y in range(-3, 3):
            grid.log_odds[6, y] = 5  # a wall leaving only a one-cell gap at y=3
        self.assertIsNone(route(grid, (1, 0), (11, 0), clearance_cells=2))
        self.assertIsNotNone(route(grid, (1, 0), (11, 0), clearance_cells=0))

    def test_single_hit_cell_blocks_itself_but_not_the_corridor(self):
        grid = OccupancyMap()
        for x in range(0, 20):
            grid.log_odds[x, -6] = grid.log_odds[x, 6] = 5
            for y in range(-5, 6):
                grid.log_odds[x, y] = -3
        grid.log_odds[10, 0] = 2  # one hit, partly cleared by later free rays
        path = route(grid, (1, 0), (18, 0), clearance_cells=4)
        self.assertIsNotNone(path)
        self.assertNotIn((10, 0), path)
        grid.log_odds[10, 0] = 5  # confirmed: now padded, closing the 2.2 m corridor
        self.assertIsNone(route(grid, (1, 0), (18, 0), clearance_cells=4))


if __name__ == "__main__":
    unittest.main()
