"""Frontier progress and exit decisions of the live explorer, on synthetic sensor maps."""
import math
import unittest

from sensing.exploration import Explorer
from sensing.mapping import OccupancyMap
from sensing.planning import route


def pose(time, cell=None, xy=None, resolution=0.2):
    x, y = xy if xy is not None else ((cell[0] + 0.5) * resolution, (cell[1] + 0.5) * resolution)
    return {"time": time, "position": [x, y, 1.4], "healthy": True}


def u_corridor():
    """Walled U: east along the bottom, north up the east side, west along the top.

    Its only unknown edge is the top arm's open west end, above the start."""
    grid = OccupancyMap()
    free = {(x, y) for x in range(0, 41) for y in range(0, 11)}
    free |= {(x, y) for x in range(30, 41) for y in range(0, 41)}
    free |= {(x, y) for x in range(0, 41) for y in range(30, 41)}
    for x, y in free:
        grid.log_odds[x, y] = -3
    for x, y in free:
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                near = (x + dx, y + dy)
                if near not in free and not (near[0] < 0 and near[1] >= 30):
                    grid.log_odds[near] = 5
    return grid


def ring(grid, centre, radius_m, keep=lambda angle: True):
    for step in range(720):
        angle = step * math.pi / 360
        if keep(angle):
            grid.log_odds[grid.cell(centre[0] + radius_m * math.cos(angle),
                                    centre[1] + radius_m * math.sin(angle))] = 5


class ExplorationTests(unittest.TestCase):
    def test_route_leading_away_from_frontier_is_not_a_stall(self):
        # runs/20261114-132148-31363: the straight line to a frontier grew while the drone
        # followed its route, and the frontier was abandoned after 20 s.
        grid = u_corridor()
        explorer = Explorer(grid)
        start = (5, 5)
        target = explorer.plan(pose(0, start))
        frontier = tuple(target["frontier"])
        self.assertEqual(frontier[0], 0)
        self.assertGreaterEqual(frontier[1], 30)
        path = route(grid, start, frontier, clearance_cells=4)
        straight = [math.dist(cell, frontier) for cell in path]
        self.assertGreater(max(straight[:60]), straight[0] + 10)  # the route first leads away
        for second, cell in enumerate(path[1:], 1):
            target = explorer.plan(pose(second, cell))
            if explorer.frontier != frontier:
                break
        self.assertEqual(explorer.stalled_replans, 0)
        self.assertIn(frontier, explorer.visited_frontiers)
        self.assertGreater(second, 60)

    def test_hovering_without_route_progress_still_stalls(self):
        grid = u_corridor()
        explorer = Explorer(grid)
        for second in range(25):
            explorer.plan(pose(second, (5, 5)))
        self.assertEqual(explorer.stalled_replans, 1)
        # The only frontier was abandoned; with nothing else left it is tried again.
        target = explorer.plan(pose(25, (5, 5)))
        self.assertEqual(target["state"], "exploring")
        self.assertIsNotNone(explorer.frontier)

    def test_open_junction_walled_within_six_metres_is_rejected_as_exit(self):
        # runs/20261114-135442-31363: a loop junction with walls 1.8 m away read >= 180 deg
        # open at 3 m, and the drone landed inside the maze.
        grid = OccupancyMap()
        centre = (15.0, 0.0)
        ring(grid, centre, 4.5)
        explorer = Explorer(grid)
        committed = False
        for second in range(45):
            x = centre[0] + min(second, 10) * 0.2
            target = explorer.plan(pose(second, xy=(x, centre[1])))
            committed = committed or target["exit_commitment"]
            self.assertNotEqual(target["state"], "escaped")
        self.assertTrue(committed)
        self.assertEqual(len(explorer.rejected_exits), 1)
        self.assertFalse(target["exit_commitment"])

    def test_open_half_plane_beyond_walls_confirms_exit(self):
        grid = OccupancyMap()
        centre = (15.0, 0.0)
        # Maze walls behind (west of) the drone only.
        ring(grid, centre, 4.5, keep=lambda angle: math.cos(angle) < -0.3)
        explorer = Explorer(grid)
        states = []
        for second in range(20):
            target = explorer.plan(pose(second, xy=(centre[0] + second * 0.3, centre[1])))
            states.append(target["state"])
            if target["state"] == "escaped":
                break
        self.assertEqual(states[-1], "escaped")
        self.assertGreaterEqual(target["open_sector_6m_deg"], 200)

    def test_degenerate_odometry_beyond_the_gate_still_confirms_exit(self):
        # runs/20261118-205724-49600: past the gate only the outer wall remained, the odometry
        # stayed degenerate, and a healthy-only commitment never came.
        grid = OccupancyMap()
        centre = (15.0, 0.0)
        ring(grid, centre, 4.5, keep=lambda angle: math.cos(angle) < -0.3)
        explorer = Explorer(grid)
        for second in range(20):
            state = {**pose(second, xy=(centre[0] + second * 0.3, centre[1])),
                     "healthy": False, "reason": "corridor degeneracy"}
            target = explorer.plan(state)
            if target["state"] == "escaped":
                break
        self.assertEqual(target["state"], "escaped")

    def test_other_unhealthy_estimates_never_commit_to_an_exit(self):
        grid = OccupancyMap()
        centre = (15.0, 0.0)
        ring(grid, centre, 4.5, keep=lambda angle: math.cos(angle) < -0.3)
        explorer = Explorer(grid)
        for second in range(20):
            state = {**pose(second, xy=(centre[0] + second * 0.3, centre[1])),
                     "healthy": False, "reason": "scan mismatch"}
            target = explorer.plan(state)
            self.assertFalse(target["exit_commitment"])


if __name__ == "__main__":
    unittest.main()
