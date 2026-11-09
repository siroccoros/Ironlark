"""The generated maze: the same seed gives the same world, and truth.json describes the walls the world contains.

Geometry is checked on the boxes themselves, not on the generator's grid: a passage is open when the point between two
cell centres lies in no wall, the exit when its gate does, and every route in truth.json is walked through open space.
"""
import contextlib
import io
import json
import re
import sys
import tempfile
import unittest
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "tools"))

import generate_maze  # noqa: E402

SEEDS = range(20)


def inside(point, wall, margin=0.0):
    """Whether an (x, y) point lies within a wall's footprint, grown by margin."""
    return all(abs(point[i] - wall["centre"][i]) <= wall["size"][i] / 2 + margin for i in range(2))


def blocked(point, truth, margin=0.0):
    return any(inside(point, wall, margin) for wall in truth["walls"])


def cell_centre(truth, cell):
    pitch, start = truth["cell_pitch_m"], truth["start"]["cell"]
    return ((cell[1] - start[1]) * pitch, (cell[0] - start[0]) * pitch)


def between(truth, a, b):
    (ax, ay), (bx, by) = cell_centre(truth, a), cell_centre(truth, b)
    return ((ax + bx) / 2, (ay + by) / 2)


def open_passages(truth):
    """The passages as the walls leave them: every pair of adjacent cells with no wall between their centres."""
    rows, cols = truth["parameters"]["rows"], truth["parameters"]["cols"]
    found = set()
    for r in range(rows):
        for c in range(cols):
            for n in ((r + 1, c), (r, c + 1)):
                if n[0] < rows and n[1] < cols and not blocked(between(truth, (r, c), n), truth):
                    found.add(((r, c), n))
    return found


def steps(passages, source, goal):
    """Breadth-first steps from source to goal over the passages, or None."""
    links = {}
    for a, b in passages:
        links.setdefault(a, []).append(b)
        links.setdefault(b, []).append(a)
    depth, queue = {source: 0}, deque([source])
    while queue:
        cell = queue.popleft()
        for n in links.get(cell, []):
            if n not in depth:
                depth[n] = depth[cell] + 1
                queue.append(n)
    return depth.get(goal)


def tuples(pairs):
    return {(tuple(a), tuple(b)) for a, b in pairs}


class Determinism(unittest.TestCase):
    def test_the_same_seed_writes_the_same_bytes(self):
        with tempfile.TemporaryDirectory() as one, tempfile.TemporaryDirectory() as two:
            first = generate_maze.write(one, generate_maze.generate(11))
            second = generate_maze.write(two, generate_maze.generate(11))
            for a, b in zip(first, second):
                self.assertEqual(a.read_bytes(), b.read_bytes(), a.name)

    def test_different_seeds_give_different_mazes(self):
        self.assertNotEqual(generate_maze.generate(1)["walls"], generate_maze.generate(2)["walls"])


class Geometry(unittest.TestCase):
    def test_the_walls_leave_open_exactly_the_passages_truth_names(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            self.assertEqual(open_passages(truth), tuples(truth["passages"]), f"seed {seed}")

    def test_the_boundary_is_closed_but_for_the_exit(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            rows, cols, half = truth["parameters"]["rows"], truth["parameters"]["cols"], truth["cell_pitch_m"] / 2
            gates = []
            for r in range(rows):
                for c in range(cols):
                    x, y = cell_centre(truth, (r, c))
                    for d_row, d_col in generate_maze.SIDES.values():
                        if not (0 <= r + d_row < rows and 0 <= c + d_col < cols):
                            point = (x + d_col * half, y + d_row * half)
                            if not blocked(point, truth):
                                gates.append(point)
            self.assertEqual(len(gates), 1, f"seed {seed}: openings at {gates}")
            self.assertFalse(blocked(truth["exit"]["gate"], truth), f"seed {seed}: the gate is walled")

    def test_the_walls_stand_on_the_ground_at_the_set_height_and_thickness(self):
        truth = generate_maze.generate(3, wall_height_m=4.0, wall_thickness_m=0.3)
        for wall in truth["walls"]:
            self.assertAlmostEqual(wall["centre"][2] - wall["size"][2] / 2, 0)
            self.assertAlmostEqual(wall["size"][2], 4.0)
            self.assertAlmostEqual(min(wall["size"][:2]), 0.3)

    def test_every_joint_is_solid(self):
        """No gap a LiDAR ray could pass through: wherever a wall meets a grid node, the node's whole
        thickness-by-thickness square is wall."""
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            rows, cols = truth["parameters"]["rows"], truth["parameters"]["cols"]
            pitch, t = truth["cell_pitch_m"], truth["parameters"]["wall_thickness_m"]
            x0, y0 = (c - pitch / 2 for c in cell_centre(truth, (0, 0)))
            for i in range(cols + 1):
                for j in range(rows + 1):
                    node = (x0 + i * pitch, y0 + j * pitch)
                    arms = [(node[0] + dx, node[1] + dy)
                            for dx, dy in ((pitch / 2, 0), (-pitch / 2, 0), (0, pitch / 2), (0, -pitch / 2))]
                    if not any(blocked(arm, truth) for arm in arms):
                        continue
                    for dx in (-0.49 * t, 0, 0.49 * t):
                        for dy in (-0.49 * t, 0, 0.49 * t):
                            self.assertTrue(blocked((node[0] + dx, node[1] + dy), truth),
                                            f"seed {seed}: a gap at node {(i, j)}")

    def test_the_start_is_clear_at_the_origin(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            self.assertEqual(truth["start"]["position"], [0.0, 0.0])
            clearance = truth["parameters"]["corridor_m"] / 2 - 0.01
            self.assertFalse(blocked((0, 0), truth, clearance), f"seed {seed}")

    def test_a_given_start_is_kept(self):
        truth = generate_maze.generate(5, start=(3, 4))
        self.assertEqual(truth["start"]["cell"], [3, 4])
        self.assertEqual(truth["shortest_path"]["cells"][0], [3, 4])


class Topology(unittest.TestCase):
    def test_no_loops_gives_a_perfect_maze(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed, loops=0)
            rows, cols = truth["parameters"]["rows"], truth["parameters"]["cols"]
            self.assertEqual(len(truth["passages"]), rows * cols - 1)
            self.assertTrue(all(b["length_m"] is None for b in truth["shortest_path"]["if_blocked"]),
                            f"seed {seed}: a perfect maze has one route")

    def test_loops_add_passages_and_alternative_routes(self):
        with_alternatives = 0
        for seed in SEEDS:
            truth = generate_maze.generate(seed, loops=0.1)
            rows, cols = truth["parameters"]["rows"], truth["parameters"]["cols"]
            interior = rows * (cols - 1) + cols * (rows - 1)
            self.assertEqual(len(truth["passages"]), rows * cols - 1 + round(0.1 * (interior - rows * cols + 1)))
            with_alternatives += any(b["length_m"] is not None for b in truth["shortest_path"]["if_blocked"])
        self.assertGreater(with_alternatives, len(SEEDS) // 2)

    def test_every_cell_is_reachable(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            passages, exit_cell = open_passages(truth), tuple(truth["exit"]["cell"])
            rows, cols = truth["parameters"]["rows"], truth["parameters"]["cols"]
            for r in range(rows):
                for c in range(cols):
                    self.assertIsNotNone(steps(passages, (r, c), exit_cell), f"seed {seed}: cell {(r, c)}")


class ShortestPath(unittest.TestCase):
    def test_the_path_walks_open_passages_from_start_to_exit_and_is_shortest(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            path, passages = truth["shortest_path"], open_passages(truth)
            cells = [tuple(c) for c in path["cells"]]
            self.assertEqual(cells[0], tuple(truth["start"]["cell"]))
            self.assertEqual(cells[-1], tuple(truth["exit"]["cell"]))
            for a, b in zip(cells, cells[1:]):
                self.assertIn(generate_maze.passage(a, b), passages, f"seed {seed}: {a} -> {b} is walled")
            self.assertEqual(len(cells) - 1, steps(passages, cells[0], cells[-1]), f"seed {seed}")

    def test_its_points_and_length_agree_with_its_cells(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            path = truth["shortest_path"]
            points = path["points"]
            self.assertEqual(points[-1], truth["exit"]["gate"])
            walked = sum(((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5 for a, b in zip(points, points[1:]))
            self.assertAlmostEqual(walked, path["length_m"], places=3)
            for a, b in zip(points, points[1:]):  # every straight leg through open space
                for k in range(1, 20):
                    p = (a[0] + (b[0] - a[0]) * k / 20, a[1] + (b[1] - a[1]) * k / 20)
                    self.assertFalse(blocked(p, truth), f"seed {seed}: {p}")

    def test_a_detour_is_never_shorter_and_null_only_when_the_exit_is_cut_off(self):
        for seed in SEEDS:
            truth = generate_maze.generate(seed)
            passages = open_passages(truth)
            start, exit_cell = tuple(truth["start"]["cell"]), tuple(truth["exit"]["cell"])
            for blocked_ in truth["shortest_path"]["if_blocked"]:
                cut = generate_maze.passage(*map(tuple, blocked_["passage"]))
                remaining = steps(passages - {cut}, start, exit_cell)
                if blocked_["length_m"] is None:
                    self.assertIsNone(remaining)
                else:
                    self.assertGreaterEqual(blocked_["length_m"], truth["shortest_path"]["length_m"])
                    pitch = truth["cell_pitch_m"]
                    self.assertAlmostEqual(blocked_["length_m"], truth["shortest_path"]["length_m"]
                                           + (remaining - len(truth["shortest_path"]["cells"]) + 1) * pitch, 3)


class World(unittest.TestCase):
    def setUp(self):
        self.truth = generate_maze.generate(4)
        self.text = generate_maze.world(self.truth)

    def solids(self):
        """(translation x y z, geometry DEF, box size x y z, name, boundingObject USE) of each top-level box Solid."""
        found = []
        for block in self.text.split("\nSolid {\n")[1:]:
            block = block.split("\n}\n")[0]
            box = re.search(r"geometry DEF (\S+) Box \{ size (\S+) (\S+) (\S+) \}", block)
            if box:
                found.append((*re.search(r"^  translation (\S+) (\S+) (\S+)$", block, re.M).groups(),
                              box[1], box[2], box[3], box[4], re.search(r'^  name "([^"]+)"$', block, re.M)[1],
                              re.search(r"^  boundingObject USE (\S+)$", block, re.M)[1]))
        return found

    def test_every_wall_collides_with_its_own_visible_box(self):
        walls = [s for s in self.solids() if s[7].startswith("wall ")]
        self.assertEqual(len(walls), len(self.truth["walls"]))
        for solid in walls:
            self.assertEqual(solid[3], solid[8])

    def test_the_world_places_the_walls_truth_names(self):
        walls = [s for s in self.solids() if s[7].startswith("wall ")]
        for solid, wall in zip(walls, self.truth["walls"]):
            self.assertEqual([float(v) for v in solid[0:3]], wall["centre"])
            self.assertEqual([float(v) for v in solid[4:7]], wall["size"])

    def test_the_ground_reaches_past_the_maze_where_truth_says(self):
        ground = next(s for s in self.solids() if s[7] == "ground")
        lo, hi = self.truth["bounds"]["min"], self.truth["bounds"]["max"]
        extent = self.truth["ground"]
        self.assertAlmostEqual(float(ground[2]) + float(ground[6]) / 2, extent["top_z"])
        for i in range(2):
            centre, size = float(ground[i]), float(ground[4 + i])
            self.assertLessEqual(centre - size / 2, lo[i] - 5)
            self.assertGreaterEqual(centre + size / 2, hi[i] + 5)
            self.assertAlmostEqual(centre - size / 2, extent["min"][i], places=3)
            self.assertAlmostEqual(centre + size / 2, extent["max"][i], places=3)

    def test_the_drone_and_the_observer_are_in_place(self):
        self.assertIn("DEF IRONLARK_DRONE IronlarkDrone {\n  translation 0 0 0.09", self.text)
        self.assertIn('controller "flight_observer"', self.text)
        self.assertIn('EXTERNPROTO "../protos/IronlarkDrone.proto"', self.text)

    def test_the_world_carries_no_truth_beyond_its_geometry(self):
        """Exit, start cell and shortest path are the evaluator's: the world never names them."""
        for word in ("exit", "shortest", "gate", "truth.json\""):
            self.assertNotIn(word, self.text.lower().replace("truth.json beside it", ""))


class Command(unittest.TestCase):
    def test_writes_both_files(self):
        with tempfile.TemporaryDirectory() as out, contextlib.redirect_stdout(io.StringIO()):
            generate_maze.main(["--seed", "9", "--out", out, "--rows", "5", "--cols", "6", "--loops", "0"])
            truth = json.loads(Path(out, "truth.json").read_text())
            self.assertEqual((truth["seed"], truth["parameters"]["rows"], truth["parameters"]["cols"]), (9, 5, 6))
            self.assertEqual(Path(out, "maze.wbt").read_text(), generate_maze.world(truth))

    def test_refuses_impossible_parameters(self):
        for kwargs in ({"rows": 1}, {"corridor_m": 0}, {"loops": 1.5}, {"start": (8, 0)}):
            with self.assertRaises(ValueError, msg=kwargs):
                generate_maze.generate(1, **kwargs)


if __name__ == "__main__":
    unittest.main()
