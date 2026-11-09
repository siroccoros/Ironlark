"""A seed into a maze world: a Webots world of box walls on stock Webots, and truth.json beside it for the evaluator.

    generate_maze.py --seed 7 --out DIR     writes DIR/maze.wbt and DIR/truth.json

The maze is a grid of square cells carved by a randomized depth-first search (one route between any two cells), then
braided: a fraction (--loops) of the remaining interior walls is removed, so a blocked corridor can leave another
route. One boundary wall has an opening, the exit. The start cell is the one farthest from the exit unless given, and
sits at the world origin, where the drone and the home pad are.

Frames are Webots' world, ENU, in metres: columns run along +x (east), rows along +y (north). Collinear wall segments
merge into one box; every box overlaps its neighbours at the joints, so no ray passes between them. Each wall's
collision box is its visible box (boundingObject USE of the shape's geometry): what the LiDAR sees is what the drone
hits.

truth.json is for the evaluator only: navigation code never reads it. It holds the walls, the ground's extent (its top
at z = 0), the passages, the start, the exit, the bounds (leaving them through the exit is escaping) and the shortest
path, with, for each of the path's passages, the shortest route when that passage is blocked (null when nothing else
reaches the exit).

The same seed and parameters give the same bytes. Standard library only.
"""
import argparse
import json
import random
from collections import deque
from pathlib import Path

VERSION = 1
SIDES = {"north": (1, 0), "south": (-1, 0), "east": (0, 1), "west": (0, -1)}  # (row, column) steps
DEFAULTS = {"rows": 8, "cols": 8, "corridor_m": 2.5, "wall_height_m": 3.0, "wall_thickness_m": 0.2, "loops": 0.1}
GROUND_MARGIN_M = 10.0  # open ground around the maze, so the drone has somewhere to escape to
DRONE_Z_M = 0.09  # the drone's resting height in worlds/flight_foundation.wbt


def passage(a, b):
    """The passage between two adjacent cells, independent of direction."""
    return (a, b) if a < b else (b, a)


def neighbours(cell, rows, cols):
    row, col = cell
    for d_row, d_col in SIDES.values():
        r, c = row + d_row, col + d_col
        if 0 <= r < rows and 0 <= c < cols:
            yield (r, c)


def carve(rng, rows, cols):
    """A perfect maze: the set of open passages of a randomized depth-first search from cell (0, 0)."""
    open_ = set()
    seen = {(0, 0)}
    stack = [(0, 0)]
    while stack:
        cell = stack[-1]
        unseen = [n for n in neighbours(cell, rows, cols) if n not in seen]
        if not unseen:
            stack.pop()
            continue
        nxt = rng.choice(unseen)
        open_.add(passage(cell, nxt))
        seen.add(nxt)
        stack.append(nxt)
    return open_


def braid(rng, open_, rows, cols, loops):
    """Remove round(loops * remaining interior walls) more walls, each one adding a loop."""
    closed = sorted({passage((r, c), n) for r in range(rows) for c in range(cols)
                     for n in neighbours((r, c), rows, cols)} - open_)
    return open_ | set(rng.sample(closed, round(loops * len(closed))))


def distances(open_, rows, cols, source, blocked=None):
    """Breadth-first steps from source to every reachable cell, with its predecessor."""
    previous = {source: None}
    queue = deque([source])
    while queue:
        cell = queue.popleft()
        for n in neighbours(cell, rows, cols):
            p = passage(cell, n)
            if n not in previous and p in open_ and p != blocked:
                previous[n] = cell
                queue.append(n)
    return previous


def route(open_, rows, cols, start, goal, blocked=None):
    """The cells of a shortest route from start to goal, or None."""
    previous = distances(open_, rows, cols, start, blocked)
    if goal not in previous:
        return None
    cells = [goal]
    while cells[-1] != start:
        cells.append(previous[cells[-1]])
    return cells[::-1]


def layout(rng, rows, cols, loops, start=None):
    """The maze as cells: open passages, the exit (cell and side) and the start cell."""
    open_ = braid(rng, carve(rng, rows, cols), rows, cols, loops)
    boundary = [((r, c), side) for r in range(rows) for c in range(cols) for side, (dr, dc) in SIDES.items()
                if not (0 <= r + dr < rows and 0 <= c + dc < cols) and (r, c) != start]
    exit_cell, exit_side = rng.choice(boundary)
    if start is None:
        depth = {}
        for cell, previous in distances(open_, rows, cols, exit_cell).items():  # breadth-first order
            depth[cell] = 0 if previous is None else depth[previous] + 1
        start = max(sorted(depth), key=depth.get)  # the first farthest cell, row-major
    return open_, exit_cell, exit_side, start


def walls(open_, rows, cols, exit_cell, exit_side):
    """Every wall segment present: ('h', line, column) lies on the line below row `line`; ('v', line, row) on the
    line left of column `line`."""
    present = set()
    for r in range(rows + 1):
        for c in range(cols):
            if not (0 < r < rows and passage((r - 1, c), (r, c)) in open_):
                present.add(("h", r, c))
    for c in range(cols + 1):
        for r in range(rows):
            if not (0 < c < cols and passage((r, c - 1), (r, c)) in open_):
                present.add(("v", c, r))
    row, col = exit_cell
    present.discard({"north": ("h", row + 1, col), "south": ("h", row, col),
                     "east": ("v", col + 1, row), "west": ("v", col, row)}[exit_side])
    return present


def boxes(present, rows, cols, origin, pitch, thickness, height):
    """Wall segments merged along each grid line into axis-aligned boxes: (centre xyz, size xyz)."""
    x0, y0 = origin  # the corner where grid lines 0 meet, at the wall centrelines

    def runs(kind, lines, length):
        for line in range(lines):
            along = [i for i in range(length) if (kind, line, i) in present]
            while along:
                first = last = along.pop(0)
                while along and along[0] == last + 1:
                    last = along.pop(0)
                yield line, first, last

    out = []
    for line, first, last in runs("h", rows + 1, cols):
        span = (last - first + 1) * pitch + thickness
        out.append(((x0 + (first + last + 1) / 2 * pitch, y0 + line * pitch, height / 2), (span, thickness, height)))
    for line, first, last in runs("v", cols + 1, rows):
        span = (last - first + 1) * pitch + thickness
        out.append(((x0 + line * pitch, y0 + (first + last + 1) / 2 * pitch, height / 2), (thickness, span, height)))
    return [(tuple(round(v, 4) for v in centre), tuple(round(v, 4) for v in size)) for centre, size in out]


def generate(seed, rows=8, cols=8, corridor_m=2.5, wall_height_m=3.0, wall_thickness_m=0.2, loops=0.1, start=None):
    """truth (a dict, as truth.json holds it) for a seed and parameters."""
    if rows < 2 or cols < 2:
        raise ValueError("a maze needs at least 2 x 2 cells")
    if corridor_m <= 0 or wall_height_m <= 0 or wall_thickness_m <= 0:
        raise ValueError("corridor width, wall height and wall thickness must be positive")
    if not 0 <= loops <= 1:
        raise ValueError("loops is a fraction of the interior walls: 0 to 1")
    if start is not None:
        start = tuple(start)
        if not (0 <= start[0] < rows and 0 <= start[1] < cols):
            raise ValueError(f"start cell {start} is outside the {rows} x {cols} maze")
    rng = random.Random(seed)
    open_, exit_cell, exit_side, start = layout(rng, rows, cols, loops, start)
    pitch = corridor_m + wall_thickness_m

    def centre(cell):  # cell centres, the start cell at the origin
        return [round((cell[1] - start[1]) * pitch, 4), round((cell[0] - start[0]) * pitch, 4)]

    origin = (-(start[1] + 0.5) * pitch, -(start[0] + 0.5) * pitch)
    lo = [round(origin[0] - wall_thickness_m / 2, 4), round(origin[1] - wall_thickness_m / 2, 4)]
    hi = [round(origin[0] + cols * pitch + wall_thickness_m / 2, 4),
          round(origin[1] + rows * pitch + wall_thickness_m / 2, 4)]
    d_row, d_col = SIDES[exit_side]
    reach = pitch / 2 + wall_thickness_m / 2  # from a boundary cell's centre to the boundary's outer face
    exit_centre = centre(exit_cell)
    gate = [round(exit_centre[0] + d_col * reach, 4), round(exit_centre[1] + d_row * reach, 4)]
    half = corridor_m / 2
    opening = [[round(gate[0] - abs(d_row) * half, 4), round(gate[1] - abs(d_col) * half, 4)],
               [round(gate[0] + abs(d_row) * half, 4), round(gate[1] + abs(d_col) * half, 4)]]

    def length(cells):
        return round((len(cells) - 1) * pitch + reach, 4)

    cells = route(open_, rows, cols, start, exit_cell)
    blocked = []
    for a, b in zip(cells, cells[1:]):
        detour = route(open_, rows, cols, start, exit_cell, blocked=passage(a, b))
        blocked.append({"passage": [list(a), list(b)], "length_m": None if detour is None else length(detour)})
    return {
        "version": VERSION,
        "generator": "tools/generate_maze.py",
        "frame": "Webots world, ENU, metres; columns along +x, rows along +y; the start cell's centre at the origin",
        "seed": seed,
        "parameters": {"rows": rows, "cols": cols, "corridor_m": corridor_m, "wall_height_m": wall_height_m,
                       "wall_thickness_m": wall_thickness_m, "loops": loops},
        "cell_pitch_m": round(pitch, 4),
        "bounds": {"min": lo, "max": hi},
        "ground": {"min": [round(v - GROUND_MARGIN_M, 4) for v in lo],
                   "max": [round(v + GROUND_MARGIN_M, 4) for v in hi], "top_z": 0.0, "thickness_m": 0.2},
        "walls": [{"centre": list(c), "size": list(s)}
                  for c, s in boxes(walls(open_, rows, cols, exit_cell, exit_side), rows, cols, origin, pitch,
                                    wall_thickness_m, wall_height_m)],
        "passages": [[list(a), list(b)] for a, b in sorted(open_)],
        "start": {"cell": list(start), "position": centre(start)},
        "exit": {"cell": list(exit_cell), "side": exit_side, "gate": gate, "opening": opening},
        "shortest_path": {"cells": [list(c) for c in cells], "points": [centre(c) for c in cells] + [gate],
                          "length_m": length(cells), "if_blocked": blocked},
    }


def number(v):
    """A coordinate for the world file: to the tenth of a millimetre, without trailing zeros."""
    text = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if text == "-0" else text


def encode(value, level=0):
    """JSON with one wall, passage or path point per line."""
    indent = " " * level
    if isinstance(value, dict):
        items = [f"{indent} {json.dumps(k)}: {encode(v, level + 1)}" for k, v in value.items()]
        return "{\n" + ",\n".join(items) + f"\n{indent}}}"
    if isinstance(value, list) and value and isinstance(value[0], (dict, list)):
        items = [f"{indent} {json.dumps(v, ensure_ascii=False)}" for v in value]
        return "[\n" + ",\n".join(items) + f"\n{indent}]"
    return json.dumps(value, ensure_ascii=False)


def world(truth, proto="../protos/IronlarkDrone.proto"):
    """The Webots world (text) for a truth dict."""
    lo, hi = truth["ground"]["min"], truth["ground"]["max"]
    centre = [(lo[i] + hi[i]) / 2 for i in range(2)]
    ground = [hi[i] - lo[i] for i in range(2)]
    p = truth["parameters"]
    out = [
        "#VRML_SIM R2025a utf8",
        f"# Generated by tools/generate_maze.py, seed {truth['seed']}: {p['rows']} x {p['cols']} cells, corridors "
        f"{number(p['corridor_m'])} m, walls {number(p['wall_height_m'])} m, loops {number(p['loops'])}.",
        "# Do not edit: regenerate. The evaluator's truth is truth.json beside it.",
        "",
        f'EXTERNPROTO "{proto}"',
        "",
        "WorldInfo {",
        f'  title "Ironlark — maze, seed {truth["seed"]}"',
        "  basicTimeStep 2",
        "  FPS 30",
        "  randomSeed 7",
        "}",
        "Viewpoint {",
        "  orientation -0.174545 0.356882 0.917698 0.979312",
        "  position -7 -9 6",
        '  follow "Ironlark"',
        "}",
        "Background {",
        "  skyColor [ 0.65 0.78 0.9 ]",
        "}",
        "DirectionalLight {",
        "  direction -0.4 0.3 -1",
        "  intensity 1",
        "  ambientIntensity 0.5",
        "}",
        "Solid {",
        f"  translation {number(round(centre[0], 4))} {number(round(centre[1], 4))} -0.1",
        "  children [",
        "    Shape {",
        "      appearance PBRAppearance {",
        "        baseColor 0.32 0.42 0.33",
        "        roughness 1",
        "        metalness 0",
        "      }",
        f"      geometry DEF GROUND_BOX Box {{ size {number(round(ground[0], 4))} {number(round(ground[1], 4))} 0.2 }}",
        "    }",
        "  ]",
        '  name "ground"',
        "  boundingObject USE GROUND_BOX",
        "}",
        "# The home pad, on the start cell. Its top is flush with the ground; no navigation marker.",
        "Solid {",
        "  translation 0 0 0.001",
        "  children [",
        "    Shape {",
        "      appearance PBRAppearance {",
        "        baseColor 0.9 0.65 0.15",
        "        roughness 1",
        "        metalness 0",
        "      }",
        "      geometry Cylinder { height 0.002 radius 0.8 }",
        "    }",
        "  ]",
        '  name "home pad"',
        "}",
    ]
    for i, wall in enumerate(truth["walls"]):
        x, y, z = wall["centre"]
        sx, sy, sz = wall["size"]
        out += [
            "Solid {",
            f"  translation {number(x)} {number(y)} {number(z)}",
            "  children [",
            "    Shape {",
            "      appearance PBRAppearance {",
            "        baseColor 0.62 0.6 0.56",
            "        roughness 0.9",
            "        metalness 0",
            "      }",
            f"      geometry DEF WALL_{i} Box {{ size {number(sx)} {number(sy)} {number(sz)} }}",
            "    }",
            "  ]",
            f'  name "wall {i}"',
            f"  boundingObject USE WALL_{i}",
            "}",
        ]
    out += [
        "DEF IRONLARK_DRONE IronlarkDrone {",
        f"  translation 0 0 {number(DRONE_Z_M)}",
        '  name "Ironlark"',
        '  controller "flight_bridge"',
        "}",
        "Robot {",
        '  name "flight evaluator"',
        "  supervisor TRUE",
        '  controller "flight_observer"',
        "}",
        "",
    ]
    return "\n".join(out)


def write(out_dir, truth, proto="../protos/IronlarkDrone.proto"):
    """DIR/maze.wbt and DIR/truth.json; returns their paths."""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    wbt, truth_json = out_dir / "maze.wbt", out_dir / "truth.json"
    wbt.write_text(world(truth, proto))
    truth_json.write_text(encode(truth) + "\n")
    return wbt, truth_json


def cell(text):
    row, _, col = text.partition(",")
    return int(row), int(col)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--out", required=True, help="directory for maze.wbt and truth.json")
    parser.add_argument("--rows", type=int, default=DEFAULTS["rows"])
    parser.add_argument("--cols", type=int, default=DEFAULTS["cols"])
    parser.add_argument("--corridor", type=float, default=DEFAULTS["corridor_m"], help="corridor width, m")
    parser.add_argument("--wall-height", type=float, default=DEFAULTS["wall_height_m"], help="m")
    parser.add_argument("--wall-thickness", type=float, default=DEFAULTS["wall_thickness_m"], help="m")
    parser.add_argument("--loops", type=float, default=DEFAULTS["loops"],
                        help="fraction of the remaining interior walls removed after carving, 0 to 1")
    parser.add_argument("--start", type=cell, help="start cell ROW,COL (default: the cell farthest from the exit)")
    parser.add_argument("--proto", default="../protos/IronlarkDrone.proto",
                        help="the drone's EXTERNPROTO path as the world names it")
    args = parser.parse_args(argv)
    try:
        truth = generate(args.seed, args.rows, args.cols, args.corridor, args.wall_height, args.wall_thickness,
                         args.loops, args.start)
    except ValueError as error:
        parser.error(str(error))
    wbt, truth_json = write(args.out, truth, args.proto)
    path = truth["shortest_path"]
    print(f"{wbt}: {len(truth['walls'])} walls; exit {truth['exit']['side']} of cell {truth['exit']['cell']}, "
          f"shortest path {path['length_m']} m over {len(path['cells'])} cells")
    print(truth_json)


if __name__ == "__main__":
    main()
