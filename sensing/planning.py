"""Grid routes and frontiers over a sensor-built occupancy map.

Unknown cells are explicit. Frontier exploration targets reachable free cells
beside unknown space; a route through unknown cells must be requested explicitly.
"""
import heapq
import math
from collections import deque


NEIGHBOURS = ((1, 0), (-1, 0), (0, 1), (0, -1))
# A hit adds 3 log-odds and a free ray takes 1 away, so a cell at 4 or more was hit in at least
# two map updates. Lone single-hit cells inside a corridor (log-odds 1-2 in
# runs/20261114-134724-31363) otherwise pad a 0.8 m disc and close the corridor's centre lane.
PADDED_LOG_ODDS = 4


def widest_open_sector(grid, position, radius_m=3.0, bins=72, with_heading=False):
    """Largest angular gap without a nearby mapped obstacle, in degrees."""
    px, py = position
    blocked = [False] * bins
    for (x, y), odds in grid.log_odds.items():
        if odds <= 0:
            continue
        dx = (x + 0.5) * grid.resolution - px
        dy = (y + 0.5) * grid.resolution - py
        distance = math.hypot(dx, dy)
        if distance > radius_m:
            continue
        if distance < 0.25:
            return (0.0, 0.0) if with_heading else 0.0
        direction = math.atan2(dy, dx)
        half_width = math.asin(min(1.0, 0.25 / distance))
        for index in range(bins):
            angle = -math.pi + (index + 0.5) * 2 * math.pi / bins
            if abs((angle - direction + math.pi) % (2 * math.pi) - math.pi) <= half_width:
                blocked[index] = True
    longest = current = end = 0
    for index, occupied in enumerate(blocked * 2):
        current = 0 if occupied else current + 1
        if current > longest:
            longest, end = current, index
    width = min(longest, bins) * 360.0 / bins
    centre = end - (longest - 1) / 2
    heading = -math.pi + (centre + 0.5) * 2 * math.pi / bins
    return (width, heading) if with_heading else width


def adjacent(cell):
    x, y = cell
    for dx, dy in NEIGHBOURS:
        yield x + dx, y + dy


def obstacle_padding(grid, clearance_cells=1):
    """Cells within the drone clearance of a confirmed obstacle -> squared cell distance to it.

    Unconfirmed occupied cells are not padded, but routes still never enter them."""
    padding = {}
    for cell, odds in grid.log_odds.items():
        if odds < PADDED_LOG_ODDS:
            continue
        x, y = cell
        for dx in range(-clearance_cells, clearance_cells + 1):
            for dy in range(-clearance_cells, clearance_cells + 1):
                squared = dx * dx + dy * dy
                if squared <= clearance_cells * clearance_cells:
                    near = (x + dx, y + dy)
                    padding[near] = min(squared, padding.get(near, squared))
    return padding


def blocked_cells(grid, clearance_cells=1):
    """Pad measured obstacles by a configurable drone clearance."""
    return set(obstacle_padding(grid, clearance_cells))


def passable(padding, cell, nxt):
    """Outside the padding, or climbing out of it from a padded start.

    Drift or a newly mapped wall can leave the drone inside the clearance it is
    meant to keep. Treating its own cell as a dead end then hides every route;
    instead a search may leave the padding along strictly increasing distance
    from the nearest obstacle, which never moves it closer to a wall."""
    return nxt not in padding or (cell in padding and padding[nxt] > padding[cell])


def reconstruct(previous, goal):
    route = [goal]
    while previous[route[-1]] is not None:
        route.append(previous[route[-1]])
    return route[::-1]


def route(grid, start, goal, allow_unknown=False, clearance_cells=1):
    """A* route in grid cells. Unknown costs four times as much as observed free."""
    start, goal = tuple(start), tuple(goal)
    padding = obstacle_padding(grid, clearance_cells)
    if goal in padding or grid.state(goal) == "occupied":
        return None
    if not allow_unknown and grid.state(goal) == "unknown":
        return None
    known = [*grid.log_odds, start, goal]
    pad = 10
    xmin, xmax = min(cell[0] for cell in known) - pad, max(cell[0] for cell in known) + pad
    ymin, ymax = min(cell[1] for cell in known) - pad, max(cell[1] for cell in known) + pad
    best = {start: 0}
    previous = {start: None}
    queue = [(abs(goal[0] - start[0]) + abs(goal[1] - start[1]), 0, start)]
    while queue:
        _, cost, cell = heapq.heappop(queue)
        if cost != best[cell]:
            continue
        if cell == goal:
            return reconstruct(previous, goal)
        for nxt in adjacent(cell):
            if not (xmin <= nxt[0] <= xmax and ymin <= nxt[1] <= ymax) or not passable(padding, cell, nxt):
                continue
            state = grid.state(nxt)
            if state == "occupied" or (state == "unknown" and not allow_unknown):
                continue
            next_cost = cost + (4 if state == "unknown" else 1)
            if next_cost < best.get(nxt, math.inf):
                best[nxt] = next_cost
                previous[nxt] = cell
                estimate = abs(goal[0] - nxt[0]) + abs(goal[1] - nxt[1])
                heapq.heappush(queue, (next_cost + estimate, next_cost, nxt))
    return None


def nearest_frontier(grid, start, visited=(), clearance_cells=1, min_steps=1):
    """Shortest safe route to observed free space adjacent to the unknown."""
    start = tuple(start)
    padding = obstacle_padding(grid, clearance_cells)
    visited = set(visited)
    queue = deque([start])
    previous = {start: None}
    while queue:
        cell = queue.popleft()
        if (cell != start and cell not in visited and grid.state(cell) == "free"
                and cell not in padding and any(grid.state(nxt) == "unknown" for nxt in adjacent(cell))):
            path = reconstruct(previous, cell)
            if len(path) - 1 >= min_steps:
                return path
        for nxt in adjacent(cell):
            if nxt not in previous and passable(padding, cell, nxt) and grid.state(nxt) == "free":
                previous[nxt] = cell
                queue.append(nxt)
    return None
