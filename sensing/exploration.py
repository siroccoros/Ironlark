"""Frontier exploration and exit detection over the live sensor map.

Decisions use only the odometer's estimate and the occupancy map built from
LiDAR; the generated maze's truth is never read here.
"""
import math

from sensing.planning import nearest_frontier, route, widest_open_sector

# Confirming an exit needs open space this far out, not just around the drone.
EXIT_CONFIRM_RADIUS_M = 6.0
EXIT_CONFIRM_SECTOR_DEG = 200


class Explorer:
    def __init__(self, grid):
        self.map = grid
        self.frontier = None
        self.waypoint_cell = None
        self.waypoint_frontier = None
        self.frontier_best_distance = math.inf
        self.frontier_progress_time = None
        self.visited_frontiers = set()
        # Around frontiers abandoned without route progress. A preference, not proof of no way
        # through: they become eligible again when no other frontier remains.
        self.stalled_frontiers = set()
        self.replans = 0
        self.blocked_replans = 0
        self.stalled_replans = 0
        self.open_scans = 0
        self.exit_candidate_scans = 0
        self.exit_heading = None
        self.exit_start = None
        self.exit_start_time = None
        self.rejected_exits = []

    def plan(self, state):
        """Keep a frontier until reached or blocked; return the navigation target."""
        here = self.map.cell(*state["position"][:2])
        px, py = state["position"][:2]
        open_sector, open_heading = widest_open_sector(self.map, (px, py), with_heading=True)
        # A junction whose walls a maze loop removed is open for 3 m but walled within 6 m
        # (<= 25 deg in runs/20261114-135442-31363); beyond a real exit it stays >= 210 deg.
        far_sector, far_heading = widest_open_sector(self.map, (px, py), radius_m=EXIT_CONFIRM_RADIUS_M,
                                                     with_heading=True)
        if open_sector >= 360:
            # Nothing mapped within 3 m: that sector's heading is arbitrary, so face the open
            # side of the wider view instead of possibly back into the maze.
            open_heading = far_heading
        # Beyond an exit gate only the outer wall remains, so the odometry is degenerate along it
        # until landing (runs/20261118-205724-49600 stayed so for 14 s past the gate). That
        # reason does not bar a commitment; inside nine batch mazes no reading reached 180 deg.
        usable = state["healthy"] or state.get("reason") == "corridor degeneracy"
        candidate = (usable and math.hypot(px, py) > 10 and open_sector >= 180
                     and all(math.dist((px, py), rejected) > 2.5 for rejected in self.rejected_exits))
        if self.exit_heading is None:
            self.exit_candidate_scans = self.exit_candidate_scans + 1 if candidate else 0
            if self.exit_candidate_scans >= 2:
                self.exit_heading, self.exit_start = open_heading, (px, py)
                self.exit_start_time = state["time"]
        if self.exit_heading is not None:
            forward = ((px - self.exit_start[0]) * math.cos(self.exit_heading)
                       + (py - self.exit_start[1]) * math.sin(self.exit_heading))
            if state["time"] - self.exit_start_time > 30:
                # Unconfirmed after 30 s: not an exit. Exploration resumes and this spot cannot
                # trigger a commitment again.
                self.rejected_exits.append(self.exit_start)
                self.exit_heading = self.exit_start = self.exit_start_time = None
                self.exit_candidate_scans = self.open_scans = 0
            else:
                self.open_scans = (self.open_scans + 1 if forward >= 1.5 and open_sector >= 220
                                   and far_sector >= EXIT_CONFIRM_SECTOR_DEG else 0)
        escaped = self.exit_heading is not None and self.open_scans >= 3
        path = None
        if self.frontier is not None:
            if state["position"][2] < 0.5:
                # Mapping starts before takeoff; waiting on the ground is not a stalled route.
                self.frontier_progress_time = state["time"]
            distance = math.dist(here, self.frontier) * self.map.resolution
            if distance < 0.35:
                self.visited_frontiers.add(self.frontier)
                self.frontier = None
            else:
                path = route(self.map, here, self.frontier, clearance_cells=4)
                # Progress is the remaining route, not the straight line: maze routes often
                # lead away from their goal first (runs/20261114-132148-31363 was abandoned
                # while following its route at 0.25 m/s).
                remaining = (len(path) - 1) * self.map.resolution if path is not None else None
                if path is None:
                    self.visited_frontiers.add(self.frontier)
                    self.frontier = None
                    self.replans += 1
                    self.blocked_replans += 1
                elif remaining < self.frontier_best_distance - 0.3:
                    self.frontier_best_distance = remaining
                    self.frontier_progress_time = state["time"]
                elif (self.frontier_progress_time is not None
                      and state["time"] - self.frontier_progress_time > 20):
                    # A grid route can be traversable on paper yet physically unreachable.
                    fx, fy = self.frontier
                    self.stalled_frontiers.update((fx + dx, fy + dy)
                                                  for dx in range(-5, 6) for dy in range(-5, 6))
                    self.frontier = None
                    path = None
                    self.replans += 1
                    self.stalled_replans += 1
        if self.frontier is None and not escaped:
            path = None
            for excluded in (self.visited_frontiers | self.stalled_frontiers, self.visited_frontiers):
                path = (nearest_frontier(self.map, here, excluded, clearance_cells=4, min_steps=4)
                        or nearest_frontier(self.map, here, excluded, clearance_cells=4))
                if path is not None:
                    break
            if path is not None and path[-1] in self.stalled_frontiers:
                self.stalled_frontiers.clear()  # a fresh chance for every abandoned region
            if path is not None:
                self.frontier = path[-1]
                self.frontier_best_distance = math.inf
                self.frontier_progress_time = state["time"]
                self.replans += 1
        waypoint = path[min(4, len(path) - 1)] if path else None
        if path and self.waypoint_frontier == self.frontier and self.waypoint_cell is not None:
            remaining = math.dist(here, self.waypoint_cell) * self.map.resolution
            if (remaining > 0.35
                    and route(self.map, here, self.waypoint_cell, clearance_cells=4) is not None):
                waypoint = self.waypoint_cell
        self.waypoint_cell = waypoint
        self.waypoint_frontier = self.frontier
        waypoint_xy = ([(waypoint[0] + 0.5) * self.map.resolution,
                        (waypoint[1] + 0.5) * self.map.resolution] if waypoint else None)
        if self.exit_heading is not None:
            waypoint_xy = [self.exit_start[0] + 3 * math.cos(self.exit_heading),
                           self.exit_start[1] + 3 * math.sin(self.exit_heading)]
        target = {"time": state["time"], "state": "escaped" if escaped else
                  "exploring" if waypoint_xy else "no_frontier",
                  "waypoint_xy_m": waypoint_xy,
                  "odometry_healthy": state["healthy"],
                  "frontier": self.frontier, "replans": self.replans,
                  "blocked_replans": self.blocked_replans,
                  "stalled_replans": self.stalled_replans,
                  "open_sector_deg": open_sector,
                  "open_sector_6m_deg": far_sector,
                  "rejected_exits": len(self.rejected_exits),
                  "exit_commitment": self.exit_heading is not None,
                  "exit_commitment_age_s": (state["time"] - self.exit_start_time
                                            if self.exit_heading is not None else None),
                  "known_cells": len(self.map.log_odds)}
        return target
