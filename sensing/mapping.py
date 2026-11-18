"""Small local occupancy map built solely from LiDAR returns and estimated pose.

Cells absent from ``log_odds`` are unknown. A ray marks traversed cells free and
its measured wall endpoint occupied. The map has no maze size or wall priors.
"""
import math

import numpy as np


class OccupancyMap:
    def __init__(self, resolution=0.2, lidar_offset=(0.0, 0.0, 0.083),
                 horizontal_resolution=900, free_ray_layer=30):
        self.resolution = resolution
        self.lidar_offset = lidar_offset
        self.horizontal_resolution = horizontal_resolution
        self.free_ray_layer = free_ray_layer
        self.log_odds = {}
        self.scans = 0

    def cell(self, x, y):
        return math.floor(x / self.resolution), math.floor(y / self.resolution)

    @staticmethod
    def ray(start, end):
        """Every grid cell crossed by a segment, including both ends."""
        x0, y0 = start
        x1, y1 = end
        dx, dy = abs(x1 - x0), abs(y1 - y0)
        sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
        error = dx - dy
        while True:
            yield x0, y0
            if (x0, y0) == (x1, y1):
                break
            twice = 2 * error
            if twice > -dy:
                error -= dy
                x0 += sx
            if twice < dx:
                error += dx
                y0 += sy

    def update(self, pose, cloud):
        """Use wall-height points; ground and wall tops cannot block a planar route.

        Pose is the odometer's local ENU [x, y, z], yaw. The LiDAR frame has the
        same axes as base_link, with the configured translation applied here.
        One endpoint per 10 cm sensor cell limits dense surfaces' vote count.
        """
        x, y, z = pose["position"]
        yaw = pose["yaw"]
        raw = np.asarray(cloud, dtype=float)
        points = raw[:, :3]
        points = points[np.isfinite(points).all(axis=1)]
        points = points + np.asarray(self.lidar_offset)
        distance = np.linalg.norm(points[:, :2], axis=1)
        points = points[(points[:, 2] > -0.25) & (points[:, 2] < 1.15)
                        & (z + points[:, 2] > 0.3)
                        & (distance > 0.45) & (distance < 16.0)]
        if len(points):
            bins = np.floor(points[:, :2] / 0.1).astype(np.int32)
            _, indices = np.unique(bins, axis=0, return_index=True)
            points = points[indices]
        c, s = math.cos(yaw), math.sin(yaw)
        origin = self.cell(x, y)
        free, occupied = set(), set()
        for px, py, _ in points:
            endpoint = self.cell(x + c * px - s * py, y + s * px + c * py)
            cells = list(self.ray(origin, endpoint))
            free.update(cells[:-1])
            occupied.add(endpoint)
        # This selected layer looks slightly down. A missing return within the
        # rig's range therefore measures open space along the ray; upper layers
        # can see over walls and must not be used this way.
        first = self.free_ray_layer * self.horizontal_resolution
        if len(raw) >= first + self.horizontal_resolution:
            layer = raw[first:first + self.horizontal_resolution]
            missing = np.flatnonzero(np.isinf(layer[:, 0]) & np.isinf(layer[:, 1]))
            for column in missing:
                azimuth = math.pi - (column + 0.5) * 2 * math.pi / self.horizontal_resolution + yaw
                endpoint = self.cell(x + 10 * math.cos(azimuth), y + 10 * math.sin(azimuth))
                free.update(self.ray(origin, endpoint))
        free.difference_update(occupied)
        for cell in free:
            self.log_odds[cell] = max(-5, self.log_odds.get(cell, 0) - 1)
        for cell in occupied:
            self.log_odds[cell] = min(5, self.log_odds.get(cell, 0) + 3)
        self.scans += 1
        return len(occupied)

    def state(self, cell):
        value = self.log_odds.get(cell)
        return "unknown" if value is None else "occupied" if value > 0 else "free"

    def snapshot(self):
        return {"resolution_m": self.resolution, "scans": self.scans,
                "cells": [[x, y, odds] for (x, y), odds in sorted(self.log_odds.items())]}
