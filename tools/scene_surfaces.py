"""Distance from world points to the surfaces a generated world draws: every Shape's Mesh, at each of its
instances (Webots patch 0003: 3 x 4 float32 rows, local to world), and the home pad. Impostors stay out (range
sensors skip them: patch 0013); every band of visibilityRange counts (a LiDAR return may come off either level of
a ground-cover asset). Plain numpy, so it runs where the bag is read.

A mesh's triangles are hashed into the cubes their bounding boxes overlap, in its own frame, each cube at least
RADIUS in the world: a point's neighbouring cubes hold every triangle within RADIUS of it. Farther surfaces read
RADIUS.
"""
import re
from pathlib import Path

import numpy as np

RADIUS = 0.5
PAD = {"centre": (0.0, 0.0, -0.018), "height": 0.04, "radius": 0.8}  # the home pad (tools/build_forest_world.py)
REGION = 2.0  # world cubes pairing points with instances' bounding spheres


def shapes(world):
    """(mesh url, instances url or None) of every Shape with a Mesh, impostors left out."""
    found = []
    for start in (m.end() for m in re.finditer(r"\bShape \{", world)):
        depth, i = 1, start
        while depth:
            depth += {"{": 1, "}": -1}.get(world[i], 0)
            i += 1
        body = world[start:i]
        mesh = re.search(r'geometry Mesh \{ url "([^"]+)" \}', body)
        if mesh and "impostor TRUE" not in body:
            instances = re.search(r'instancesUrl \[ "([^"]+)" \]', body)
            found.append((mesh.group(1), instances.group(1) if instances else None))
    return found


def obj_triangles(path):
    """Triangles (n, 3, 3) of an OBJ of triangles (corners v, v/vt, v//vn or v/vt/vn, alike throughout)."""
    lines = Path(path).read_text().splitlines()
    vertices = [line[2:] for line in lines if line.startswith("v ")]
    faces = [line[2:] for line in lines if line.startswith("f ")]
    v = np.fromstring(" ".join(vertices), dtype=float, sep=" ").reshape(len(vertices), -1)[:, :3]  # maybe a colour
    f = np.fromstring(" ".join(faces).replace("/", " "), dtype=np.int64, sep=" ")
    if len(f) % (3 * len(faces)):
        raise ValueError(f"{path}: faces other than triangles, or corners unlike each other")
    f = f.reshape(len(faces), 3, -1)[:, :, 0]
    return v[np.where(f > 0, f - 1, f + len(v))]


def keys(cells):
    return ((cells[..., 0] + (1 << 20)) << 42) | ((cells[..., 1] + (1 << 20)) << 21) | (cells[..., 2] + (1 << 20))


class Grid:
    """Triangles (or boxes: corner pairs) hashed into the cubes of edge cell their bounding boxes overlap."""

    def __init__(self, triangles, cell):
        self.cell = cell
        self.low, self.high = triangles.min(axis=1), triangles.max(axis=1)
        lo = np.floor(self.low / cell).astype(np.int64)
        span = np.floor(self.high / cell).astype(np.int64) - lo + 1
        count = span.prod(axis=1)
        owner = np.repeat(np.arange(len(triangles)), count)
        k = np.arange(count.sum()) - np.repeat(np.cumsum(count) - count, count)
        s = span[owner]
        key = keys(lo[owner] + np.stack([k // (s[:, 1] * s[:, 2]), k // s[:, 2] % s[:, 1], k % s[:, 2]], axis=1))
        order = np.argsort(key, kind="stable")
        self.key, self.owner = key[order], owner[order]

    def candidates(self, points, reach):
        """(point index, triangle index) of the triangles whose bounding boxes lie within reach (at most cell) of
        each point, once per cube they share with the cubes within reach of the point."""
        scaled = points / self.cell
        base = np.floor(scaled).astype(np.int64)
        within = reach / self.cell
        side = {-1: scaled - base < within, 0: np.ones(scaled.shape, bool), 1: base + 1 - scaled < within}
        found_point, found_owner = [], []
        for offset in [(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)]:
            here = np.flatnonzero(side[offset[0]][:, 0] & side[offset[1]][:, 1] & side[offset[2]][:, 2])
            key = keys(base[here] + offset)
            lo, hi = np.searchsorted(self.key, key, "left"), np.searchsorted(self.key, key, "right")
            count = hi - lo
            point = np.repeat(here, count)
            owner = self.owner[np.repeat(lo - (np.cumsum(count) - count), count) + np.arange(count.sum())]
            gap = np.maximum(np.maximum(self.low[owner] - points[point], points[point] - self.high[owner]), 0.0)
            close = np.einsum("ij,ij->i", gap, gap) <= reach * reach
            found_point.append(point[close])
            found_owner.append(owner[close])
        return np.concatenate(found_point), np.concatenate(found_owner)


def triangle_distance(p, a, b, c):
    """Distance from points p to triangles abc, all (n, 3) (Ericson, Real-Time Collision Detection 5.1.5)."""
    ab, ac, ap = b - a, c - a, p - a
    d1, d2 = np.einsum("ij,ij->i", ab, ap), np.einsum("ij,ij->i", ac, ap)
    bp, cp = p - b, p - c
    d3, d4 = np.einsum("ij,ij->i", ab, bp), np.einsum("ij,ij->i", ac, bp)
    d5, d6 = np.einsum("ij,ij->i", ab, cp), np.einsum("ij,ij->i", ac, cp)
    va, vb, vc = d3 * d6 - d5 * d4, d5 * d2 - d1 * d6, d1 * d4 - d3 * d2
    with np.errstate(invalid="ignore", divide="ignore"):
        denominator = va + vb + vc
        v, w = vb / denominator, vc / denominator
        closest = a + ab * v[:, None] + ac * w[:, None]  # inside the face
        regions = [
            (d1 <= 0) & (d2 <= 0), a,
            (d3 >= 0) & (d4 <= d3), b,
            (d6 >= 0) & (d5 <= d6), c,
            (vc <= 0) & (d1 >= 0) & (d3 <= 0), a + ab * (d1 / (d1 - d3))[:, None],
            (vb <= 0) & (d2 >= 0) & (d6 <= 0), a + ac * (d2 / (d2 - d6))[:, None],
            (va <= 0) & (d4 - d3 >= 0) & (d5 - d6 >= 0),
            b + (c - b) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)))[:, None],
        ]
    for condition, point in reversed(list(zip(regions[::2], regions[1::2]))):
        closest = np.where(condition[:, None], point, closest)
    distance = np.linalg.norm(p - closest, axis=1)
    return np.where(np.isfinite(distance), distance, np.inf)  # a degenerate triangle is no surface


def pad_distance(points):
    x, y, z = (points - PAD["centre"]).T
    radial = np.maximum(np.hypot(x, y) - PAD["radius"], 0.0)
    vertical = np.maximum(np.abs(z) - PAD["height"] / 2, 0.0)
    return np.hypot(radial, vertical)


class Surfaces:
    """The surfaces of a world (its .wbt and the meshes beside it) near a set of places (rows x, y, z, within
    near metres of one): distance(points) is each point's distance to the nearest, at most RADIUS. Instances are
    turned, mirrored and scaled alike on every axis (as tools/build_forest_world.py places them), so distances are
    measured in each mesh's own frame and scaled."""

    def __init__(self, wbt, places, near, folder=None):
        folder = Path(folder or Path(wbt).parent)
        places = np.asarray(places, float)
        self.meshes = []  # (triangles in the mesh's frame, each instance's scale, world to mesh as 3 x 4 rows)
        centres, radii, owners = [], [], []
        for mesh_url, instances_url in shapes(Path(wbt).read_text()):
            rows = (np.fromfile(folder / instances_url, "<f4").astype(float).reshape(-1, 3, 4)
                    if instances_url else np.hstack([np.eye(3), np.zeros((3, 1))])[None])
            stretch = np.linalg.svd(rows[:, :, :3], compute_uv=False)
            if np.any(stretch[:, 0] > stretch[:, 2] * (1 + 1e-4)):
                raise ValueError(f"{instances_url}: an instance scaled unevenly")
            local = obj_triangles(folder / mesh_url)
            vertices = local.reshape(-1, 3)
            centre = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
            radius = np.linalg.norm(vertices - centre, axis=1).max()
            world_centre = np.einsum("nij,j->ni", rows[:, :, :3], centre) + rows[:, :, 3]
            world_radius = radius * stretch[:, 0] + RADIUS
            gap = np.min([np.linalg.norm(world_centre - p, axis=1) for p in places], axis=0) - world_radius
            keep = gap < near
            if not keep.any():
                continue
            rows, scale = rows[keep], stretch[keep, 0]
            inverse = np.linalg.inv(rows[:, :, :3])
            self.meshes.append((local, scale, np.concatenate([inverse, -inverse @ rows[:, :, 3:]], axis=2)))
            centres.append(world_centre[keep])
            radii.append(world_radius[keep])
            owners.append(np.column_stack([np.full(keep.sum(), len(self.meshes) - 1), np.arange(keep.sum())]))
        self.centres, self.radii, self.owners = np.concatenate(centres), np.concatenate(radii), np.concatenate(owners)
        self.grids = {}

    def _pairs(self, points):
        """(point index, sphere index) of every instance sphere holding a point: small spheres through cubes, the
        few large ones (terrain, whole trees) tested against every point."""
        small, large = np.flatnonzero(self.radii < 4 * REGION), np.flatnonzero(self.radii >= 4 * REGION)
        centres, radii = self.centres[small], self.radii[small, None]
        point, sphere = Grid(np.stack([centres - radii, centres + radii], axis=1), REGION).candidates(points, 0.0)
        points_, spheres_ = [point], [small[sphere]]
        for first in range(0, len(points), 1000):
            p, s = np.nonzero(np.linalg.norm(points[first:first + 1000, None] - self.centres[large], axis=2)
                              < self.radii[large])
            points_.append(p + first)
            spheres_.append(large[s])
        point, sphere = np.concatenate(points_), np.concatenate(spheres_)
        inside = np.linalg.norm(points[point] - self.centres[sphere], axis=1) < self.radii[sphere]
        return point[inside], sphere[inside]

    def distance(self, points, chunk=500_000):
        points = np.asarray(points, float)
        best = np.minimum(pad_distance(points), RADIUS)
        # Most points lie on a surface: first only the triangles within a fifth of RADIUS, then for the points
        # with none that near all within RADIUS.
        for reach in (RADIUS / 5, RADIUS):
            open_ = np.flatnonzero(best >= RADIUS / 5)
            point, sphere = self._pairs(points[open_])
            point = open_[point]
            mesh_of, instance_of = self.owners[sphere].T
            for m, (triangles, scale, inverse) in enumerate(self.meshes):
                pick = mesh_of == m
                if not pick.any():
                    continue
                p, instance = point[pick], instance_of[pick]
                local = np.einsum("nij,nj->ni", inverse[instance, :, :3], points[p]) + inverse[instance, :, 3]
                # RADIUS in the world is at most RADIUS / (least scale) here: cubes of that edge, or as large as
                # most triangles.
                if m not in self.grids:
                    cell = max(RADIUS / scale.min(), np.percentile(np.ptp(triangles, axis=1).max(axis=1), 90))
                    self.grids[m] = Grid(triangles, cell)
                pair, triangle = self.grids[m].candidates(local, reach / scale.min())
                for start in range(0, len(pair), chunk):
                    q, t = pair[start:start + chunk], triangle[start:start + chunk]
                    found = triangle_distance(local[q], triangles[t, 0], triangles[t, 1], triangles[t, 2])
                    np.minimum.at(best, p[q], found * scale[instance[q]])
        return np.minimum(best, RADIUS)
