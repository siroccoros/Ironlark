"""Record evaluation ground truth, enforce GPS-denied zones, and drive the smart spectator camera.

Never sends commands or leaks truth to autonomy.
Hotkeys in the Webots 3D window:
  1  Smart Chase (auto-transitions to tight indoor chase cam inside buildings so the roof never blocks view)
  2  Close Tail Cam
  3  Outdoor Overview
  4  Free Mouse (releases Viewpoint control)
"""

import json
import math
import os
import time
from pathlib import Path

from controller import Supervisor


def in_box(position, box):
    return (box["x"][0] <= position[0] <= box["x"][1]
            and box["y"][0] <= position[1] <= box["y"][1]
            and box["z"][0] <= position[2] <= box["z"][1])


def look_at(eye, target):
    """Webots Viewpoint SFRotation mapping +X forward and +Z up from eye toward target."""
    fx, fy, fz = (b - a for a, b in zip(eye, target))
    n = math.sqrt(fx * fx + fy * fy + fz * fz) or 1.0
    fx, fy, fz = fx / n, fy / n, fz / n
    rx, ry, rz = fy, -fx, 0.0  # right = forward x world_up
    rn = math.sqrt(rx * rx + ry * ry + rz * rz)
    if rn < 1e-8:
        rx, ry, rz, rn = 0.0, -1.0, 0.0, 1.0
    rx, ry, rz = rx / rn, ry / rn, rz / rn
    ux, uy, uz = (ry * fz - rz * fy, rz * fx - rx * fz, rx * fy - ry * fx)
    m = [[fx, -rx, ux], [fy, -ry, uy], [fz, -rz, uz]]
    tr = m[0][0] + m[1][1] + m[2][2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw, qx, qy, qz = 0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        qw, qx, qy, qz = (m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        qw, qx, qy, qz = (m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        qw, qx, qy, qz = (m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s
    ang = 2.0 * math.acos(max(-1.0, min(1.0, qw)))
    s = math.sqrt(max(1e-12, 1.0 - qw * qw))
    return [qx / s, qy / s, qz / s, ang]


def _wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _extract_obstacle_aabbs(robot, pad=0.38):
    """Collect world-space AABBs for all solid & tree obstacles so the camera spring-arm never clips them."""
    ignore = {"ground", "home pad", "Ironlark", "flight evaluator"}
    ignore_suffixes = ("_floor", "_apron", "_rim", "_lane", "_arch")
    boxes = []
    root = robot.getRoot()
    if not root:
        return boxes

    def add_box(pos, rot, sx, sy, sz):
        hx = 0.5 * (abs(rot[0]) * sx + abs(rot[1]) * sy + abs(rot[2]) * sz) + pad
        hy = 0.5 * (abs(rot[3]) * sx + abs(rot[4]) * sy + abs(rot[5]) * sz) + pad
        hz = 0.5 * (abs(rot[6]) * sx + abs(rot[7]) * sy + abs(rot[8]) * sz) + pad
        boxes.append((pos[0] - hx, pos[0] + hx, pos[1] - hy, pos[1] + hy, pos[2] - hz, pos[2] + hz))

    def add_pose_geom(pos, rot, pnode):
        tf = pnode.getField("translation")
        lt = tf.getSFVec3f() if tf else (0.0, 0.0, 0.0)
        cx = pos[0] + rot[0] * lt[0] + rot[1] * lt[1] + rot[2] * lt[2]
        cy = pos[1] + rot[3] * lt[0] + rot[4] * lt[1] + rot[5] * lt[2]
        cz = pos[2] + rot[6] * lt[0] + rot[7] * lt[1] + rot[8] * lt[2]
        pch = pnode.getField("children")
        if not pch or pch.getCount() == 0:
            return
        geom = pch.getMFNode(0)
        if geom is None:
            return
        gtype = geom.getTypeName()
        if gtype == "Box":
            sf = geom.getField("size")
            if sf:
                add_box((cx, cy, cz), rot, *sf.getSFVec3f())
        elif gtype == "Cylinder":
            r = geom.getField("radius").getSFFloat() + pad
            h = 0.5 * geom.getField("height").getSFFloat() + pad
            boxes.append((cx - r, cx + r, cy - r, cy + r, cz - h, cz + h))
        elif gtype == "Cone":
            r = geom.getField("bottomRadius").getSFFloat() + pad
            h = 0.5 * geom.getField("height").getSFFloat() + pad
            boxes.append((cx - r, cx + r, cy - r, cy + r, cz - h, cz + h))

    children = root.getField("children")
    for i in range(children.getCount()):
        node = children.getMFNode(i)
        if node is None:
            continue
        ntype = node.getTypeName()
        # Official Webots tree PROTOs: approximate trunk+canopy AABB so camera orbits around them cleanly.
        if ntype in ("Pine", "Oak", "Cypress"):
            pos = node.getPosition()
            r = 1.6 if ntype == "Oak" else 1.2
            boxes.append((pos[0] - r, pos[0] + r, pos[1] - r, pos[1] + r, pos[2], pos[2] + 9.0))
            continue
        if ntype != "Solid":
            continue
        nf = node.getField("name")
        name = nf.getSFString() if nf else ""
        if name in ignore or any(name.endswith(s) for s in ignore_suffixes):
            continue
        bf = node.getField("boundingObject")
        bo = bf.getSFNode() if bf else None
        if bo is None:
            continue
        pos = node.getPosition()
        rot = node.getOrientation()
        btype = bo.getTypeName()
        if btype == "Box":
            sf = bo.getField("size")
            if sf:
                add_box(pos, rot, *sf.getSFVec3f())
        elif btype == "Pose":
            add_pose_geom(pos, rot, bo)
        elif btype == "Group":
            gch = bo.getField("children")
            if not gch:
                continue
            for j in range(gch.getCount()):
                pnode = gch.getMFNode(j)
                if pnode is not None and pnode.getTypeName() == "Pose":
                    add_pose_geom(pos, rot, pnode)
    return boxes


def _axis_angle_matrix(rot):
    x, y, z, a = rot
    n = math.sqrt(x * x + y * y + z * z) or 1.0
    x, y, z = x / n, y / n, z / n
    c, s, t = math.cos(a), math.sin(a), 1.0 - math.cos(a)
    return [[t * x * x + c, t * x * y - s * z, t * x * z + s * y],
            [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
            [t * x * z - s * y, t * y * z + s * x, t * z * z + c]]


def _compose(T, local_t, local_R):
    """World transform (t, R) of a child at local (t, R) under parent T = (t, R)."""
    pt, pR = T
    t = [pt[i] + sum(pR[i][k] * local_t[k] for k in range(3)) for i in range(3)]
    R = [[sum(pR[i][k] * local_R[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return t, R


def export_collision_geometry(robot, skip_defs):
    """Truth for the evaluator only: every collision primitive (Webots boundingObject) in world frame.

    Walks PROTO internals too (getBaseNodeField), so trees, props and nested solids appear with the
    exact shapes Webots collides against. Meshes, planes and elevation grids (ground, terrain rim)
    are skipped and counted.
    """
    prims, skipped = [], {}
    eye = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]

    def field(node, name):
        return node.getBaseNodeField(name) if node.isProto() else node.getField(name)

    def geom(node, T, owner):
        if node is None:
            return
        kind = node.getTypeName() if not node.isProto() else node.getBaseTypeName()
        if kind in ("Pose", "Transform"):
            tf, rf = field(node, "translation"), field(node, "rotation")
            T = _compose(T, tf.getSFVec3f() if tf else [0.0, 0.0, 0.0],
                         _axis_angle_matrix(rf.getSFRotation()) if rf else eye)
            ch = field(node, "children")
            for i in range(ch.getCount() if ch else 0):
                geom(ch.getMFNode(i), T, owner)
        elif kind == "Group":
            ch = field(node, "children")
            for i in range(ch.getCount() if ch else 0):
                geom(ch.getMFNode(i), T, owner)
        elif kind == "Shape":
            g = field(node, "geometry")
            geom(g.getSFNode() if g else None, T, owner)
        elif kind == "Box":
            prims.append({"type": "box", "name": owner, "t": T[0], "R": T[1], "size": field(node, "size").getSFVec3f()})
        elif kind in ("Cylinder", "Capsule"):
            r, h = field(node, "radius").getSFFloat(), field(node, "height").getSFFloat()
            prims.append({"type": "cyl", "name": owner, "t": T[0], "R": T[1], "radius": r,
                          "height": h + (2.0 * r if kind == "Capsule" else 0.0)})
        elif kind == "Sphere":
            prims.append({"type": "sphere", "name": owner, "t": T[0], "radius": field(node, "radius").getSFFloat()})
        else:
            skipped[kind] = skipped.get(kind, 0) + 1

    def walk(node, parent_T):
        if node is None or node.getDef() in skip_defs:
            return
        base = node.getBaseTypeName()
        if base in ("Robot",):
            return
        if base == "Solid":
            pose = node.getPose()
            T = ([pose[3], pose[7], pose[11]], [[pose[0], pose[1], pose[2]], [pose[4], pose[5], pose[6]],
                                                [pose[8], pose[9], pose[10]]])
            nf = field(node, "name")
            bo = field(node, "boundingObject")
            geom(bo.getSFNode() if bo else None, T, nf.getSFString() if nf else base)
        elif base in ("Pose", "Transform"):
            tf, rf = field(node, "translation"), field(node, "rotation")
            T = _compose(parent_T, tf.getSFVec3f() if tf else [0.0, 0.0, 0.0],
                         _axis_angle_matrix(rf.getSFRotation()) if rf else eye)
        else:
            T = parent_T
        ch = field(node, "children")
        for i in range(ch.getCount() if ch else 0):
            walk(ch.getMFNode(i), T)

    children = robot.getRoot().getField("children")
    for i in range(children.getCount()):
        walk(children.getMFNode(i), ([0.0, 0.0, 0.0], eye))
    return {"primitives": prims, "skipped": skipped}


def _ray_aabb(origin, delta, box):
    """Slab ray-AABB test returning earliest hit fraction t in (0, 1], or None if no hit."""
    if (box[0] <= origin[0] <= box[1] and box[2] <= origin[1] <= box[3] and box[4] <= origin[2] <= box[5]):
        return None  # anchor itself inside padded margin: ignore so arm never collapses to zero
    tmin, tmax = 0.0, 1.0
    for axis in range(3):
        o = origin[axis]
        d = delta[axis]
        lo, hi = box[2 * axis], box[2 * axis + 1]
        if abs(d) < 1e-9:
            if o < lo or o > hi:
                return None
        else:
            inv = 1.0 / d
            t0 = (lo - o) * inv
            t1 = (hi - o) * inv
            if t0 > t1:
                t0, t1 = t1, t0
            if t0 > tmin:
                tmin = t0
            if t1 < tmax:
                tmax = t1
            if tmax < tmin:
                return None
    return tmin if 0.02 < tmin <= 1.0 else None


def _arm_clearance(origin, desired, boxes):
    """Smallest ray hit fraction t in (0, 1] along origin -> desired across all obstacle AABBs."""
    delta = (desired[0] - origin[0], desired[1] - origin[1], desired[2] - origin[2])
    best = 1.0
    for box in boxes:
        t = _ray_aabb(origin, delta, box)
        if t is not None and t < best:
            best = t
    return best


class RealTimePacer:
    """Holds simulated time to the wall clock, one physics step at a time, and sizes the frame rate.

    Webots' own real-time mode (WbSimulationWorld::step) measures each step and sleeps in whole
    milliseconds and steers a running mean toward the step length: with 2 ms steps the truncation
    reads steps as shorter than they are, and flight ran at 0.76x real time. Instead Webots runs in
    fast mode and this pacer, called between steps, sleeps until each step's wall-clock deadline.
    Time lost to a slow step (a sensor render, a frame) is made up by the following steps; a lag
    longer than MAX_LAG is dropped rather than replayed fast.

    The camera and LiDAR render in Webots' main thread on a fixed cycle, stalling it 10-20 ms, and
    the frame drawn right after a stall showed the world that much late. The pacer learns each
    step's cost by its phase in the cycle and runs ahead of a stall by its cost, so the stall ends
    on time.
    """

    MAX_LAG = 0.1  # s
    CYCLE = 0.1  # s: sensor renders repeat every 100 ms (config/sensors.json: LiDAR 10 Hz, camera 20 Hz)
    STALL = 0.003  # s: a phase costing this much more than the median step holds a sensor render

    def __init__(self, step_s):
        self.step_s = step_s
        self.origin = None  # wall time at simulated time 0
        self.cost = [0.0] * round(self.CYCLE / step_s)  # s: Webots' wall time per step, by phase
        self.median = 0.0
        self.stalls = []  # (phase, excess cost) of the stalling phases
        self.quiet = 0.0005  # s: a step with no render
        self.render = 0.007  # s: a frame's cost to the step it precedes
        self.extra = 0.0  # s: added to the frame interval while real time has no headroom
        self.requested = None  # wall time of the last step request
        self.wall = self.slept = 0.0

    def start(self, sim_time):
        self.origin = time.perf_counter() - sim_time
        self.requested = None

    def wait(self, sim_time):
        """Called once the step ending at sim_time has returned; returns when to request the next."""
        if self.origin is None:
            return
        now = time.perf_counter()
        n = len(self.cost)
        phase = round(sim_time / self.step_s) % n
        if self.requested is not None:
            took = now - self.requested
            self.wall += took
            i = (phase - 1) % n
            self.cost[i] += 0.05 * (took - self.cost[i])
            if self.cost[i] - self.median < self.STALL:
                if took > self.quiet + 0.0025:
                    self.render += 0.05 * (took - self.quiet - self.render)
                else:
                    self.quiet += 0.05 * (took - self.quiet)
        ahead = self.origin + sim_time - now
        if ahead < -self.MAX_LAG:
            self.origin -= ahead + self.MAX_LAG
        # Steps between now and a stall each gain step_s - 2 * quiet on the wall clock.
        gain = self.step_s - 2.0 * self.quiet
        lead = 0.0
        for stall_phase, excess in self.stalls:
            lead = max(lead, excess - ((stall_phase - phase) % n) * gain)
        ahead -= lead
        if ahead > 0.0:
            time.sleep(ahead)
        after = time.perf_counter()
        self.slept += after - now
        self.wall += after - now
        self.requested = after

    def update(self, target):
        """Refresh the stall map; return the WorldInfo FPS that makes Webots draw `target` frames per second.

        Webots draws a frame when more than 1000/FPS whole milliseconds have passed since the last
        frame ended (WbView3D::refresh), so a frame interval is that threshold, up to one step more,
        and the frame's own cost: at FPS 60 it drew 41 frames per second. The threshold is set to
        leave the target interval, and widened while the pacer had under 10% of the time to spare.
        """
        self.median = sorted(self.cost)[len(self.cost) // 2]
        self.stalls = [(i, c - self.median) for i, c in enumerate(self.cost) if c - self.median > self.STALL]
        if self.wall > 0.0:
            spare = self.slept / self.wall
            self.extra = max(0.0, self.extra + (0.001 if spare < 0.1 else -0.001 if spare > 0.2 else 0.0))
            self.wall = self.slept = 0.0
        threshold = 1.0 / target - self.render - self.step_s / 2 + self.extra
        return target if threshold >= 1.0 / target else min(1.0 / max(threshold, self.step_s), 4.0 * target)


class SmartCamera:
    """Cinematic heading-aligned spring-arm camera with obstacle pull-in so view is never blocked.

    Updated on every physics step: Webots renders a frame after whichever step its frame timer
    lands on, so a camera moved less often than the drone lets the drone slide between camera
    updates and snap back (a 100 Hz sawtooth of several pixels, aliased by the frame rate).
    Smoothing gains are per 10 ms and converted to the step, so the feel is step-rate independent.
    """

    REACH = 6.5  # m: covers the longest spring-arm (4.9 m) with margin

    def __init__(self, robot, interior_boxes, step_s):
        self.robot = robot
        self.view = robot.getFromDef("IRONLARK_VIEW")
        self.interiors = [b for b in (interior_boxes or []) if b]
        self.active_interior = None
        self.boxes = _extract_obstacle_aabbs(robot)
        self.near_boxes, self.near_centre = [], None
        self.k = step_s / 0.010
        self.mode = "chase"
        self.eye = None
        self.target = None
        self.cam_yaw = 0.0
        self.indoor_blend = 0.0
        self.arm_scale = 1.0
        self.keyboard = robot.getKeyboard()
        self.keyboard.enable(20)
        if self.view:
            self.pos_field = self.view.getField("position")
            self.ori_field = self.view.getField("orientation")
            self._hud()

    def _hud(self):
        tags = [("1", "CHASE", "chase"), ("2", "TAIL", "tail"), ("3", "OVERVIEW", "overview"), ("4", "FREE", "free")]
        label = "   ".join(f"[{k} {name}]" if m == self.mode else f"{k} {name}" for k, name, m in tags)
        self.robot.setLabel(0, label, 0.02, 0.94, 0.045, 0xF0F4F8, 0.0, "Arial")

    def _arm_point(self, x, y, z, yaw, dist, height, shoulder):
        fx, fy = math.cos(yaw), math.sin(yaw)
        rx, ry = fy, -fx
        ez = z + height
        if self.active_interior and self.indoor_blend > 0.25:
            ez = min(self.active_interior["z"][1] - 0.55, ez)
        return [x - dist * fx + shoulder * rx, y - dist * fy + shoulder * ry, max(0.55, ez)]

    def _gain(self, per_10ms):
        """Exponential smoothing gain for one step, equivalent to per_10ms applied every 10 ms."""
        return 1.0 - (1.0 - per_10ms) ** self.k

    def _obstacles_near(self, x, y, z):
        """Obstacle boxes the arm can reach, refreshed after the drone moves 1 m (the full list is long)."""
        c = self.near_centre
        if c is None or abs(x - c[0]) > 1.0 or abs(y - c[1]) > 1.0 or abs(z - c[2]) > 1.0:
            r = self.REACH + 1.0
            self.near_centre = (x, y, z)
            self.near_boxes = [b for b in self.boxes
                               if b[0] - r <= x <= b[1] + r and b[2] - r <= y <= b[3] + r and b[4] - r <= z <= b[5] + r]
        return self.near_boxes

    def _nearest_interior(self, x, y, margin=3.6):
        for box in self.interiors:
            if (box["x"][0] - margin <= x <= box["x"][1] + margin
                    and box["y"][0] - margin <= y <= box["y"][1] + margin):
                return box
        return None

    def update(self, pos, yaw):
        if not self.view:
            return
        key = self.keyboard.getKey()
        while key != -1:
            modes = {ord("1"): "chase", ord("2"): "tail", ord("3"): "overview", ord("4"): "free"}
            if key in modes and modes[key] != self.mode:
                self.mode = modes[key]
                self._hud()
            key = self.keyboard.getKey()
        if self.mode == "free":
            return

        x, y, z = pos
        # Smooth cinematic orbit behind the drone's heading.
        self.cam_yaw = _wrap(self.cam_yaw + self._gain(0.035) * _wrap(yaw - self.cam_yaw))

        hit_int = self._nearest_interior(x, y)
        if hit_int is not None:
            self.active_interior = hit_int
        in_zone = 1.0 if hit_int is not None else 0.0
        self.indoor_blend += self._gain(0.035) * (in_zone - self.indoor_blend)
        b = self.indoor_blend

        if self.mode == "overview":
            desired = [-18.0, -18.0, 24.0]
            desired_target = [max(-24.0, min(24.0, x * 0.6)), max(-24.0, min(24.0, y * 0.6)), max(1.0, z)]
            alpha_pos = 0.06
        else:
            if self.mode == "tail":
                dist, height, shoulder, look = 2.15, 0.62, 0.0, 1.8
            else:
                dist = 4.6 * (1.0 - b) + 2.65 * b
                height = 1.55 * (1.0 - b) + 0.72 * b
                shoulder = 0.48 * (1.0 - b) + 0.20 * b
                look = 2.1 * (1.0 - b) + 1.5 * b

            anchor = (x, y, z + 0.14)
            boxes = self._obstacles_near(x, y, z)
            cand = self._arm_point(x, y, z, self.cam_yaw, dist, height, shoulder)
            t_best = _arm_clearance(anchor, cand, boxes)

            # If a pillar, tree, or wall blocks the spring-arm, try a subtle ±24 deg side-orbit first.
            if t_best < 0.96:
                for dyaw in (-0.42, 0.42):
                    alt = self._arm_point(x, y, z, self.cam_yaw + dyaw, dist, height * 0.9, 0.0)
                    t_alt = _arm_clearance(anchor, alt, boxes)
                    if t_alt > t_best + 0.18:
                        cand, t_best = alt, t_alt

            target_scale = 1.0 if t_best >= 0.99 else max(0.34, t_best - 0.04)
            # Fast pull-in so walls/trees never block the drone; silky slow glide-out once clear.
            self.arm_scale += self._gain(0.28 if target_scale < self.arm_scale else 0.04) * (target_scale - self.arm_scale)
            scale = min(self.arm_scale, target_scale) if t_best < 0.99 else self.arm_scale
            desired = [
                anchor[0] + scale * (cand[0] - anchor[0]),
                anchor[1] + scale * (cand[1] - anchor[1]),
                max(0.45, anchor[2] + scale * (cand[2] - anchor[2])),
            ]
            fx, fy = math.cos(self.cam_yaw), math.sin(self.cam_yaw)
            desired_target = [x + look * fx, y + look * fy, max(0.55, z + 0.06)]
            alpha_pos = 0.22 if t_best < 0.98 else 0.075

        if self.eye is None:
            self.eye = list(desired)
            self.target = list(desired_target)
        else:
            alpha_pos, alpha_target = self._gain(alpha_pos), self._gain(0.12)
            self.eye = [a + alpha_pos * (c - a) for a, c in zip(self.eye, desired)]
            self.target = [a + alpha_target * (c - a) for a, c in zip(self.target, desired_target)]
        self.pos_field.setSFVec3f(self.eye)
        self.ori_field.setSFRotation(look_at(self.eye, self.target))


robot = Supervisor()
step_ms = int(robot.getBasicTimeStep())
drone = robot.getFromDef("IRONLARK_DRONE")
drone.enablePoseTracking(step_ms)  # getPose() then reads the pose sent with each step, no extra round trip
destination = os.environ.get("IRONLARK_RUN_DIR")
if destination is None:
    raise SystemExit("Start this world through ./ironlark so its evaluation output has a destination.")
run = Path(destination)

sim_meta_path = run / "sim-meta.json"
sim_meta = json.loads(sim_meta_path.read_text()) if sim_meta_path.is_file() else {}
gps_denied_boxes = sim_meta.get("gps_denied_boxes") or (
    [sim_meta["gps_denied_box"]] if sim_meta.get("gps_denied_box") else []
)
interior_boxes = sim_meta.get("interior_boxes") or (
    [sim_meta["interior_box"]] if sim_meta.get("interior_box") else []
)
camera = SmartCamera(robot, interior_boxes, step_ms / 1000.0)
last_denied = None
if sim_meta:
    # Evaluator truth (tests/motion_quality.py): collision geometry, and contacts while airborne.
    tmp = run / "world-obstacles.json.tmp"
    tmp.write_text(json.dumps(export_collision_geometry(robot, {"IRONLARK_DRONE"})))
    tmp.replace(run / "world-obstacles.json")
    drone.enableContactPointsTracking(step_ms * 50, True)

# Controllers start once the world has loaded; the launcher waits for this.
(run / "world-ready").touch()

children = robot.getRoot().getField("children")
world_info = next(children.getMFNode(i) for i in range(children.getCount())
                  if children.getMFNode(i).getTypeName() == "WorldInfo")
fps = world_info.getField("FPS")
world_fps = frame_fps = fps.getSFFloat()
pacer = RealTimePacer(step_ms / 1000.0)
fast_start = os.environ.get("IRONLARK_FAST_START") == "1"
movie_path = os.environ.get("IRONLARK_MOVIE")
movie_state = "idle" if movie_path else "off"  # idle -> recording -> finalizing -> done
movie_stop_at = None
saw_landing = False
flight_state = run / "flight-state.json"
robot.simulationSetMode(Supervisor.SIMULATION_MODE_FAST)  # paced by RealTimePacer, see there
if fast_start:
    fps.setSFFloat(10.0)
else:
    pacer.start(robot.getTime())

# Every step: the spectator camera (see SmartCamera) and real-time pacing. Every 100 ms (10 Hz, as
# evaluators expect): ground truth, the GPS-denied zone flag, and the end of the fast start. Every
# second: the frame rate (see RealTimePacer.update); the world's FPS is the frame rate it gets.
with (run / "ground_truth.jsonl").open("w", buffering=1) as stream:
    while robot.step(step_ms) != -1:
        now = robot.getTime()
        pose = drone.getPose()
        position = [pose[3], pose[7], pose[11]]
        camera.update(position, math.atan2(pose[4], pose[0]))
        now_ms = round(now * 1000)
        if now_ms % 1000 == 0 and not fast_start and movie_state != "recording":
            if (wanted := round(pacer.update(world_fps))) != frame_fps:
                frame_fps = wanted
                fps.setSFFloat(float(frame_fps))
        if now_ms % 100 == 0:
            state = None
            if (fast_start or movie_state != "off") and flight_state.exists():
                try:
                    state = json.loads(flight_state.read_text()).get("state")
                except (OSError, ValueError):
                    state = None
            if fast_start and state and state not in ("CONNECTING", "WAITING_FOR_POSITION"):
                fps.setSFFloat(30.0 if movie_path else float(frame_fps))
                pacer.start(now)
                fast_start = False
            if movie_state == "idle" and state in ("ARMING", "TAKING_OFF"):
                fps.setSFFloat(30.0)
                robot.movieStartRecording(movie_path, 1920, 1080, 0, 80, 1, False)
                movie_state = "recording"
            elif movie_state == "recording":
                if state == "LANDING":
                    saw_landing = True
                if saw_landing and movie_stop_at is None and (
                    state in ("COMPLETED", "STANDBY") or (run / "autonomy_report.json").exists()
                ):
                    movie_stop_at = now + 2.0
                if movie_stop_at is not None and now >= movie_stop_at:
                    robot.movieStopRecording()
                    movie_state = "finalizing"
            elif movie_state == "finalizing":
                if robot.movieIsReady() or robot.movieFailed():
                    (run / "movie-done").write_text("failed\n" if robot.movieFailed() else "ok\n")
                    movie_state = "done"
            denied = any(in_box(position, box) for box in gps_denied_boxes)
            if gps_denied_boxes and denied != last_denied:
                tmp = run / "gps-denied.json.tmp"
                tmp.write_text(json.dumps({"denied": denied, "time": now}) + "\n")
                tmp.replace(run / "gps-denied.json")
                last_denied = denied
            sample = {"time": now, "position": position, "velocity": drone.getVelocity(),
                      "orientation": [pose[0], pose[1], pose[2], pose[4], pose[5], pose[6],
                                      pose[8], pose[9], pose[10]],
                      "gps_denied": denied}
            if sim_meta and position[2] > 0.4:
                contacts = drone.getContactPoints(True)
                if contacts:
                    sample["contacts"] = [[round(c, 3) for c in cp.getPoint()] for cp in contacts[:4]]
            stream.write(json.dumps(sample) + "\n")
        pacer.wait(now)
