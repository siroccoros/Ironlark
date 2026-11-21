"""Unit tests for the autonomy stack: LIO, 3D occupancy mapping, 3D A* planning, and mission judge."""
import math
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from autonomy.gcs import encode_rgb_png, occupancy_slice_2d  # noqa: E402
from autonomy.mapping import OccupancyMap  # noqa: E402
from autonomy.mavlink_interface import MavlinkPilot  # noqa: E402
from autonomy import trajectory as tj  # noqa: E402
from autonomy.mission import MissionExecutive  # noqa: E402
from autonomy.odometry import LidarInertialOdometry, rodrigues  # noqa: E402
from autonomy.planner import line_is_clear, plan_path  # noqa: E402
from tests import mission_eval  # noqa: E402
from worlds import generate  # noqa: E402


def _synthetic_room_cloud():
    """Dense 3D room + interior pillar point cloud in world coordinates."""
    pts = []
    xs = np.linspace(-10, 10, 40)
    ys = np.linspace(-10, 10, 40)
    zs = np.linspace(0, 5, 16)
    # Floor & ceiling
    gx, gy = np.meshgrid(xs, ys)
    pts.append(np.column_stack([gx.ravel(), gy.ravel(), np.zeros(gx.size)]))
    pts.append(np.column_stack([gx.ravel(), gy.ravel(), np.full(gx.size, 5.0)]))
    # Four walls
    wy, wz = np.meshgrid(ys, zs)
    pts.append(np.column_stack([np.full(wy.size, 10.0), wy.ravel(), wz.ravel()]))
    pts.append(np.column_stack([np.full(wy.size, -10.0), wy.ravel(), wz.ravel()]))
    wx, wz = np.meshgrid(xs, zs)
    pts.append(np.column_stack([wx.ravel(), np.full(wx.size, 10.0), wz.ravel()]))
    pts.append(np.column_stack([wx.ravel(), np.full(wx.size, -10.0), wz.ravel()]))
    # Central pillar
    theta = np.linspace(0, 2 * math.pi, 36, endpoint=False)
    pt, pz = np.meshgrid(theta, zs)
    pts.append(np.column_stack([2.0 + 0.8 * np.cos(pt.ravel()), 1.0 + 0.8 * np.sin(pt.ravel()), pz.ravel()]))
    return np.vstack(pts)


class TestOdometry(unittest.TestCase):
    def test_tracks_6dof_motion_on_synthetic_lidar_and_imu(self):
        rng = np.random.default_rng(42)
        world_cloud = _synthetic_room_cloud()
        lio = LidarInertialOdometry(initial_position=(0.0, 0.0, 2.0))

        true_p = np.array([0.0, 0.0, 2.0])
        true_R = np.eye(3)
        dt_scan = 0.1
        vel = np.array([1.2, -0.4, 0.15])
        omega = np.array([0.0, 0.0, 0.25])  # gentle yaw turn

        for step in range(20):
            t = step * dt_scan
            if step > 0:
                # 25 IMU steps per 0.1 s LiDAR scan with realistic white noise.
                for k in range(25):
                    ti = (step - 1) * dt_scan + (k + 1) * (dt_scan / 25)
                    true_R = true_R @ rodrigues(omega * (dt_scan / 25))
                    gyro_meas = omega + rng.normal(0, 0.002, 3)
                    accel_meas = true_R.T @ np.array([0.0, 0.0, 9.81]) + rng.normal(0, 0.03, 3)
                    lio.on_imu(ti, gyro_meas, accel_meas)
                true_p = true_p + vel * dt_scan

            # Transform world cloud into body frame with realistic 2 cm range noise.
            body_pts = (world_cloud - true_p) @ true_R + rng.normal(0, 0.02, world_cloud.shape)
            p_est, R_est, _, healthy = lio.on_lidar(t, body_pts)
            self.assertTrue(healthy)

        pos_err = float(np.linalg.norm(p_est - true_p))
        rot_err_deg = math.degrees(math.acos(np.clip((np.trace(R_est.T @ true_R) - 1.0) * 0.5, -1.0, 1.0)))
        self.assertLess(pos_err, 0.15, f"LIO drifted {pos_err:.3f} m over 2 s")
        self.assertLess(rot_err_deg, 1.5, f"LIO yaw/attitude drifted {rot_err_deg:.2f} deg")


class TestMappingAndPlanning(unittest.TestCase):
    def test_raycasting_carves_free_marks_wall_and_finds_frontiers(self):
        omap = OccupancyMap(origin=(-5, -5, 0), size=(20, 10, 5), resolution=0.5, inflate_xy=0.8)
        # Wall at x = 6.0 spanning y in [-3, 3], z in [0.5, 4.0]
        ys, zs = np.meshgrid(np.linspace(-3, 3, 25), np.linspace(0.5, 4.0, 15))
        wall = np.column_stack([np.full(ys.size, 6.0), ys.ravel(), zs.ravel()])
        for _ in range(3):
            omap.update(sensor_pos=(0.0, 0.0, 2.0), world_points=wall, stride=1)

        # Space between sensor and wall is free; wall itself is occupied; behind wall stays unknown.
        self.assertTrue(omap.free[tuple(omap.world_to_idx([3.0, 0.0, 2.0]))])
        self.assertTrue(omap.occupied[tuple(omap.world_to_idx([6.0, 0.0, 2.0]))])
        self.assertTrue(omap.unknown[tuple(omap.world_to_idx([10.0, 0.0, 2.0]))])
        self.assertFalse(omap.is_safe([5.5, 0.0, 2.0]))  # inside inflation margin

        # Frontiers exist at the edge of carved free space.
        frontiers = omap.frontiers({"x": [-4, 14], "y": [-4, 4], "z": [1.5, 3.0]}, current_pos=(0.0, 0.0, 2.0))
        self.assertTrue(len(frontiers) > 0)
        # Fast separable obstacle dilation must match scipy.ndimage.binary_dilation bit-for-bit.
        from scipy.ndimage import binary_dilation
        np.testing.assert_array_equal(omap.inflated_obstacles(), binary_dilation(omap.occupied, structure=omap._struct))

    def test_astar_routes_around_wall_and_shortcuts(self):
        omap = OccupancyMap(origin=(-2, -8, 0), size=(20, 16, 5), resolution=0.5, inflate_xy=0.8)
        # Solid wall at x = 8, y in [-4, +4] (open passage around y = ±5.5).
        ys, zs = np.meshgrid(np.linspace(-4, 4, 33), np.linspace(0.2, 4.5, 18))
        wall = np.column_stack([np.full(ys.size, 8.0), ys.ravel(), zs.ravel()])
        for _ in range(3):
            omap.update(sensor_pos=(2.0, 0.0, 2.0), world_points=wall, stride=1)

        path = plan_path(omap, start=(2.0, 0.0, 2.0), goal=(14.0, 0.0, 2.0), allow_unknown=True)
        self.assertIsNotNone(path)
        self.assertFalse(line_is_clear(omap, (2.0, 0.0, 2.0), (14.0, 0.0, 2.0)))
        for a, b in zip(path, path[1:]):
            self.assertTrue(line_is_clear(omap, a, b, allow_unknown=True))
        # Path must detour laterally around the wall (|y| > 4.5).
        self.assertGreater(max(abs(p[1]) for p in path), 4.5)

    def _wall_map(self):
        """A wall x in [4, 4.5], y in [-6, 1.5], all in view of a sensor at (1, 0): its end is a sharp corner."""
        omap = OccupancyMap(origin=(-2, -8, 0), size=(20, 16, 5), resolution=0.5)
        ys, zs = np.meshgrid(np.linspace(-6.0, 1.5, 31), np.linspace(0.3, 4.5, 15))
        wall = np.column_stack([np.full(ys.size, 4.2), ys.ravel(), zs.ravel()])
        ring = np.array([[1 + 11 * math.cos(a), 11 * math.sin(a), z]
                         for a in np.linspace(0, 2 * math.pi, 240) for z in (0.3, 1.5, 2.5, 3.5)])
        for _ in range(4):
            omap.update(sensor_pos=(1.0, 0.0, 2.2), world_points=np.vstack([wall, ring]), stride=1)
        return omap

    def _fly(self, traj, dt=tj.DT):
        ts = np.arange(traj.t0, traj.t_end + 0.5, dt)
        return ts, [traj.sample(t) for t in ts]

    def test_trajectory_rounds_corners_but_never_through_inflated_obstacles(self):
        omap = self._wall_map()
        view = tj.MapView(omap.copy())
        path = plan_path(omap, (1.0, -2.0, 2.2), (8.0, -2.0, 2.2), z_bounds=(2.0, 2.6))
        self.assertIsNotNone(path)
        self.assertGreater(len(path), 2)
        # Full-size rounding at the wall's end would cut into its safety margin ...
        naive = tj.geometry(tj.control_points(path, [tj.Limits().corner_r] * len(path)))[0]
        self.assertTrue(view.blocked(naive).any())
        # ... the built trajectory shrinks it: its geometry is clear, every flown sample keeps the margin.
        traj = tj.build(path, (np.array(path[0]), np.zeros(3), np.zeros(3), 0.0, 0.0), view, tj.Limits(), 0.0)
        self.assertIsNotNone(traj)
        self.assertFalse(view.blocked(traj.P).any())
        _, states = self._fly(traj)
        flown = np.array([s[0] for s in states])
        self.assertGreater(float(view.clearance(flown).min()), 0.95)

    def test_trajectory_ends_at_rest_on_its_goal_without_overshoot(self):
        omap = self._wall_map()
        view = tj.MapView(omap.copy())
        path = plan_path(omap, (1.0, -2.0, 2.2), (8.0, -2.0, 2.2), z_bounds=(2.0, 2.6))
        traj = tj.build(path, (np.array(path[0]), np.zeros(3), np.zeros(3), 0.0, 0.0), view, tj.Limits(), 0.0)
        ts, states = self._fly(traj)
        arc = np.array([traj.arc_at(t)[0] for t in ts])
        self.assertTrue(np.all(np.diff(arc) >= -1e-9))  # never backs up
        self.assertTrue(np.all(np.array([traj.arc_at(t)[1] for t in ts]) >= 0.0))
        p, v, a, _, rate = states[-1]
        np.testing.assert_allclose(p, path[-1], atol=1e-6)
        self.assertLess(float(np.linalg.norm(v)), 1e-9)
        self.assertLess(float(np.linalg.norm(a)), 1e-9)
        self.assertEqual(rate, 0.0)

    def test_trajectory_respects_speed_acceleration_jerk_and_yaw_limits(self):
        omap = self._wall_map()
        view = tj.MapView(omap.copy())
        lim = tj.Limits()
        path = plan_path(omap, (1.0, -2.0, 2.2), (8.0, -2.0, 2.2), z_bounds=(2.0, 2.6))
        traj = tj.build(path, (np.array(path[0]), np.zeros(3), np.zeros(3), 0.0, 0.0), view, lim, 0.0)
        _, states = self._fly(traj)
        v = np.array([s[1] for s in states])
        a = np.array([s[2] for s in states])
        rate = np.array([s[4] for s in states])
        self.assertLessEqual(float(np.linalg.norm(v[:, :2], axis=1).max()), lim.v_max + 1e-6)
        self.assertLessEqual(float(np.linalg.norm(a[:, :2], axis=1).max()), math.hypot(lim.a_brake, lim.a_lat) + 0.05)
        jerk = np.linalg.norm(np.diff(a[:, :2], axis=0), axis=1) / tj.DT
        self.assertLessEqual(float(jerk.max()), 3.0)  # PSC_NE_JERK: ArduPilot would smooth anything sharper
        self.assertLessEqual(float(np.abs(rate).max()), lim.yaw_rate + 1e-9)

    def test_replans_brakes_and_retimes_continue_the_trajectory_exactly(self):
        omap = self._wall_map()
        view = tj.MapView(omap.copy())
        lim = tj.Limits()
        path = plan_path(omap, (1.0, -2.0, 2.2), (8.0, -2.0, 2.2), z_bounds=(2.0, 2.6))
        traj = tj.build(path, (np.array(path[0]), np.zeros(3), np.zeros(3), 0.0, 0.0), view, lim, 0.0)
        for t_r in (1.0, 2.7, 4.1, 6.3):
            start = traj.sample(t_r)
            new_path = plan_path(omap, start[0], (2.0, 6.0, 2.2), z_bounds=(2.0, 2.6))
            for nxt in (tj.build(new_path, start, view, lim, t_r), tj.brake(traj, t_r, lim),
                        tj.retime(traj, t_r, view, lim)):
                self.assertIsNotNone(nxt)
                got = nxt.sample(t_r)
                np.testing.assert_allclose(got[0], start[0], atol=1e-6)
                np.testing.assert_allclose(got[1], start[1], atol=1e-6)
                self.assertAlmostEqual(abs(tj._wrap(got[3] - start[3])), 0.0, places=6)
                self.assertAlmostEqual(got[4], start[4], places=6)
                # A new curve starts straight along the velocity, so the acceleration feed-forward may
                # drop its centripetal part (at most a_lat); ArduPilot's PSC_NE_JERK smooths that step.
                self.assertLessEqual(float(np.linalg.norm(got[2] - start[2])), lim.a_lat + 1e-6)
                self.assertLess(float(np.linalg.norm(nxt.sample(t_r + tj.DT)[0] - got[0])), 0.06)

    def test_brake_slows_along_the_curve_without_reversing(self):
        omap = self._wall_map()
        view = tj.MapView(omap.copy())
        path = plan_path(omap, (1.0, -2.0, 2.2), (8.0, -2.0, 2.2), z_bounds=(2.0, 2.6))
        traj = tj.build(path, (np.array(path[0]), np.zeros(3), np.zeros(3), 0.0, 0.0), view, tj.Limits(), 0.0)
        stop = tj.brake(traj, 3.0, tj.Limits())
        speeds = [stop.arc_at(t)[1] for t in np.arange(3.0, stop.t_end + 0.2, tj.DT)]
        self.assertTrue(np.all(np.array(speeds) >= 0.0))
        self.assertTrue(np.all(np.diff(speeds) <= 1e-9))
        self.assertEqual(speeds[-1], 0.0)

    def test_yaw_turns_the_short_way_across_pi(self):
        lim = tj.Limits()
        traj = tj.build([np.zeros(3), np.array([-5.0, -0.9, 0.0])],
                        (np.zeros(3), np.zeros(3), np.zeros(3), math.radians(170.0), 0.0), None, lim, 0.0)
        _, states = self._fly(traj)
        yaws = np.unwrap([s[3] for s in states])
        self.assertLess(abs(yaws[-1] - yaws[0]), math.radians(25.0))  # 170 -> -170 deg is +20, not -340

    def test_streamer_preserves_earlier_pending_trajectory_across_replan(self):
        from autonomy.mission import SetpointStreamer

        class StaticPilot:
            closing = True  # don't spin background thread; test queue logic directly
            sim_time = 0.0

        streamer = SetpointStreamer(StaticPilot())
        lim = tj.Limits()
        t1 = tj.build([np.zeros(3), np.array([10.0, 0.0, 0.0])],
                      (np.zeros(3), np.zeros(3), np.zeros(3), 0.0, 0.0), None, lim, 0.0)
        streamer.start(t1)
        # Commit an earlier brake at t0=2.0, then a later replan stitched to that brake at t0=2.5.
        brk = tj.brake(t1, 2.0, lim)
        streamer.commit(brk)
        st25 = streamer.at(2.5).sample(2.5)
        t2 = tj.build([st25[0], st25[0] + np.array([4.0, 0.0, 0.0])], st25, None, lim, 2.5)
        streamer.commit(t2)
        # At t=2.2 (between 2.0 and 2.5), the brake trajectory must still govern — never overwritten!
        self.assertIs(streamer.at(2.2), brk)
        self.assertIs(streamer.at(2.5), t2)
        p_before, v_before = brk.sample(2.5)[:2]
        p_after, v_after = t2.sample(2.5)[:2]
        np.testing.assert_allclose(p_before, p_after, atol=1e-6)
        np.testing.assert_allclose(v_before, v_after, atol=1e-6)

    def test_plan_path_snaps_goal_inside_obstacle_to_safe_point(self):
        omap = OccupancyMap(origin=(-2, -6, 0), size=(14, 12, 5), resolution=0.5, inflate_xy=0.8)
        xs, ys, zs = np.meshgrid(np.linspace(4.2, 5.8, 5), np.linspace(-0.8, 0.8, 5), np.linspace(0.5, 4.0, 8))
        rock = np.column_stack([xs.ravel(), ys.ravel(), zs.ravel()])
        for _ in range(3):
            omap.update(sensor_pos=(1.0, 0.0, 2.0), world_points=rock, stride=1)
        # Clicking right inside the rock (5.0, 0.0) from just outside it must never return the unsafe rock center.
        path = plan_path(omap, start=(2.75, 0.0, 2.0), goal=(5.0, 0.0, 2.0), allow_unknown=True)
        self.assertIsNotNone(path)
        self.assertTrue(omap.is_safe(path[-1]))

    def test_gcs_map_slice_and_png_encoder(self):
        omap = OccupancyMap(origin=(-2, -2, 0), size=(6, 6, 4), resolution=0.5)
        sl = occupancy_slice_2d(omap)
        self.assertEqual((sl["nx"], sl["ny"]), (12, 12))
        png = encode_rgb_png(np.zeros((16, 20, 3), dtype=np.uint8))
        self.assertTrue(png.startswith(b"\x89PNG\r\n\x1a\n"))

    def test_pilot_event_is_thread_safe_under_concurrent_gcs_clicks(self):
        with tempfile.TemporaryDirectory() as td:
            pilot = MavlinkPilot(state_path=Path(td) / "flight-state.json")
            errors = []

            def worker(tid):
                try:
                    for i in range(60):
                        pilot.event(f"STATE_{tid}", f"step {i}")
                except Exception as exc:
                    errors.append(exc)

            threads = [threading.Thread(target=worker, args=(k,)) for k in range(4)]
            for t in threads:
                t.start()
            for t in threads:
                t.join()
            self.assertEqual(errors, [])
            self.assertEqual(len(pilot.events), 240)

    def test_hold_brakes_along_the_trajectory_then_keeps_streaming_rest(self):
        class FakePilot:
            closing = False
            armed = True
            ekf_source = 1
            ekf_local_webots = None

            def __init__(self):
                self.sim_time = 10.0
                self.sent = []

            def wait_message(self, timeout):
                time.sleep(0.0005)
                self.sim_time += 0.02  # autopilot time runs while the executive waits

            def send_pva_target(self, p, v, a, yaw, rate):
                self.sent.append((self.sim_time, np.array(p), np.array(v)))

            def event(self, name, detail=None):
                pass

            def update_gps_ekf_transition(self, healthy):
                pass

            def yaw_webots(self):
                return 0.0

        class FakePerception:
            def snapshot(self):
                return (np.zeros(3), np.eye(3), np.zeros(3), True, 10)

        pilot = FakePilot()
        mgr = MissionExecutive(pilot, FakePerception(), brief={})
        line = tj.build([np.array([0.0, 0.0, 2.2]), np.array([30.0, 0.0, 2.2])],
                        (np.array([0.0, 0.0, 2.2]), np.zeros(3), np.zeros(3), 0.0, 0.0), None, tj.Limits(), 10.0)
        mgr.streamer.start(line)
        while pilot.sim_time < 16.0:  # cruising
            time.sleep(0.001)
        held = threading.Thread(target=mgr.hold_loop, daemon=True)
        held.start()
        while pilot.sim_time < 26.0:
            time.sleep(0.001)
        mgr.preempt.set()
        held.join(timeout=2.0)
        pilot.closing = True

        after = [(t, p, v) for t, p, v in pilot.sent if t >= 16.0]
        vx = np.array([v[0] for _, _, v in after])
        self.assertGreater(vx[0], 1.0)  # it was moving when told to hold
        self.assertTrue(np.all(vx >= -1e-9))  # braked forward along the line, never back
        self.assertTrue(np.all(np.diff(vx) <= 1e-3))  # (the line was still creeping up to cruise speed)
        self.assertTrue(np.all(np.abs(vx[-20:]) < 1e-9))  # at rest, and the rest setpoint keeps streaming
        np.testing.assert_allclose(after[-1][1], after[-20][1], atol=1e-9)
        gaps = np.diff([t for t, _, _ in pilot.sent])
        self.assertLessEqual(float(gaps.max()), 0.04 + 1e-9)


class TestMissionJudge(unittest.TestCase):
    def _sample_mission(self):
        _, meta = generate.mission_world(seed=7)
        # Synthetic clean mission trajectory: pad -> cave -> explore -> pad.
        waypoints = [(0, 0, 0.09), (0, 0, 2.2), (18, 0, 2.2), (24, 0, 2.2), (32, -4, 2.2),
                     (32, 5, 2.2), (24, 0, 2.2), (18, 0, 2.2), (0, 0, 2.2), (0, 0, 0.05)]
        truth, lio = [], []
        t = 0.0
        for a, b in zip(waypoints, waypoints[1:]):
            for k in range(20):
                u = k / 20.0
                pos = [a[i] + u * (b[i] - a[i]) for i in range(3)]
                denied = pos[0] >= 20.5
                truth.append({"time": round(t, 2), "position": pos, "orientation": [1, 0, 0, 0, 1, 0, 0, 0, 1],
                              "gps_denied": denied})
                lio.append({"time": round(t, 2), "position": [pos[0] + 0.05, pos[1] - 0.03, pos[2]]})
                t += 0.1
        report = {
            "passed": True,
            "frontiers_visited": 3,
            "interior_free_volume_m3": 420.0,
            "gps_lost_observed": True,
            "ekf_source_switches": [{"sim_time": 4.0, "source": 2}, {"sim_time": 12.0, "source": 1}],
        }
        return truth, lio, report, meta

    def test_valid_mission_passes(self):
        truth, lio, report, meta = self._sample_mission()
        res = mission_eval.judge_mission(truth, lio, report, meta)
        self.assertGreater(res["interior_travel_m"], 15.0)
        self.assertLess(res["lio_rmse_m"], 0.2)

    def test_missing_ekf_switch_or_crash_fails(self):
        truth, lio, report, meta = self._sample_mission()
        report["ekf_source_switches"] = []
        with self.assertRaisesRegex(RuntimeError, "ExternalNav"):
            mission_eval.judge_mission(truth, lio, report, meta)

        truth, lio, report, meta = self._sample_mission()
        truth[40]["orientation"] = [1, 0, 0, 0, -1, 0, 0, 0, -0.8]
        with self.assertRaisesRegex(RuntimeError, "flipped"):
            mission_eval.judge_mission(truth, lio, report, meta)


if __name__ == "__main__":
    unittest.main()
