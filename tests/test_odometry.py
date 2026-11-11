"""Geometry and frame regression for sensor odometry and its EKF handoff."""
import io
import math
import sys
import unittest
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
try:
    from sensing.odometry import ScanOdometry  # noqa: E402
    from ros.lidar_odometry import mavlink_payload  # noqa: E402
except ModuleNotFoundError as error:
    if error.name not in ("scipy", "rclpy", "sensor_msgs", "nav_msgs", "pymavlink"):
        raise
    ScanOdometry = None
    mavlink_payload = None


def walls(corner=True):
    line = np.arange(-6.0, 6.0, 0.08)
    height = np.arange(-0.05, 1.05, 0.15)
    a = np.array([(x, 2.0, z) for x in line for z in height])
    b = np.array([(x, -2.0, z) for x in line for z in height])
    if corner:
        c = np.array([(3.0, y, z) for y in line for z in height])
        return np.vstack((a, b, c))
    return np.vstack((a, b))


@unittest.skipIf(ScanOdometry is None, "odometry dependencies are installed in the ROS image")
class ScanMatching(unittest.TestCase):
    def test_recorded_return_times_set_snapshot_time_without_changing_points(self):
        odom = ScanOdometry((0, 0, 0))
        times = np.linspace(0.8, 0.9, 100)
        raw = np.column_stack((np.full(100, 2.0), np.ones(100),
                               np.full(100, 0.2), times))
        stamp, corrected = odom.prepare_scan(1.0, raw)
        self.assertTrue(odom.timing_valid)
        self.assertAlmostEqual(stamp, 0.9)
        self.assertAlmostEqual(odom.scan_duration, 0.1)
        self.assertAlmostEqual(odom.acquisition_delay, 0.1)
        self.assertTrue(np.array_equal(corrected, raw[:, :3]))

    def test_zeroed_first_scan_times_are_not_invented(self):
        odom = ScanOdometry((0, 0, 0))
        raw = np.column_stack((np.ones(100), np.zeros(100), np.zeros(100), np.zeros(100)))
        stamp, corrected = odom.prepare_scan(0.1, raw)
        self.assertFalse(odom.timing_valid)
        self.assertAlmostEqual(stamp, 0.1)
        self.assertTrue(np.array_equal(corrected, raw[:, :3]))

    def test_scan_rewinds_gyro_to_snapshot_then_restores_publication_attitude(self):
        odom = ScanOdometry((0, 0, 0))
        for time_s in np.arange(0.8, 1.001, 0.01):
            odom.imu(float(time_s), 1.0)
        xyz = walls()
        times = np.linspace(0.8, 0.9, len(xyz))
        state = odom.scan(1.0, np.column_stack((xyz, times)))
        self.assertAlmostEqual(state["time"], 0.9)
        self.assertAlmostEqual(state["yaw"], 0.1, places=2)
        self.assertAlmostEqual(odom.pose[2], 0.2, places=2)

    def test_corner_constrains_planar_pose(self):
        odom = ScanOdometry((0, 0, 0))
        cloud = walls()
        odom.scan(0.1, cloud)
        shifted = cloud - np.array([0.12, 0.05, 0])
        state = odom.scan(0.2, shifted)
        self.assertTrue(state["healthy"], state["reason"])
        self.assertLess(np.linalg.norm(np.array(state["position"][:2]) - [0.12, 0.05]), 0.04)

    def test_straight_corridor_is_unobservable(self):
        odom = ScanOdometry((0, 0, 0))
        cloud = walls(False)
        odom.scan(0.1, cloud)
        state = odom.scan(0.2, cloud - np.array([0.12, 0.05, 0]))
        self.assertFalse(state["healthy"])
        self.assertEqual(state["reason"], "corridor degeneracy")

    def test_external_nav_has_ned_position_and_covariance(self):
        state = {"time": 2.0, "position": [1, 2, 3], "yaw": 0.0,
                 "velocity": [0.1, 0.2, 0.3], "healthy": True,
                 "covariance_z": 0.05,
                 "covariance_xy_yaw": [[0.04, 0, 0], [0, 0.09, 0], [0, 0, 0.01]]}
        packet = mavlink_payload(state)
        self.assertEqual(packet[3:6], (1, -2, -3))
        for actual, expected in zip(packet[7:10], (0.1, -0.2, -0.3)):
            self.assertAlmostEqual(actual, expected)
        self.assertAlmostEqual(abs(packet[6][0]), 1.0)
        self.assertEqual(packet[13][0], 0.04)
        self.assertTrue(math.isnan(packet[14][0]))
        self.assertEqual(packet[-1], 80)
        from pymavlink import mavutil
        output = io.BytesIO()
        mavutil.mavlink.MAVLink(output).odometry_send(*packet)
        self.assertGreater(len(output.getvalue()), 0)

    def test_velocity_rotates_with_body_heading(self):
        state = {"time": 2.0, "position": [0, 1, 0], "yaw": math.pi / 2,
                 "velocity": [0, 1, 0], "healthy": True, "covariance_z": 0.05,
                 "covariance_xy_yaw": [[0.04, 0, 0], [0, 0.04, 0], [0, 0, 0.01]]}
        packet = mavlink_payload(state)
        self.assertEqual(packet[3:6], (0, -1, 0))
        self.assertAlmostEqual(packet[7], 1.0)
        self.assertAlmostEqual(packet[8], 0.0)


if __name__ == "__main__":
    unittest.main()
