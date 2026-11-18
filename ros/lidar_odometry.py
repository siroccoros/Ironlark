"""Live LiDAR odometry and optional MAVLink external navigation, on Webots /clock time."""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

os.environ.setdefault("MAVLINK20", "1")  # ODOMETRY is a MAVLink 2 message

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sensing import frames  # noqa: E402
from sensing.odometry import ScanOdometry  # noqa: E402
from sensing.mapping import OccupancyMap  # noqa: E402
from sensing.exploration import Explorer  # noqa: E402


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cloud_xyzt(message):
    if message.point_step != 20 or message.is_bigendian:
        raise ValueError("unexpected LiDAR point layout")
    data = np.frombuffer(message.data, dtype=np.dtype({"names": ["x", "y", "z", "time"],
                                                       "formats": ["<f4"] * 4,
                                                       "offsets": [0, 4, 8, 16], "itemsize": 20}))
    return np.column_stack((data["x"], data["y"], data["z"], data["time"]))


# EKF3 fuses an external-navigation sample as if it were VISO_DELAY_MS old. Late-flight DDS
# delivery once stalled for up to a second (see ros/fastdds.xml), so a scan older than this behind
# the newest IMU sample is not sent; EKF3 coasts on the IMU instead.
EXTERNAL_NAV_MAX_LAG_S = 0.3


def external_nav_fresh(scan_time, latest_imu_time, limit=EXTERNAL_NAV_MAX_LAG_S):
    """Whether a scan is recent enough to give EKF3 as a current pose measurement."""
    return latest_imu_time is None or latest_imu_time - scan_time <= limit


def mavlink_payload(state):
    """Convert local FLU odometry to MAVLink's fixed local FRD and moving body FRD."""
    from pymavlink import mavutil
    yaw = state["yaw"]
    q_local = (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))
    q = frames.odom_attitude_to_local_frd(q_local)
    position = frames.odom_flu_to_local_frd(state["position"])
    velocity = frames.odom_velocity_to_body_frd(state["velocity"], q_local)
    cov = state["covariance_xy_yaw"]
    position_covariance = [0.0] * 21
    position_covariance[0], position_covariance[6] = max(cov[0][0], 0.01), max(cov[1][1], 0.01)
    position_covariance[11] = max(state["covariance_z"], 0.01)
    position_covariance[15] = position_covariance[18] = 0.1
    position_covariance[20] = max(cov[2][2], 0.01)
    quality = 80 if state["healthy"] else -1
    return (int(state["time"] * 1e6), mavutil.mavlink.MAV_FRAME_LOCAL_FRD,
            mavutil.mavlink.MAV_FRAME_BODY_FRD, *position, (q[3], q[0], q[1], q[2]),
            *velocity, 0.0, 0.0, 0.0, position_covariance, [0.0] * 21, 0, 0, quality)


class LidarNode(Node):
    def __init__(self, external_nav=False, explore=False, inject_unhealthy=False):
        super().__init__("ironlark_lidar_odometry")
        config = json.loads((ROOT / "config/sensors.json").read_text())
        self.estimate = ScanOdometry(config["lidar"]["translation"])
        self.map = OccupancyMap(lidar_offset=config["lidar"]["translation"],
                                horizontal_resolution=config["lidar"]["horizontal_resolution"],
                                free_ray_layer=config["lidar"]["layers"] - 2)
        self.scan_count = 0
        self.external_nav = external_nav
        self.explore = explore
        self.explorer = Explorer(self.map) if explore else None
        self.mav = None
        self.odometry = self.create_publisher(Odometry, "/odom/lidar", 20)
        self.health = self.create_publisher(String, "/odom/health", 20)
        self.latest_imu_time = None
        # Evaluation only: the mission writes this file to inject unhealthy estimates in flight.
        self.unhealthy_trigger = Path("/run/ironlark") / "inject-unhealthy-odometry" if inject_unhealthy else None
        self.stale_scans = 0
        # Reliable, as before: best-effort readers of the bridge's large scans were followed by
        # the bridge dying silently in one flight (runs/20261114-125809-17842).
        self.create_subscription(Imu, "/imu/data", self.on_imu, 100)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_scan, 10)
        self.report = (Path("/run/ironlark") / "odometry.jsonl").open("w", buffering=1)
        self.navigation_log = ((Path("/run/ironlark") / "navigation.jsonl").open("w", buffering=1)
                               if explore else None)

    def on_imu(self, message):
        self.latest_imu_time = seconds(message.header.stamp)
        self.estimate.imu(self.latest_imu_time, message.angular_velocity.z,
                          message.angular_velocity.x, message.angular_velocity.y,
                          (message.linear_acceleration.x, message.linear_acceleration.y,
                           message.linear_acceleration.z))

    def on_scan(self, message):
        cloud = cloud_xyzt(message)
        stamp = seconds(message.header.stamp)
        state = self.estimate.scan(stamp, cloud)
        if self.unhealthy_trigger is not None and self.unhealthy_trigger.exists():
            state = {**state, "healthy": False, "reason": "injected unhealthy estimate"}
        lag = None if self.latest_imu_time is None else self.latest_imu_time - stamp
        fresh = external_nav_fresh(stamp, self.latest_imu_time)
        if self.external_nav:
            # Before mapping and planning: EKF3 gets the pose as soon as it is estimated.
            if fresh:
                self.send_mavlink(state)
            else:
                self.stale_scans += 1
        self.report.write(json.dumps({**state, "delivery_lag_s": None if lag is None else round(lag, 4),
                                      "received_wall_s": round(time.monotonic(), 3),
                                      "external_nav_sent": self.external_nav and fresh}) + "\n")
        self.scan_count += 1
        odom = Odometry()
        odom.header = message.header
        odom.header.stamp.sec = int(state["time"])
        odom.header.stamp.nanosec = int(round((state["time"] - int(state["time"])) * 1e9))
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z = state["position"]
        yaw = state["yaw"]
        odom.pose.pose.orientation.z = math.sin(yaw / 2)
        odom.pose.pose.orientation.w = math.cos(yaw / 2)
        body_velocity = frames.odom_velocity_to_body_flu(
            state["velocity"], (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z = body_velocity
        covariance = state["covariance_xy_yaw"]
        for index, value in ((0, covariance[0][0]), (7, covariance[1][1]), (14, state["covariance_z"]),
                             (21, 0.1), (28, 0.1), (35, covariance[2][2])):
            odom.pose.covariance[index] = max(value, 0.01)
        self.odometry.publish(odom)
        self.health.publish(String(data=state["reason"]))
        if self.scan_count % 10 == 0:
            if state["healthy"]:
                self.map.update(state, cloud)
            if self.explore:
                self.plan_exploration(state)

    def plan_exploration(self, state):
        target = self.explorer.plan(state)
        path = Path("/run/ironlark") / "navigation_target.json"
        temporary = path.with_suffix(".tmp")
        temporary.write_text(json.dumps(target) + "\n")
        temporary.replace(path)
        self.navigation_log.write(json.dumps(target) + "\n")

    def send_mavlink(self, state):
        from pymavlink import mavutil
        if self.mav is None:
            try:
                self.mav = mavutil.mavlink_connection("tcp:127.0.0.1:5761", source_system=254,
                                                     source_component=191, retries=0)
                self.get_logger().info("connected to SITL external navigation port 5761")
            except OSError:
                return
        try:
            self.mav.mav.odometry_send(*mavlink_payload(state))
        except OSError:
            self.mav.close()
            self.mav = None


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--external-nav", action="store_true")
    parser.add_argument("--explore", action="store_true")
    parser.add_argument("--inject-unhealthy", action="store_true")
    args = parser.parse_args()
    rclpy.init()
    node = LidarNode(args.external_nav, args.explore, args.inject_unhealthy)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        (Path("/run/ironlark") / "occupancy_map.json").write_text(json.dumps(node.map.snapshot()) + "\n")
        node.report.close()
        if node.navigation_log:
            node.navigation_log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
