"""Live LiDAR odometry and optional MAVLink external navigation, on Webots /clock time."""
import argparse
import json
import math
import os
import sys
from pathlib import Path

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu, PointCloud2
from std_msgs.msg import String

os.environ.setdefault("MAVLINK20", "1")  # ODOMETRY is a MAVLink 2 message

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from sensing import frames  # noqa: E402
from sensing.odometry import ScanOdometry  # noqa: E402
from sensing.pointcloud import cloud_xyzt, seconds  # noqa: E402


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
            *velocity, 0.0, 0.0, 0.0, position_covariance, [math.nan] + [0.0] * 20, 0, 0, quality)


class LidarNode(Node):
    def __init__(self, external_nav=False):
        super().__init__("ironlark_lidar_odometry")
        config = json.loads((ROOT / "config/sensors.json").read_text())
        self.estimate = ScanOdometry(config["lidar"]["translation"])
        self.external_nav = external_nav
        self.mav = None
        self.odometry = self.create_publisher(Odometry, "/odom/lidar", 20)
        self.health = self.create_publisher(String, "/odom/health", 20)
        self.create_subscription(Imu, "/imu/data", self.on_imu, 100)
        self.create_subscription(PointCloud2, "/lidar/points", self.on_scan, 10)
        self.report = (Path("/run/ironlark") / "odometry.jsonl").open("w", buffering=1)

    def on_imu(self, message):
        self.estimate.imu(seconds(message.header.stamp), message.angular_velocity.z,
                          message.angular_velocity.x, message.angular_velocity.y,
                          (message.linear_acceleration.x, message.linear_acceleration.y,
                           message.linear_acceleration.z))

    def on_scan(self, message):
        state = self.estimate.scan(seconds(message.header.stamp), cloud_xyzt(message))
        self.report.write(json.dumps(state) + "\n")
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
        odom.twist.covariance[0] = -1.0  # velocity covariance is not estimated
        covariance = state["covariance_xy_yaw"]
        for index, value in ((0, covariance[0][0]), (7, covariance[1][1]), (14, state["covariance_z"]),
                             (21, 0.1), (28, 0.1), (35, covariance[2][2])):
            odom.pose.covariance[index] = max(value, 0.01)
        self.odometry.publish(odom)
        self.health.publish(String(data=state["reason"]))
        if self.external_nav:
            self.send_mavlink(state)

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
    args = parser.parse_args()
    rclpy.init()
    node = LidarNode(args.external_nav)
    try:
        rclpy.spin(node)
    finally:
        node.report.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
