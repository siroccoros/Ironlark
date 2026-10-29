"""ROS 2 side of the sensor stream: publish what the Webots controller sends, in Webots time.

Listens on config/sensors.json's stream address for sensing/protocol.py messages and publishes
/clock, /imu/data, /lidar/points, /camera/image_raw, /camera/camera_info and static transforms
from base_link to each sensor. Ground truth never reaches it (the controller writes truth.bin). Frames
follow REP-103 (sensing/frames.py): Webots' world and bodies are ENU and FLU. When the controller
finishes, it writes how many messages it published to published.json in the run folder (argument 1),
before acknowledging: the launcher stops the recorder once the bag holds them all.
"""
import array
import json
import math
import socket
import sys
import threading
import time
import traceback
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from sensing import frames, protocol  # noqa: E402


def stamp(time_ns):
    return Time(sec=time_ns // 1_000_000_000, nanosec=time_ns % 1_000_000_000)


class Bridge(Node):
    def __init__(self, config):
        super().__init__("ironlark_bridge")
        self.config = config
        # Reliable, and deep: a subscriber (the recorder) that falls behind loses nothing. Depths
        # match ros/record_qos.yaml: large for small high-rate messages, modest for images.
        def reliable(depth):
            return QoSProfile(depth=depth, reliability=ReliabilityPolicy.RELIABLE)

        self.clock = self.create_publisher(Clock, "/clock", reliable(20000))
        self.imu = self.create_publisher(Imu, "/imu/data", reliable(10000))
        self.points = self.create_publisher(PointCloud2, "/lidar/points", reliable(200))
        self.image = self.create_publisher(Image, "/camera/image_raw", reliable(200))
        self.camera_info = self.create_publisher(CameraInfo, "/camera/camera_info", reliable(200))
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.static_tf = self.create_publisher(TFMessage, "/tf_static", latched)
        self.static_published = False
        self.published = 0  # messages published, the static transforms included

        imu = config["imu"]
        noise = (imu["accelerometer_noise"] ** 2, imu["gyro_noise"] ** 2)
        self.accel_covariance = [noise[0], 0.0, 0.0, 0.0, noise[0], 0.0, 0.0, 0.0, noise[0]]
        self.gyro_covariance = [noise[1], 0.0, 0.0, 0.0, noise[1], 0.0, 0.0, 0.0, noise[1]]
        self.point_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="ring", offset=12, datatype=PointField.INT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        ]

    def publish(self, publisher, msg):
        publisher.publish(msg)
        self.published += 1

    def publish_static(self, time_ns):
        transforms = []

        def add(parent, child, translation, quaternion):
            t = TransformStamped()
            t.header.stamp = stamp(time_ns)
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = translation
            (t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
             t.transform.rotation.w) = quaternion
            transforms.append(t)

        for key in ("imu", "lidar", "camera"):
            sensor = self.config[key]
            add("base_link", sensor["frame_id"], sensor["translation"],
                frames.axis_angle_to_quaternion(sensor["rotation"]))
        camera = self.config["camera"]
        add(camera["frame_id"], camera["optical_frame_id"], (0.0, 0.0, 0.0), frames.CAMERA_LINK_TO_OPTICAL)
        self.publish(self.static_tf, TFMessage(transforms=transforms))
        self.static_published = True

    def on_message(self, kind, time_ns, payload):
        if not self.static_published:
            self.publish_static(time_ns)
        header_stamp = stamp(time_ns)
        if kind == protocol.CLOCK:
            self.publish(self.clock, Clock(clock=header_stamp))
        elif kind == protocol.IMU:
            gx, gy, gz, ax, ay, az = protocol.VECTOR6.unpack(payload)
            msg = Imu()
            msg.header.stamp, msg.header.frame_id = header_stamp, self.config["imu"]["frame_id"]
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = gx, gy, gz
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = ax, ay, az
            msg.angular_velocity_covariance = self.gyro_covariance
            msg.linear_acceleration_covariance = self.accel_covariance
            msg.orientation_covariance[0] = -1.0  # no orientation: the IMU measures rates and acceleration (REP-145)
            self.publish(self.imu, msg)
        elif kind == protocol.LIDAR:
            (count,) = protocol.COUNT.unpack_from(payload)
            msg = PointCloud2()
            msg.header.stamp, msg.header.frame_id = header_stamp, self.config["lidar"]["frame_id"]
            msg.height, msg.width = 1, count
            msg.fields = self.point_fields
            msg.is_bigendian, msg.is_dense = False, True  # hits only: rays that hit nothing are left out
            msg.point_step = protocol.LIDAR_POINT_BYTES
            msg.row_step = count * protocol.LIDAR_POINT_BYTES
            # array('B'): rclpy copies it; assigning bytes checks each byte in Python.
            msg.data = array.array("B", payload[protocol.COUNT.size:])
            self.publish(self.points, msg)
        elif kind == protocol.IMAGE:
            width, height = protocol.SIZE.unpack_from(payload)
            camera = self.config["camera"]
            msg = Image()
            msg.header.stamp, msg.header.frame_id = header_stamp, camera["optical_frame_id"]
            msg.height, msg.width, msg.encoding, msg.step = height, width, "bgra8", 4 * width
            msg.data = array.array("B", payload[protocol.SIZE.size:])
            self.publish(self.image, msg)
            info = CameraInfo()
            info.header = msg.header
            info.width, info.height, info.distortion_model = width, height, "plumb_bob"
            focal = width / 2.0 / math.tan(camera["field_of_view"] / 2.0)
            cx, cy = width / 2.0, height / 2.0
            info.k = [focal, 0.0, cx, 0.0, focal, cy, 0.0, 0.0, 1.0]
            info.d = [0.0] * 5
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [focal, 0.0, cx, 0.0, 0.0, focal, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
            self.publish(self.camera_info, info)

def serve(node, address, run):
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(address)
    server.listen(1)
    node.get_logger().info(f"waiting for the Webots controller on {address[0]}:{address[1]}")
    while rclpy.ok():
        connection, _ = server.accept()
        node.get_logger().info("controller connected")
        received, started = 0, time.monotonic()
        try:
            while True:
                node.on_message(*protocol.read_message(connection))
                received += 1
                if received % 5000 == 0:
                    rate = received / (time.monotonic() - started)
                    node.get_logger().info(f"{received} messages, {rate:.0f} per second")
        except EOFError:
            # The controller finished (protocol.finish): every message it sent was read and published.
            node.get_logger().info(f"controller finished: {received} messages, {node.published} published")
            temporary = run / "published.json.tmp"
            temporary.write_text(json.dumps({"published": node.published}) + "\n")
            temporary.replace(run / "published.json")
            try:
                connection.sendall(protocol.ACK.pack(received))
            except OSError:
                pass
        except Exception:  # closing tells the controller, which reports the stream failed
            node.get_logger().error(f"controller stream failed after {received} messages:\n{traceback.format_exc()}")
        connection.close()


def main():
    config = json.loads((ROOT / "config/sensors.json").read_text())
    rclpy.init()
    node = Bridge(config)
    stream = config["stream"]
    threading.Thread(target=serve, args=(node, (stream["host"], stream["port"]), Path(sys.argv[1])),
                     daemon=True).start()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
