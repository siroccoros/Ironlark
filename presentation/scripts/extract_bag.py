"""Dump a recorded flight's bag into arrays the presentation's data step reads (runs in the ROS 2 container).

    python3 extract_bag.py RUN OUT

OUT/truth.npy (t, x, y, z, qx, qy, qz, qw), OUT/imu.npy (t, gyro xyz, accel xyz), OUT/static.json,
OUT/scans/<ns>.npy (x, y, z, ring, time: the LiDAR frame), OUT/frames/<ns>.npy (every second camera image, BGRA).
"""
import json
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "/project/tools")
from bag_reader import open_bag  # noqa: E402
from rclpy.serialization import deserialize_message  # noqa: E402
from rosidl_runtime_py.utilities import get_message  # noqa: E402

run, out = Path(sys.argv[1]), Path(sys.argv[2])
(out / "scans").mkdir(parents=True, exist_ok=True)
(out / "frames").mkdir(parents=True, exist_ok=True)
reader = open_bag(run / "bag")
types = {t.name: t.type for t in reader.get_all_topics_and_types()}
truth, imu, static, images = [], [], {}, 0
point = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("ring", "<i4"), ("time", "<f4")])
while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == "/clock" or topic == "/ground_truth/velocity" or topic == "/camera/camera_info":
        continue
    if topic == "/camera/image_raw":
        images += 1
        if images % 2:
            continue
    msg = deserialize_message(data, get_message(types[topic]))
    if topic == "/tf_static":
        for t in msg.transforms:
            r, tr = t.transform.rotation, t.transform.translation
            static[t.child_frame_id] = [[tr.x, tr.y, tr.z], [r.x, r.y, r.z, r.w]]
        continue
    stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
    if topic == "/ground_truth/pose":
        p, o = msg.pose.position, msg.pose.orientation
        truth.append((stamp / 1e9, p.x, p.y, p.z, o.x, o.y, o.z, o.w))
    elif topic == "/imu/data":
        g, a = msg.angular_velocity, msg.linear_acceleration
        imu.append((stamp / 1e9, g.x, g.y, g.z, a.x, a.y, a.z))
    elif topic == "/lidar/points":
        np.save(out / "scans" / f"{stamp}.npy", np.frombuffer(bytes(msg.data), point))
    elif topic == "/camera/image_raw":
        np.save(out / "frames" / f"{stamp}.npy",
                np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 4))
np.save(out / "truth.npy", np.array(truth))
np.save(out / "imu.npy", np.array(imu))
(out / "static.json").write_text(json.dumps(static, indent=2))
print(len(truth), len(imu), len(list((out / "scans").iterdir())), len(list((out / "frames").iterdir())))
