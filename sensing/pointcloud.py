"""Decode the bridge's timestamped PointCloud2 layout for sensor consumers."""
import numpy as np


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def cloud_xyzt(message):
    if message.point_step != 20 or message.is_bigendian:
        raise ValueError("unexpected LiDAR point layout")
    data = np.frombuffer(message.data, dtype=np.dtype({"names": ["x", "y", "z", "time"],
                                                       "formats": ["<f4"] * 4,
                                                       "offsets": [0, 4, 8, 16], "itemsize": 20}))
    return np.column_stack((data["x"], data["y"], data["z"], data["time"]))
