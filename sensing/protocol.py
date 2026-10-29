"""Wire format from the Webots controller to the ROS 2 bridge: framed messages over TCP.

Each message: a 20-byte header (magic, type, Webots time in nanoseconds, payload length), then the
payload. Little-endian. Plain Python (struct), so both sides load it without extra packages.
At the end the controller shuts down its side; the bridge reads to the end of the stream, answers
with the number of messages it read (uint64) and closes.

Ground truth never enters the stream: the controller writes it to truth.bin in the run folder (TRUTH
records), for checks and evaluation only.
"""
import socket
from pathlib import Path
import struct

MAGIC = b"IRLK"
HEADER = struct.Struct("<4sBxxxqI")

CLOCK = 1  # no payload: Webots time advanced to this step
IMU = 2  # angular velocity, linear acceleration: 6 doubles (an IMU measures no orientation)
LIDAR = 3  # uint32 count, then count returns (x, y, z float32, layer int32, time float32: s after the stamp)
IMAGE = 4  # uint16 width, uint16 height, then width * height BGRA bytes, rows from the top
# 5: ground truth until it left the stream (truth.bin); kept unused so old and new streams never mix.

VECTOR6 = struct.Struct("<6d")
# truth.bin: Webots time (ns), position, orientation (x, y, z, w), linear velocity, world ENU.
TRUTH = struct.Struct("<q10d")
COUNT = struct.Struct("<I")
SIZE = struct.Struct("<HH")
ACK = struct.Struct("<Q")
LIDAR_POINT_BYTES = 20


def pack(kind, time_ns, payload=b""):
    return HEADER.pack(MAGIC, kind, time_ns, len(payload)) + payload


def read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        chunk = stream.recv(size - len(data))
        if not chunk:
            raise EOFError
        data += chunk
    return bytes(data)


def read_message(stream):
    """(kind, time_ns, payload) from a socket; EOFError when it closes."""
    magic, kind, time_ns, length = HEADER.unpack(read_exact(stream, HEADER.size))
    if magic != MAGIC:
        raise ValueError("stream out of sync")
    return kind, time_ns, read_exact(stream, length)


def finish(stream):
    """End a stream: the number of messages the bridge acknowledges reading."""
    stream.shutdown(socket.SHUT_WR)
    return ACK.unpack(read_exact(stream, ACK.size))[0]


def read_truth(path):
    """truth.bin's records: (time_ns, position, orientation, velocity) tuples, in time order."""
    data = Path(path).read_bytes()
    if len(data) % TRUTH.size:
        raise ValueError(f"{path} ends inside a record: cut short")
    return [(r[0], r[1:4], r[4:8], r[8:11]) for r in TRUTH.iter_unpack(data)]
