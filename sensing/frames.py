"""Single source of truth for Ironlark's coordinate frames and quaternion math.

Conventions:
- Webots world: ENU (x east, y north, z up); robot bodies: FLU (x forward, y left, z up).
  ROS 2 (REP-103) uses the exact same conventions, so sensor frames pass from Webots to ROS unchanged.
- ArduPilot SITL via upstream Webots_Python maps Webots world (x, y, z) -> SITL NED (x, -y, -z)
  and body FLU -> FRD (x, -y, -z), keeping yaw=0 along Webots +X. Use webots_to_sitl_ned() /
  sitl_ned_to_webots() for MAVLink local-NED messages (VISION_POSITION_ESTIMATE, GUIDED setpoints).
- Standard geographic ENU <-> NED conversions are also provided below.
- Quaternions are (x, y, z, w).
"""
import math


def enu_to_ned(v):
    x, y, z = v
    return (y, x, -z)


def flu_to_frd(v):
    x, y, z = v
    return (x, -y, -z)


def webots_to_sitl_ned(v):
    """Webots world (x, y, z) -> ArduPilot SITL local NED, matching upstream Webots_Python."""
    return (v[0], -v[1], -v[2])


def sitl_ned_to_webots(v):
    """ArduPilot SITL local NED -> Webots world (x, y, z) (self-inverse)."""
    return (v[0], -v[1], -v[2])


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def quaternion_inverse(q):
    return (-q[0], -q[1], -q[2], q[3])


def rotate_vector(q, v):
    x, y, z, w = q
    return quaternion_multiply(quaternion_multiply(q, (*v, 0.0)), (-x, -y, -z, w))[:3]


def axis_angle_to_quaternion(rotation):
    """Webots rotation [axis x, y, z, angle] as a quaternion."""
    x, y, z, angle = rotation
    norm = math.sqrt(x * x + y * y + z * z) or 1.0
    s = math.sin(angle / 2) / norm
    return (x * s, y * s, z * s, math.cos(angle / 2))


def quaternion_to_euler(q):
    """Intrinsic ZYX Euler angles (roll, pitch, yaw) in radians from quaternion (x, y, z, w)."""
    x, y, z, w = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return (roll, pitch, yaw)


def webots_euler_to_sitl_ned(rpy):
    """Webots FLU/world (roll, pitch, yaw) -> ArduPilot SITL FRD/NED (roll, -pitch, -yaw)."""
    return (rpy[0], -rpy[1], -rpy[2])


# A camera_link (FLU, looking along +x) to its optical frame (x right, y down, z forward).
CAMERA_LINK_TO_OPTICAL = (-0.5, 0.5, -0.5, 0.5)

# World ENU to NED and body FLU to FRD, as rotations: q_ned_frd = ENU_TO_NED * q_enu_flu * FLU_TO_FRD.
ENU_TO_NED = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
FLU_TO_FRD = (1.0, 0.0, 0.0, 0.0)


def attitude_enu_flu_to_ned_frd(q):
    return quaternion_multiply(quaternion_multiply(ENU_TO_NED, q), FLU_TO_FRD)
