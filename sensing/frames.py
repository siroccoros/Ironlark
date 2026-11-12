"""The one place that defines Ironlark's frames.

Webots world: ENU (x east, y north, z up); its robot bodies: FLU (x forward, y left, z up). Those
are ROS REP-103's map/odom and base_link conventions, so data passes from Webots to ROS unchanged.
ArduPilot works in NED (x north, y east, z down) and FRD bodies (x forward, y right, z down).
The pinned Webots flight bridge sends (Webots x, -y, -z) to SITL: its local forward axis
therefore aligns with Webots +x at the initial heading. The odometer's local origin also starts
along that heading; its MAVLink LOCAL_FRD conversion is separate from a standard ENU to NED map
conversion. The functions below own both conversions.
Quaternions are (x, y, z, w).
"""
import math


def enu_to_ned(v):
    x, y, z = v
    return (y, x, -z)


def flu_to_frd(v):
    x, y, z = v
    return (x, -y, -z)


def quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz)


def axis_angle_to_quaternion(rotation):
    """Webots rotation [axis x, y, z, angle] as a quaternion."""
    x, y, z, angle = rotation
    norm = math.sqrt(x * x + y * y + z * z) or 1.0
    s = math.sin(angle / 2) / norm
    return (x * s, y * s, z * s, math.cos(angle / 2))


# A camera_link (FLU, looking along +x) to its optical frame (x right, y down, z forward).
CAMERA_LINK_TO_OPTICAL = (-0.5, 0.5, -0.5, 0.5)

# World ENU to NED and body FLU to FRD, as rotations: q_ned_frd = ENU_TO_NED * q_enu_flu * FLU_TO_FRD.
ENU_TO_NED = (math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0)
FLU_TO_FRD = (1.0, 0.0, 0.0, 0.0)


def attitude_enu_flu_to_ned_frd(q):
    return quaternion_multiply(quaternion_multiply(ENU_TO_NED, q), FLU_TO_FRD)


def odom_flu_to_local_frd(v):
    """The scan odometer's origin is its initial body heading; its local x is forward."""
    return flu_to_frd(v)


def webots_to_sitl_local(v):
    """Pinned Webots flight bridge's FDM mapping: Webots (x,y,z) -> SITL (x,-y,-z)."""
    return flu_to_frd(v)


def odom_attitude_to_local_frd(q):
    """Local FLU body attitude into the fixed initial-heading FRD frame."""
    return quaternion_multiply(quaternion_multiply(FLU_TO_FRD, q), FLU_TO_FRD)


def odom_velocity_to_body_frd(velocity, attitude_local_flu):
    q = odom_attitude_to_local_frd(attitude_local_flu)
    inverse = (-q[0], -q[1], -q[2], q[3])
    vector = (*odom_flu_to_local_frd(velocity), 0.0)
    return quaternion_multiply(quaternion_multiply(inverse, vector), q)[:3]


def odom_velocity_to_body_flu(velocity, attitude_local_flu):
    q = attitude_local_flu
    inverse = (-q[0], -q[1], -q[2], q[3])
    return quaternion_multiply(quaternion_multiply(inverse, (*velocity, 0.0)), q)[:3]
