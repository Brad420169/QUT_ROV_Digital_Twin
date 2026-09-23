"""Small reusable control/math helper functions."""

import math

from tf_transformations import euler_from_quaternion, quaternion_from_euler


def apply_deadzone(value: float, deadzone: float) -> float:
    if abs(value) < deadzone:
        return 0.0

    sign = 1.0 if value > 0.0 else -1.0
    return sign * (abs(value) - deadzone) / (1.0 - deadzone)


def trigger_to_command(raw: float) -> float:
    if raw > 0.0:
        return 0.0
    return min(1.0, abs(raw))


def clamp(value: float, minimum: float = -1.0, maximum: float = 1.0) -> float:
    return max(minimum, min(maximum, value))


def wrap_to_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    return quaternion_to_rpy(x, y, z, w)[2]


def quaternion_to_rpy(x: float, y: float, z: float, w: float):
    """Roll, pitch, yaw in radians (ZYX convention)."""
    return euler_from_quaternion([x, y, z, w])


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    """Inverse of quaternion_to_rpy. Returns (x, y, z, w)."""
    return tuple(quaternion_from_euler(roll, pitch, yaw))