"""Geometry utilities for openarmx_scenario_ui.

Pure-Python SE3 / quaternion / RPY helpers that are shared across multiple
modules (scenario_action_client, ik_check, cartesian_control_tab).  No ROS2
imports; no PyQt5 imports — pure math only so the module can be imported from
any context.

Coordinate conventions
----------------------
  RPY (roll, pitch, yaw) : ZYX intrinsic Euler angles (radians).
  Quaternion             : (qx, qy, qz, qw) — ROS / geometry_msgs convention.
"""

from __future__ import annotations

import math
from typing import Tuple


# ---------------------------------------------------------------------------
# Basic RPY ↔ quaternion conversion
# ---------------------------------------------------------------------------

def rpy_to_quat(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
    """Convert ZYX intrinsic Euler angles (rad) to quaternion (qx, qy, qz, qw)."""
    cr, sr = math.cos(roll / 2.0),  math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0),   math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,   # qx
        cr * sp * cy + sr * cp * sy,   # qy
        cr * cp * sy - sr * sp * cy,   # qz
        cr * cp * cy + sr * sp * sy,   # qw
    )


def quat_to_rpy(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert quaternion (qx, qy, qz, qw) to ZYX intrinsic Euler angles (rad).

    Returns (roll, pitch, yaw).
    """
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (qw * qy - qz * qx)
    pitch = (math.copysign(math.pi / 2.0, sinp)
             if abs(sinp) >= 1.0 else math.asin(sinp))
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


# ---------------------------------------------------------------------------
# Pose dict helpers
# ---------------------------------------------------------------------------

def pose_dict_to_se3_components(
    pose: dict,
) -> Tuple[Tuple[float, float, float], Tuple[float, float, float, float]]:
    """Extract translation and quaternion from a pose dict.

    Accepts dicts that carry either:
      * quaternion keys   qx, qy, qz, qw  (preferred), OR
      * RPY keys          roll, pitch, yaw  (converted on the fly).

    Returns:
        ((x, y, z), (qx, qy, qz, qw))
    """
    xyz = (float(pose["x"]), float(pose["y"]), float(pose["z"]))
    if "qw" in pose:
        q = (float(pose["qx"]), float(pose["qy"]),
             float(pose["qz"]), float(pose["qw"]))
    else:
        q = rpy_to_quat(pose["roll"], pose["pitch"], pose["yaw"])
    return xyz, q


def interp_pose(start: dict, goal: dict, t: float) -> dict:
    """Linearly interpolate between two pose dicts at parameter t ∈ [0, 1].

    Position is linearly interpolated; orientation uses SLERP.
    Both dicts must contain x/y/z and either qx/qy/qz/qw or roll/pitch/yaw.
    Returns a new dict with x/y/z, qx/qy/qz/qw, roll/pitch/yaw, and
    frame_id (taken from start if present).
    """
    t = max(0.0, min(1.0, float(t)))
    (sx, sy, sz), (sqx, sqy, sqz, sqw) = pose_dict_to_se3_components(start)
    (gx, gy, gz), (gqx, gqy, gqz, gqw) = pose_dict_to_se3_components(goal)

    # Linear position interpolation
    ix = sx + t * (gx - sx)
    iy = sy + t * (gy - sy)
    iz = sz + t * (gz - sz)

    # SLERP for orientation
    dot = sqx * gqx + sqy * gqy + sqz * gqz + sqw * gqw
    # Ensure shortest path
    if dot < 0.0:
        gqx, gqy, gqz, gqw = -gqx, -gqy, -gqz, -gqw
        dot = -dot
    dot = min(1.0, dot)
    if dot > 0.9995:
        # Nearly identical — linear fallback + renormalize
        iqx = sqx + t * (gqx - sqx)
        iqy = sqy + t * (gqy - sqy)
        iqz = sqz + t * (gqz - sqz)
        iqw = sqw + t * (gqw - sqw)
    else:
        theta_0 = math.acos(dot)
        sin_theta_0 = math.sin(theta_0)
        theta = theta_0 * t
        sin_theta = math.sin(theta)
        s0 = math.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        iqx = s0 * sqx + s1 * gqx
        iqy = s0 * sqy + s1 * gqy
        iqz = s0 * sqz + s1 * gqz
        iqw = s0 * sqw + s1 * gqw

    # Normalize
    n = math.sqrt(iqx * iqx + iqy * iqy + iqz * iqz + iqw * iqw)
    if n > 1e-9:
        iqx, iqy, iqz, iqw = iqx / n, iqy / n, iqz / n, iqw / n

    roll, pitch, yaw = quat_to_rpy(iqx, iqy, iqz, iqw)
    result: dict = {
        "x": ix, "y": iy, "z": iz,
        "qx": iqx, "qy": iqy, "qz": iqz, "qw": iqw,
        "roll": roll, "pitch": pitch, "yaw": yaw,
    }
    if "frame_id" in start:
        result["frame_id"] = start["frame_id"]
    return result
