"""Joint definitions for the openarmx Joint Control tab.

Adapted from the Ti5 `tr_works_joint_control_ui` package for the openarmx
bimanual arm (7 + 7 DOF) plus two prismatic gripper fingers.

Arm joints are handled in DEGREES; gripper fingers in METERS (prismatic).
Limits are loaded from config/joint_limits.yaml (editable without rebuild),
falling back to the defaults below.
"""

from __future__ import annotations

import math
import os

import yaml

# --- UI scale factors (shared between joint_control_tab and other widgets) ----
ARM_SCALE = 10        # slider int = degrees * 10
GRIP_SCALE = 10000    # slider int = meters * 10000  (0.044 m -> 440)

# --- Frame name constants -----------------------------------------------------
# Bimanual base frame used by cyclo MoveL and TF lookups.
CYCLO_BASE_FRAME = "openarmx_body_link0"


def _poses_dir() -> str:
    """Return the canonical directory for saved pose JSON files.

    Respects the OPENARMX_SCENARIOS_DIR environment variable; falls back to
    ~/openarmx_ws/scenarios/poses.
    """
    env = os.environ.get("OPENARMX_SCENARIOS_DIR")
    base = env if env and os.path.isdir(env) else os.path.expanduser(
        "~/openarmx_ws/scenarios")
    return os.path.join(base, "poses")

# --- Arm joints ---------------------------------------------------------------
LEFT_ARM_JOINTS = [f"openarmx_left_joint{i}" for i in range(1, 8)]
RIGHT_ARM_JOINTS = [f"openarmx_right_joint{i}" for i in range(1, 8)]
ARM_JOINT_NAMES = LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS

# --- Gripper finger joints (prismatic, meters) --------------------------------
LEFT_GRIPPER = "openarmx_left_finger_joint1"
RIGHT_GRIPPER = "openarmx_right_finger_joint1"
GRIPPER_NAMES = [LEFT_GRIPPER, RIGHT_GRIPPER]

# All joints we publish to /joint_states for SIL visualisation.
JOINT_NAMES = ARM_JOINT_NAMES + GRIPPER_NAMES

# --- HIL trajectory controllers ----------------------------------------------
# JointTrajectoryController subscribes to <controller>/joint_trajectory.
LEFT_TRAJ_TOPIC = "/left_joint_trajectory_controller/joint_trajectory"
RIGHT_TRAJ_TOPIC = "/right_joint_trajectory_controller/joint_trajectory"
# GripperActionController action namespaces (control_msgs/GripperCommand).
LEFT_GRIPPER_ACTION = "/left_gripper_controller/gripper_cmd"
RIGHT_GRIPPER_ACTION = "/right_gripper_controller/gripper_cmd"

# --- Default limits -----------------------------------------------------------
# Arm limits in degrees (converted from v10 joint_limits.yaml radians).
_ARM_LIMITS_DEG = {
    1: (-71.6, 171.9),
    2: (-97.4, 97.4),
    3: (-90.0, 90.0),
    4: (0.0, 103.1),
    5: (-85.9, 85.9),
    6: (-43.0, 43.0),
    7: (-80.2, 80.2),
}
_DEFAULT_LIMITS = {}
for _i in range(1, 8):
    _DEFAULT_LIMITS[f"openarmx_left_joint{_i}"] = _ARM_LIMITS_DEG[_i]
    _DEFAULT_LIMITS[f"openarmx_right_joint{_i}"] = _ARM_LIMITS_DEG[_i]

# Gripper limits in meters (prismatic finger, from openarmx_hand.xacro).
GRIPPER_LIMITS_M = {
    LEFT_GRIPPER: (0.0, 0.044),
    RIGHT_GRIPPER: (0.0, 0.044),
}


def _load_limits() -> dict:
    """Load joint limits from config/joint_limits.yaml, fallback to defaults.

    Returns a single dict keyed by full joint name. Arm values are degrees,
    gripper values are meters.
    """
    limits = dict(_DEFAULT_LIMITS)
    limits.update(GRIPPER_LIMITS_M)
    try:
        from ament_index_python.packages import get_package_share_directory
        cfg = os.path.join(
            get_package_share_directory("openarmx_scenario_ui"),
            "config", "joint_limits.yaml")
        if os.path.isfile(cfg):
            with open(cfg) as f:
                data = yaml.safe_load(f) or {}
            for name, vals in data.items():
                if isinstance(vals, list) and len(vals) == 2:
                    limits[name] = (float(vals[0]), float(vals[1]))
    except Exception:
        pass
    return limits


# {joint_name: (lo, hi)} — arms in deg, grippers in meters.
JOINT_LIMITS = _load_limits()
# Backwards-friendly alias used by the arm UI (degrees only).
JOINT_LIMITS_DEG = {n: JOINT_LIMITS[n] for n in ARM_JOINT_NAMES}

# --- Mirror mapping -----------------------------------------------------------
# left_jointN <-> right_jointN. MIRROR_SIGN applies a per-index sign when
# copying a value across the body. Default -1 (negate), matching the Ti5 UI;
# adjust per joint if the openarmx URDF axes need same-sign mirroring.
MIRROR_SIGN = {i: -1.0 for i in range(1, 8)}

MIRROR_PAIRS = {}
for _i in range(1, 8):
    _l = f"openarmx_left_joint{_i}"
    _r = f"openarmx_right_joint{_i}"
    MIRROR_PAIRS[_l] = (_r, MIRROR_SIGN[_i])
    MIRROR_PAIRS[_r] = (_l, MIRROR_SIGN[_i])

# --- Preset positions ---------------------------------------------------------
# HOME: everything at zero.
HOME_POSITION_DEG = {name: 0.0 for name in ARM_JOINT_NAMES}

# INIT: a mild ready pose (elbow joint4 slightly bent). Adjust to taste.
INIT_POSITION_DEG = {name: 0.0 for name in ARM_JOINT_NAMES}
INIT_POSITION_DEG["openarmx_left_joint4"] = 45.0
INIT_POSITION_DEG["openarmx_right_joint4"] = 45.0

# Gripper presets (meters): fully open.
HOME_GRIPPER_M = {name: 0.0 for name in GRIPPER_NAMES}
INIT_GRIPPER_M = {name: 0.044 for name in GRIPPER_NAMES}


def spinbox_name(joint_name: str) -> str:
    return f"spin_{joint_name}"


def is_gripper(joint_name: str) -> bool:
    return joint_name in GRIPPER_NAMES
