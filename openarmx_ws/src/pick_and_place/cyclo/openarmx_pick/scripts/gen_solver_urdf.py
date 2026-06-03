#!/usr/bin/env python3
"""Generate a single-arm OpenArmX URDF for the cyclo_control QP MoveL solver.

The cyclo_control KinematicsSolver (Pinocchio) treats every movable joint in the
URDF as a controllable DOF. For a single-arm task we therefore:

  * keep the LEFT arm's 7 revolute joints movable (the 7-DOF chain), and
  * freeze (convert to ``fixed``) the RIGHT arm joints and ALL gripper finger
    joints so they become static geometry rather than extra DOF.

Because ``world -> openarmx_body_link0`` is an identity fixed joint, the model
root frame coincides with ``openarmx_body_link0``. That is exactly the frame the
vision pipeline publishes grasp poses in, so the solver FK needs no extra TF.

Collision handling is staged. cyclo's KinematicsSolver always calls
``buildGeom(COLLISION)`` + ``addAllCollisionPairs()``; with no SRDF every pair
(including adjacent links) is active, which makes the collision-CBF QP infeasible.
``--no-collision`` strips every ``<collision>`` so buildGeom yields an empty
geometry model -> the solver runs with joint-limit + singularity CBF only. Use
that for first bring-up, then regenerate with collisions + an SRDF.

Usage:
  gen_solver_urdf.py IN.urdf OUT.urdf [--arm left|right] [--no-collision] [--strip-visual]
"""
import argparse
import sys
import xml.etree.ElementTree as ET

# Joint sub-elements that only make sense for a movable joint.
_MOVABLE_ONLY = ("axis", "limit", "dynamics", "mimic", "safety_controller")


def freeze_joint(joint: ET.Element) -> None:
    """Turn a movable joint into a fixed one in place."""
    joint.set("type", "fixed")
    for tag in _MOVABLE_ONLY:
        for el in joint.findall(tag):
            joint.remove(el)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("input")
    ap.add_argument("output")
    ap.add_argument("--arm", choices=["left", "right"], default="left",
                    help="arm that stays movable (default: left)")
    ap.add_argument("--no-collision", action="store_true",
                    help="strip all <collision> (stage-1 bring-up, empty geom model)")
    ap.add_argument("--strip-visual", action="store_true",
                    help="also strip <visual> to shrink the file")
    args = ap.parse_args(argv)

    keep = args.arm                       # movable arm prefix token
    drop = "right" if keep == "left" else "left"

    tree = ET.parse(args.input)
    root = tree.getroot()

    frozen, kept = [], []
    for joint in root.findall("joint"):
        name = joint.get("name") or ""
        jtype = joint.get("type")
        if jtype not in ("revolute", "prismatic", "continuous"):
            continue
        # Freeze the other arm and every finger joint; keep this arm's arm-joints.
        is_finger = "finger" in name
        is_drop_arm = f"_{drop}_" in name
        is_keep_arm_joint = (f"_{keep}_joint" in name) and not is_finger
        if is_keep_arm_joint:
            kept.append(name)
        elif is_drop_arm or is_finger:
            freeze_joint(joint)
            frozen.append(name)
        else:
            # Unknown movable joint -> freeze defensively so DOF stays exactly the arm.
            freeze_joint(joint)
            frozen.append(name)

    if args.no_collision:
        for link in root.findall("link"):
            for col in link.findall("collision"):
                link.remove(col)
    if args.strip_visual:
        for link in root.findall("link"):
            for vis in link.findall("visual"):
                link.remove(vis)

    tree.write(args.output, encoding="utf-8", xml_declaration=True)

    print(f"[gen_solver_urdf] movable arm = {keep}  ({len(kept)} DOF)")
    for n in kept:
        print(f"    keep   {n}")
    print(f"[gen_solver_urdf] frozen {len(frozen)} joints (other arm + fingers)")
    print(f"[gen_solver_urdf] collision = {'STRIPPED' if args.no_collision else 'kept'}"
          f"  visual = {'STRIPPED' if args.strip_visual else 'kept'}")
    print(f"[gen_solver_urdf] wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
