#!/usr/bin/env python3
"""Generate a BIMANUAL OpenArmX URDF for the cyclo_control QP VR / MoveL solver.

The cyclo_control KinematicsSolver (Pinocchio) treats every movable joint in the
URDF as a controllable DOF. For a bimanual task we therefore:

  * keep BOTH arms' 7 revolute joints movable (14-DOF total chain), and
  * freeze (convert to ``fixed``) the gripper finger joints (4 total) so they
    become static geometry rather than extra DOF.

Because ``world -> openarmx_body_link0`` is an identity fixed joint, the model
root frame coincides with ``openarmx_body_link0``. That is exactly the frame
both the vision pipeline and the RViz interactive markers publish poses in, so
the solver FK needs no extra TF.

Collision handling is staged. cyclo's KinematicsSolver always calls
``buildGeom(COLLISION)`` + ``addAllCollisionPairs()``; with no SRDF every pair
(including adjacent links) is active, which makes the collision-CBF QP
infeasible. ``--no-collision`` strips every ``<collision>`` so buildGeom yields
an empty geometry model -> the solver runs with joint-limit + singularity CBF
only. Use that for first bring-up, then regenerate with collisions + an SRDF
to enable inter-arm self-collision CBF.

Companion to ``openarmx_pick/scripts/gen_solver_urdf.py`` (single-arm). Both
share the freeze-other-DOF strategy; this script differs by freezing nothing
on the arm chains.

Usage:
  gen_bimanual_urdf.py IN.urdf OUT.urdf [--no-collision] [--strip-visual]
"""
import argparse
import sys
import xml.etree.ElementTree as ET

# Joint sub-elements that only make sense for a movable joint.
_MOVABLE_ONLY = ("axis", "limit", "dynamics", "mimic", "safety_controller")
_MOVABLE_TYPES = ("revolute", "prismatic", "continuous")


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
    ap.add_argument("--no-collision", action="store_true",
                    help="strip all <collision> (stage-1 bring-up, empty geom model)")
    ap.add_argument("--strip-visual", action="store_true",
                    help="also strip <visual> to shrink the file")
    args = ap.parse_args(argv)

    tree = ET.parse(args.input)
    root = tree.getroot()

    frozen, kept = [], []
    for joint in root.findall("joint"):
        name = joint.get("name") or ""
        jtype = joint.get("type")
        if jtype not in _MOVABLE_TYPES:
            continue
        is_finger = "finger" in name
        # Arm chain joint = openarmx_(left|right)_joint{1..7}
        is_arm_joint = ("_left_joint" in name or "_right_joint" in name) and not is_finger
        if is_arm_joint:
            kept.append(name)
        else:
            # Fingers + any other movable joint -> freeze defensively.
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

    print(f"[gen_bimanual_urdf] kept {len(kept)} arm DOF (left + right):")
    for n in kept:
        print(f"    keep   {n}")
    print(f"[gen_bimanual_urdf] frozen {len(frozen)} joints (fingers + other movable)")
    for n in frozen:
        print(f"    freeze {n}")
    print(f"[gen_bimanual_urdf] collision = {'STRIPPED' if args.no_collision else 'kept'}"
          f"  visual = {'STRIPPED' if args.strip_visual else 'kept'}")
    print(f"[gen_bimanual_urdf] wrote {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
