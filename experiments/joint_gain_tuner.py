#!/usr/bin/env python3
"""Per-axis (per-joint) step-response tuner for the OpenArmX MIT-mode position loop.

The position-tracking loop lives entirely in the Robstride motor (MIT/impedance
mode). The only tunable knobs are the per-joint KP (stiffness) and KD (damping)
exposed as ROS2 params `kp_jointN` / `kd_jointN` on the hardware_params node. The
JointTrajectoryController is a pure position pass-through, so there is no ros2_control
PID to tune.

This tool, for ONE joint (axis) at a time:
  1. reads current KP/KD from the hardware_params node,
  2. optionally sets a new KP/KD (or sweeps a list of them),
  3. commands a small step on that joint (all other joints held at their current pose),
  4. logs the response (target vs actual from /joint_states),
  5. computes steady-state error, overshoot, settling time, rise time,
  6. prints a table and recommends the gain with the smallest steady-state error that
     keeps overshoot under a threshold.

Because there is NO integral term (MIT mode is PD-on-error), a residual steady-state
error of tau_load/kp is expected and is exactly what KP tuning minimises here.

SAFETY: start with a small --delta, a slow --move-time, in a safe pose, e-stop ready.
Robot must be launched first (controller_manager + hardware active).

Examples
--------
  # single measurement at the current gains, right arm joint 2, +0.1 rad step
  ros2 run ... NO -- this is a plain script:
  python3 joint_gain_tuner.py --arm right --joint 2 --delta 0.1

  # set KP=200 KD=6 first, then measure
  python3 joint_gain_tuner.py --arm right --joint 2 --set-kp 200 --set-kd 6

  # sweep KP over a list (KD fixed at current or --set-kd), recommend best
  python3 joint_gain_tuner.py --arm right --joint 2 --sweep-kp 100,150,200,300 --set-kd 6

  # single-arm (no left_/right_ prefix)
  python3 joint_gain_tuner.py --arm none --joint 4 --delta 0.08
"""
from __future__ import annotations

import argparse
import csv
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def arm_defaults(arm: str):
    """Return (joint_prefix, controller_name, params_node) for an --arm choice."""
    if arm in ("right", "r"):
        return "right_", "/right_joint_trajectory_controller", "/openarmx_right_hardware_params"
    if arm in ("left", "l"):
        return "left_", "/left_joint_trajectory_controller", "/openarmx_left_hardware_params"
    # single arm: no prefix
    return "", "/joint_trajectory_controller", "/openarmx_hardware_params"


class GainTuner(Node):
    def __init__(self, prefix: str, controller: str, params_node: str):
        super().__init__("joint_gain_tuner")
        self.prefix = prefix
        self.controller = controller.rstrip("/")
        self.params_node = params_node.rstrip("/")
        self.arm_joints = [f"openarmx_{prefix}joint{i}" for i in range(1, 8)]

        # latest /joint_states position by joint name
        self._pos = {}
        # best-effort sensor QoS matches joint_state_broadcaster
        js_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                            history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(JointState, "/joint_states", self._on_js, js_qos)

        self.cmd_pub = self.create_publisher(
            JointTrajectory, f"{self.controller}/joint_trajectory", 10)

        self.get_cli = self.create_client(GetParameters, f"{self.params_node}/get_parameters")
        self.set_cli = self.create_client(SetParameters, f"{self.params_node}/set_parameters")

    # ---- callbacks -------------------------------------------------------
    def _on_js(self, msg: JointState):
        for name, p in zip(msg.name, msg.position):
            self._pos[name] = p

    # ---- helpers ---------------------------------------------------------
    def wait_for_states(self, timeout=10.0):
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if all(j in self._pos for j in self.arm_joints):
                return True
        return False

    def current_pose(self):
        return [self._pos[j] for j in self.arm_joints]

    def get_gains(self, joint_idx: int):
        """Read kp_joint{idx}, kd_joint{idx} (1-based) from the params node."""
        if not self.get_cli.wait_for_service(timeout_sec=3.0):
            return None, None
        req = GetParameters.Request()
        req.names = [f"kp_joint{joint_idx}", f"kd_joint{joint_idx}"]
        fut = self.get_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        if res is None or len(res.values) < 2:
            return None, None
        return res.values[0].double_value, res.values[1].double_value

    def set_gains(self, joint_idx: int, kp=None, kd=None):
        if kp is None and kd is None:
            return True
        if not self.set_cli.wait_for_service(timeout_sec=3.0):
            self.get_logger().error("set_parameters service unavailable")
            return False
        req = SetParameters.Request()
        for name, val in (("kp", kp), ("kd", kd)):
            if val is None:
                continue
            p = Parameter()
            p.name = f"{name}_joint{joint_idx}"
            p.value = ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                     double_value=float(val))
            req.parameters.append(p)
        fut = self.set_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        res = fut.result()
        ok = res is not None and all(r.successful for r in res.results)
        if not ok:
            self.get_logger().error("failed to set gains")
        return ok

    def send_pose(self, positions, move_time: float):
        """Command all 7 arm joints to `positions` over move_time seconds."""
        traj = JointTrajectory()
        traj.joint_names = list(self.arm_joints)
        pt = JointTrajectoryPoint()
        pt.positions = [float(p) for p in positions]
        sec = int(move_time)
        pt.time_from_start = Duration(sec=sec, nanosec=int((move_time - sec) * 1e9))
        traj.points.append(pt)
        self.cmd_pub.publish(traj)

    def record_step(self, joint_idx: int, target: float, duration: float):
        """Publish a step on one joint, log (t_rel, actual) of that joint for `duration`s."""
        jname = self.arm_joints[joint_idx - 1]
        base = self.current_pose()
        start = base[joint_idx - 1]
        base[joint_idx - 1] = target

        samples = []  # (t_rel, actual)
        self.send_pose(base, move_time=max(0.2, self._move_time))
        t0 = time.monotonic()
        while rclpy.ok() and time.monotonic() - t0 < duration:
            rclpy.spin_once(self, timeout_sec=0.02)
            samples.append((time.monotonic() - t0, self._pos.get(jname, float("nan"))))
        return start, samples


def metrics(start, target, samples, settle_tol, settle_window):
    """Compute step-response metrics from (t_rel, actual) samples."""
    step = target - start
    ts = [t for t, _ in samples]
    ys = [y for _, y in samples if not math.isnan(y)]
    if not ys or abs(step) < 1e-9:
        return None
    tend = ts[-1]
    final_pts = [y for t, y in samples if t >= tend - settle_window and not math.isnan(y)]
    final = sum(final_pts) / len(final_pts) if final_pts else ys[-1]
    sse = target - final  # signed steady-state error
    sign = 1.0 if step > 0 else -1.0
    peak_beyond = max(((y - target) * sign for _, y in samples if not math.isnan(y)), default=0.0)
    overshoot = max(0.0, peak_beyond) / abs(step) * 100.0
    # settling time: last instant outside the tolerance band around target
    settle_t = 0.0
    for t, y in samples:
        if not math.isnan(y) and abs(y - target) > settle_tol:
            settle_t = t
    # rise time 10% -> 90% of the step
    lo, hi = start + 0.1 * step, start + 0.9 * step

    def cross(level):
        for t, y in samples:
            if math.isnan(y):
                continue
            if (step > 0 and y >= level) or (step < 0 and y <= level):
                return t
        return None
    t_lo, t_hi = cross(lo), cross(hi)
    rise = (t_hi - t_lo) if (t_lo is not None and t_hi is not None and t_hi >= t_lo) else float("nan")
    return dict(step=step, final=final, sse=sse, sse_abs=abs(sse),
                overshoot=overshoot, settling=settle_t, rise=rise)


def fmt_row(label, m):
    return (f"  {label:<16} sse {m['sse']*1000:+7.2f} mrad ({math.degrees(m['sse']):+6.3f}deg)  "
            f"overshoot {m['overshoot']:5.1f}%  settle {m['settling']:5.2f}s  rise {m['rise']:5.2f}s")


def ascii_plot(start, target, samples, width=64, height=14):
    """No-dependency terminal plot of a step response (matplotlib is broken under
    numpy 2.x on this PC). '.' = target setpoint, '*' = actual position."""
    pts = [(t, y) for t, y in samples if not math.isnan(y)]
    if not pts:
        return "  (no samples to plot)"
    t0, t1 = pts[0][0], pts[-1][0]
    span = (t1 - t0) or 1.0
    ys = [y for _, y in pts] + [start, target]
    ymin, ymax = min(ys), max(ys)
    if ymax - ymin < 1e-9:
        ymax = ymin + 1e-6

    def row_of(y):
        r = int(round((ymax - y) / (ymax - ymin) * (height - 1)))
        return min(height - 1, max(0, r))

    grid = [[" "] * width for _ in range(height)]
    for c in range(width):  # target line first (so actual overwrites it)
        grid[row_of(target)][c] = "."
    for c in range(width):
        tt = t0 + span * c / max(1, width - 1)
        nearest = min(pts, key=lambda p: abs(p[0] - tt))
        grid[row_of(nearest[1])][c] = "*"

    lines = []
    for i, gr in enumerate(grid):
        yv = ymax - (ymax - ymin) * i / (height - 1)
        lines.append(f"  {yv:+8.4f} |" + "".join(gr))
    lines.append("  " + " " * 9 + "+" + "-" * width)
    lines.append(f"  {'':11}t=0{' ' * (width - 7)}t={t1:.1f}s   (. target, * actual)")
    return "\n".join(lines)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", default="right", choices=["right", "left", "none", "r", "l"],
                    help="arm side (none = single-arm, no prefix)")
    ap.add_argument("--joint", type=int, required=True, choices=range(1, 8),
                    help="arm joint index 1..7 (gripper J8 excluded for safety)")
    ap.add_argument("--delta", type=float, default=0.10, help="step size [rad] (default 0.10)")
    ap.add_argument("--move-time", type=float, default=1.0,
                    help="commanded ramp time to the step target [s]")
    ap.add_argument("--duration", type=float, default=4.0,
                    help="record window after the command [s]")
    ap.add_argument("--settle-tol", type=float, default=0.01,
                    help="settling-time tolerance band around target [rad]")
    ap.add_argument("--settle-window", type=float, default=1.0,
                    help="averaging window at the tail for steady-state error [s]")
    ap.add_argument("--set-kp", type=float, default=None, help="set this KP before measuring")
    ap.add_argument("--set-kd", type=float, default=None, help="set this KD before measuring")
    ap.add_argument("--sweep-kp", default=None,
                    help="comma list of KP values to sweep, e.g. 100,150,200,300")
    ap.add_argument("--sweep-kd", default=None,
                    help="comma list of KD values to sweep (mutually exclusive with --sweep-kp)")
    ap.add_argument("--max-overshoot", type=float, default=10.0,
                    help="overshoot%% ceiling for the recommendation (default 10)")
    ap.add_argument("--plot", choices=["ascii", "none"], default="ascii",
                    help="terminal response plot (matplotlib is unavailable here under numpy 2.x)")
    ap.add_argument("--return-home", action="store_true",
                    help="command the joint back to its start pose after each trial")
    ap.add_argument("--controller", default=None, help="override controller name")
    ap.add_argument("--params-node", default=None, help="override hardware_params node name")
    ap.add_argument("--csv", default=None, help="write per-trial metrics to this CSV file")
    args = ap.parse_args()

    if args.sweep_kp and args.sweep_kd:
        ap.error("use only one of --sweep-kp / --sweep-kd per run")

    prefix, controller, params_node = arm_defaults(args.arm)
    controller = args.controller or controller
    params_node = args.params_node or params_node

    rclpy.init()
    node = GainTuner(prefix, controller, params_node)
    try:
        if not node.wait_for_states():
            node.get_logger().error(
                "no /joint_states for the arm joints - is the robot launched? "
                f"expected joints: {node.arm_joints}")
            return
        cur_kp, cur_kd = node.get_gains(args.joint)
        jname = node.arm_joints[args.joint - 1]
        start_pos = node.current_pose()[args.joint - 1]
        node.get_logger().info(
            f"[{jname}] current KP={cur_kp} KD={cur_kd}  pos={start_pos:+.4f} rad  "
            f"step={args.delta:+.3f} rad")

        node._move_time = args.move_time

        # build the trial list
        if args.sweep_kp:
            trials = [("kp", float(v)) for v in args.sweep_kp.split(",")]
        elif args.sweep_kd:
            trials = [("kd", float(v)) for v in args.sweep_kd.split(",")]
        else:
            trials = [(None, None)]  # single measurement at (optionally set) gains

        # apply any one-shot set before a non-sweep run
        if not (args.sweep_kp or args.sweep_kd):
            node.set_gains(args.joint, kp=args.set_kp, kd=args.set_kd)

        rows = []
        traces = []  # (label, start, target, samples) for plotting
        for kind, val in trials:
            if kind == "kp":
                node.set_gains(args.joint, kp=val, kd=args.set_kd)
            elif kind == "kd":
                node.set_gains(args.joint, kp=args.set_kp, kd=val)
            kp_used, kd_used = node.get_gains(args.joint)
            time.sleep(0.3)

            start = node.current_pose()[args.joint - 1]
            target = start + args.delta
            s0, samples = node.record_step(args.joint, target, args.duration)
            m = metrics(s0, target, samples, args.settle_tol, args.settle_window)
            if m is None:
                node.get_logger().warn("not enough samples for metrics, skipping trial")
                continue
            m["kp"], m["kd"] = kp_used, kd_used
            rows.append(m)
            label = f"KP={kp_used:g} KD={kd_used:g}"
            traces.append((label, s0, target, samples))
            print(fmt_row(label, m))
            if args.plot == "ascii" and len(trials) == 1:
                print(ascii_plot(s0, target, samples))

            if args.return_home or (kind is not None):
                pose = node.current_pose()
                pose[args.joint - 1] = s0
                node.send_pose(pose, move_time=max(0.5, args.move_time))
                t0 = time.monotonic()
                while rclpy.ok() and time.monotonic() - t0 < max(0.8, args.move_time + 0.5):
                    rclpy.spin_once(node, timeout_sec=0.02)

        # ranked table + recommendation across a sweep
        if len(rows) > 1:
            print("\nRANKED (by |steady-state error|):")
            for r in sorted(rows, key=lambda r: r["sse_abs"]):
                print(fmt_row(f"KP={r['kp']:g} KD={r['kd']:g}", r))
            ok = [r for r in rows if r["overshoot"] <= args.max_overshoot]
            pick = min(ok or rows, key=lambda r: r["sse_abs"])
            print("\nRECOMMENDATION (smallest |steady-state error| within overshoot ceiling "
                  f"{args.max_overshoot:g}%):")
            print(f"  -> KP={pick['kp']:g} KD={pick['kd']:g}  "
                  f"sse={pick['sse']*1000:+.2f} mrad  overshoot={pick['overshoot']:.1f}%")
            if not ok:
                print("  (no trial met the overshoot ceiling; raise KD or lower the ceiling)")
            if args.plot == "ascii":
                for label, s0, tgt, samples in traces:
                    print(f"\n[{label}]")
                    print(ascii_plot(s0, tgt, samples))

        if args.csv and rows:
            with open(args.csv, "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=["kp", "kd", "step", "final", "sse",
                                                  "sse_abs", "overshoot", "settling", "rise"])
                w.writeheader()
                for r in rows:
                    w.writerow(r)
            print(f"\nwrote {len(rows)} rows -> {args.csv}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
