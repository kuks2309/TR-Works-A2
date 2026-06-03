"""Linear-path reachability checker (Pinocchio numerical IK).

Used by the Cartesian jog UI to verify that a commanded Cartesian linear motion
is feasible BEFORE sending it to cyclo. cyclo's QP uses slack and will silently
fall back to the closest reachable direction without emitting controller_error,
so unreachable jogs would otherwise drift sideways instead of failing loudly.

Algorithm: interpolate start_pose -> goal_pose into N waypoints (in SE3), then
solve damped-least-squares Newton-Raphson IK at each waypoint, warm-started
from the previous step. If any waypoint fails to converge or violates joint
limits, the linear motion is reported unreachable.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import pinocchio as pin


@dataclass
class LinearIKResult:
    """Result of a linear-path reachability check."""
    reachable: bool
    reason: str
    waypoint_index: int
    n_waypoints: int


def _dict_to_se3(pose: dict) -> pin.SE3:
    """Accept {x,y,z,qx,qy,qz,qw} OR {x,y,z,roll,pitch,yaw} (Z-Y-X intrinsic)."""
    if "qw" in pose:
        q = pin.Quaternion(pose["qw"], pose["qx"], pose["qy"], pose["qz"])
    else:
        # ZYX intrinsic Euler (yaw, pitch, roll) — matches _rpy_to_quat in
        # scenario_action_client.py.
        cr, sr = np.cos(pose["roll"] * 0.5),  np.sin(pose["roll"] * 0.5)
        cp, sp = np.cos(pose["pitch"] * 0.5), np.sin(pose["pitch"] * 0.5)
        cy, sy = np.cos(pose["yaw"] * 0.5),   np.sin(pose["yaw"] * 0.5)
        qw = cr*cp*cy + sr*sp*sy
        qx = sr*cp*cy - cr*sp*sy
        qy = cr*sp*cy + sr*cp*sy
        qz = cr*cp*sy - sr*sp*cy
        q = pin.Quaternion(qw, qx, qy, qz)
    q.normalize()
    t = np.array([pose["x"], pose["y"], pose["z"]])
    # NOTE: on this machine pinocchio 3.9.0 <-> numpy have an ABI mismatch, so
    # EVERY numpy<->Eigen conversion (SE3(R,t), SE3(q,t), even .translation
    # setters) raises Boost.Python.ArgumentError. The whole IK pre-check is
    # therefore non-functional here; _on_jog_press wraps the call in try/except
    # so the cyclo jog still publishes. Rebuild pinocchio against this numpy to
    # restore the pre-check.
    return pin.SE3(q.toRotationMatrix(), t)


def _interp_se3(a: pin.SE3, b: pin.SE3, t: float) -> pin.SE3:
    """SE3 geodesic interpolation: a * exp(t * log(a^-1 * b))."""
    rel = a.actInv(b)
    twist = pin.log6(rel).vector * float(t)
    return a * pin.exp6(twist)


class LinearReachabilityChecker:
    """Per-arm reachability checker using Pinocchio + damped LS IK."""

    def __init__(
        self,
        urdf_path: str,
        tcp_frame_name: str,
        max_iter: int = 80,
        tol: float = 5e-4,
        damping: float = 1e-3,
        step_gain: float = 0.5,
    ):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        if not self.model.existFrame(tcp_frame_name):
            raise ValueError(
                f"TCP frame '{tcp_frame_name}' not in URDF {urdf_path}"
            )
        self.tcp_id = self.model.getFrameId(tcp_frame_name)
        self.q_lo = np.array(self.model.lowerPositionLimit)
        self.q_hi = np.array(self.model.upperPositionLimit)
        self.nq = self.model.nq
        self.max_iter = max_iter
        self.tol = tol
        self.damping = damping
        self.step_gain = step_gain

    def numerical_ik(
        self,
        target_se3: pin.SE3,
        q_seed: np.ndarray,
    ) -> Tuple[np.ndarray, bool, str]:
        """Damped LS Newton-Raphson IK in the LOCAL frame.

        Returns (q, ok, reason). reason in {"ok","joint_limit","no_convergence"}.
        """
        q = np.array(q_seed, dtype=float).copy()
        if q.shape[0] != self.nq:
            return q, False, f"q_seed dim {q.shape[0]} != model.nq {self.nq}"
        for _ in range(self.max_iter):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.tcp_id)
            current = self.data.oMf[self.tcp_id]
            err = pin.log6(current.actInv(target_se3)).vector
            if np.linalg.norm(err) < self.tol:
                if np.any(q < self.q_lo - 1e-6) or np.any(q > self.q_hi + 1e-6):
                    return q, False, "joint_limit"
                return q, True, "ok"
            J = pin.computeFrameJacobian(
                self.model, self.data, q, self.tcp_id, pin.LOCAL
            )
            H = J.T @ J + self.damping * np.eye(self.model.nv)
            dq = np.linalg.solve(H, J.T @ err)
            q = pin.integrate(self.model, q, dq * self.step_gain)
            q = np.clip(q, self.q_lo, self.q_hi)
        return q, False, "no_convergence"

    def check_linear_path(
        self,
        start_pose: dict,
        goal_pose: dict,
        q_seed: np.ndarray,
        n_steps: int = 10,
    ) -> LinearIKResult:
        """Verify each waypoint along the linear path is IK-solvable."""
        start_se3 = _dict_to_se3(start_pose)
        goal_se3 = _dict_to_se3(goal_pose)
        q = np.array(q_seed, dtype=float).copy()
        for i in range(n_steps + 1):
            t = i / n_steps if n_steps > 0 else 1.0
            wp = _interp_se3(start_se3, goal_se3, t)
            q, ok, reason = self.numerical_ik(wp, q)
            if not ok:
                return LinearIKResult(
                    reachable=False,
                    reason=reason,
                    waypoint_index=i,
                    n_waypoints=n_steps,
                )
        return LinearIKResult(
            reachable=True, reason="ok",
            waypoint_index=n_steps, n_waypoints=n_steps,
        )
