#!/usr/bin/env python3
"""왼팔 pick 시퀀스 — ptp_box_align 액션 서버 없이 직접 IK + JTC 토픽 발행.

시퀀스(사용자 지시):
  1. 박스 검출 (이미 검출된 /detected_boxes 자세 사용, YOLO 재검출 안 함)
  2. 검출 위치이동: 왼팔 TCP -> (box.x, box.y, 0.8) 접근
  3. 그리퍼 개방 (0.044)
  4. 높이만 z=0.75
  5. 그리퍼 80% (0.0088)
  6. 다시 z=0.8 (그리퍼 80% 유지 — 건드리지 않음)

IK는 ptp_box_align_node.cpp 와 동일한 방식:
  /robot_description 풀모델 -> 왼팔 7-DOF 축소, 제어 프레임 openarmx_left_hand_tcp,
  damped-least-squares(CLIK) + 매 스텝 limit clamp + neutral 시드 후 랜덤 재시작.
"""
import sys
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import String
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand

SIDE = "left"
ARM_PREFIX = f"openarmx_{SIDE}_joint"
TCP_FRAME = f"openarmx_{SIDE}_hand_tcp"
FALLBACK_BOX = (0.160187, 0.0650303, 0.786038)

# IK 파라미터 (ptp_box_align_node.cpp 기본값과 동일)
IK_EPS, IK_MAX_ITER, IK_DT, IK_DAMP, IK_RESTARTS = 1e-4, 1000, 0.1, 1e-6, 20
MOVE_TIME = 6.0           # JTC 이동 시간 [s]
SETTLE = 0.7              # 이동 후 정착 여유 [s]

# 수직 하향 자세 R180 P0 Y0 = Rx(pi) = diag(1,-1,-1)
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


class PickSeq(Node):
    def __init__(self):
        super().__init__("ptp_pick_sequence_left")
        latched = QoSProfile(depth=1,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.urdf = None
        self.box = None
        self.create_subscription(String, "/robot_description", self._urdf_cb, latched)
        self.create_subscription(PoseArray, "/detected_boxes", self._box_cb, latched)
        self.traj_pub = self.create_publisher(
            JointTrajectory, f"/{SIDE}_joint_trajectory_controller/joint_trajectory", 10)
        self.grip = ActionClient(self, GripperCommand,
                                 f"/{SIDE}_gripper_controller/gripper_cmd")
        self.model = self.data = None
        self.fid = None
        self.qidx = []
        self.jnames = []

    # -------- callbacks
    def _urdf_cb(self, msg):
        if self.urdf is None:
            self.urdf = msg.data

    def _box_cb(self, msg):
        if msg.poses:
            p = msg.poses[0].position
            self.box = (p.x, p.y, p.z)

    # -------- helpers
    def wait(self, sec):
        end = self.get_clock().now().nanoseconds + int(sec * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def build_model(self):
        full = pin.buildModelFromXML(self.urdf)
        ref = pin.neutral(full)
        lock = [j for j in range(1, len(full.joints))
                if full.joints[j].nq >= 1 and not full.names[j].startswith(ARM_PREFIX)]
        self.model = pin.buildReducedModel(full, lock, ref)
        self.data = self.model.createData()
        if not self.model.existFrame(TCP_FRAME):
            raise RuntimeError(f"frame {TCP_FRAME} not in reduced model")
        self.fid = self.model.getFrameId(TCP_FRAME)
        for j in range(1, len(self.model.joints)):
            if self.model.joints[j].nq == 1:
                self.jnames.append(self.model.names[j])
                self.qidx.append(self.model.joints[j].idx_q)
        print(f"[model] reduced nq={self.model.nq}, joints={self.jnames}")

    def _clamp(self, q):
        for qi in self.qidx:
            q[qi] = min(max(q[qi], self.model.lowerPositionLimit[qi]),
                        self.model.upperPositionLimit[qi])

    def _ik_descent(self, oMdes, q):
        err = np.zeros(6)
        for _ in range(IK_MAX_ITER):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.fid)
            iMd = self.data.oMf[self.fid].actInv(oMdes)
            err = pin.log6(iMd).vector
            res = float(np.linalg.norm(err))
            if res < IK_EPS:
                return q, res, True
            J = pin.computeFrameJacobian(self.model, self.data, q, self.fid)
            J = -pin.Jlog6(iMd.inverse()) @ J
            JJt = J @ J.T
            JJt[np.diag_indices_from(JJt)] += IK_DAMP
            v = -J.T @ np.linalg.solve(JJt, err)
            q = pin.integrate(self.model, q, v * IK_DT)
            self._clamp(q)
        return q, float(np.linalg.norm(err)), False

    def solve_ik(self, x, y, z):
        oMdes = pin.SE3(R_DOWN, np.array([x, y, z]))
        best_q, best_res = pin.neutral(self.model), 1e18
        for attempt in range(IK_RESTARTS + 1):
            qa = pin.neutral(self.model) if attempt == 0 \
                else pin.randomConfiguration(self.model)
            q, res, conv = self._ik_descent(oMdes, qa)
            if res < best_res:
                best_res, best_q = res, q
            if conv:
                best_q, best_res = q, res
                break
        pin.forwardKinematics(self.model, self.data, best_q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        tcp = self.data.oMf[self.fid].translation
        err_mm = float(np.linalg.norm(tcp - np.array([x, y, z])) * 1000.0)
        return best_q, best_res, err_mm, tcp

    # -------- position-only IK (orientation 자유; 자세 제어 조건 해제)
    def _ik_descent_pos(self, p_des, q):
        err = np.zeros(3)
        for _ in range(IK_MAX_ITER):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.fid)
            err = p_des - self.data.oMf[self.fid].translation
            if np.linalg.norm(err) < IK_EPS:
                return q, float(np.linalg.norm(err)), True
            J6 = pin.computeFrameJacobian(self.model, self.data, q, self.fid,
                                          pin.LOCAL_WORLD_ALIGNED)
            Jp = J6[:3, :]
            JJt = Jp @ Jp.T
            JJt[np.diag_indices_from(JJt)] += IK_DAMP
            v = Jp.T @ np.linalg.solve(JJt, err)
            q = pin.integrate(self.model, q, v * IK_DT)
            self._clamp(q)
        return q, float(np.linalg.norm(err)), False

    def solve_ik_pos(self, x, y, z, seed=None):
        p = np.array([x, y, z])
        best_q = pin.neutral(self.model) if seed is None else seed
        best_res = 1e18
        for attempt in range(IK_RESTARTS + 1):
            if attempt == 0:
                qa = best_q.copy()
            else:
                qa = pin.randomConfiguration(self.model)
            q, res, conv = self._ik_descent_pos(p, qa)
            if res < best_res:
                best_res, best_q = res, q
            if conv:
                best_q, best_res = q, res
                break
        pin.forwardKinematics(self.model, self.data, best_q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        tcp = self.data.oMf[self.fid].translation.copy()
        return best_q, best_res, float(np.linalg.norm(tcp - p) * 1000.0), tcp

    def max_reach_z(self, x, y, tol_mm=5.0, zhi=1.05, zlo=0.35, steps=36):
        """위에서부터 스캔해 도달 가능한 최대 Z를 찾는다(자세 자유)."""
        best = None
        for k in range(steps + 1):
            z = zhi - (zhi - zlo) * k / steps
            _, _, err_mm, _ = self.solve_ik_pos(x, y, z)
            if err_mm <= tol_mm:
                best = z
                break
        return best

    def send_traj(self, q, label):
        # 컨트롤러가 토픽을 구독할 때까지 대기
        for _ in range(50):
            if self.traj_pub.get_subscription_count() > 0:
                break
            self.wait(0.1)
        traj = JointTrajectory()
        traj.joint_names = self.jnames
        pt = JointTrajectoryPoint()
        pt.positions = [float(q[qi]) for qi in self.qidx]
        pt.time_from_start = Duration(sec=int(MOVE_TIME),
                                      nanosec=int((MOVE_TIME % 1) * 1e9))
        traj.points = [pt]
        self.traj_pub.publish(traj)
        deg = [f"{np.degrees(q[qi]):+.1f}" for qi in self.qidx]
        print(f"[move] {label}: q(deg)={deg}  (subs={self.traj_pub.get_subscription_count()})")
        self.wait(MOVE_TIME + SETTLE)

    def gripper(self, position, effort, label):
        if not self.grip.wait_for_server(timeout_sec=5.0):
            print(f"[grip] {label}: server unavailable!")
            return
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = float(effort)
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print(f"[grip] {label}: goal rejected")
            return
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=10.0)
        r = rfut.result().result
        print(f"[grip] {label}: pos={r.position:.4f} reached={r.reached_goal} "
              f"stalled={r.stalled}")
        self.wait(0.5)


def main():
    rclpy.init()
    node = PickSeq()
    print("=== 왼팔 pick 시퀀스 (action server 없이) ===")

    # /robot_description 수신 대기
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.urdf:
            break
    if not node.urdf:
        print("ERROR: /robot_description 미수신")
        return 1
    node.build_model()

    # 1) 박스 검출 — 이미 검출된 /detected_boxes 자세 사용
    node.wait(2.0)
    if node.box is not None:
        bx, by, bz = node.box
        print(f"[detect] /detected_boxes 사용: ({bx:.4f}, {by:.4f}, {bz:.4f})")
    else:
        bx, by, bz = FALLBACK_BOX
        print(f"[detect] 폴백(이미 검출된 알려진 자세): ({bx:.4f}, {by:.4f}, {bz:.4f})")

    # --reach: 자세 자유(position-only) 도달성/최대Z 점검 (로봇 미동작)
    if "--reach" in sys.argv:
        print("--- REACH CHECK: position-only IK, 자세 제어 해제 ---")
        # 6D(자세 구속) 비교
        _, r6, e6, t6 = node.solve_ik(bx, by, 0.75)
        print(f"[6D 구속] z=0.75: err={e6:.1f}mm (자세 수직하향)")
        # position-only
        for z in (0.80, 0.75):
            _, r, e, t = node.solve_ik_pos(bx, by, z)
            print(f"[pos-only] z={z}: err={e:.2f}mm tcp=({t[0]:.3f},{t[1]:.3f},{t[2]:.3f}) "
                  f"{'OK' if e < 5 else 'UNREACH'}")
        zmax = node.max_reach_z(bx, by)
        print(f"[pos-only] 최대 도달 Z @ (x={bx:.3f},y={by:.3f}): "
              f"{zmax if zmax is not None else '없음(도달불가)'}")
        node.destroy_node(); rclpy.shutdown(); return 0

    dry = "--dry" in sys.argv

    def step_move(z, label):
        q, res, err_mm, tcp = node.solve_ik(bx, by, z)
        print(f"[ik] {label} z={z}: residual={res:.2e} err={err_mm:.3f}mm "
              f"tcp=({tcp[0]:.3f},{tcp[1]:.3f},{tcp[2]:.3f})")
        if not dry:
            node.send_traj(q, label)

    if dry:
        print("--- DRY RUN: IK 계산만, 로봇 미동작 ---")
        for z in (0.80, 0.75, 0.80):
            step_move(z, f"dry z={z}")
        print("=== DRY RUN 완료 ===")
        node.destroy_node()
        rclpy.shutdown()
        return 0

    # 2) 검출 위치이동: (bx,by,0.8)
    step_move(0.80, "approach z=0.8")
    # 3) 그리퍼 개방
    node.gripper(0.044, 20.0, "open")
    # 4) 높이만 z=0.75
    step_move(0.75, "descend z=0.75")
    # 5) 그리퍼 80% (0.0088)
    node.gripper(0.0088, 50.0, "close 80%")
    # 6) 다시 z=0.8 (그리퍼 미관여 = 80% 유지)
    step_move(0.80, "retreat z=0.8")

    print("=== 시퀀스 완료 (그리퍼 80% 유지) ===")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
