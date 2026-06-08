#!/usr/bin/env python3
"""왼팔 pick 시퀀스 v2 — 자세 제어 해제(position-only IK) + 접근/상승은 forward position
컨트롤러(모터각도 직접, JTC 미사용, rate-limit), 하강(0.75)만 JTC.

사용자 지시 시퀀스:
  1. 박스 검출 (카메라->라즈베리파이; /detected_boxes 사용)
  2. 접근 이동: 왼팔 TCP -> (box.x, box.y, 0.80), 자세 자유  [JTC 없이: forward position]
  4. 하강: z=0.75                                          [JTC 사용]
  5. 박스 잡기 80% close
  6. 다시 위로 z=0.80                                       [JTC 없이: forward position]

forward position = /left_arm_position_controller/commands (7관절, 그리퍼 제외).
JTC <-> arm_position_controller 는 배타적이라 switch_controller 로 교체.
안전: 스위치 후 '현재 위치'부터 시작해 목표까지 rate-limit 램프(급동작 방지).

기본 실행은 DRY(계획만). 실제 동작은 '--run'.
"""
import sys
import os
import math
import time as _time
import yaml
import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,
                       QoSHistoryPolicy)
from std_msgs.msg import String, Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import SwitchController

# --side=right 로 오른팔 구동 (기본 left). 좌/우 별도 실행.
SIDE = "right" if "--side=right" in sys.argv else "left"
ARM_PREFIX = f"openarmx_{SIDE}_joint"
TCP_FRAME = f"openarmx_{SIDE}_hand_tcp"
JTC = f"{SIDE}_joint_trajectory_controller"
FPOS = f"{SIDE}_arm_position_controller"
# INIT 자세 (deg): 2026-06-07 오른팔 핸드 가이드 자세를 등록(테이블 충돌 회피용 상향).
# 좌우 미러 규칙(실증, FK perr=0mm): j4(팔꿈치)만 동일, 나머지(j1·j2·j3·j5·j6·j7) 전부 부호반전.
#   s = (-1,-1,-1,+1,-1,-1,-1).  옛 INIT([-50,0,0,100,0,0,-50]↔[50,0,0,100,0,0,50])과도 일치.
#   우(등록): [-42.8,  46.3, -14.3, 106.5,  41.7, -23.4, -42.1]
#   좌(미러): [ 42.8, -46.3,  14.3, 106.5, -41.7,  23.4,  42.1]
INIT_DEG = ([42.8, -46.3, 14.3, 106.5, -41.7, 23.4, 42.1] if SIDE == "left"
            else [-42.8, 46.3, -14.3, 106.5, 41.7, -23.4, -42.1])

# 박스 접근(진입) 지점 (deg): 2026-06-07 핸드 가이드로 캡처한 entry 자세
# (right_grasp_reference_dataset.yaml, pose_type=entry). 파지 성공 시 이 지점으로 가서 오픈(놓기).
# 좌우 미러: j4 만 동일, 나머지 부호반전 (s=-1,-1,-1,+1,-1,-1,-1).
#   우(기록): [ 2.27,  17.79, -36.32, 110.37,  8.36, -41.27, -14.14]
#   좌(미러): [-2.27, -17.79,  36.32, 110.37, -8.36,  41.27,  14.14]
APPROACH_POINT_DEG = ([-2.27, -17.79, 36.32, 110.37, -8.36, 41.27, 14.14] if SIDE == "left"
                      else [2.27, 17.79, -36.32, 110.37, 8.36, -41.27, -14.14])

# 박스 드랍(놓기) 위치 (deg): 2026-06-07 핸드 가이드로 캡처한 drop 자세
# (right_grasp_reference_dataset.yaml, pose_type=drop). 박스 접근 지점 경유 후 여기서 오픈(드롭).
# 좌우 미러: j4 만 동일, 나머지 부호반전.
#   우(기록): [ 43.05, -1.29, -34.39, 54.43,  23.62, -22.91, -1.40]
#   좌(미러): [-43.05,  1.29,  34.39, 54.43, -23.62,  22.91,  1.40]
DROP_POINT_DEG = ([-43.05, 1.29, 34.39, 54.43, -23.62, 22.91, 1.40] if SIDE == "left"
                  else [43.05, -1.29, -34.39, 54.43, 23.62, -22.91, -1.40])

APPROACH_Z = 0.88          # 접근/상승 높이. 박스 상단(책상0.72+9cm=0.81) 위로 진입해야 박스 안 침.
                           # (0.80은 박스top 아래라 진입 중 그리퍼가 박스를 치고 밀었음 -> 파지실패)
DESCEND_Z = 0.75           # 런타임에 박스 z 추종으로 덮어씀(아래)
# 하강 z 박스 z 추종: descend_z = max(DESCEND_FLOOR, box_z - GRASP_DEPTH).
# 고정 0.75면 z 낮은 박스(예 0.739)에서 그리퍼가 박스 위에서 닫혀 빈손 -> 박스 z 따라 내림.
# DESCEND_FLOOR(0.72)로 테이블 충돌 보호("0.72 이상이면 진행").
# 검출 z 는 박스 상부에 가깝다(9cm 박스: 책상 0.72 + 높이 0.09 -> 윗면 0.81, 검출 z≈0.79).
# descend = box_z - GRASP_DEPTH. 0.03 이면 윗면보다 ~5cm 박혀 9cm 박스를 쳐서 빈손 ->
# 깊이를 줄여 박스 상부를 잡는다(0.01 -> descend≈0.78, 윗면 ~3cm 아래).
GRASP_DEPTH = 0.01         # 검출 z(상부) 아래로 내리는 깊이 (작을수록 위에서 잡음)
DESCEND_FLOOR = 0.72       # 하강 하한 (이 밑으로는 안 내림)
GRIP_80 = 0.0             # 파지 닫기 명령=완전닫힘. 빈손 바닥 실측 좌0.0007/우0.0018(우 오프셋 높음),
                          # 박스 물면 박스 폭에서 멈춤 -> 빈손 바닥과 벌어져 위치로 파지 판별.
GRIP_OPEN = 0.044          # 완전 열림 (이동 중 핸드 열림 유지)
GRIP_EFFORT = 1.0         # 무른 박스 으깸 방지(50->10->5->1). 최소 힘으로 자연폭 파지(검출 쉬움)
                          # 주의: 너무 약하면 운반 중 미끄러질 수 있음
# 파지검증 임계(GRIP_80=0.0 이므로 임계=margin, 절대값). 빈손바닥 좌0.0007/우0.0018 위,
# 얇은 파지(으깬 우박스 0.0040) 아래. 단일 임계로 양팔 성립.
GRIP_DETECT_MARGIN = 0.003   # 절대 임계 폴백(캘리브 실패 시). 평소엔 self.grip_thr 사용
GRIP_CALIB_MARGIN = 0.001    # 기동 캘리브: 임계 = 측정한 빈손바닥 + 이 값(상대 판정). 드리프트 흡수

IK_EPS, IK_MAX_ITER, IK_DT, IK_DAMP, IK_RESTARTS = 1e-4, 1000, 0.1, 1e-6, 20
RAMP_RATE_DPS = 90.0     # forward position 최대 관절속도 [deg/s] (튜닝: --rate)
RAMP_HZ = 50.0
JTC_MOVE_TIME = 0.8      # JTC 하강 시간 [s] (튜닝: --descend)
INIT_TIME = 0.85          # INIT 복귀 시간 [s] (튜닝: --init)
SETTLE = 0.1             # 이동 후 정착 여유 [s] (튜닝: --settle)
GRASP_SETTLE = 0.55       # 파지(비차단) 후 정착 [s] (튜닝: --grasp-settle)
WARMUP_T = 0.1           # JTC 스위치 후 워밍업 궤적 시간 [s]

# 중간자세(테이블 충돌 방지): INIT 자세 유지한 채 X +7cm 앞으로, z>=0.75 로 이동
MID_X_FWD = 0.07         # INIT TCP 에서 X 앞으로 [m]
MID_MIN_Z = 0.75         # 중간자세 최소 높이 [m]
PICK_MAX_X = 0.46        # 이 X[m] 이상은 큰(place) 박스 검출 -> 미니박스 픽 대상에서 제외

# 파지 자세 pitch 상한 [deg]. 비스듬은 유리하나 이 이상 앞으로 숙이면 파지 실패 ->
# |pitch|<=PITCH_MAX 해를 우선 채택(자연 슬랜티드 유지), 과도하면 다른 IK 해 탐색.
# 2026-06-07: 사용자가 핸드 가이드로 시연한 far-right graspable 자세가 tilt~50°(P-49.6) ->
# 45° 캡이 그 좋은 각도를 걸러내 far-right 실패 -> 55° 로 상향(시연 50°+여유 5°).
# 기준 자세: experiments/right_grasp_reference_pose.yaml
PITCH_MAX = 55.0

# 접근 IK 후보: 유효(pitch<=PITCH_MAX, |yaw|<=YAW_MAX) 후보를 N_CAND 개 모을 때까지 랜덤
# 리스타트를 MAX_TRIES 까지 시도(조기종료) -> 그 중 접근 이동량 최소(최적경로) 선택.
# 유효 0개면 뒤틀린 자세를 채택하지 않고 상위에서 안전 중단(빈손/손목뒤틀림 방지).
N_CAND = 3
MAX_TRIES = 24
YAW_MAX = 25.0

# 수직 하향 자세 R180 P0 Y0 = Rx(pi) = diag(1,-1,-1)
R_DOWN = np.array([[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]])


def _argf(flag, default):
    for a in sys.argv:
        if a.startswith(flag + "="):
            try:
                return float(a.split("=", 1)[1])
            except ValueError:
                pass
    return default


CONFIG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           f"pick_seq_config_{SIDE}.yaml")


def load_grasp_offset():
    """pick_seq_config.yaml 의 grasp_offset (x,y,z) [m] 로드. 없으면 0."""
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return 0.0, 0.0, 0.0
    o = cfg.get("grasp_offset") or {}
    return float(o.get("x", 0.0)), float(o.get("y", 0.0)), float(o.get("z", 0.0))


def load_zheights():
    """pick_seq_config.yaml 의 approach_z, descend_z [m]. 없으면 None(기본 유지)."""
    try:
        with open(CONFIG_PATH) as f:
            cfg = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return None, None
    az = cfg.get("approach_z")
    dz = cfg.get("descend_z")
    return (float(az) if az is not None else None,
            float(dz) if dz is not None else None)


class PickV2(Node):
    def __init__(self):
        super().__init__("ptp_pick_seq_v2_left")
        latched = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                             durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self.urdf = None
        self.box = None
        self.boxes = []
        self.state = {}
        self.create_subscription(String, "/robot_description", self._urdf_cb, latched)
        self.create_subscription(PoseArray, "/detected_boxes", self._box_cb, latched)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.fpos_pub = self.create_publisher(Float64MultiArray, f"/{FPOS}/commands", 10)
        self.jtc_pub = self.create_publisher(JointTrajectory, f"/{JTC}/joint_trajectory", 10)
        self.grip = ActionClient(self, GripperCommand, f"/{SIDE}_gripper_controller/gripper_cmd")
        self.switch = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.model = self.data = None
        self.fid = None
        self.qidx = []
        self.jnames = []

    def _urdf_cb(self, m):
        if self.urdf is None:
            self.urdf = m.data

    def _box_cb(self, m):
        if m.poses:
            self.boxes = [(p.position.x, p.position.y, p.position.z) for p in m.poses]
            # 측면 분담(중앙 Y=0 기준): 오른쪽(Y<0)=오른팔, 왼쪽(Y>0)=왼팔.
            # 그 중 X 최소(가까울수록 쉽게 잡음 — far 신전/droop 회피)를 선택.
            # X>=PICK_MAX_X 는 큰(place) 박스 검출 -> 픽 대상 제외. 측면(중앙 Y=0)으로 좌우 분담.
            if SIDE == "right":
                cand = [b for b in self.boxes if b[1] < 0.0 and b[0] < PICK_MAX_X]
            else:
                cand = [b for b in self.boxes if b[1] > 0.0 and b[0] < PICK_MAX_X]
            # 엄격 분리: 자기 측면에 박스 없으면 안 잡고 대기(None). 반대편으로 절대 안 넘어감.
            self.box = min(cand, key=lambda b: b[0]) if cand else None

    def _js_cb(self, m):
        for n, p in zip(m.name, m.position):
            self.state[n] = p

    def wait(self, sec):
        end = self.get_clock().now().nanoseconds + int(sec * 1e9)
        while self.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    # ---------- model + IK
    def build_model(self):
        full = pin.buildModelFromXML(self.urdf)
        lock = [j for j in range(1, len(full.joints))
                if full.joints[j].nq >= 1 and not full.names[j].startswith(ARM_PREFIX)]
        self.model = pin.buildReducedModel(full, lock, pin.neutral(full))
        self.data = self.model.createData()
        self.fid = self.model.getFrameId(TCP_FRAME)
        for j in range(1, len(self.model.joints)):
            if self.model.joints[j].nq == 1:
                self.jnames.append(self.model.names[j])
                self.qidx.append(self.model.joints[j].idx_q)

    def _clamp(self, q):
        for qi in self.qidx:
            q[qi] = min(max(q[qi], self.model.lowerPositionLimit[qi]),
                        self.model.upperPositionLimit[qi])

    def _ik_pos_descent(self, p_des, q):
        err = np.zeros(3)
        for _ in range(IK_MAX_ITER):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.fid)
            err = p_des - self.data.oMf[self.fid].translation
            if np.linalg.norm(err) < IK_EPS:
                return q, float(np.linalg.norm(err)), True
            J = pin.computeFrameJacobian(self.model, self.data, q, self.fid,
                                         pin.LOCAL_WORLD_ALIGNED)[:3, :]
            JJt = J @ J.T
            JJt[np.diag_indices_from(JJt)] += IK_DAMP
            q = pin.integrate(self.model, q, (J.T @ np.linalg.solve(JJt, err)) * IK_DT)
            self._clamp(q)
        return q, float(np.linalg.norm(err)), False

    def solve_pos(self, x, y, z, seed, pitch_max=PITCH_MAX):
        """position-only IK (자세 자유, 비스듬 파지 유리). 단 pitch(앞으로 숙임)가 과도하면
        파지 실패하므로 |pitch|<=pitch_max 해를 우선 채택(없으면 |pitch| 최소). 자연 슬랜티드 유지."""
        p = np.array([x, y, z])
        best = None                          # (abs_pitch, q, res, err, tcp)
        fb_q, fb_res = seed.copy(), 1e18      # 미도달시 최저잔차 폴백
        for attempt in range(IK_RESTARTS + 1):
            qa = seed.copy() if attempt == 0 else pin.randomConfiguration(self.model)
            q, res, _ = self._ik_pos_descent(p, qa)
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.fid)
            M = self.data.oMf[self.fid]
            tcp = M.translation.copy()
            err = float(np.linalg.norm(tcp - p) * 1000.0)
            if res < fb_res:
                fb_res, fb_q = res, q.copy()
            if err <= 5.0:
                apitch = abs(math.degrees(pin.rpy.matrixToRpy(M.rotation)[1]))
                if best is None or apitch < best[0]:
                    best = (apitch, q.copy(), res, err, tcp)
                if apitch <= pitch_max:       # 적당한 pitch면 채택 (자연 슬랜티드)
                    break
        if best is not None:
            _, q, res, err, tcp = best
            return q, res, err, tcp
        pin.forwardKinematics(self.model, self.data, fb_q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        tcp = self.data.oMf[self.fid].translation.copy()
        return fb_q, fb_res, float(np.linalg.norm(tcp - p) * 1000.0), tcp

    # -------- 6D IK (수직하향 자세 구속; 책상 걸림 방지)
    def _ik6_descent(self, oMdes, q):
        err = np.zeros(6)
        for _ in range(IK_MAX_ITER):
            pin.forwardKinematics(self.model, self.data, q)
            pin.updateFramePlacement(self.model, self.data, self.fid)
            iMd = self.data.oMf[self.fid].actInv(oMdes)
            err = pin.log6(iMd).vector
            if np.linalg.norm(err) < IK_EPS:
                return q, float(np.linalg.norm(err)), True
            J = pin.computeFrameJacobian(self.model, self.data, q, self.fid)
            J = -pin.Jlog6(iMd.inverse()) @ J
            JJt = J @ J.T
            JJt[np.diag_indices_from(JJt)] += IK_DAMP
            q = pin.integrate(self.model, q, (-J.T @ np.linalg.solve(JJt, err)) * IK_DT)
            self._clamp(q)
        return q, float(np.linalg.norm(err)), False

    def solve_vdown(self, x, y, z, seed):
        oMdes = pin.SE3(R_DOWN, np.array([x, y, z]))
        best_q, best_res = seed.copy(), 1e18
        for attempt in range(IK_RESTARTS + 1):
            qa = seed.copy() if attempt == 0 else pin.randomConfiguration(self.model)
            q, res, conv = self._ik6_descent(oMdes, qa)
            if res < best_res:
                best_res, best_q = res, q
            if conv:
                best_q, best_res = q, res
                break
        pin.forwardKinematics(self.model, self.data, best_q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        tcp = self.data.oMf[self.fid].translation.copy()
        return best_q, best_res, float(np.linalg.norm(tcp - np.array([x, y, z])) * 1000.0), tcp

    def solve_pose(self, x, y, z, R, seed):
        """위치(x,y,z) + 주어진 회전 R 로 6D IK (자세 구속). seed 시드."""
        oMdes = pin.SE3(np.asarray(R, dtype=float), np.array([x, y, z]))
        best_q, best_res = seed.copy(), 1e18
        for attempt in range(IK_RESTARTS + 1):
            qa = seed.copy() if attempt == 0 else pin.randomConfiguration(self.model)
            q, res, conv = self._ik6_descent(oMdes, qa)
            if res < best_res:
                best_res, best_q = res, q
            if conv:
                best_q, best_res = q, res
                break
        pin.forwardKinematics(self.model, self.data, best_q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        tcp = self.data.oMf[self.fid].translation.copy()
        return best_q, best_res, float(np.linalg.norm(tcp - np.array([x, y, z])) * 1000.0), tcp

    def init_tcp_pose(self):
        """INIT_DEG 구성에서의 hand_tcp 위치[m]+회전(R) (FK)."""
        q = pin.neutral(self.model)
        for k, qi in enumerate(self.qidx):
            q[qi] = math.radians(INIT_DEG[k])
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        M = self.data.oMf[self.fid]
        return M.translation.copy(), M.rotation.copy()

    def grasp_pose_rpy(self, q):
        """주어진 q 에서 hand_tcp 의 위치[m]와 RPY[deg] (body_link0 기준)."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        M = self.data.oMf[self.fid]
        return M.translation.copy(), np.degrees(pin.rpy.matrixToRpy(M.rotation))

    def solve_pick_optimal(self, bx, by, seed, n=N_CAND):
        """박스로 가는 접근 IK(Inverse Kinematics)만 후보 탐색. 시드 1 + 랜덤 리스타트를
        MAX_TRIES 까지 시도하며 파지 유효(|pitch|<=PITCH_MAX, |yaw|<=YAW_MAX, err<=5mm) 접근
        후보를 모은다(유효 n개 모이면 조기종료). 유효 후보 중 접근 이동량(현재 관절 대비 최대
        관절 변화) 최소를 선택. 하강은 선택 접근에서 1회만 풀이(수직 하강은 단일해로 충분).
        유효 0개면 뒤틀린 자세를 채택하지 않고 ok=False 로 상위가 안전 중단(빈손/손목뒤틀림 방지).
        파지 유효성은 접근 자세 RPY 로 판정(하강은 접근에서 시드 -> 자세 거의 동일).
        반환 (qa, ea, ta, qd, ed, td, info, ok)."""
        cur = self.arm_q()
        valids = []   # (motion, pitch, yaw, qa, ea, ta)
        tries = 0
        for i in range(MAX_TRIES):
            tries += 1
            sd = seed if i == 0 else pin.randomConfiguration(self.model)
            qa, _, ea, ta = self.solve_pos(bx, by, APPROACH_Z, sd)
            _, arpy = self.grasp_pose_rpy(qa)
            pitch, yaw = abs(float(arpy[1])), abs(float(arpy[2]))
            if ea <= 5.0 and pitch <= PITCH_MAX and yaw <= YAW_MAX:
                motion = float(np.degrees(np.max(np.abs(self.target_arm(qa) - cur))))
                valids.append((motion, pitch, yaw, qa, ea, ta))
                if len(valids) >= n:
                    break
        if not valids:
            info = (f"유효 0개/{tries}회 — |yaw|<={YAW_MAX:.0f} & |pitch|<={PITCH_MAX:.0f} "
                    f"접근 없음(뒤틀림뿐). 안전 중단 요망")
            return None, 99.0, None, None, 99.0, None, info, False
        valids.sort(key=lambda c: c[0])                # 접근 이동량 최소(최적경로)
        motion, pitch, yaw, qa, ea, ta = valids[0]
        qd, _, ed, td = self.solve_pos(bx, by, DESCEND_Z, qa)   # 하강은 선택 접근에서 1회만(자유 IK)
        info = (f"접근 유효 {len(valids)}개/{tries}회 선택 "
                f"이동 {motion:.0f}deg / pitch {pitch:.0f}deg / yaw {yaw:.0f}deg (하강 1회)")
        return qa, ea, ta, qd, ed, td, info, True

    def _ref_model(self):
        """grasp 기준자세 모델 1회 로드·캐시. 자기 side 모델 우선, 없으면 right 폴백."""
        if getattr(self, "_refm", "x") != "x":
            return self._refm
        import yaml
        base = os.path.dirname(os.path.abspath(__file__))
        native = os.path.join(base, f"{SIDE}_grasp_reference_model.yaml")
        if os.path.exists(native):
            self._refm = yaml.safe_load(open(native))
            self._refm_native = True          # 자기 side 데이터로 만든 모델 -> yaw 그대로
        else:
            fb = os.path.join(base, "right_grasp_reference_model.yaml")
            self._refm = yaml.safe_load(open(fb)) if os.path.exists(fb) else None
            self._refm_native = False         # right 모델 폴백 -> left 는 yaw 미러
        return self._refm

    def solve_pick_refmodel(self, bx, by, seed):
        """grasp 기준자세 모델: 박스 (X,|Y|) -> 목표 tilt(평면회귀) -> 목표 자세 R.
        접근·하강 모두 그 R 로 6D IK(자세 구속) -> 접근 자세=하강 자세=기준자세
        (자유 IK 의 수직화/뒤틀림 없음). 모델은 오른팔 기반 -> 왼팔은 yaw 부호반전(미러).
        반환 (qa, ea, ta, qd, ed, td, info, ok)."""
        m = self._ref_model()
        if not m:
            return (None, 99.0, None, None, 99.0, None,
                    "기준자세 모델 없음(build_grasp_reference_model.py 먼저)", False)
        tp = m["tilt_plane"]
        tilt = tp["a_X"] * bx + tp["b_absY"] * abs(by) + tp["c"]
        lo, hi = m.get("tilt_clamp_deg", [10.0, 60.0])
        tilt = max(lo, min(hi, float(tilt)))
        roll0 = float(m.get("roll_deg", 180.0))
        # native(자기 side 데이터) 모델이면 yaw 그대로, right 모델 폴백을 left 가 쓰면 미러(부호반전)
        yaw_flip = -1.0 if (SIDE == "left" and not getattr(self, "_refm_native", False)) else 1.0
        yaw0 = float(m.get("yaw_deg", 0.0)) * yaw_flip
        R = pin.rpy.rpyToMatrix(math.radians(roll0), math.radians(-tilt), math.radians(yaw0))
        # 접근(박스 위 경유점): APPROACH_Z 부터 낮춰가며 '도달 가능한 가장 높은' 접근을 선택.
        # 근접 박스는 z=0.88 도달 불가하지만 더 낮은(박스 위) 접근은 도달 -> 학습 파지를 살림.
        # 하한은 박스 위 유지(DESCEND_Z+0.05). 하강(실제 파지=학습 자세)만 엄격 판정.
        qa = ta = None
        ea = 99.0
        az = APPROACH_Z
        z = APPROACH_Z
        while z >= DESCEND_Z + 0.05 - 1e-9:
            q_, _, e_, t_ = self.solve_pose(bx, by, z, R, seed)
            if e_ < ea:
                qa, ta, ea, az = q_, t_, e_, z
            if e_ <= 8.0:
                break
            z -= 0.01
        qd, _, ed, td = self.solve_pose(bx, by, DESCEND_Z, R, qa)
        ok = (ed <= 5.0 and ea <= 35.0)        # 파지(하강) 엄격, 접근은 도달가능 best
        info = (f"기준자세 tilt={tilt:.1f}° (roll{roll0:.0f}/yaw{yaw0:+.1f}) "
                f"접근z={az:.2f} err={ea:.1f}mm 하강err={ed:.1f}mm")
        return qa, ea, ta, qd, ed, td, info, ok

    def gripper_tilt_deg(self, q):
        """그리퍼 접근축(hand_tcp z)이 수직하향(-Z)에서 벗어난 각도 [deg]. 수직=0."""
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacement(self.model, self.data, self.fid)
        R = self.data.oMf[self.fid].rotation
        cosang = max(-1.0, min(1.0, -float(R[2, 2])))
        return math.degrees(math.acos(cosang))

    def arm_q(self):
        """현재 측정 7관절 (j1..j7 순서)."""
        return np.array([self.state.get(n, 0.0) for n in self.jnames])

    def seed_full(self):
        """현재 측정값을 reduced-model q 벡터에 채운 시드."""
        q = pin.neutral(self.model)
        for k, qi in enumerate(self.qidx):
            q[qi] = self.state.get(self.jnames[k], 0.0)
        return q

    def target_arm(self, qfull):
        return np.array([qfull[qi] for qi in self.qidx])

    # ---------- controller switch
    def do_switch(self, activate, deactivate):
        if not self.switch.wait_for_service(timeout_sec=5.0):
            raise RuntimeError("switch_controller service 없음")
        req = SwitchController.Request()
        req.activate_controllers = activate
        req.deactivate_controllers = deactivate
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout = Duration(sec=5, nanosec=0)
        fut = self.switch.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=8.0)
        ok = fut.result() is not None and fut.result().ok
        print(f"   switch activate={activate} deactivate={deactivate} -> ok={ok}")
        return ok

    # ---------- moves
    def forward_ramp(self, q_target_arm, label):
        """현재 측정값에서 목표까지 rate-limit 램프 (forward position)."""
        if not np.all(np.isfinite(q_target_arm)):
            print(f"   [{label}] NaN/inf 타깃 — 램프 중단(발행 안 함)")
            return
        cmd = self.arm_q()
        max_step = math.radians(RAMP_RATE_DPS) / RAMP_HZ
        n = 0
        while True:
            d = q_target_arm - cmd
            if np.max(np.abs(d)) < 1e-3:
                break
            cmd = cmd + np.clip(d, -max_step, max_step)
            self.fpos_pub.publish(Float64MultiArray(data=[float(v) for v in cmd]))
            self.wait(1.0 / RAMP_HZ)
            n += 1
            if n > 600:
                print(f"   [{label}] 램프 스텝 상한(600) 도달 — 중단")
                break
        # settle: 목표 유지
        for _ in range(int(RAMP_HZ * SETTLE)):
            self.fpos_pub.publish(Float64MultiArray(data=[float(v) for v in q_target_arm]))
            self.wait(1.0 / RAMP_HZ)
        print(f"   [{label}] forward-ramp 완료 ({n} steps)")

    def _jtc_send(self, q_target_arm, t, label):
        for _ in range(50):
            if self.jtc_pub.get_subscription_count() > 0:
                break
            self.wait(0.1)
        traj = JointTrajectory()
        traj.joint_names = self.jnames
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in q_target_arm]
        pt.time_from_start = Duration(sec=int(t), nanosec=int((t % 1) * 1e9))
        traj.points = [pt]
        self.jtc_pub.publish(traj)
        print(f"   [{label}] JTC 발행 ({t}s)")

    def jtc_move(self, q_target_arm, label, t=None):
        if t is None:
            t = JTC_MOVE_TIME
        self._jtc_send(q_target_arm, t, label)
        self.wait(t + SETTLE)

    def grip_status(self, label="grip-check"):
        """핑거 위치로 파지 성공 판정. 박스 물리면 명령(0.0088)보다 높은 폭에서 stall."""
        for _ in range(15):
            rclpy.spin_once(self, timeout_sec=0.05)
        g = self.state.get(f"openarmx_{SIDE}_finger_joint1", 0.0)
        # 임계: 기동 캘리브로 측정한 빈손 바닥 기준(self.grip_thr). 없으면 절대값 폴백.
        thr = getattr(self, "grip_thr", GRIP_80 + GRIP_DETECT_MARGIN)
        gripped = g > thr
        print(f"   [{label}] finger={g:.4f}m (임계 {thr:.4f}) -> "
              f"{'파지 OK(박스 물림)' if gripped else '미파지 의심(빈 손)'}")
        return gripped

    def goto_init(self, t=None):
        """INIT(j1=50,j4=100,j7=50)로 복귀(JTC). 스위치 직후 첫 궤적 무효화 방지 위해
        현재 위치 워밍업 궤적 1회 후 INIT 발행."""
        if t is None:
            t = INIT_TIME
        init = np.array([math.radians(d) for d in INIT_DEG])
        # 짧은 워밍업 발행로 스위치 직후 JTC 레퍼런스 정렬 (최소 대기)
        self._jtc_send(self.arm_q(), WARMUP_T, "warmup")
        self.wait(0.1)
        self._jtc_send(init, t, "INIT")
        self.wait(t + SETTLE)

    def goto_joint(self, deg, label, t=None):
        """임의 관절각(deg)으로 JTC 이동(스위치 직후 워밍업 궤적 후 목표 발행)."""
        if t is None:
            t = INIT_TIME
        tgt = np.array([math.radians(d) for d in deg])
        self._jtc_send(self.arm_q(), WARMUP_T, "warmup")
        self.wait(0.1)
        self._jtc_send(tgt, t, label)
        self.wait(t + SETTLE)

    def gripper(self, position, label, block=True, settle_after=None):
        if settle_after is None:
            settle_after = SETTLE
        if not self.grip.wait_for_server(timeout_sec=5.0):
            print(f"   [{label}] gripper server 없음")
            return
        g = GripperCommand.Goal()
        g.command.position = float(position)
        g.command.max_effort = GRIP_EFFORT
        fut = self.grip.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=5.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print(f"   [{label}] gripper rejected")
            return
        if not block:
            # 결과(특히 박스 stall=stall_timeout 10s) 안 기다리고 짧게 정착 후 진행
            print(f"   [{label}] gripper 비차단 전송 -> {settle_after}s 정착")
            self.wait(settle_after)
            return
        rfut = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rfut, timeout_sec=15.0)
        res = rfut.result()
        if res is None:
            print(f"   [{label}] gripper 결과 타임아웃(박스 파지 stall로 추정) — 계속 진행")
        else:
            r = res.result
            print(f"   [{label}] gripper pos={r.position:.4f} reached={r.reached_goal} stalled={r.stalled}")
        self.wait(0.4)


def main():
    run = "--run" in sys.argv
    global RAMP_RATE_DPS, JTC_MOVE_TIME, INIT_TIME, SETTLE, APPROACH_Z, DESCEND_Z
    RAMP_RATE_DPS = _argf("--rate", RAMP_RATE_DPS)
    JTC_MOVE_TIME = _argf("--descend", JTC_MOVE_TIME)
    INIT_TIME = _argf("--init", INIT_TIME)
    SETTLE = _argf("--settle", SETTLE)
    _az, _dz = load_zheights()
    if _az is not None:
        APPROACH_Z = _az
    if _dz is not None:
        DESCEND_Z = _dz
    rclpy.init()
    node = PickV2()
    print(f"=== 왼팔 pick v2 (자세해제, 접근/상승=forward position, 하강=JTC) "
          f"[{'RUN' if run else 'DRY'}] ===")
    print(f"[tune] rate={RAMP_RATE_DPS}dps descend={JTC_MOVE_TIME}s "
          f"init={INIT_TIME}s settle={SETTLE}s  approach_z={APPROACH_Z} descend_z={DESCEND_Z}")

    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.urdf and node.state:
            break
    if not node.urdf:
        print("ERROR: /robot_description 미수신"); return 1
    node.build_model()
    node.wait(1.5)

    if node.box is None:
        print("ERROR: /detected_boxes 없음 (검출 먼저)"); return 1
    if len(node.boxes) > 1:
        side_lbl = "오른쪽 Y<0" if SIDE == "right" else "왼쪽 Y>0"
        srt = sorted(node.boxes, key=lambda b: b[0])
        print(f"[0] 박스 {len(node.boxes)}개 검출 -> {SIDE}팔 담당({side_lbl}) 중 X 최소(가까운) 선택")
        for b in srt:
            same = (b[1] < 0.0) if SIDE == "right" else (b[1] > 0.0)
            mark = " <= 선택" if b == node.box else ("" if same else "  (반대측면 제외)")
            print(f"     X={b[0]:.3f} Y={b[1]:+.3f} Z={b[2]:.3f}{mark}")
    rbx, rby, rbz = node.box
    ox, oy, oz = load_grasp_offset()
    bx, by, bz = rbx + ox, rby + oy, rbz + oz
    print(f"[1] 검출 박스 ({rbx:.4f}, {rby:.4f}, {rbz:.4f}) "
          f"+ 오프셋({ox:+.3f},{oy:+.3f},{oz:+.3f}) -> 타깃 ({bx:.4f}, {by:.4f}, {bz:.4f})")
    DESCEND_Z = max(DESCEND_FLOOR, bz - GRASP_DEPTH)   # 박스 z 추종(하한 0.72)
    print(f"   [하강 z] 박스 z {bz:.3f} -> descend_z {DESCEND_Z:.3f} "
          f"(max({DESCEND_FLOOR}, z-{GRASP_DEPTH}))")

    # IK: 수직하향(자세구속) 우선 -> 도달불가(>5mm)면 자세자유 폴백. 현재 자세 시드.
    # 기본 = 자유(position-only). --vdown 시에만 수직하향 우선(+자유 폴백).
    vdown_pref = "--vdown" in sys.argv
    seed = node.seed_full()

    ik_times = []

    def solve(z, sd):
        t0 = _time.monotonic()
        if vdown_pref:
            q, r, e, t = node.solve_vdown(bx, by, z, sd)
            if e <= 5.0:
                ik_times.append(_time.monotonic() - t0)
                return q, r, e, t, "vdown"
            print(f"   (z={z}: 수직하향 도달불가 err={e:.1f}mm -> 자세자유 폴백)")
        q, r, e, t = node.solve_pos(bx, by, z, sd)
        ik_times.append(_time.monotonic() - t0)
        return q, r, e, t, "free"

    t_ik = _time.monotonic()
    grasp_ok = True
    if vdown_pref:
        qa, ra, ea, ta, ma = solve(APPROACH_Z, seed)
        qd, rd, ed, td, md = solve(DESCEND_Z, qa)
        opt_info = "vdown(후보선택 미적용)"
    elif "--optimal" in sys.argv:
        # (구방식 폴백) 접근만 후보 탐색 -> 파지 유효 중 이동량 최소
        qa, ea, ta, qd, ed, td, opt_info, grasp_ok = node.solve_pick_optimal(bx, by, seed, N_CAND)
        ma = md = "free"
    else:
        # 기본: grasp 기준자세 모델(박스 위치 -> 목표 자세). 접근=하강=기준자세.
        qa, ea, ta, qd, ed, td, opt_info, grasp_ok = node.solve_pick_refmodel(bx, by, seed)
        ma = md = "refmodel"
        if not grasp_ok:
            # 기준자세 6D IK 도달불가(자세 경직) -> 자유 IK 후보탐색 폴백
            print(f"   [기준자세 도달불가: {opt_info}] -> 자유 IK 폴백")
            qa, ea, ta, qd, ed, td, opt_info, grasp_ok = node.solve_pick_optimal(bx, by, seed, N_CAND)
            ma = md = "free(폴백)"
    if not grasp_ok:
        print(f"   [IK 자세결정] {opt_info}")
        print("!! SAFE-ABORT: 유효 파지자세 없음/모델 없음/도달불가 — 실행 안 함. 박스 위치·모델 확인.")
        node.destroy_node(); rclpy.shutdown(); return 1
    qr, rr, er, tr, mr = qa, 0.0, ea, ta, ma   # 상승=접근 자세 복귀(재풀이 불필요)
    ik_ms = (_time.monotonic() - t_ik) * 1000.0
    # 중간자세(테이블 충돌 방지): INIT 자세 유지 + X+7cm + z>=0.75
    it_pos, it_R = node.init_tcp_pose()
    mx, my, mz = it_pos[0] + MID_X_FWD, it_pos[1], max(float(it_pos[2]), MID_MIN_Z)
    q_mid, _, e_mid, t_mid = node.solve_pose(mx, my, mz, it_R, seed)
    mid = node.target_arm(q_mid)
    print(f"[2a] 중간자세 (INIT XY {it_pos[0]:.3f},{it_pos[1]:.3f} -> X+{MID_X_FWD} z>={MID_MIN_Z}) "
          f"타깃 ({mx:.3f}, {my:.3f}, {mz:.3f}) 자세유지 err={e_mid:.2f}mm")
    print(f"   [IK 자세모드] 접근={ma} 하강={md} 상승={mr}")
    print(f"   [IK 자세결정] {opt_info}")
    print(f"   [IK 계산시간] 합 {ik_ms:.0f}ms")
    tilt = node.gripper_tilt_deg(qd)
    print(f"   [그리퍼 기울기] 하강자세 tilt={tilt:.1f}° (수직=0°; 적당히 비스듬이 파지에 유리)")
    gp, grpy = node.grasp_pose_rpy(qd)
    print(f"   [파지자세] pos=({gp[0]:.3f}, {gp[1]:.3f}, {gp[2]:.3f})m  "
          f"RPY=(R {grpy[0]:.1f}, P {grpy[1]:.1f}, Y {grpy[2]:.1f})deg")
    appr, desc, retr = node.target_arm(qa), node.target_arm(qd), node.target_arm(qr)
    cur = node.arm_q()
    print(f"[2] 접근 z={APPROACH_Z}: err={ea:.2f}mm tcp=({ta[0]:.3f},{ta[1]:.3f},{ta[2]:.3f}) "
          f"이동량(최대관절)={np.degrees(np.max(np.abs(appr-cur))):.1f}deg")
    print(f"[4] 하강 z={DESCEND_Z}: err={ed:.2f}mm tcp=({td[0]:.3f},{td[1]:.3f},{td[2]:.3f}) "
          f"이동량={np.degrees(np.max(np.abs(desc-appr))):.1f}deg")
    print(f"[6] 상승 z={APPROACH_Z}: err={er:.2f}mm 이동량={np.degrees(np.max(np.abs(retr-desc))):.1f}deg")
    # ---- 안전 가드: NaN/과대오차/과대이동이면 실행 전 중단 (NaN 명령 → 팔 오작동 방지) ----
    cur = node.arm_q()
    problems = []
    for nm, q, e in (("중간자세", mid, e_mid), ("접근", appr, ea),
                     ("하강", desc, ed), ("상승", retr, er)):
        if not np.all(np.isfinite(q)):
            problems.append(f"{nm}:NaN/inf")
        elif e > 5.0:
            problems.append(f"{nm}:err{e:.1f}mm")
    if np.all(np.isfinite(appr)):
        mv = np.degrees(np.max(np.abs(appr - cur)))
        if mv > 120.0:
            problems.append(f"접근이동{mv:.0f}deg(>120)")
    if problems:
        print(f"!! SAFE-ABORT: {problems} — 실행 안 함(NaN/과대 모션 방지). 재검출/재시도 요망.")
        if run:
            node.destroy_node(); rclpy.shutdown(); return 1
    print(f"   접근 관절(deg)={[f'{d:+.0f}' for d in np.degrees(appr)]}")
    print(f"   하강 관절(deg)={[f'{d:+.0f}' for d in np.degrees(desc)]}")

    if not run:
        print("=== DRY: 동작/스위치 안 함. 실제 실행은 --run ===")
        node.destroy_node(); rclpy.shutdown(); return 0

    # ---- retreat-only: 현재(파지) 상태에서 z=0.8 상승만 (그리퍼 미관여) ----
    if "--retreat-only" in sys.argv:
        print("[retreat-only] z=0.8 상승 (forward position, 그리퍼 미관여=박스 파지 유지)")
        node.do_switch([FPOS], [JTC])
        node.forward_ramp(retr, "상승")
        node.do_switch([JTC], [FPOS])
        print("=== 상승 완료 (박스 파지 유지) ===")
        node.destroy_node(); rclpy.shutdown(); return 0

    # ---- release-init: 현재(파지) 상태에서 3초 후 펴고 INIT 원위치만 ----
    if "--release-init" in sys.argv:
        print("[release-init] 3초 대기 -> 그리퍼 open(release) -> INIT 원위치")
        node.wait(3.0)
        node.gripper(GRIP_OPEN, "release")
        node.goto_init()
        print("=== release + INIT 완료 ===")
        node.destroy_node(); rclpy.shutdown(); return 0

    # ---- 실제 실행 (단계별 타이밍 계측) ----
    laps = []

    def lap(label, fn):
        t0 = _time.monotonic()
        fn()
        dt = _time.monotonic() - t0
        laps.append((label, dt))
        print(f"   ⏱ {label}: {dt:.2f}s")

    t_all = _time.monotonic()
    print("[0] 그리퍼 open")
    lap("0_gripper_open", lambda: node.gripper(GRIP_OPEN, "open"))

    print("[2] 접근 (JTC->forward: 중간자세 -> 접근)")
    lap("2a_switch_fpos", lambda: node.do_switch([FPOS], [JTC]))
    lap("2b_mid", lambda: node.forward_ramp(mid, "중간자세"))
    lap("2c_approach_ramp", lambda: node.forward_ramp(appr, "접근"))

    print("[4] 하강 z=0.75 (forward->JTC)")
    lap("4a_switch_jtc", lambda: node.do_switch([JTC], [FPOS]))
    lap("4b_descend_jtc", lambda: node.jtc_move(desc, "하강"))

    print("[5] 그리퍼 80% close (비차단)")
    lap("5_grasp", lambda: node.gripper(GRIP_80, "80%", block=False, settle_after=GRASP_SETTLE))

    print("[6] 상승 (JTC->forward)")
    lap("6a_switch_fpos", lambda: node.do_switch([FPOS], [JTC]))
    lap("6b_retreat_ramp", lambda: node.forward_ramp(retr, "상승"))
    gripped = node.grip_status("파지검증")

    print("[end] JTC 복귀")
    lap("end_switch_jtc", lambda: node.do_switch([JTC], [FPOS]))

    if gripped:
        # 파지 성공 -> 접근 지점 -> 드랍 위치 -> 드롭 -> (복귀: 접근 지점 경유) -> INIT
        print("[7] 파지 성공 -> 접근 지점 -> 드랍 위치 -> 드롭 -> 접근 지점 경유 -> INIT")
        lap("7a_to_approach_pt", lambda: node.goto_joint(APPROACH_POINT_DEG, "박스접근지점"))
        lap("7b_to_drop_pt", lambda: node.goto_joint(DROP_POINT_DEG, "박스드랍위치"))
        lap("8_drop", lambda: node.gripper(GRIP_OPEN, "release(drop)", block=False, settle_after=0.3))
        lap("8b_back_approach_pt", lambda: node.goto_joint(APPROACH_POINT_DEG, "박스접근지점(복귀)"))
        lap("9_goto_init", lambda: node.goto_init())
    else:
        # 파지 실패(빈 손) -> 놓기 생략, 곧장 홈 복귀
        print("[7] 파지 실패(빈 손) -> 놓기 생략, 홈 복귀")
        lap("9_goto_init", lambda: node.goto_init())

    total = _time.monotonic() - t_all
    print("\n=== 단계별 타이밍 분석 ===")
    for label, dt in laps:
        print(f"  {label:18s}: {dt:6.2f}s  ({dt / total * 100:4.1f}%)")
    print(f"  {'TOTAL':18s}: {total:6.2f}s")
    print("=== 시퀀스 완료 ("
          + ("파지 성공: 접근->드랍 드롭->접근 경유->INIT" if gripped else "파지 실패: 홈 복귀(놓기 생략)") + ") ===")
    node.destroy_node(); rclpy.shutdown(); return 0


if __name__ == "__main__":
    sys.exit(main())
