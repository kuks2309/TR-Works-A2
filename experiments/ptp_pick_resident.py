#!/usr/bin/env python3
"""상주(resident) pick&place 노드 — 모델·rclpy 1회 로드 후 상주하여 매 pick 을 빠르게(~1s 셋업).

ptp_pick_seq_v2_left.py(PickV2)의 검증된 메서드를 그대로 재사용하되, 스크립트는 건드리지 않고
흐름만 이 파일에서 재현(run_pick). 좌/우 각 1개 프로세스(SIDE 모듈전역 때문).

서비스(std_srvs/Trigger):
  ~/pick_once   : 정해진 색(param 'color')에서 자기 측면 가장 가까운(min-X) 박스 1개 pick&place
  ~/auto_start  : 연속 loop 시작 (param 'color')
  ~/auto_stop   : 연속 loop 중지
파라미터:
  color : 'mini-box-red' | 'mini-box-yellow' | 'mini-box-green' | 'auto'(전체)
상태 토픽: ~/status (std_msgs/String) — 'idle/picking/ok/empty/no_box ...'

사용: python3 ptp_pick_resident.py --side right
"""
import fcntl
import os
import sys
import time

import numpy as np

ARM_LOCK_PATH = "/tmp/openarmx_arm_pick.lock"   # 양팔 동시 동작 금지(프로세스 간 뮤텍스)

def _pop_side(argv):
    """--side=X / --side X 둘 다 처리 -> (side, --side 인자 제거된 나머지)."""
    side, out, i = "left", [], 0
    while i < len(argv):
        a = argv[i]
        if a.startswith("--side="):
            side = a.split("=", 1)[1]; i += 1; continue
        if a == "--side" and i + 1 < len(argv):
            side = argv[i + 1]; i += 2; continue
        out.append(a); i += 1
    return side, out


_side, _rest = _pop_side(sys.argv)
# core 는 모듈로드 시 '--side=X'(등호)만 읽으므로 등호형으로 정규화해 주입
sys.argv = [_rest[0], f"--side={_side}"] + _rest[1:]
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy  # noqa: E402
from rclpy.action import ActionClient  # noqa: E402
from std_srvs.srv import Trigger  # noqa: E402
from std_msgs.msg import String, Bool  # noqa: E402
import ptp_pick_seq_v2_left as core  # noqa: E402

ALL_COLORS = ["mini-box-red", "mini-box-yellow", "mini-box-green",
              "mini-box-blue", "mini-box-orange"]   # box_perception 5색 (mini-box-seg 제외)


def run_pick(node, run=True):
    """검증된 main() 흐름 재현. node.box(측면 min-X)가 채워진 상태에서 호출.
    반환 dict {ok, gripped, box, finger, info}."""
    if node.box is None:
        return {"ok": False, "gripped": False, "info": "no_box"}
    rbx, rby, rbz = node.box
    ox, oy, oz = core.load_grasp_offset()
    bx, by, bz = rbx + ox, rby + oy, rbz + oz
    core.DESCEND_Z = max(core.DESCEND_FLOOR, bz - core.GRASP_DEPTH)   # 박스 z 추종(하한 0.72)
    seed = node.seed_full()

    # 자세 결정: 기준자세 모델 -> 도달불가면 자유 IK 폴백
    qa, ea, ta, qd, ed, td, info, ok = node.solve_pick_refmodel(bx, by, seed)
    if not ok:
        qa, ea, ta, qd, ed, td, info, ok = node.solve_pick_optimal(bx, by, seed, core.N_CAND)
    if not ok:
        return {"ok": False, "gripped": False, "info": f"unreachable: {info}"}
    qr, er, tr = qa, ea, ta                       # 상승=접근 자세 복귀

    # 중간자세(테이블 충돌 방지)
    it_pos, it_R = node.init_tcp_pose()
    mx, my, mz = it_pos[0] + core.MID_X_FWD, it_pos[1], max(float(it_pos[2]), core.MID_MIN_Z)
    q_mid, _, e_mid, _ = node.solve_pose(mx, my, mz, it_R, seed)
    mid = node.target_arm(q_mid)
    appr, desc, retr = node.target_arm(qa), node.target_arm(qd), node.target_arm(qr)
    cur = node.arm_q()

    # 안전 가드
    problems = []
    for nm, q, e in (("중간", mid, e_mid), ("접근", appr, ea), ("하강", desc, ed), ("상승", retr, er)):
        if not np.all(np.isfinite(q)):
            problems.append(f"{nm}:NaN")
        elif e > 5.0:
            problems.append(f"{nm}:err{e:.1f}")
    if np.all(np.isfinite(appr)) and np.degrees(np.max(np.abs(appr - cur))) > 120.0:
        problems.append("접근이동>120")
    g = node.grasp_pose_rpy(qd)[1]
    if not run:
        return {"ok": True, "gripped": None, "info": f"DRY {info} tilt~{abs(g[1]):.0f}",
                "problems": problems}
    if problems:
        return {"ok": False, "gripped": False, "info": f"SAFE-ABORT {problems}"}

    FPOS, JTC = core.FPOS, core.JTC
    # 이동 중엔 그리퍼를 닫아(좁게) 스냅 방지, 박스 바로 위(접근점)에서 열고 하강(top-down)
    node.gripper(core.GRIP_80, "닫고 이동", block=False, settle_after=0.3)
    node.do_switch([FPOS], [JTC])
    node.forward_ramp(mid, "중간자세")
    node.forward_ramp(appr, "접근")
    node.gripper(core.GRIP_OPEN, "파지위치-열기")
    node.do_switch([JTC], [FPOS])
    node.jtc_move(desc, "하강")
    node.gripper(core.GRIP_80, "파지", block=False, settle_after=core.GRASP_SETTLE)
    node.do_switch([FPOS], [JTC])
    node.forward_ramp(retr, "상승")
    gripped = node.grip_status("파지검증")
    node.do_switch([JTC], [FPOS])
    if gripped:
        node.goto_joint(core.APPROACH_POINT_DEG, "박스접근지점")
        node.goto_joint(core.DROP_POINT_DEG, "박스드랍위치")
        node.gripper(core.GRIP_OPEN, "drop", block=False, settle_after=0.3)
        node.goto_joint(core.APPROACH_POINT_DEG, "박스접근지점(복귀)")
    # INIT 은 항상 그리퍼 열림 상태로 복귀 (실패 시 쥐고 있던 것도 여기서 놓음)
    node.gripper(core.GRIP_OPEN, "INIT 복귀-열기", block=False, settle_after=0.2)
    node.goto_init()
    box = tuple(round(v, 4) for v in (rbx, rby, rbz))
    return {"ok": True, "gripped": bool(gripped), "box": box, "info": info}


class ResidentPick:
    def __init__(self):
        self.node = core.PickV2()
        # 모델 1회 로드 — 스택이 늦게 떠도 기다림(최대 ~120s, 진행 로그)
        waited = 0.0
        while rclpy.ok() and not (self.node.urdf and self.node.state):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            waited += 0.1
            if waited % 5.0 < 0.1:
                miss = ("/robot_description" if not self.node.urdf else "/joint_states")
                print(f"[{core.SIDE}] 스택 대기 중... {miss} 미수신 ({waited:.0f}s)", flush=True)
            if waited > 120.0:
                raise RuntimeError("스택 미기동(120s) — L0 하드웨어/컨트롤러 먼저 올리세요")
        self.node.build_model()
        self.node.declare_parameter("color", "mini-box-red")
        self._color_val = "mini-box-red"   # /pick_color 토픽으로 UI가 갱신
        self.detect = ActionClient(self.node, _detect_action_type(), "/yolov8_node/detect")
        self.status_pub = self.node.create_publisher(String, "~/status", 10)
        self.node.create_subscription(String, "/pick_color", self._on_color, 10)
        self.node.create_service(Trigger, "~/pick_once", self._srv_pick_once)
        self.node.create_service(Trigger, "~/auto_start", self._srv_auto_start)
        self.node.create_service(Trigger, "~/auto_stop", self._srv_auto_stop)
        # 양팔 동시동작 제어: 기본=단일팔(뮤텍스로 충돌방지). /allow_dual_arm True(UI 체크박스)면 동시 허용
        self._allow_dual = False
        self.node.create_subscription(Bool, "/allow_dual_arm", self._on_dual, 10)
        self._lock_fd = open(ARM_LOCK_PATH, "w")
        self._have_lock = False
        self._auto = False
        self._pick_req = False     # 콜백은 플래그만; 실제 흐름은 메인 루프(중첩 스핀 방지)
        self._busy = False
        self._ok = 0
        self._ng = 0
        self._calibrate_grip()   # 빈손 바닥 측정 -> 상대 임계(self.node.grip_thr). 끝나면 INIT=열림
        self._pub_status(f"resident {core.SIDE} 준비 완료")

    def _calibrate_grip(self):
        """기동 시 빈손 닫힘 위치(바닥)를 재서 파지 임계=바닥+margin 설정(재시작 드리프트 흡수).
        절차: 열기 -> 빈손 닫기(비차단, stall 대기 회피) -> finger 중앙값 측정 -> 다시 열기(INIT)."""
        jn = f"openarmx_{core.SIDE}_finger_joint1"
        self.node.gripper(core.GRIP_OPEN, "calib-열기", block=False, settle_after=0.8)
        self.node.gripper(core.GRIP_80, "calib-빈손닫기", block=False, settle_after=1.8)
        vals = []
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)
            vals.append(self.node.state.get(jn, 0.0))
        floor = sorted(vals)[len(vals) // 2]            # 중앙값
        self.node.grip_thr = floor + core.GRIP_CALIB_MARGIN
        self.node.gripper(core.GRIP_OPEN, "calib-복귀(INIT 열림)", block=False, settle_after=0.3)
        self._pub_status(f"파지 캘리브: 빈손바닥={floor:.4f} -> 임계={self.node.grip_thr:.4f}")

    def tick(self):
        """메인 루프에서 호출 — 요청 플래그가 있으면 pick 1회 실행(콜백 밖)."""
        if self._busy:
            return
        if self._pick_req:
            self._pick_req = False
            self._busy = True
            try:
                self._do_one(self._color())
            finally:
                self._busy = False
        elif self._auto:
            self._busy = True
            try:
                self._do_one(self._color())
            finally:
                self._busy = False

    def _pub_status(self, s):
        self.status_pub.publish(String(data=s))
        print(f"[{core.SIDE}] {s}", flush=True)

    def _on_color(self, msg):
        if msg.data:
            self._color_val = msg.data        # 'mini-box-red' | ... | 'auto'

    def _on_dual(self, msg):
        self._allow_dual = bool(msg.data)     # True=양팔 동시 허용, False=단일팔(뮤텍스)

    def _acquire_arm_lock(self):
        """양팔 뮤텍스 비차단 획득. 타팔이 쥐고 있으면 False(이번 틱 양보)."""
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self._have_lock = True
            return True
        except (BlockingIOError, OSError):
            return False

    def _release_arm_lock(self):
        if self._have_lock:
            try:
                fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
            except OSError:
                pass
            self._have_lock = False

    def _color(self):
        return self._color_val or "mini-box-red"

    def _trigger_detect(self, color):
        # 자동=5색 콤마 1회(루프 시 /detected_boxes 덮어써져 마지막 색만 남음).
        prompt = ",".join(ALL_COLORS) if color == "auto" else color
        if not self.detect.wait_for_server(timeout_sec=3.0):
            return
        goal = _detect_goal(prompt)
        fut = self.detect.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=20.0)
        gh = fut.result()
        if gh and gh.accepted:
            rfut = gh.get_result_async()
            rclpy.spin_until_future_complete(self.node, rfut, timeout_sec=20.0)
        for _ in range(20):
            rclpy.spin_once(self.node, timeout_sec=0.05)   # _box_cb 갱신

    def _do_one(self, color):
        # 충돌방지: 단일팔 모드면 뮤텍스 획득(실패=타팔 동작중 -> 양보). _allow_dual 이면 동시 허용.
        if not self._allow_dual and not self._acquire_arm_lock():
            self._pub_status("대기(타팔 동작중)")
            return {"ok": False, "info": "busy_other_arm"}
        try:
            self.node.box = None
            self._trigger_detect(color)
            if self.node.box is None:
                self._pub_status("no_box")
                return {"ok": False, "info": "no_box"}
            self._pub_status(f"picking {core.SIDE} box=({self.node.box[0]:.2f},{self.node.box[1]:+.2f})")
            r = run_pick(self.node, run=True)
            if r.get("gripped"):
                self._ok += 1; self._pub_status(f"ok box={r.get('box')} [성공 {self._ok}]")
            elif r["ok"]:
                self._ng += 1; self._pub_status(f"empty box={r.get('box')} [빈손 {self._ng}]")
            else:
                self._pub_status(r["info"])
            return r
        finally:
            self._release_arm_lock()

    def _srv_pick_once(self, req, resp):
        # 콜백은 플래그만(중첩 스핀 방지). 결과는 ~/status 토픽으로.
        self._pick_req = True
        resp.success = True; resp.message = "queued"; return resp

    def _srv_auto_start(self, req, resp):
        self._auto = True; self._pub_status("AUTO 시작")
        resp.success = True; resp.message = "auto on"; return resp

    def _srv_auto_stop(self, req, resp):
        self._auto = False; self._pub_status(f"AUTO 중지 (성공 {self._ok}/빈손 {self._ng})")
        resp.success = True; resp.message = "auto off"; return resp


def _detect_action_type():
    from yolov8_detection_msgs.action import DetectBox
    return DetectBox


def _detect_goal(prompt):
    from yolov8_detection_msgs.action import DetectBox
    g = DetectBox.Goal()
    g.prompts = prompt
    g.confidence = 0.5
    g.publish_annotated = True
    return g


def main():
    rclpy.init()
    rp = ResidentPick()
    try:
        # 수동 스핀 루프: 콜백 처리(spin_once) 후 tick()으로 흐름 실행(중첩 스핀 방지)
        while rclpy.ok():
            rclpy.spin_once(rp.node, timeout_sec=0.05)
            rp.tick()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            rp.node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
