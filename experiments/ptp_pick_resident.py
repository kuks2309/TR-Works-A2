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
MOTION_PATH_LOCK = "/tmp/openarmx_motion_path.lock"  # C++ Hover(EX) vs resident pick(SH) 경로 상호배타
AUTO_BOX_MAX_AGE = 30.0  # /detected_boxes 가 이보다 오래되면 stale → 픽 스킵. 검출 게이팅이 모션(~15s)
                         # 중 검출을 멈추므로 픽 사이클보다 길게(정상 게이팅을 stale 로 오판 방지)
AUTO_FAULT_LIMIT = 5     # auto 에서 연속 '고장' 이 횟수면 자동정지(무한 busy-retry 방지)

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
from rclpy.qos import (QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy,  # noqa: E402
                       QoSHistoryPolicy)
from std_srvs.srv import Trigger  # noqa: E402
from std_msgs.msg import String, Bool  # noqa: E402
import ptp_pick_seq_v2_left as core  # noqa: E402

# [기술부채] /pick_color·~/status 공용 latched QoS — 늦게 뜨거나 재시작한 노드도 마지막 값 즉시 수신.
LATCHED = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                     durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                     history=QoSHistoryPolicy.KEEP_LAST)

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

    # 안전 가드: 하강(실제 파지)은 엄격 5mm, 접근/상승/중간(경유점, 적응형이라 오차 큼)은 느슨 35mm.
    # (refmodel 의 ea<=35mm 수락과 일치시킴 — 안 그러면 근접 박스가 가드에 걸려 SAFE-ABORT)
    problems = []
    for nm, q, e, lim in (("중간", mid, e_mid, 35.0), ("접근", appr, ea, 35.0),
                          ("하강", desc, ed, 5.0), ("상승", retr, er, 35.0)):
        if not np.all(np.isfinite(q)):
            problems.append(f"{nm}:NaN")
        elif e > lim:
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
    node.gripper(core.GRIP_80, "닫고 이동", block=False, settle_after=0.15)
    if not node.do_switch([FPOS], [JTC]):   # [기술부채] 전환 실패 시 비활성 컨트롤러 발행(무동작) 방지
        return {"ok": False, "gripped": False, "info": "switch_fail(중간/접근: JTC→FPOS)"}
    c = node.forward_ramp(mid, "중간자세", settle=False)   # 접근으로 통과(무정지)
    node.forward_ramp(appr, "접근", seed=c)               # 직전 cmd 이어받기 → seam 불연속 제거
    node.gripper(core.GRIP_OPEN, "파지위치-열기")
    if not node.do_switch([JTC], [FPOS]):
        return {"ok": False, "gripped": False, "info": "switch_fail(하강: FPOS→JTC)"}
    node.jtc_move(desc, "하강")
    node.gripper(core.GRIP_80, "파지", block=False, settle_after=core.GRASP_SETTLE)
    if not node.do_switch([FPOS], [JTC]):
        return {"ok": False, "gripped": False, "info": "switch_fail(상승: JTC→FPOS)"}
    node.forward_ramp(retr, "상승")
    gripped = node.grip_status("파지검증")
    # 컨테이너(드롭) 이동도 forward position (설계 확정: 상승/컨테이너 이동=forward).
    # 상승 직후 FPOS 가 활성이므로 그대로 두고 박스접근지점→드랍→복귀를 forward_ramp 로 이동.
    # 기존 goto_joint(JTC)은 호출마다 워밍업+0.85s 라 뒷절반이 느렸음 → 설계 일치 + 단축.
    if gripped:
        c = node.forward_ramp(np.radians(core.APPROACH_POINT_DEG), "박스접근지점", settle=False)  # 드랍으로 통과
        node.forward_ramp(np.radians(core.DROP_POINT_DEG), "박스드랍위치", seed=c)  # 직전 cmd 이어받기 → 접근→드랍 한 번에 연속
        node.gripper(core.GRIP_OPEN, "drop", block=False, settle_after=0.15)
        node.forward_ramp(np.radians(core.APPROACH_POINT_DEG), "박스접근지점(복귀)")
    # INIT 복귀만 JTC (설계대로). FPOS→JTC 전환 후 복귀.
    node.gripper(core.GRIP_OPEN, "INIT 복귀-열기", block=False, settle_after=0.1)
    if not node.do_switch([JTC], [FPOS]):
        return {"ok": False, "gripped": bool(gripped), "info": "switch_fail(INIT: FPOS→JTC)"}
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
        # [기술부채] config(pick_seq_config_{side}.yaml) approach_z 를 resident 경로에도 반영
        # (이전엔 core.main() 에서만 읽혀 무시 → 모듈기본 0.88 사용). descend_z 는 런타임 박스추종이라 생략.
        _az, _dz = core.load_zheights()
        if _az is not None:
            core.APPROACH_Z = _az
        # [기술부채] ~/status latched(pub) → 늦게 붙은 UI 도 마지막 상태 즉시 수신.
        self.status_pub = self.node.create_publisher(String, "~/status", LATCHED)
        # [기술부채] /pick_color latched 구독 → 재시작 후에도 마지막 수동 색 즉시 수신(기본색 오발 방지).
        self.node.create_subscription(String, "/pick_color", self._on_color, LATCHED)
        self.node.create_service(Trigger, "~/pick_once", self._srv_pick_once)
        self.node.create_service(Trigger, "~/auto_start", self._srv_auto_start)
        self.node.create_service(Trigger, "~/auto_stop", self._srv_auto_stop)
        # 양팔 동시동작 제어: 기본=단일팔(뮤텍스로 충돌방지). /allow_dual_arm True(UI 체크박스)면 동시 허용
        self._allow_dual = False
        self.node.create_subscription(Bool, "/allow_dual_arm", self._on_dual, 10)
        # 락 파일은 /tmp 쓰기 실패 시 '조용히 진행' 금지 — 모션 안전상 기동을 중단한다.
        # (경로 상호배타: resident 는 모션 동안 SH 공유락, C++ execute 는 EX 배타락 — 별개 파일.)
        try:
            self._lock_fd = open(ARM_LOCK_PATH, "w")
            self._path_fd = open(MOTION_PATH_LOCK, "w")
        except OSError as e:
            raise RuntimeError(f"락 파일 생성 실패({e}) — /tmp 쓰기 권한 확인") from e
        self._have_lock = False
        self._have_path = False
        self._auto = False
        self._pick_req = False     # 콜백은 플래그만; 실제 흐름은 메인 루프(중첩 스핀 방지)
        self._busy = False
        self._ok = 0
        self._ng = 0
        self._last_motion_end = None   # 마지막 모션 종료 시각(sense-plan-act 재검출 핸드셰이크)
        self._fault_streak = 0         # auto 연속 고장 카운트(진전 watchdog)
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
            except Exception as e:                  # 한 픽의 예외가 resident 를 죽이지 않도록
                import traceback
                self._pub_status(f"pick 예외(생존): {e}")
                traceback.print_exc()
            finally:
                self._busy = False
        elif self._auto:
            self._busy = True
            try:
                # dual 모드는 양팔 독립 픽이라 sense-plan-act 재검출 핸드셰이크를 끔(직렬화 방지).
                # 단일팔 모드만 freshness 강제(깨끗한 sense-plan-act). staleness 가드는 양쪽 다 적용.
                r = self._do_one(self._color(), require_fresh=not self._allow_dual)
                self._watch_auto(r)
            except Exception as e:
                import traceback
                self._watch_auto({"ok": False, "info": "exception"})
                self._pub_status(f"auto pick 예외(생존): {e}")
                traceback.print_exc()
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

    def _acquire_path_lock(self):
        """경로 SH(공유) 락 비차단 획득. C++ Hover(EX) 진행 중이면 False(이번 틱 양보)."""
        try:
            fcntl.flock(self._path_fd, fcntl.LOCK_SH | fcntl.LOCK_NB)
            self._have_path = True
            return True
        except (BlockingIOError, OSError):
            return False

    def _release_path_lock(self):
        if self._have_path:
            try:
                fcntl.flock(self._path_fd, fcntl.LOCK_UN)
            except OSError:
                pass
            self._have_path = False

    def _color(self):
        return self._color_val or "mini-box-red"

    def _do_one(self, color, require_fresh=False):
        # 검출/이동 분리: 검출은 box_detect_loop 노드가 연속 수행 -> 여기선 트리거 없이
        # 최신 /detected_boxes(=self.node.box) 만 소비한다. 최신 반영 위해 짧게 스핀.
        for _ in range(3):
            rclpy.spin_once(self.node, timeout_sec=0.02)
        box = self.node.box
        if box is None:
            self._pub_status("no_box")
            return {"ok": False, "info": "no_box"}
        # [3] staleness 가드: 검출이 끊겼는데(box_detect_loop 죽음 등) 과거 박스로 무한 픽 방지.
        stamp = self.node.box_stamp
        if stamp is not None:
            age = (self.node.get_clock().now() - stamp).nanoseconds / 1e9
            if age > AUTO_BOX_MAX_AGE:
                self._pub_status(f"stale box ({age:.1f}s) — 검출 끊김?")
                return {"ok": False, "info": "stale_box"}
        # [3] sense-plan-act 핸드셰이크(auto): 직전 모션 이후의 '새' 검출이 와야 다음 픽.
        if require_fresh and self._last_motion_end is not None and (
                stamp is None or stamp <= self._last_motion_end):
            self._pub_status("재검출 대기(모션 후 새 검출 전)")
            return {"ok": False, "info": "await_fresh"}
        # 모션(run_pick)만 뮤텍스 — 타팔 모션 중이면 양보(다음 틱 즉시 재시도, 검출 트리거 없음).
        if not self._allow_dual and not self._acquire_arm_lock():
            self._pub_status("대기(타팔 동작중)")
            return {"ok": False, "info": "busy_other_arm"}
        # 경로 상호배타(SH): C++ Hover(디버그)가 EX 락 보유 중이면 양보(컨트롤러 충돌 방지).
        if not self._acquire_path_lock():
            self._release_arm_lock()
            self._pub_status("대기(Hover 디버그 동작중)")
            return {"ok": False, "info": "busy_cpp_hover"}
        try:
            self.node.box = box                     # 잠금 시점 박스로 고정해 픽
            self._pub_status(f"picking {core.SIDE} box=({box[0]:.2f},{box[1]:+.2f})")
            r = run_pick(self.node, run=True)
            if r.get("gripped"):
                self._ok += 1; self._pub_status(f"ok box={r.get('box')} [성공 {self._ok}]")
            elif r["ok"]:
                self._ng += 1; self._pub_status(f"empty box={r.get('box')} [빈손 {self._ng}]")
            else:
                self._pub_status(r["info"])
            return r
        finally:
            self._release_path_lock()
            self._release_arm_lock()
            self._last_motion_end = self.node.get_clock().now()  # 재검출 핸드셰이크 기준

    def _watch_auto(self, r):
        """auto 진전 감시: 연속 '고장'(도달불가/SAFE-ABORT/switch 실패/예외) 시 auto 자동정지.
        no_box/busy_*/await_fresh/stale_box = 정상 대기(고장 아님) → streak 리셋.
        (stale_box 는 검출 게이팅·간헐 검출로 정상 발생 → 고장 집계 금지: 안 움직이는 팔 오정지 방지.)"""
        info = (r or {}).get("info", "")
        faults = ("unreachable", "SAFE-ABORT", "switch_fail", "exception")
        if any(k in info for k in faults):
            self._fault_streak += 1
            if self._fault_streak >= AUTO_FAULT_LIMIT:
                self._auto = False
                self._fault_streak = 0
                self._pub_status(f"AUTO 자동정지: 연속 고장 {AUTO_FAULT_LIMIT}회 (마지막: {info})")
        else:
            self._fault_streak = 0

    def _srv_pick_once(self, req, resp):
        # 콜백은 플래그만(중첩 스핀 방지). 결과는 ~/status 토픽으로.
        self._pick_req = True
        resp.success = True; resp.message = "queued"; return resp

    def _srv_auto_start(self, req, resp):
        self._auto = True; self._pub_status("AUTO 시작")
        resp.success = True; resp.message = "auto on"; return resp

    def _srv_auto_stop(self, req, resp):
        # soft-stop: self._auto=False 만 세팅 → 현재 진행 중인 1픽은 끝까지 가고 다음 사이클부터 정지.
        # (즉시정지=현재위치 hold 는 executor 분리 필요, 별도 트랙.)
        self._auto = False; self._fault_streak = 0
        self._pub_status(f"AUTO 정지 요청 — 현재 동작 완료 후 정지 (성공 {self._ok}/빈손 {self._ng})")
        resp.success = True; resp.message = "auto off (current pick finishes)"; return resp


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
