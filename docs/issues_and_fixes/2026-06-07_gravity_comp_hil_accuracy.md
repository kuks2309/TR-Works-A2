# 2026-06-07 — 중력 보상 HIL 위치정확도 검증 + g_scale 최적화 + effort 컨트롤러 언로드 결함

## 목적
실로봇에서 **중력 보상(gravity compensation)이 위치 오차를 줄이는지** 정량 검증하고
보상 계수 `g_scale` 최적값을 결정한다.

## 방법 (HIL A/B)
- 좌팔 TCP(Tool Center Point)를 body_link0 기준 **3×3 격자**(중심 (0.20,0.15), ±0.05m,
  z=0.75, 자세 top-down RPY(180,0,0))로 이동.
- 이동: 검증된 DLS IK(Inverse Kinematics)(pinocchio `buildReducedModel` + per-step
  관절한계 clamp, ptp 백엔드와 동일, sanity (0.20,0.20,0.72)=0.04mm) → q\* 산출 →
  JTC(Joint Trajectory Controller) `FollowJointTrajectory` action.
- 측정: 정착 후 **실제 관절 엔코더 → FK(Forward Kinematics)**(+ tf2 교차검증, 일치)로
  실제 TCP(Tool Center Point) 산출 → 명령 좌표와의 위치오차(mm).
- A/B: 각 점에서 보상 ON vs OFF(`gravity_comp_node` 의 `enable_compensation` 파라미터
  토글, persistent 서비스 클라이언트 — 노드 churn 없음). **9점 × 10회**.
- z 결정: (0.20,±0.15)에서 top-down 유지는 **z=0.72~0.78만 가능, z≥0.80 IK 미수렴**
  (그 높이에선 top-down 자세 물리적 불가) → 중앙 **z=0.75** 채택. 사용자 언급 0.85 불가.
- 스크립트: `/tmp/grav_hil_test.py` (mode: verify/run/diag/gsweep).

## 결과 1 — 중력 보상은 오차를 크게 줄인다 (좌팔, g=1.0, 90회)
- **전체 평균: OFF 43.0mm → ON 9.0mm (79% 감소).** 모든 점에서 감소, 반복도 std<0.3mm.
- OFF 오차는 리치(X)에 비례해 커짐(X=0.15→0.25: 20→63mm = 모멘트암↑). ON은 리치 무관하게
  작게 유지 → **부하 의존 처짐을 제거**. 효과는 고부하(먼 리치)에서 최대(감소 ~59mm,
  (0.25,0.20)에서 61.5→2.1mm, 96%↓).

### 우팔 (J4 모터 수리 후, g=0.95, 90회)
J4 모터 수리 후 우팔(중심 (0.20,−0.15)) 동일 테스트:
- **전체 평균: OFF 44.3mm → ON 9.4mm (79% 감소)** — 좌팔과 동일 수준.
- **J4 처짐 ON 범위 −0.43~+0.23°(전 90회), 이상치·진동·폭주 없음** → J4 수리 검증 완료.
  (보상 OFF 시 J4 −4.3~−4.5° 처짐 → ON 시 ≈0° 보상.) (rosbag `grav_hil_right_g095_*`)

## 결과 2 — g_scale 최적화: 키우면 악화, 최적은 pose-dependent
`g_scale` 스윕 (중심 + 고부하점, 런타임 파라미터):

| g_scale | (0.20,0.15) ON | (0.20,0.10) ON | j4 토크 | j4 처짐 |
|---|---|---|---|---|
| 0.80 | 4.4 | 12.9 | 3.2 | −1.0° |
| **0.90** | **3.6** | 10.0 | 3.7 | −0.9° |
| **0.95** | 5.3 | **8.9** | 3.9 | −0.7° |
| 1.00 | 10.4 | 11.6 | 4.1 | +0.5° |
| 1.10~1.40 | 14→34 | 15→28 | 4.5→5.9 | +0.7~1.5° |

- **g_scale ↑(>1.0): 일관되게 악화** — 과보상으로 팔이 목표보다 위로 오버슈트.
- 최적은 **1.0보다 아래**. 모델 g(q)가 실제 중력을 약간 과대평가(또는 JTC 위치루프가 일부
  분담)해서 g=1.0이면 살짝 과보상.

격자 전역 재검증 (좌팔, g=0.90 vs g=1.0, 각 90회, ON 위치오차 mm):

| 점 | g=1.0 | g=0.90 |
|---|---|---|
| (0.15,+0.15) | 15.1 | **6.2** |
| (0.20,+0.15) 중심 | 10.4 | **3.8** |
| (0.20,+0.20) | 5.8 | 9.7 |
| (0.25,+0.20) | 2.1 | 5.7 |
| **평균** | **8.98** | **8.04** (~10%↓) |

- g=0.90은 **평균 ~10% 개선**이나 **균일하지 않음**(4점 개선/5점 악화). g=1.0에서 과보상이던
  점(중심, (0.15,0.15))은 크게 개선, 이미 잘 맞던 점(먼 +0.20Y)은 부족보상으로 악화.
- **최적 g_scale이 자세마다 다름** → 단일 전역 스케일엔 한계. 잔존오차(~5-13mm)의 본질은
  자세별 모델 g(q) 오차 + 마찰. 진짜 균일 개선엔 자세별/마찰 보정 필요.
- (OFF 평균 43.0 vs 42.2 — 보상 끄면 g_scale 무관, sanity 일치.)

### 채택: **g_scale = 0.95** (평균 절충)
- 반영: `gravity_comp.launch.py` default 1.0→0.95, Launch Manager 체크박스 preset
  `g_scale:=1.0→0.95`, 런타임 `ros2 param set` 적용.
- 참고: 픽 대상이 주로 먼 리치(X=0.25)면 그 영역은 g=1.0이 더 정확(2-5mm)하므로,
  운용 영역에 따라 재튜닝 여지 있음. (bringup `openarmx.bimanual.launch.py`는 별도로
  g_scale=1.05 하드코딩 + `enable_forward_effort:=false` 게이트 — 현재 미사용 경로.)

## 발견된 결함 — effort 컨트롤러가 조용히 언로드되는 launch 취약점 (HIGH)
검증 초반 **보상 ON/OFF가 처짐에 0.01°도 차이 없음**(joint4 −4.85° 동일). 진단(diag):
발행 토크는 0↔3.87N·m 정확히 토글되는데 **joint4 위치 불변** → 피드포워드가 모터에 안 닿음.

- 원인: `forward_effort_controller`가 **언로드 상태**(`joint4/effort [unclaimed]`, 컨트롤러
  목록에 없음). `gravity_comp_node`는 토픽에 발행하지만 받아서 effort 인터페이스
  (`tau_commands_` → MIT `param.torque`, v10_simple_hardware.cpp:588)로 넘길 컨트롤러가 없음.
- 경위: `gravity_comp.launch.py`의 effort spawner가 **`--unload-on-kill`** 인데, spawner
  프로세스가 죽으면(예: 초기 CM 미준비 타임아웃, 부분 종료) `--unload-on-kill`이 컨트롤러를
  언로드. 그러면 **launch 부모·gravity_comp_node는 살아있는 채로 컨트롤러만 사라진
  "반쪽 상태"** → 보상이 조용히 무력화(증상 없이). "아까는 됐는데 지금 안 됨"의 정체.
- 임시 조치: effort 컨트롤러를 `--unload-on-kill` 없이 재스폰 → `joint4/effort [claimed]`
  복구 → 보상 즉시 작동(중심점 OFF 39mm → ON 5.6mm 재확인).
- **수정 완료**: `gravity_comp.launch.py`의 spawner에서 `--unload-on-kill` 제거 → spawner는
  컨트롤러 활성화 후 종료하고 컨트롤러는 **잔존**(spawner/launch가 죽어도 언로드 안 됨).
  OFF 클린정지는 `enable_compensation=false`(0 발행)가 보장(컨트롤러 잔존하되 0 출력 →
  잔여 토크 없음). `launch_manager_tab.py` OFF 핸들러·preset 주석도 갱신.

## 데이터/자산
- rosbag: `experiments/rosbags/grav_hil_left_072713` (g=1.0). g=0.90 rosbag은 SQLite
  `database is locked`로 실패(스크립트 출력에 측정값 전량 보존되어 손실 없음).
- 스크립트: `/tmp/grav_hil_test.py`, `/tmp/ik_reach.py`(read-only 도달성 검증).

## 완료/미완
- 우팔(0.20,−0.15) HIL 테스트 — **완료** (J4 수리 후, 결과 1 우팔 절 참조. OFF44→ON9mm, J4 정상).
- launch `--unload-on-kill` 취약점 코드 수정 — **완료** (커밋 b3bfcb0).
