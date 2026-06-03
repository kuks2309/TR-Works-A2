# 검증 스크립트 및 검증 현황

분석일: 2026-06-03
패키지: openarmx_pick

## 분석 범위

이 문서는 `openarmx_pick` 패키지에 포함된 3개의 독립 검증 스크립트(`verify_solver.py`, `verify_grasp.py`, `verify_e2e.py`)의 동작 원리, 측정 내용, 실행 방법, 공식 기록된 PASS 결과를 분석하고, 추가로 live camera(실 카메라) 검증 현황과 테스트 커버리지의 공백을 정리한다. 소스 코드 수정은 일절 수행하지 않으며, 모든 진술은 코드 또는 README에서 직접 확인한 사실에 근거한다.

---

## 1. 검증 구조 개요

검증은 파이프라인 단계에 따라 세 계층으로 분리된다.

| 스크립트 | Stage | 검증 대상 | 외부 의존성 |
|---|---|---|---|
| `scripts/verify_solver.py` | A | QP(Quadratic Program, 이차계획법)+CBF(Control Barrier Function, 제어 장벽 함수) MoveL 솔버 | solver launch 필요, 실 로봇 불필요 |
| `scripts/verify_grasp.py` | B | grasp_pose_node 의 파지 자세 합성 | grasp_pose_node 실행 필요, 카메라 불필요 |
| `scripts/verify_e2e.py` | E2E | 박스 검출→파지 자세→MoveL→솔버→EE(End-Effector, 엔드이펙터) 수렴 전 과정 | solver + grasp_pose_node(auto_send:=true) |

세 스크립트 모두 실제 로봇 하드웨어 없이 실행 가능하며, 합성 데이터(synthetic data)로 ROS2 토픽을 직접 퍼블리시하여 각 컴포넌트를 구동한다.

---

## 2. Stage A: 솔버 검증 — `verify_solver.py`

### 2.1 동작 원리

`Verifier` 노드는 두 가지 역할을 동시에 수행한다.

**시뮬레이션된 로봇 상태 피드백**

100Hz(타이머 주기 0.01 s)로 `/joint_states`를 퍼블리시한다. 초기 상태는 7개 왼팔 관절 모두 0.0 rad(home)이다. 솔버가 `/openarmx/left_arm/joint_trajectory`로 명령을 내릴 때마다 `_on_cmd` 콜백이 그 첫 번째 point의 position을 현재 측정 상태 `self.q`에 덮어씌운다(`scripts/verify_solver.py:49-56`). 이로써 "명령 → 측정 → 재명령" 폐루프(closed-loop)가 pure-simulation으로 형성된다.

**목표 자세 발송**

1 s 타이머(`_send_goal_once`)가 최초 1회만 MoveL 메시지를 퍼블리시한다(`scripts/verify_solver.py:62-77`). 목표 위치는 `openarmx_body_link0` 프레임 기준 (x=0.10, y=0.15, z=0.00), orientation은 identity(w=1.0)이며 `time_from_start.sec = 3`이다.

### 2.2 측정 항목

- **관절 명령 수렴 추이**: 50 command 주기마다(`_cmd_count % 50 == 0`) 현재 관절각 벡터를 로그로 출력한다(`scripts/verify_solver.py:58-60`). 수렴 여부는 로그를 시각적으로 확인하며, 자동 수치 assert는 없다.
- **joint-limit CBF 동작**: 관절이 한계(limit)에 도달하면 솔버의 CBF 제약이 그 관절을 클램프(clamp)해야 한다. README `Verification status`에서 "joints clamp exactly at limits → joint-limit CBF confirmed"로 기술되어 있으나, 스크립트 자체에 자동화된 limit-clamp assert 코드는 없다. 사용자가 로그에서 직접 확인하는 방식이다.

### 2.3 실행 방법

```bash
# 1) solver launch (cyclo_ws + openarmx_ws overlay 소싱 후)
ros2 launch openarmx_pick openarmx_movel.launch.py

# 2) 별도 터미널에서
python3 scripts/verify_solver.py
```

스크립트 상단 docstring(`scripts/verify_solver.py:8-11`)에 위 순서가 명시되어 있다. `KeyboardInterrupt`로 종료한다.

### 2.4 공식 PASS 결과

README `Verification status (2026-05-31)`:

> Stage A (solver port) — PASS. MoveL goal → QP drives `joint_command` to an IK solution; joints clamp exactly at limits → joint-limit CBF confirmed.

수치 오차 기준은 README에 명시되지 않았다. PASS는 "관절각이 IK 해(解)를 향해 단조롭게 변화하고 한계에서 클램프된다"는 정성적 판단이다.

---

## 3. Stage B: 파지 자세 검증 — `verify_grasp.py`

### 3.1 합성 박스 클라우드 생성

`make_cloud` 함수가 `openarmx_body_link0` 프레임에서 평탄한 박스 상면을 모사하는 `PointCloud2`를 생성한다(`scripts/verify_grasp.py:29-49`).

| 파라미터 | 값 |
|---|---|
| 중심 (CX, CY, CZ) | (0.50, 0.10, 0.20) m |
| 장축(long, +Y 방향) 길이 | 0.16 m |
| 단축(short, +X 방향) 길이 | 0.08 m |
| 포인트 간격 | 0.004 m (장·단축 모두) |

클라우드를 `/box_plane/cloud`에 0.2 s 주기로 퍼블리시하며(`scripts/verify_grasp.py:57`), `grasp_pose_node`가 `/openarmx/grasp_pose`를 발행하면 `_on_pose` 콜백이 첫 번째 응답만 처리한다(`scripts/verify_grasp.py:66-68`).

합성 클라우드가 이미 `openarmx_body_link0` 프레임 기준이므로, `grasp_pose_node` 내부의 TF(Transform, 좌표 변환) 조회는 identity 변환을 반환한다. 이는 TF 체인 자체를 검증 범위 밖에 두는 설계 선택이다.

### 3.2 측정 항목 및 assert 조건

`_on_pose` 콜백에서 세 개의 boolean 조건을 직접 계산하고 종합 판정을 출력한다(`scripts/verify_grasp.py:65-91`).

| 검사 | 계산 방법 | PASS 조건 |
|---|---|---|
| `ok_pos` | 파지 위치의 x, y 편차 | `abs(x - 0.50) < 0.02` **and** `abs(y - 0.10) < 0.02` |
| `ok_down` | approach 축(tool +z의 base frame 방향) z 성분 | `approach[2] < -0.9` (거의 수직 하강) |
| `ok_open` | opening 축 x 성분 절댓값 | `abs(opening[0]) > 0.9` (단축 방향 정렬) |

approach 축과 opening 축은 quaternion의 회전행렬 열벡터로부터 직접 계산한다(`scripts/verify_grasp.py:70-73`). 판정 문자열은 `"PASS"` 또는 `"CHECK"`이며, 세 조건이 모두 참일 때만 PASS이다.

**예상 파지 자세 수치** (docstring 기준):
- position: `(+0.500, +0.100, +0.195)` — 중심(z=0.20)에서 `grasp_depth`(0.005 m)만큼 아래
- approach axis: `(0, 0, -1)` — 수직 하강
- opening axis: `(+/-1, 0, 0)` — 박스 단축(+X)

### 3.3 실행 방법

```bash
# grasp_pose_node를 먼저 기동 (auto_send:=false 권장)
ros2 run openarmx_pick grasp_pose_node --ros-args -p auto_send:=false

# 별도 터미널에서
python3 scripts/verify_grasp.py
```

`KeyboardInterrupt`로 종료하며, `rclpy.shutdown()` 시 예외를 무시하는 방어 코드가 포함되어 있다(`scripts/verify_grasp.py:103-106`).

### 3.4 공식 PASS 결과

README:

> Stage B (grasp synthesis) — PASS. Synthetic box → grasp pos err ≈ 2 mm, approach `(0,0,-1)`, opening on the box short axis.

위치 오차 약 2 mm, approach 벡터 (0,0,-1), opening 벡터가 박스 단축에 정렬됨이 확인되었다.

---

## 4. End-to-End 검증 — `verify_e2e.py`

### 4.1 동작 원리

`E2E` 노드는 Stage A와 Stage B의 시뮬레이션을 결합한다. 세 개의 주기 작업을 동시에 수행한다.

- **`_js_tick` (100 Hz)**: `/joint_states` 퍼블리시. 초기 home 상태(7관절 모두 0.0)에서 시작하며, 솔버 명령이 오면 `_on_cmd`가 즉시 `self.q`를 갱신한다(`scripts/verify_e2e.py:61-75`).
- **`_cloud_tick` (5 Hz)**: 합성 박스 클라우드를 `/box_plane/cloud`에 반복 퍼블리시한다. E2E 박스 중심은 (CX=0.28, CY=0.14, CZ=0.10), 단축 0.08 m, 장축 0.16 m이다(`scripts/verify_e2e.py:26,31-32`).
- **`_report` (1 Hz)**: EE 현재 위치와 pre-grasp 목표 위치의 오차를 매 초 출력한다.

### 4.2 측정 항목 및 PASS 조건

`_report` 함수는 두 가지 오차를 계산한다(`scripts/verify_e2e.py:91-98`).

| 오차 | 계산 | PASS 조건 |
|---|---|---|
| 3D 유클리드 거리 `err` | `‖ee_xyz − pregrasp_xyz‖` | (로그 표시만, assert 없음) |
| XY 평면 거리 `exy` | `‖ee_xy − pregrasp_xy‖` | `exy < 0.03 m` (30 mm) → PASS |

pre-grasp 목표는 `grasp_pose_node`가 내보내는 `/openarmx/grasp_pose` 위치에서 z 방향으로 `PRE_H = 0.10 m`을 더한 값이다(`scripts/verify_e2e.py:81-83`). `auto_send:=true` 모드에서 `grasp_pose_node`가 pre-grasp MoveL을 자동으로 솔버에 발송하기 때문에, 이 계산이 실제 명령 목표와 일치한다.

PASS 판정은 EE의 XY 수렴에만 의존한다. z축 수렴(하강 정확도)은 수치로 측정되지만 PASS 조건에는 포함되지 않는다.

### 4.3 실행 방법

```bash
# solver + grasp_pose_node (auto_send:=true)
ros2 launch openarmx_pick openarmx_pick.launch.py auto_send:=true

# 별도 터미널에서
python3 scripts/verify_e2e.py
```

README 상단 docstring에 동일한 실행 순서가 명시되어 있다(`scripts/verify_e2e.py:4-5`).

### 4.4 공식 PASS 결과

README:

> End-to-end (sim) — PASS. EE converges within ≈ 6–12 mm of the pre-grasp.

XY 수렴 판정 임계값은 30 mm이지만, 실제 관측된 수렴 오차는 6–12 mm로 훨씬 타이트하다. 3D 오차는 z축 잔류 오차가 포함되어 이보다 클 수 있다.

---

## 5. Live Camera 검증 현황

README `Verification status (2026-05-31)`:

> Live camera — PASS. Real D435 + cardboard box (top at body z ≈ 0.204 m, height 17.2 cm) → stable top-down grasp pose in the base frame. Robot not moved.

실제 Intel RealSense D435 카메라와 골판지 박스를 이용한 검증이 수행되었다. 박스 상면 body z ≈ 0.204 m, 박스 높이 17.2 cm 조건에서 `grasp_pose_node`가 기반 프레임에서 안정적인 top-down 파지 자세를 출력함을 확인하였다.

단, "Robot not moved"라고 명시된 바와 같이 이 검증은 파지 자세 계산까지만이며, 실제 팔 동작(MoveL 명령 실행 → 관절 구동 → 물체 파지)은 수행되지 않았다.

---

## 6. 스크립트별 assert 방식 비교

| 스크립트 | 자동화된 수치 assert | 출력 형식 | 종료 트리거 |
|---|---|---|---|
| `verify_solver.py` | 없음 | 50회마다 관절각 로그 | Ctrl-C |
| `verify_grasp.py` | 있음 (`ok_pos`, `ok_down`, `ok_open` 3개) | 첫 응답 수신 후 `PASS`/`CHECK` 출력 | Ctrl-C (첫 판정 후 계속 실행) |
| `verify_e2e.py` | 있음 (`exy < 0.03`) | 1 Hz 주기 오차 + 조건 충족 시 `PASS` 출력 | Ctrl-C |

`verify_solver.py`만 자동 판정이 없으며, 사용자가 로그를 읽고 수렴 여부를 직접 판단해야 한다.

---

## 7. 테스트 커버리지 공백

### 7.1 Joint-limit CBF 자동 검증 부재

Stage A 스크립트는 관절 한계 클램프를 자동으로 assert하지 않는다. 관절이 실제로 limit에서 멈추는지는 로그를 육안으로 확인해야 한다. 수치 기준(예: `abs(q[i] - limit[i]) < tol`)을 갖춘 자동 assert가 없다.

### 7.2 z축 수렴 미검증 (E2E)

`verify_e2e.py`의 PASS 조건은 XY 평면 거리만 사용한다(`exy < 0.03`). z축 오차는 로그에 표시되지만 독립적인 PASS 기준이 없다. pre-grasp 높이 정확도(z 방향)는 실제 파지에서 중요한 인자인데, 현재 검증에서는 정량적으로 판정되지 않는다.

### 7.3 TF 변환 경로 검증 없음

`verify_grasp.py`는 합성 클라우드를 이미 `openarmx_body_link0` 프레임에서 생성하므로, `grasp_pose_node`의 `tf_buffer.lookup_transform` 경로(카메라 프레임 → base 프레임)가 실제로 통과되지 않는다. 카메라 보정(extrinsic calibration) 결과가 포함된 실제 TF 체인의 정확도는 live camera 검증에서만 간접적으로 확인된다.

### 7.4 다중 박스 후보 선택 미검증

README "Not yet done"에 명시된 바와 같이 `box_plane` 노드는 최대 3개의 상면 후보를 동시에 발행할 수 있다. 현재 검증 스크립트는 단일 박스 클라우드만 사용하므로, 다중 후보가 존재할 때 `grasp_pose_node`가 파지 자세를 어떻게 선택(또는 혼동)하는지 검증하지 않는다.

### 7.5 실제 관절 구동 통합 미검증

Live camera PASS는 파지 자세 계산에만 해당한다. `joint_command` → `forward_position_controller` → 실 팔 구동 → 물체 파지의 전 과정은 아직 검증되지 않았다. README "Not yet done" 4번 항목에서 "wire `joint_command` to the OpenArmX `forward_position_controller`"를 차기 작업으로 명시한다.

### 7.6 Pick FSM(Finite State Machine, 유한 상태 기계) 미구현

현재 검증은 pre-grasp 호버(hover) 위치까지만 다룬다. 하강(descend) → 그리퍼 닫기 → 들어올리기(lift) 시퀀스를 포함하는 전체 pick FSM은 구현 및 검증이 완료되지 않았다(README "Not yet done" 2번 항목).

### 7.7 충돌 CBF 미검증

Stage-1 URDF에서 충돌 메시(collision mesh)가 제거된 상태이므로, 자기충돌 회피(self-collision avoidance) CBF 제약은 검증 범위 밖이다. Stage-2 충돌 CBF는 URDF 재생성 및 SRDF(Semantic Robot Description Format, 시맨틱 로봇 기술 형식) 추가 후 별도 검증이 필요하다.

---

## 8. 요약 표

| 항목 | 상태 | 비고 |
|---|---|---|
| Stage A: QP 솔버 MoveL 수렴 | PASS (2026-05-31) | 정성적 로그 확인, 수치 assert 없음 |
| Stage A: Joint-limit CBF 클램프 | PASS (2026-05-31) | 로그 육안 확인, 자동 assert 없음 |
| Stage B: 합성 박스 파지 위치 오차 | PASS ≈ 2 mm (2026-05-31) | `ok_pos` 임계값 20 mm, 자동 assert 있음 |
| Stage B: approach 방향 (0,0,-1) | PASS (2026-05-31) | `ok_down`: z < -0.9, 자동 assert 있음 |
| Stage B: opening 방향 (단축 정렬) | PASS (2026-05-31) | `ok_open`: \|x\| > 0.9, 자동 assert 있음 |
| E2E: XY 수렴 ≈ 6–12 mm | PASS (2026-05-31) | 임계값 30 mm, 자동 assert 있음 |
| E2E: Z 수렴 | 측정만, 판정 없음 | 별도 assert 기준 미설정 |
| Live camera 파지 자세 | PASS (2026-05-31) | 팔 동작 없이 자세 계산만 확인 |
| 실 팔 구동 통합 | 미완료 | `forward_position_controller` 연결 필요 |
| Pick FSM (하강-파지-들기) | 미완료 | 구현 미착수 |
| Stage-2 충돌 CBF | 미완료 | URDF 재생성 + SRDF 필요 |
| 다중 박스 후보 선택 | 미완료 | 파지 자세 점프 문제 미해결 |
