# cyclo_control 종합 분석 리포트

> **작성일**: 2026-05-14
> **대상 저장소**: [ROBOTIS-GIT/cyclo_control](https://github.com/ROBOTIS-GIT/cyclo_control)
> **클론 위치**: `/home/openarmx/TR-Works/kkw/China/cyclo_control`
> **분석 범위**: 패키지 6개, ROS 노드 10개, 코어 컨트롤러 5개, 리타게팅 모듈 3개

---

## Executive Summary

ROBOTIS Physical AI 라인업(AI Worker 양팔, OpenMANIPULATOR-X, OpenMANIPULATOR-Y)을 한 코드베이스로 제어하는 ROS 2 Jazzy 모션 컨트롤러 스택. 5가지 텔레옵 흐름(MoveL / MoveJ / VR / Leader / 인터랙티브 마커)을 **단일 QP 추상화** 위에서 통합하며, 모든 명령은 결국 다음 형태의 QP로 귀결된다:

```
min  ½·q̇ᵀ P q̇ + qᵀ q̇ + ρ·Σsᵢ        (P = Jᵀ·W·J 또는 W)
 s.t.  q̇_lb  ≤  q̇  ≤  q̇_ub
       q̇ ≥ −α(q − q_min) − s_qmin     ── 관절 하한 CBF
       q̇ ≤  α(q_max − q) + s_qmax     ── 관절 상한 CBF
       ∇dᵢᵀ·q̇ + sᵢ ≥ −α(dᵢ − d_safe)   ── 충돌쌍 i별 CBF (활성 시)
       sᵢ ≥ 0
```

- **Pinocchio**: FK, Jacobian, 충돌쌍 거리·그래디언트(자체 미분식)
- **OSQP-Eigen**: 위 QP를 100 Hz로 풀이, 희소 패턴 캐시로 재초기화 회피
- **CBF + 슬랙 변수**: 한계·충돌을 *연성*으로 풀어 infeasible 방지하면서 사실상 강제

이 리포트는 위 아키텍처를 (1) 시스템 토폴로지, (2) 핵심 라이브러리, (3) ROS 노드 10종, (4) 텔레옵 파이프라인, (5) 안전 메커니즘, (6) 평가·한계의 6단계로 분해한다.

---

## 1. 시스템 토폴로지

### 1.1 패키지 의존성

```
                    ┌────────────────────────────┐
                    │    osqp_eigen_vendor       │ (벤더된 osqp-eigen)
                    └────────────────────────────┘
                                  ▲
                                  │
                    ┌────────────────────────────┐
                    │  cyclo_motion_controller_  │
                    │           core              │ ── Pinocchio + Eigen + OSQP
                    │  ─────────────────────────  │
                    │  KinematicsSolver           │
                    │  QPBase (추상)              │
                    │  VRController               │
                    │  AIWorkerMoveJController    │
                    │  OpenManipulatorMoveL/JCtl  │
                    │  retargeting/* (Python)     │
                    └────────────────────────────┘
                            ▲                ▲
                            │                │
                            │                ├──── cyclo_motion_controller_models (URDF/SRDF)
                            │                │
   ┌─────────────────────┐  │  ┌──────────────────────────────┐
   │ cyclo_motion_       │  │  │ cyclo_motion_controller_     │
   │ controller_ros      │──┘  │ ros_py                        │
   │  (C++ 노드 10개)     │     │  (arm/hand retargeting)       │
   └─────────────────────┘     └──────────────────────────────┘
            ▲
            │
            └── robotis_interfaces (MoveL.msg 등; vcs로 가져옴)
```

### 1.2 코어 → ROS 노드 매핑 (`cyclo_motion_controller_ros/CMakeLists.txt`)

총 10개 C++ 실행 파일:

| Executable | 코어 백엔드 | 소스 |
|---|---|---|
| `ai_worker_movel_controller_node` | `controllers::AIWorkerMoveLController` (=`VRController`) | `src/nodes/ai_worker/ai_worker_movel_controller_node.cpp` |
| `ai_worker_movej_controller_node` | `controllers::AIWorkerMoveJController` | `src/nodes/ai_worker/ai_worker_movej_controller_node.cpp` |
| `vr_controller_node` | `controllers::VRController` | `src/nodes/ai_worker/vr_controller_node.cpp` |
| `leader_controller_node` | KinematicsSolver만 (FK 전용) | `src/nodes/ai_worker/leader_controller_node.cpp` |
| `omx_movel_controller_node` | `controllers::OpenManipulatorMoveLController` | `src/nodes/omx/omx_movel_controller_node.cpp` |
| `omx_movej_controller_node` | `controllers::OpenManipulatorMoveJController` | `src/nodes/omx/omx_movej_controller_node.cpp` |
| `omy_movel_controller_node` | `controllers::OpenManipulatorMoveLController` | `src/nodes/omy/omy_movel_controller_node.cpp` |
| `omy_movej_controller_node` | `controllers::OpenManipulatorMoveJController` | `src/nodes/omy/omy_movej_controller_node.cpp` |
| `interactive_marker_node` | (FK + RViz Interactive Marker) | `src/utils/eef_interactive_marker_node.cpp` |
| `reference_checker_node` | (점프 감지) | `src/utils/reference_checker_node.cpp` |

OMX/OMY는 **같은 코어 컨트롤러를 URDF만 바꿔서** 재사용한다 — 두 패키지에서 컨트롤러 코드가 동일하다는 뜻.

---

## 2. 핵심 라이브러리 분석

### 2.1 `KinematicsSolver` (`cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp`)

Pinocchio의 얇은 래퍼로, **충돌쌍별 거리 그래디언트 계산**이 가장 비싼 부분이자 차별점.

#### 생성자 (`:34-120`)
```
1. urdf 파일 존재 확인 → pinocchio::urdf::buildModel
2. pinocchio::COLLISION 지오메트리 로딩 → addAllCollisionPairs()
3. SRDF 제공 시 → removeCollisionPairs (자체 충돌 제외쌍 적용)
4. dof_ = model_.nq
5. q_lb_, q_ub_, qdot_lb_=-qdot_ub_  ← URDF 한계
6. frames 순회 → BODY(링크)/JOINT 분류, root_link 결정
```

> SRDF 없으면 *모든 충돌쌍* 활성 → 솔버 부담 폭증. `disable_gripper_collisions:=true` 같은 옵션은 modified SRDF 로 전환하는 방식.

#### `updateState(q, qdot)` (`:126-156`)
```cpp
forwardKinematics(model, data, q, qdot);
computeJointJacobians(model, data, q);
updateFramePlacements(model, data);
```
이 한 번의 호출로 `data_.oMf[*]` (모든 프레임 placement) 와 Jacobian 캐시가 모두 갱신된다. 이후 `getPose`, `getJacobian` 은 그냥 캐시 조회.

#### `getCollisionPairDistances(with_grad, with_graddot, verbose)` (`:250-362`)

핵심 알고리즘 — **거리 함수의 관절좌표 그래디언트 ∂d/∂q 를 해석적으로 계산**.

거리 함수: `d_i = (p_B - p_A) · n̂`, `n̂ = (p_B - p_A) / ‖p_B - p_A‖`

```
for each collision pair (A, B):
  jointA, jointB ← parent joints
  pA, pB        ← FCL 최근접점 (geom_data_.distanceResults[idx].nearest_points)
  n̂            ← unit(pB - pA)

  J_jointA, J_jointB ← LOCAL_WORLD_ALIGNED 6D Jacobian
  rA = pA - oMi[jointA].translation   // 조인트 원점에서 최근접점까지 변위
  rB = pB - oMi[jointB].translation

  JA = J_top(linear) - skew(rA)·J_bottom(angular)   // 점 속도 Jacobian
  JB = J_top(linear) - skew(rB)·J_bottom(angular)

  ∂d/∂q = n̂ᵀ·(JB - JA)         // 거리의 관절좌표 미분
  if d < 0:  ∂d/∂q *= -1         // 침투 시 부호 보정
```

`with_graddot=true` 옵션은 `getJointJacobianTimeVariation`을 추가로 호출해 `d̈` 항까지 산출 (현재 컨트롤러는 사용하지 않음).

#### 외부 노출 API

```cpp
updateState, computePose, computeJacobian, getPose, getJacobian,
getJointNames, getJointPosition, getJointVelocity, getJointPositionLimit,
getJointVelocityLimit, setJointVelocityBoundsByIndex,
getCollisionPairCount, getCollisionPairDistances,
hasLinkFrame, hasJointFrame, getLinkFrameVector, getJointFrameVector,
getRootLinkName, getDof
```

`setJointVelocityBoundsByIndex(idx, lower, upper)` 는 리프트 같은 패시브 조인트를 **QP 단계에서 자연스럽게 락**하는 데 쓰인다. AI Worker MoveL/VR 노드가 `lift_vel_bound=0` 으로 호출 → QP가 `q̇_lift ∈ [0, 0]` 으로 풀이.

### 2.2 `QPBase` (`include/.../optimization/qp_base.hpp`)

#### 결정 변수 / 제약 레이아웃

```
x = [decision]ⁿˣ
A_total = | I_nbc  · · · |   ← Bound 제약 (대각 항등)
          | A_ineq · · · |   ← 부등식
          | A_eq   · · · |   ← 등식

l_total, u_total 각각 nbc + nineq + neq 행
```

각 컨트롤러는 다음 4개 가상함수만 구현한다:
- `setCost()` — `P_ds_`, `q_ds_`
- `setBoundConstraint()` — `l_bound_ds_`, `u_bound_ds_`
- `setIneqConstraint()` — `A_ineq_ds_`, `l_ineq_ds_`, `u_ineq_ds_`
- `setEqConstraint()` — `A_eq_ds_`, `b_eq_ds_`

#### `solveQP()` 의 희소성 캐시 (`:148-166`)

```cpp
P_sparse = P_ds_.sparseView();
A_sparse = A_ds_.sparseView();

if (!solver_initialized_ || pattern changed) {
    solver_ = OsqpEigen::Solver();   // 새 인스턴스
    initializeSolver(P, A, q, l, u);
} else {
    solver_.updateHessianMatrix(P);
    solver_.updateLinearConstraintsMatrix(A);
    solver_.updateGradient(q);
    solver_.updateBounds(l, u);
}
```

`hasSameSparsityPattern` 은 `outerIndexPtr`/`innerIndexPtr` 만 비교 — 값이 0이 되어도 패턴은 보존. 100Hz 루프에서 매번 부등식 행의 0-패딩 만 바뀌므로 거의 항상 *fast path* (update만) 로 흘러간다.

### 2.3 컨트롤러 계층 비교표

다섯 가지 QP 컨트롤러는 **같은 결정변수 레이아웃**(`q̇, s_qmin, s_qmax, s_sing, s_sel_col`)을 공유한다. **차이는 비용 함수 형태와 트래킹 변수 개수**뿐.

| 컨트롤러 | 비용 함수 트래킹 항 | 트래킹 대상 | 입력 API |
|---|---|---|---|
| `VRController` / `AIWorkerMoveLController` | `Σᵢ ‖Jᵢ·q̇ − ẋᵢ_des‖²_Wᵢ` | 다중 링크 (양 그리퍼 + 양 팔꿈치) | `setDesiredTaskVel(map<link, Vec6>)` |
| `AIWorkerMoveJController` | `‖q̇ − q̇_des‖²_W` | 전체 관절속도 | `setDesiredJointVel(VectorXd)` |
| `OpenManipulatorMoveLController` | `‖J·q̇ − ẋ_des‖²_W` (단일 링크) | 단일 EE | `setDesiredTaskVel(Vec6)` |
| `OpenManipulatorMoveJController` | `‖q̇ − q̇_des‖²_W` | 전체 관절 (단팔) | `setDesiredJointVel(VectorXd)` |

CBF 부등식 코드(`setIneqConstraint`)는 네 클래스에서 사실상 동일 — 관절 위치 한계 + 충돌쌍 거리 한계. 즉 **본질적으로 한 QP의 비용함수 파라미터화 차이**일 뿐, 안전 보장 메커니즘은 같은 코드를 공유한다.

---

## 3. ROS 노드 상세 분석

### 3.1 AI Worker MoveL — `ai_worker_movel_controller_node`

#### 토픽 IO
| 방향 | 토픽 | 타입 | 비고 |
|---|---|---|---|
| Sub | `/joint_states` | `JointState` | 측정 |
| Sub | `/r_goal_move`, `/l_goal_move` | `robotis_interfaces/MoveL` | 작업공간 목표 + duration |
| Pub | `/leader/.../right/joint_trajectory` | `JointTrajectory` | 우팔 명령 |
| Pub | `/leader/.../left/joint_trajectory` | `JointTrajectory` | 좌팔 명령 |
| Pub | `/leader/joystick_controller_right/joint_trajectory` | `JointTrajectory` | 리프트 (옵션) |
| Pub | `/r_gripper_pose`, `/l_gripper_pose` | `PoseStamped` | FK 결과 |
| Pub | `~/controller_error` | `String` | QP 실패 등 |

#### 메인 제어 루프 (100 Hz)
```
1. joint_state 수신 가드 + 타임아웃 검사
2. q_feedback = q_desired_ (리프트만 측정값으로 덮어씀)
3. kinematics_solver_->updateState(q_feedback, qdot_)
4. 양팔 각각:
     elapsed < duration → cubic 위치 + SO(3) 회전 보간 + 피드포워드 속도 산출
     else               → 정상상태 P-제어
   computeDesiredVelocity(current, ref, ff_lin, ff_ang)
5. weights = {r_link:[10,10,10,1,1,1], l_link:[10,10,10,1,1,1]}
6. qp_controller_->setWeight(weights, 0.1·I_dof)
7. qp_controller_->setDesiredTaskVel({r_link: v_r, l_link: v_l})
8. opt_qdot = getOptJointVel()
9. q_desired_ += opt_qdot · time_step_
10. publishTrajectory(q_desired_)
```

특이점:
- **시작 자세 재설정 정책**(`syncArmStateToFeedback`): 신규 MoveL 명령 수신 시 그 팔의 측정값으로만 `q_desired_`를 재정렬 → 명령 연속성 보장하면서 점프 방지.
- 그리퍼는 작업공간 목표를 받지 않으므로 QP가 풀어준 관절속도에 따라 *자유롭게* 움직임 (조인트 한계와 비용 정규화로만 제어).

### 3.2 AI Worker MoveJ — `ai_worker_movej_controller_node`

본질적으로 **CBF 안전 필터** 노드. 입력 raw 궤적을 자체 누적 명령(`q_commanded_`)으로 미끄럽게 흡수.

#### 핵심 흐름
```
입력 raw_joint_trajectory ─► updateArmTargetFromTrajectory()
   ↳ msg.joint_names 있으면 이름 매핑 / 없으면 size로 매핑
   ↳ 그리퍼 값은 별도 변수에 저장 (QP 미관여)

100 Hz loop:
   q_feedback = q_commanded_                    (자기 출력 적분)
   q_ref = q_feedback; 양팔 목표만 덮어씀
   q̇_des = kp_joint · (q_ref - q_feedback)
   qp_filter_->setDesiredJointVel(q̇_des)
   opt_qdot = getOptJointVel()
   q_commanded_ += opt_qdot · dt
   publish [arm joints + gripper] JointTrajectory
```

→ **자기 출력 적분 + CBF 필터** 구조로 외부 명령의 점프나 한계 위반을 흡수한다. 리더팔/리타게팅 출력을 다운스트림에 흘리기 전 안전 게이트.

### 3.3 VR Controller — `vr_controller_node`

가장 복잡한 노드. 5-상태 머신:

```
[idle] ─reactivate=true─► [start_requested]
   │                          │
   │                          ▼
   │            r/l goal pose 수신 대기
   │                          │
   │                          ▼
   │            pose error 검사:
   │              pos_err > 0.15 m  OR  ori_err > 45° → 머무름
   │              둘 다 통과 → control_enabled = true
   │                          │
   │                          ▼
   │            [activate_pending] (3초)
   │                          │
   │                          ▼
   │            [running]
   │              slow_start_scale: 3~11초 사이 0→1 선형 램프
   │              VRController QP (양 그리퍼 + 양 팔꿈치 4링크)
   │                          │
   │  ◄─────reactivate=false──┘
   └─◄────reference_diverged or joint_state_timeout
```

#### 추적 변수
- 양 그리퍼 (`r/l_gripper_name`): 위치 가중치 10, 자세 가중치 1
- 양 팔꿈치 (`r/l_elbow_name`): 위치 가중치 8 (`weight_elbow_position`), 자세 가중치 0 → **null-space 활용**으로 사람 팔꿈치 모양 따라가게 만드는 핵심 트릭

#### 안전 절차
1. `startup_ref_pos_threshold` (0.15 m), `startup_ref_ori_threshold_deg` (45°) — 활성화 전 reference-vs-current 정렬 검사
2. 3초 `activate_pending` 지연
3. 8초 `slow_start_scale` 선형 램프 (속도 명령에 곱)
4. `/reference_diverged` 토픽 수신 시 즉시 중단 → 명령 stale

### 3.4 Leader Controller — `leader_controller_node`

리더팔 양팔 `JointTrajectory` 명령을 받아 그 위치를 그대로 URDF 모델에 박고 FK를 풀어 `/r_goal_pose`, `/l_goal_pose`, `/r_elbow_pose`, `/l_elbow_pose` 로 송출. 즉 **관절 명령을 작업공간 명령으로 변환**해 VR 컨트롤러 입력으로 공급하는 어댑터.

특이점:
- `lift_joint` 만은 측정 `/joint_states` 값으로 갱신해 모델 일관성 유지.
- `world` 링크가 있으면 그 frame을 base로 삼아 변환 (`computePoseInBaseFrame`).
- `reactivate` 가 false면 publish 자체를 멈춤 → 다운스트림 정지.

### 3.5 Reference Checker — `reference_checker_node`

```python
on /r_goal_pose, /l_goal_pose:
    pos_jump = ‖new_pos - prev_pos‖
    ori_jump = 2·acos(|q_prev · q_new|)
    if pos_jump > 0.1 m or ori_jump > 30°:
        publish /reference_diverged := True
```

VR 모드에서 사람 추적 데이터의 *불연속*을 감지해 즉시 컨트롤러 동작 중단을 트리거. 단순하지만 안전상 결정적.

### 3.6 Interactive Marker — `interactive_marker_node`

RViz의 6-DoF 마커를 띄워 사용자가 끌면 `MoveL` 메시지를 publish.

- `MOUSE_DOWN` → 드래깅 시작
- `POSE_UPDATE` + 드래깅 + `publish_while_dragging=true` → 매 프레임 publish
- `MOUSE_UP` → 최종 자세 publish, 드래깅 종료

런치에서 좌/우 두 개를 띄우면 양팔 MoveL 인터페이스. `MoveL.time_from_start = 0` 이라서 다운스트림이 holding 모드 (피드포워드 없는 P 제어) 로 처리.

### 3.7 OMX/OMY 컨트롤러

코어 컨트롤러 (`OpenManipulatorMoveL/JController`)는 *단일 EE* 버전이라는 점만 다르다 — `setDesiredTaskVel`이 `map`이 아닌 `Vector6d` 한 개를 받는다 (`open_manipulator_movel_controller.hpp:50`). ROS 노드는 AI Worker 와 같은 패턴(`joint_states` 수신 → 100 Hz 루프 → QP → JointTrajectory publish) 으로 단순화된 단팔 버전.

---

## 4. 텔레옵 파이프라인 종합

### 4.1 5가지 입력 모드와 전체 흐름

```
①  RViz 인터랙티브 마커        ②  명시적 MoveL publish     ③  raw JointTrajectory
   │                              │                            │
   │ MoveL                        │ MoveL                       │ JointTrajectory
   ▼                              ▼                            ▼
   /r_goal_move, /l_goal_move ─────────────────┐         ──────┐
                                                │              │
                                                ▼              ▼
                          ai_worker_movel_controller_node    ai_worker_movej_controller_node
                          (작업공간 → QP → q̇)               (관절공간 + CBF 필터)
                                                │              │
                                                └──────┬───────┘
                                                       │
                                                       ▼
                          /leader/.../{right,left}/joint_trajectory ──► (다운스트림 컨트롤러)


④  리더팔 (관절 명령)
   │ JointTrajectory
   ▼
   /leader/.../raw_joint_trajectory ──► leader_controller_node ──► /r_goal_pose, /l_goal_pose
                                                                    │
                                                                    ▼
                                                         vr_controller_node (작업공간 추종 + 팔꿈치 트래킹)


⑤  VR 헬멧/트래커 (사람 팔 PoseStamped)
   │ PoseStamped (어깨/팔꿈치/손목)
   ▼
   arm_retargeting_teleop.py
   (방향 벡터 추출 → 로봇 팔 길이 곱 → 양손 거리 우선화)
   │
   ▼
   /r_goal_pose, /l_goal_pose, /r_subgoal_pose, /l_subgoal_pose ──► vr_controller_node
                                                                    │
                                                                    ▼
                                                         reference_checker_node
                                                         (점프 감지 → /reference_diverged)
```

### 4.2 리타게팅 (Python, `cyclo_motion_controller_ros_py/scripts/`)

#### `arm_retargeting.py`
사람 어깨/팔꿈치/손목 PoseStamped → 로봇 팔 길이로 *재구성*된 골 자세. 핵심 단계:
1. 방향 벡터만 추출 (사람과 로봇의 절대 위치는 무시)
2. 로봇의 `upper_arm_length`, `forearm_length` 곱
3. 양손 거리 우선 보정 (지수 감쇠 가중)
4. 전박 길이 재투영 + 1차 저역통과
5. 사람 양손 상대 자세를 로봇 양손에 강제 (쿼터니언 곱)

#### `teleop_retargeting.py` + `cyclo_motion_controller_core/src/retargeting/optimizer.py`
손가락 리타게팅은 [dex-retargeting](https://github.com/dexsuite/dex-retargeting) DexPilot 알고리즘을 직접 차용:
- `nlopt` 으로 비선형 최소제곱 풀이
- 손가락 끝 5점 + 손목 → 로봇 손 관절 좌표
- Huber loss, projection distance, escape distance 등 DexPilot 파라미터 그대로

---

## 5. 안전 메커니즘 정리

| 계층 | 메커니즘 | 위치 |
|---|---|---|
| 솔버 | CBF 슬랙 + 큰 페널티 | 모든 `setIneqConstraint` |
| 솔버 | OSQP 희소 패턴 캐시로 100 Hz 실시간 보장 | `qp_base.hpp:148-166` |
| 노드 | `joint_state_timeout` (0.5 s) → 명령 중단, 다음 수신 시 재동기화 | 모든 컨트롤러 노드 |
| 노드 | 첫 수신 전 명령 무시 (`!q_desired_initialized_`) | 모든 컨트롤러 노드 |
| 노드 | 신규 명령 수신 시 그 팔만 측정값으로 재정렬 | `syncArmStateToFeedback` |
| VR | 활성화 전 reference-vs-current 정렬 검사 (15 cm / 45°) | `vr_controller_node.cpp:544-595` |
| VR | 3초 `activate_pending`, 8초 slow_start_scale 선형 램프 | `:608-617`, `:662-687` |
| VR | `/reference_diverged` 수신 시 즉시 중단 | `:400-413` |
| 외부 | `reference_checker_node` 가 goal 토픽 점프 감지 → divergence publish | `reference_checker_node.cpp:91-106` |
| 외부 | `reactivate_topic` (`/reactivate`) 으로 운영자 enable/disable | `:415-444` |
| 핸드오버 | `disable_gripper_collisions:=true` 시 modified SRDF로 양손 충돌 제외 | 런치 `:176-186` |

---

## 6. 평가, 한계, 개선 제안

### 6.1 강점

1. **단일 QP 추상화의 일관성** — 5개 컨트롤러가 같은 결정변수·CBF 구조를 공유. 새 로봇 추가 시 `QPBase` 상속하고 비용 4함수만 구현하면 됨.
2. **희소 패턴 캐시** — 100Hz 실시간 보장의 핵심 최적화. 매 사이클 OSQP 재초기화 회피.
3. **연성 CBF + 슬랙** — `infeasible → stall` 대신 슬랙 페널티화로 점진적 위반 허용. 실제 환경에서 강건.
4. **VR 모드의 다층 안전 게이트** — startup mismatch → 3초 지연 → 8초 slow start → reference divergence 감시. 사람 추적 노이즈에 대한 방어가 충실.
5. **모듈 분리** — core는 ROS 의존성 0, 다른 시뮬레이션/임베디드 환경에도 재사용 가능.

### 6.2 약점·한계

1. **충돌 모델 신뢰성 주의** — `KinematicsSolver` 생성자에 직접 노란색 경고 출력:
   ```
   "Collision model for the robot may not be perfect!"
   ```
   URDF 의 collision 메시 품질에 전적으로 의존. SRDF로 제외쌍을 직접 큐레이션해야 함.
2. **속도 한계만 강제, 가속/저크 미강제** — QP가 푸는 것은 순간 `q̇`. 사이클 간 가속도 제한이 없어 `slack_penalty` 활성화 등 동작 전환 시 토크 점프 발생 가능.
3. **그리퍼 폐쇄제어 부재** — MoveL/VR 모드에서 그리퍼 관절은 QP가 자유롭게 풀이. 실제 그리퍼 명령은 별도 흐름이 필요(외부 컨트롤러).
4. **`syncCommandStateToFeedback` 의 양팔 동시 리셋** — 한 팔만 명령 갱신해도 양팔 시작자세가 같이 리셋되는 경로가 있음 (MoveL `q_desired_ = q_`). 양손 협조 작업 중에는 미세한 양손 자세 점프 가능.
5. **단일 포인트 trajectory** — 입력 `JointTrajectory.points.front()` 만 사용. multi-point 궤적은 무시되어 다운스트림 보간기에 위임됨.
6. **VR slow_start 하드코딩** — 3초 지연, 8초 램프가 노드 내부 상수 (`vr_controller_node.cpp:664-666`). 파라미터화 안 됨.
7. **TODO 주석 흔적** — `vr_controller_node.cpp:463 "// ToDo: Add low pass filter"` — `qdot_` 미필터링이 인식되어 있음.

### 6.3 개선 제안

| 우선순위 | 제안 |
|---|---|
| 높음 | qdot에 1차 저역통과 필터 적용 (코드에 TODO 있음). 미분 노이즈가 QP 비용 항으로 직결됨. |
| 높음 | `slack_penalty`와 `cbf_alpha` 의 활성도를 디버그 토픽으로 노출. 슬랙이 큰 값을 갖는 빈도 = 한계 근접 빈도. |
| 중간 | 가속도 제한 부등식 추가: `\|q̈\| ≤ a_max` → `\|q̇_t - q̇_{t-1}\| ≤ a_max·dt`. QPBase에 1줄. |
| 중간 | `disable_gripper_collisions` 외에 *런타임* 충돌쌍 비활성 API. 현재는 SRDF 스왑만 가능. |
| 낮음 | `JointTrajectory` 의 다중 포인트 → cubic 보간으로 직접 처리. 현재 다운스트림 의존. |
| 낮음 | VR slow_start 파라미터화 (`activation_delay`, `slow_start_duration`). |

---

## 7. 핵심 수식 요약

### 7.1 작업공간 PD with feedforward (MoveL)
```
e_p = p_goal - p_cur
e_o = θ·k̂  (where R_err = R_goal · R_curᵀ = AxisAngle(θ, k̂))
v_des = ff_linear  + Kp_pos · e_p
ω_des = ff_angular + Kp_ori · e_o
```

### 7.2 SO(3) 큐빅 보간 (`type_define.hpp`)
```
τ   = cubic(t, 0, T, 0, 1, 0, 0)                    ── 시간 스케일링
R_t = R_0 · exp(skew(log(R_0ᵀ·R_f) · τ))             ── 회전 큐빅
ω_t = R_0 · cubicDot(t, 0, T, 0, r, 0, 0)            ── 각속도
```

### 7.3 QP 정식화 (MoveL: 양팔 + 양 팔꿈치)
```
min   Σ_{i∈{r_grip, l_grip, r_elb, l_elb}}  ‖Jᵢ·q̇ − xdotᵢ_des‖²_{Wᵢ}
      + ‖q̇‖²_{W_d}
      + ρ·(1ᵀs_qmin + 1ᵀs_qmax + s_sing + 1ᵀs_sel_col)

 s.t.  q̇_lb  ≤ q̇ ≤ q̇_ub
       I·q̇ + I·s_qmin ≥ -α·(q - q_min)
       -I·q̇ + I·s_qmax ≥ -α·(q_max - q)
       ∇dᵢᵀ·q̇ + sᵢ ≥ -α·(dᵢ - d_safe)   (활성 시)
       s_* ≥ 0
```

### 7.4 CBF 의 의미
거리 `d(q)`가 클래스-K 함수 `α·(d - d_safe)` 보다 빨리 줄어들지 않도록 강제 → `d` 가 `d_safe` 로 수렴(점근적). 슬랙은 솔버 infeasible 회피를 위한 *완화*.

---

## 8. 데이터 흐름 카탈로그

### 8.1 표준 토픽 목록

| 토픽 | 타입 | Pub/Sub 노드 |
|---|---|---|
| `/joint_states` | `JointState` | (외부) → 모든 컨트롤러 |
| `/r_goal_move`, `/l_goal_move` | `MoveL` | InteractiveMarker / 외부 → MoveL Ctrl |
| `/r_goal_pose`, `/l_goal_pose` | `PoseStamped` | Leader / Retargeting → VR Ctrl |
| `/r_subgoal_pose`, `/l_subgoal_pose` | `PoseStamped` | ArmRetargeting → VR Ctrl (팔꿈치) |
| `/r_elbow_pose`, `/l_elbow_pose` | `PoseStamped` | Leader → VR (선택) |
| `/r_gripper_pose`, `/l_gripper_pose` | `PoseStamped` | MoveL/VR Ctrl (FK 결과 publish) |
| `/.../raw_joint_trajectory` | `JointTrajectory` | Leader/외부 → MoveJ/VR Ctrl |
| `/.../joint_trajectory` | `JointTrajectory` | MoveL/MoveJ/VR Ctrl → (다운스트림) |
| `/reactivate` | `Bool` | 운영자 → VR/Leader Ctrl |
| `/reference_diverged` | `Bool` | ReferenceChecker / VR Ctrl 자체 |
| `~/controller_error` | `String` | 컨트롤러 → (로깅) |

### 8.2 표준 파라미터 (YAML 키)

`cyclo_motion_controller_ros/config/{ai_worker,omx,omy}_config.yaml` 공통:

```
control_frequency, time_step, trajectory_time
kp_position, kp_orientation        (MoveL/VR)
kp_joint                           (MoveJ)
weight_position, weight_orientation, weight_damping, weight_elbow_position
slack_penalty, cbf_alpha
collision_buffer, collision_safe_distance
joint_state_timeout
joint_states_topic, *_traj_topic, *_pose_topic
*_gripper_name, *_elbow_name, *_gripper_joint
startup_ref_pos_threshold, startup_ref_ori_threshold_deg  (VR)
lift_vel_bound                                            (AI Worker)
```

---

## 9. 결론

**cyclo_control은 단일 QP 위에 5종 텔레옵을 일관 통합한, 잘 설계된 ROS 2 모션 제어 스택**이다. 핵심 강점은 (a) CBF + 슬랙으로 인한 강건성, (b) OSQP 패턴 캐시로 인한 실시간성, (c) core/ros/models/ros_py 4계층 분리로 인한 재사용성. VR 텔레옵 모드는 startup-mismatch / activation-delay / slow-start / reference-divergence 의 4단 안전 게이트로 인간 추적 데이터의 노이즈를 견딘다.

개선 여지는 가속도 제한, qdot 저역통과, slack 활성도 모니터링 정도이며, 충돌 모델 품질(URDF 의 collision 메시 + SRDF 큐레이션)에는 사용자의 주의가 명시적으로 요구된다.

---

## 부록: 참고 라인

| 파일 | 라인 | 내용 |
|---|---|---|
| `cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp` | `:250-362` | 충돌쌍 거리 그래디언트 계산 (핵심) |
| `cyclo_motion_controller_core/include/.../optimization/qp_base.hpp` | `:148-166` | 희소 패턴 캐시 |
| `cyclo_motion_controller_core/src/controllers/ai_worker/vr_controller.cpp` | `:113-217` | VRController QP 전체 |
| `cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp` | `:319-463` | MoveL 메인 루프 |
| `cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp` | `:471-739` | VR 5-상태 머신 |
| `cyclo_motion_controller_ros/src/utils/reference_checker_node.cpp` | `:62-106` | 점프 감지 |
| `cyclo_motion_controller_ros_py/scripts/arm_retargeting.py` | 전체 | 사람 팔 → 로봇 팔 길이 재매핑 |
| `cyclo_motion_controller_core/src/retargeting/optimizer.py` | 전체 | DexPilot nlopt 손 리타게팅 |
