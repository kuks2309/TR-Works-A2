# 03. 컨트롤러 & 운동학 (Core 라이브러리)

[← 문서 목차](README.md) · [← 02. QP/CBF](02_qp_cbf_formulation.md)

`cyclo_motion_controller_core` 의 클래스들을 정리합니다. 모두 ROS 비의존이며,
입력은 Eigen 벡터, 출력은 관절 속도 `qdot` 입니다.

```
QPBase  (추상)
 ├─ OpenManipulatorMoveLController   단일 EE 데카르트
 ├─ OpenManipulatorMoveJController   단일 관절 공간
 └─ VRController                     다중 링크(양팔+팔꿈치) 태스크 맵
      └─ AIWorkerMoveLController     (VRController 그대로, 생성자만)
```

운동학(FK/Jacobian/충돌)은 모든 컨트롤러가 공유하는 `KinematicsSolver` 가 담당합니다.

---

## 1. `QPBase` (optimization/qp_base.hpp)

OSQP 표준형 `min ½xᵀPx+qᵀx s.t. l≤Ax≤u` 풀이 프레임워크. 자세한 내용은
[02_qp_cbf_formulation.md](02_qp_cbf_formulation.md) 참고.

| 멤버 | 역할 |
| --- | --- |
| `setQPsize(nx,nbc,nineqc,neqc)` | 변수/제약 크기 지정, 버퍼 0 초기화 (`assert(nbc==nx || nbc==0)`) |
| `solveQP(sol)` | cost·제약 조립 → 희소화 → (재초기화 or in-place 갱신) → OSQP 풀이 |
| `initializeSolver(...)` | 솔버 새로 생성(실패한 setup 의 stale 상태 회피), warm-start on |
| `setCost()` *(순수가상)* | `P_ds_`, `q_ds_` 채움 |
| `setBoundConstraint()` *(순수가상)* | `l_bound_ds_`, `u_bound_ds_` 채움 |
| `setIneqConstraint()` *(순수가상)* | `A_ineq_ds_`, `l_ineq_ds_`, `u_ineq_ds_` 채움 |
| `setEqConstraint()` *(순수가상)* | `A_eq_ds_`, `b_eq_ds_` 채움 |

> `setConstraint()` 가 **[bound; ineq; eq]** 순서로 전체 `A`/`l`/`u` 를 조립합니다.
> 희소 패턴이 직전과 같으면 재초기화 없이 갱신하여 비용을 줄입니다.

---

## 2. `OpenManipulatorMoveLController`

단일 엔드이펙터의 데카르트(task-space) 속도 추종.
→ 결정변수·비용·제약은 [02 §2~4](02_qp_cbf_formulation.md) 와 동일.

| API | 설명 |
| --- | --- |
| `OpenManipulatorMoveLController(robot_data, controlled_link, dt)` | `controlled_link` 가 모델에 없으면 `runtime_error` |
| `setControlledLink(name)` | 태스크 프레임 변경 (존재 검증) |
| `setDesiredTaskVel(ẋ_d)` | 목표 6D 태스크 속도(`Vector6d`) |
| `setWeights(task_w(6), damping_w(n))` | 태스크/감쇠 가중치 (damping 크기 불일치 시 무시) |
| `setControllerParams(slack_penalty, cbf_alpha, buffer, safe)` | QP 페널티·CBF 파라미터 |
| `getOptJointVel(opt_qdot)` | QP 풀이 → `qdot`. 실패 시 `false` + 영속도 |

- 생성자 기본값: `slack_penalty=1000`, `cbf_alpha=5`, `collision_buffer=0.05`, `collision_safe_distance=0.02`
  (ROS 노드가 config 값으로 덮어씀).
- `controlled_link` 가 비어 있으면 태스크 비용을 추가하지 않고 감쇠+슬랙만 남습니다.

---

## 3. `OpenManipulatorMoveJController`

단일 로봇의 **관절 공간** 속도 추종. 야코비안을 사용하지 않습니다.

| API | 설명 |
| --- | --- |
| `OpenManipulatorMoveJController(robot_data, dt)` | (controlled_link 불필요) |
| `setDesiredJointVel(q̇_d(n))` | 목표 관절 속도 (크기 일치 시에만 반영) |
| `setWeights(joint_w(n), damping_w(n))` | 관절 추종/감쇠 가중치 |
| `setControllerParams(...)` / `getOptJointVel(...)` | MoveL 과 동일 |

- 비용: `‖q̇ − q̇_d‖²_{Wⱼ} + q̇ᵀDq̇`. 제약(관절한계 CBF·충돌 CBF·슬랙)은 MoveL 과 **완전히 동일**.

---

## 4. `VRController` (양팔/다중 링크)

여러 링크의 태스크를 **맵(map)** 으로 동시에 추종하는 IK 컨트롤러.
AI Worker 양팔 텔레오퍼레이션의 코어입니다.

| API | 설명 |
| --- | --- |
| `VRController(robot_data, dt)` | 단일 모델로 여러 링크 태스크 처리 |
| `setDesiredTaskVel(map<link, ẋ_d>)` | 링크별 목표 6D 속도 (예: 좌/우 그리퍼, 좌/우 팔꿈치) |
| `setWeight(map<link, w(6)>, w_damping(n))` | 링크별 태스크 가중치 + 감쇠 |
| `setControllerParams(...)` / `getOptJointVel(...)` | 공통 |

비용(`setCost`, vr:113-148):

```
P[q̇,q̇] = Σ_link 2 Jᵢᵀ Wᵢ Jᵢ + 2 D
q[q̇]   = Σ_link (−2 Jᵢᵀ Wᵢ ẋ_d,i)
```

- 맵에 등장하는 **모든 링크**에 대해 야코비안 항을 누적 → 양팔(좌/우 그리퍼) + 팔꿈치(elbow) subgoal
  을 하나의 QP 에서 동시에 만족시킵니다.
- 맵에 가중치가 없는 링크는 `Vector6d::Ones()` 기본값.
- 제약(관절한계 CBF·충돌 CBF·슬랙)은 OpenManipulator 와 동일 구조이므로,
  **양팔 자기 충돌 회피**가 같은 QP 안에서 처리됩니다(SRDF 로 비활성화한 쌍 제외).

### `AIWorkerMoveLController`

```cpp
class AIWorkerMoveLController : public VRController { /* 생성자만 */ };
```

별도 로직 없이 `VRController` 를 그대로 상속합니다. 즉 AI Worker MoveL 의 IK 거동은
`VRController` 와 동일하며, 차이는 ROS 노드가 어떤 링크/가중치/입력을 맵에 넣는지에 있습니다
([04_ros_interface.md](04_ros_interface.md) 참고).

---

## 5. `KinematicsSolver` (kinematics/kinematics_solver.cpp)

**백엔드: Pinocchio + HPP-FCL.** URDF/SRDF 로 모델·충돌쌍을 구성하고 FK·야코비안·충돌거리를 제공합니다.

### 5.1 생성/초기화

1. `pinocchio::urdf::buildModel(urdf)` → `model_`, `data_`
2. `buildGeom(model, urdf, COLLISION, geom_model_)` → 충돌 형상
3. `geom_model_.addAllCollisionPairs()` → **모든 링크쌍** 충돌쌍 등록
4. SRDF 가 있으면 `srdf::removeCollisionPairs(...)` → **비충돌(허용) 쌍 제거**
   - ⚠️ SRDF 미제공 시 경고 후 **모든 쌍이 활성** 상태로 남습니다(과도한 제약 가능).
   - 콘솔에 `Collision model for the robot may not be perfect!` 경고를 항상 출력.
5. `dof_ = model_.nq`, 상태/한계 초기화
   - 관절 위치 한계: `model_.lower/upperPositionLimit`
   - 관절 속도 한계: `qdot_ub_ = model_.velocityLimit`, `qdot_lb_ = −qdot_ub_` (대칭)
6. 프레임 수집: BODY(=URDF `<link>`) / JOINT(=URDF `<joint>`),
   `universe` 조인트에 직접 붙은 첫 BODY 를 **root link** 로 식별

### 5.2 주요 함수

| 함수 | 내용 |
| --- | --- |
| `updateState(q,qdot)` | `forwardKinematics` + `computeJointJacobians` + `updateFramePlacements` |
| `getPose(link)` | 캐시된 `data_.oMf` 로 `Affine3d` 반환(상태 갱신 안 함) |
| `computePose(q,link)` | 임시 `Data` 로 즉석 FK (상태 비파괴) |
| `getJacobian(link)` | `getFrameJacobian(LOCAL_WORLD_ALIGNED)`, 6×dof |
| `computeJacobian(q,link)` | 임시 `Data` 로 즉석 야코비안 |
| `getJointNames()` | 일반화 좌표 순서의 관절 이름 (joint_states 정렬용) |
| `getJointPositionLimit()` / `getJointVelocityLimit()` | `(lower, upper)` pair |
| `setJointVelocityBoundsByIndex(i,lo,hi)` | 특정 관절 속도 한계 덮어쓰기 |
| `getCollisionPairCount()` | 활성 충돌쌍 수 (= 컨트롤러 슬랙/제약 `m`) |
| `getCollisionPairDistances(with_grad, with_graddot, verbose)` | 쌍별 `MinDistResult{distance, grad, grad_dot}` |

### 5.3 충돌거리 그라디언트 (CBF 입력)

`getCollisionPairDistances` 가 충돌 CBF 제약의 `∇d` 를 해석적으로 계산합니다(kinematics:290-358):

1. `computeDistances` 로 쌍별 최단거리 `d` 와 witness 점 `pA, pB` 획득
2. 접촉 법선 `n = (pB − pA)/‖·‖`
3. 각 형상의 부모 조인트 야코비안을 witness 점으로 이동(레버암 `r = p − oMi.translation`,
   `J_point = J_v − [r]× J_ω`)
4. `∇d = nᵀ (J_B − J_A)` → 거리의 관절 민감도
5. **관통 시(`d < 0`) 부호 반전** (`grad *= -1`)
6. `with_graddot=true` 면 `∇ḋ`(시간변화)도 계산. 컨트롤러에서는 `false` 로 호출(미사용).

> 즉 컨트롤러의 충돌 CBF `∇dᵀq̇ ≥ −α(d − d_safe)` 의 `∇d` 가 여기서 나옵니다.

---

## 6. 공통 수학 유틸 (common/type_define.hpp)

| 항목 | 내용 |
| --- | --- |
| `Vector6d` | 6D twist (선속도 3 + 각속도 3) |
| `MinDistResult` | `{distance, grad, grad_dot}` 충돌거리 결과 |
| `cubic` / `cubicDot` | 스칼라 3차 보간 + 시간미분 (경계 위치/속도 조건) |
| `cubicVector` / `cubicDotVector` | 벡터화(고정·동적 크기) |
| `rotationCubic` | SO(3) 위 회전행렬 3차 보간 (`R₀·exp(log(R₀ᵀR_f)·τ)`) |
| `rotationCubicDot` | 위 보간의 각속도 프로파일 |

ROS MoveL 노드는 이 함수들로 시작→목표 자세 사이를 부드럽게 보간하고
피드포워드 속도를 생성합니다.

다음: [04. ROS 인터페이스 →](04_ros_interface.md)
