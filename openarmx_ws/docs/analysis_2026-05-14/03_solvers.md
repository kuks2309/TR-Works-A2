# Solver 심층 분석

OpenArmX 워크스페이스의 모든 솔버 관련 파일을 5개 카테고리로 분류해서 분석한 결과입니다.

## 솔버 아키텍처 전체도

```
┌────────────────────────────────────────────────────────────────┐
│  계획용 (오프라인 / 저주파)                                       │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ MoveIt2 + KDL 6-DOF IK (5 ms timeout, full pose)     │    │
│  │   → Pilz 직선 플래너                                  │    │
│  └──────────────────────────────────────────────────────┘    │
│                                                                │
│  텔레오프용 (실시간 / 60-100 Hz)                                │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ openarmx_arm_driver Differential IK (VR)             │    │
│  │  - 외부 바이너리 (소스 비공개)                         │    │
│  └──────────────────────────────────────────────────────┘    │
│                                                                │
│  토크 제어용 (실시간 / 200-300 Hz)                              │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ KDL 동역학 (gravity / Coriolis / mass / Jacobian)    │    │
│  │  ├ gravity_comp        (풀 동역학, null-space 포함)   │    │
│  │  └ teleop_bimanual     (중력 + 감쇠 + 위치 홀드만)    │    │
│  └──────────────────────────────────────────────────────┘    │
│                                                                │
│  모델 데이터:                                                  │
│  ┌──────────────────────────────────────────────────────┐    │
│  │ description/config/*/v10/*.yaml (URDF xyz/rpy)       │    │
│  │  → xacro → URDF → KDL Tree → 모든 솔버               │    │
│  └──────────────────────────────────────────────────────┘    │
└────────────────────────────────────────────────────────────────┘
```

---

## 1. MoveIt2 IK Solver

**위치:** `openarmx_bimanual_moveit_config/config/`
- `kinematics.yaml`
- `openarmx_bimanual.srdf`
- `pilz_cartesian_limits.yaml`

| 항목 | 값 |
|---|---|
| 플러그인 | `kdl_kinematics_plugin/KDLKinematicsPlugin` (좌/우 동일) |
| Search resolution | 0.005 (5 mm) |
| Timeout | 0.005 s (5 ms — 매우 공격적) |
| IK 모드 | 6-DOF 풀 포즈 (KDL 기본) |
| 플래닝 그룹 | `left_arm` (7), `right_arm` (7), `left_gripper`, `right_gripper` |
| Base / Tip | `openarmx_{L,R}_link0` → `openarmx_{L,R}_hand` |
| 양팔 통합 그룹 | ❌ 없음 (좌/우 독립 계획) |
| Cartesian 플래너 | Pilz Industrial Motion Planner (1.0 m/s, 1.57 rad/s) |
| OMPL | 명시 없음 (의존성에는 포함) |

**주의사항:**
- 5 ms timeout은 7-DOF redundant arm에서 KDL 수렴 실패 위험.
- `attempts` 미지정 → MoveIt 기본값 (3-5).
- TRAC-IK / Pick-IK 교체 검토 권장.

---

## 2. 로봇 기구학 YAML

**위치:** `openarmx_description/config/`

**관례:** DH가 아닌 **URDF-style xyz/rpy** (오일러 각도).

| 파일 | 용도 |
|---|---|
| `arm/v10/kinematics.yaml` | 조인트 프레임 원점 (부모 링크 기준) |
| `arm/v10/kinematics_link.yaml` | visual/collision 메쉬 오프셋 (arm은 전부 0) |
| `arm/v10/kinematics_offset.yaml` | 좌표축 정렬용 회전 (예: joint2 roll 1.5708) |
| `body/v10/kinematics.yaml` | body_link0 원점 |
| `body/v10/kinematics_link.yaml` | body 메쉬 오프셋 |
| `hand/openarmx_hand/kinematics.yaml` | hand + finger 조인트 원점 |
| `hand/openarmx_hand/kinematics_link.yaml` | hand 메쉬 오프셋 |

**구성 요소별 DOF:**
- **Arm v10:** 7 조인트, 8 링크 (link0-link7)
- **Body v10:** 1 베이스 링크 (body_link0)
- **Hand:** 3 링크 (hand + 2 finger)

**처리 흐름:**
```
*.yaml → xacro 매크로 (openarmx-kinematics) → URDF <origin> 태그
                                          → KDL Tree
                                          → 모든 솔버
```

`kinematics + kinematics_offset` 합성 = 조인트 최종 원점.
`kinematics_link` = 메쉬 보정 (대부분 0).
조인트 한계는 별도 `limit` 필드 (`openarmx-limits` 매크로가 처리).

**의의:** v10 버전 분리 + 캘리브레이션 시 메쉬 안 건드림.

---

## 3. KDL 동역학 솔버 — openarmx_gravity_comp

**위치:**
- `openarmx_ros2/openarmx_gravity_comp/include/dynamics.hpp`
- `openarmx_ros2/openarmx_gravity_comp/src/dynamics.cpp`

### 3.1 Public API

```cpp
Dynamics(urdf_path, start_link, end_link);
Init();
NumJoints(); StartLink(); EndLink(); PrintModelSummary();
SetGravityVector(gx, gy, gz);  // 런타임 변경 가능
```

### 3.2 KDL 컴포넌트

| 컴포넌트 | 사용처 |
|---|---|
| `KDL::Tree` | `kdl_parser::treeFromUrdfModel()` |
| `KDL::Chain` | `kdl_tree.getChain(start, end)` |
| `KDL::ChainDynParam` | gravity, Coriolis, mass |
| `KDL::ChainJntToJacSolver` | Jacobian |
| `KDL::ChainFkSolverPos_recursive` | 정기구학 |

### 3.3 구현된 메서드

1. **정기구학** — `GetEECordinate()`, `GetPreEECordinate()` → SE(3) (R, p)
2. **중력 토크** — `GetGravity()` → `JntToGravity`
3. **코리올리/원심력** — `GetColiori()` → `JntToCoriolis`
4. **관성 행렬 (대각)** — `GetMassMatrixDiagonal()` → `JntToMass` 후 대각 추출
5. **Jacobian** — `GetJacobian()` (6×DOF Eigen)
6. **널 스페이스** — `GetNullSpace()` = (I − J⁺·J), `GetNullSpaceTauSpace()` = 전치

### 3.4 수치 안정성

**Pseudo-inverse 이중 구현:**
- **안정 SVD:** `tol = 1e-6 × max(m,n) × σ_max`
- **표준:** `J^T(JJ^T)^-1`
- ⚠️ **`use_stable_svd`가 false로 하드코딩** → 항상 표준 방법 사용 → 특이점 폭주 위험
- DLS (Damped Least Squares) 항 **없음**

### 3.5 중력 벡터

- **기본:** (0, 0, −9.81)
- `SetGravityVector()` 호출 시 `ChainDynParam` 솔버 재생성

### 3.6 구조 제한

**단일 체인 전용.** 양팔은 외부에서 `Dynamics` 객체 2개 생성하여 독립 사용.

### 3.7 노드 사용 패턴 (gravity_comp_node.cpp)

```
/joint_states → q
  → arm_dyn_left->GetGravity(q_left) → tau_left
  → arm_dyn_right->GetGravity(q_right) → tau_right
  → 안전 한계 클램프 (어깨 20 Nm, 손목 2 Nm)
  → /{left,right}_forward_effort_controller/commands
```

---

## 4. KDL 동역학 솔버 — openarmx_teleop_bimanual (재사용)

**위치:**
- `openarmx_teleop_bimanual/include/dynamics.hpp`
- `openarmx_teleop_bimanual/src/dynamics.cpp`
- `openarmx_teleop_bimanual/src/teleop_bimanual_with_gravitycomp_single.cpp`

### 4.1 복사 여부

완전 동일 복사는 아니고 **동일 설계의 재사용 가능 유틸리티 클래스**.
- 같은 8개 메서드, zero-allocation 인터페이스 (`double*` 입출력)
- 두 패키지가 사실상 같은 구현체를 각자 보유 → **중복**

### 4.2 Teleop 루프 사용 패턴 (200 Hz)

```cpp
arm_dyn_->GetGravity(q_, tau_g_);         // KDL 호출 1회
tau_damp_[i] = -kd_damp_ * qd_[i];        // 로컬 감쇠
// 정지 시: tau_hold_[i] = kp_hold_ * (hold_target_[i] - q_[i]);
tau_joint = tau_g_ * g_scale_ + tau_damp_ + tau_hold_;
```

### 4.3 Velocity-Based Hold 로직

```
모든 |qd_[i]| < vel_hold_thresh_ (0.02 rad/s)
  AND
hold_settle_ms_ (300 ms) 동안 유지
  ↓
hold_latched_ = true
hold_target_ = 현재 q_  (캡처)
  ↓
홀드 토크: kp_hold_ × (hold_target_ − q_)
  ↓
모션 재개 시 래치 해제
```

### 4.4 gravity_comp와의 차이

| 항목 | gravity_comp | teleop_bimanual |
|---|---|---|
| Jacobian/Null-space | ✓ 사용 | ✗ 미사용 |
| Mass matrix | ✓ 사용 | ✗ 미사용 |
| Hold 로직 | ✗ | ✓ (300 ms settling) |
| Damping | ✗ | ✓ (kd 비례) |
| g_scale 기본 | 1.05 | 0.9 |
| 사용처 | 토크 피드포워드 | 텔레오프 보조 |

### 4.5 호출 빈도

**팔당 1회/사이클.** 7-DOF 전체에 대해 한 번에 계산, 조인트별 루프 없음 (KDL 내부 벡터화).

---

## 5. VR Teleop IK Solver

**위치:**
- `openarmx_teleop_vr/openarmx_teleop_vr/openarmx_teleop_vr/openarmx_teleop_vr_node.py`
- `openarmx_teleop_vr/openarmx_teleop_vr/launch/teleop_vr.launch.py`

### 5.1 솔버 라이브러리

- **`openarmx_arm_driver.OpenArmTeleopController`** (외부 / 사전 컴파일된 Python 패키지)
- 소스 트리에 없음 → `openarmx_ws/install/` 에 바이너리로만 존재
- 동작 특성으로 보아 **Jacobian pseudo-inverse 기반 Differential IK**

### 5.2 핵심 파라미터

| 파라미터 | 기본값 | 역할 |
|---|---|---|
| `ik_iterations` | 3 | 사이클당 IK 반복 |
| `constraint_mode` | `"joint"` 또는 `"link"` | full DOF / link4 위치 제약 |
| `use_link4_ext` | true | URDF 확장 프레임 (팔꿈치 제어) |
| `slow_max_step_deg` | 1° | safe 모드 사이클당 최대 변화 |
| `fast_max_step_deg` | 12° | fast 모드 (0 = 비활성) |
| `grip_threshold` | 0.5 | 데드맨 임계값 |
| `control_rate` | 60-100 Hz | 메인 루프 |

### 5.3 데이터 흐름

```
VR PoseStamped (xyz + quat)
  → RelativePose 객체로 변환
  → TeleopInputFrame (+ grip/trigger 상태)
  → controller.step(frame)
  → 조인트 명령 (Float64MultiArray)
  → /{left,right}_forward_position_controller/commands
```

**현재 조인트 상태 동기화:** `_joint_state_callback` → 사이클당 스텝 제한에 활용.

### 5.4 모드 전환

- `/vr_right_controller/rate` 토픽 수신
- `rate ≥ 0.999` → fast 모드 (12°/cycle)
- `rate < 0.999` → safe 모드 (1°/cycle)

### 5.5 IK 종류 판단 근거

**로컬 Jacobian 기반 (Differential IK).** 근거:
1. 매우 낮은 반복 수 (3회)
2. 사이클별 작은 스텝 제한 (differential IK 특성)
3. 60-100 Hz 실시간 루프
4. NLOPT 등 수치 탐색 패턴 부재

→ MoveIt의 글로벌 KDL IK와 달리 **현재 자세 부근 선형화** 방식 → 부드럽고 응답성 좋음.

---

## 요약 비교표

| 솔버 | 종류 | 위치 | 주파수 | 용도 |
|---|---|---|---|---|
| MoveIt KDL IK | 글로벌 6-DOF | bimanual_moveit_config | 계획 시 1회 | 궤적 계획 |
| Pilz | 직선 모션 | bimanual_moveit_config | 계획 시 1회 | Cartesian 모션 |
| KDL 동역학 (gravity_comp) | 풀 동역학 | openarmx_gravity_comp | 100-200 Hz | 중력 보상 토크 |
| KDL 동역학 (teleop) | 중력만 | openarmx_teleop_bimanual | 200-300 Hz | leader-follower 보조 |
| OpenArmTeleopController | Differential IK | openarmx_arm_driver (외부) | 60-100 Hz | VR 텔레오프 |
