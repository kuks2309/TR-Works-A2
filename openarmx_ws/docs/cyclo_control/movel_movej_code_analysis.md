# AI Worker MoveL / MoveJ 컨트롤러 — 상세 코드 분석

- **분석일**: 2026-05-14
- **대상**: `controller_type:=movel` / `controller_type:=movej` 모드에서 실행되는 ROS 2 노드 및 코어 QP 컨트롤러
- **소스 경로**: `/home/openarmx/TR-Works/kkw/China/cyclo_control/`

---

## 0. 두 모드의 위치

| 모드 | ROS 노드 | 코어 컨트롤러 | 부속 노드 |
|---|---|---|---|
| `movel` | `ai_worker_movel_controller_node` | `cyclo_motion_controller::controllers::AIWorkerMoveLController` ← `VRController` 상속 | (옵션) `interactive_marker_node` ×2 |
| `movej` | `ai_worker_movej_controller_node` | `cyclo_motion_controller::controllers::AIWorkerMoveJController` (QPBase 직접 상속) | 없음 |

핵심 차이:
- **MoveL**: 작업공간(SE(3)) 목표 → Jacobian 기반 IK → QP가 비용함수에 `‖J·q̇ − ẋ_des‖²`를 최소화
- **MoveJ**: 관절공간 목표 → 목표 관절속도를 QP 비용에 직접 넣어 `‖q̇ − q̇_des‖²` 최소화 + **CBF 충돌·관절한계 보장 필터** 역할

---

## 1. MoveL 데이터 흐름

### 1.1 토픽

```
인터랙티브 마커 (선택)                       시스템(/joint_states)
   │                                              │
   │  robotis_interfaces/msg/MoveL                │  sensor_msgs/msg/JointState
   ▼                                              ▼
/r_goal_move ─┐                            jointStateCallback()
/l_goal_move ─┘                                   │
   ▼                                              ▼
rightMoveLCallback / leftMoveLCallback ── q_, qdot_, q_desired_(초기 동기화)
   │
   ├─ syncArmStateToFeedback(arm joints)      ─── (재명령 시 현재 피드백으로 시작점 재정렬)
   ├─ kinematics_solver_->updateState(q_desired_, qdot_)
   ├─ right/left_movel_start_pose_ ← FK(r/l_gripper_name_)
   ├─ right/left_movel_goal_pose_  ← poseMsgToEigen(msg.pose)
   ├─ right/left_active_motion_duration_ ← msg.time_from_start
   └─ right/left_movel_trajectory_active_ = true
                                                   ▼
                                       100Hz controlLoopCallback()
                                                   │
                                                   ▼
        ┌──────────────── 양팔 양쪽 시간경과 검사 ────────────────┐
        │  elapsed < duration : cubic spline 보간 (위치/자세/속도)│
        │  else               : 정상상태 holding (위치제어만)     │
        └─────────────────────────────────────────────────────────┘
                                                   ▼
            VRController(=AIWorkerMoveLController) QP 풀이
                                                   ▼
            opt_qdot → q_desired_ += opt_qdot · dt
                                                   ▼
            arm_l_pub_ / arm_r_pub_ / lift_pub_ 로 JointTrajectory publish
            r_gripper_pose_pub_ / l_gripper_pose_pub_ 로 FK 결과 publish
```

### 1.2 노드 클래스 구성 (`ai_worker_movel_controller_node.{hpp,cpp}`)

#### 멤버 상태

```cpp
std::shared_ptr<KinematicsSolver> kinematics_solver_;        // Pinocchio 래퍼
std::shared_ptr<AIWorkerMoveLController> qp_controller_;     // 코어 QP

Eigen::VectorXd q_, qdot_, q_desired_;                       // 측정/명령 관절상태
Eigen::Affine3d right_gripper_pose_, left_gripper_pose_;     // FK 현재 자세
Eigen::Affine3d right_movel_start_pose_, *_goal_pose_;       // 보간용 시작/목표
rclcpp::Time right_motion_start_time_, left_motion_start_time_;
double right_active_motion_duration_, left_active_motion_duration_;

bool joint_state_received_, q_desired_initialized_;
bool right/left_movel_target_initialized_;
bool right/left_movel_trajectory_active_;
bool joint_state_timeout_active_;
```

#### 초기화 (`AIWorkerMoveLController::AIWorkerMoveLController()`)

핵심 파라미터 (`ai_worker_config.yaml`로 오버라이드):

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `control_frequency` | 100 Hz | 제어 타이머 주기 |
| `time_step` | 0.01 s | `q_desired_ += opt_qdot * time_step_` 적분에 사용 |
| `kp_position` / `kp_orientation` | 50 / 50 | 작업공간 PD 게인 (P항) |
| `weight_position` / `weight_orientation` | 10 / 1 | QP 비용 트래킹 가중치 |
| `weight_damping` | 0.1 | 관절속도 정규화 |
| `slack_penalty` | 1000 | 슬랙 변수 페널티 |
| `cbf_alpha` | 5 | CBF 게인 (`cyclo_motion_controller_ros/.../node.cpp` 기본; YAML은 50) |
| `collision_buffer` / `safe_distance` | 0.05 / 0.02 | CBF 활성 거리 / 안전 거리 |
| `joint_state_timeout` | 0.5 s | 피드백 타임아웃 |
| `lift_vel_bound` | 0.0 | 0이면 리프트 조인트 락 (`setJointVelocityBoundsByIndex`) |

조인트 분류는 이름 패턴으로 한다:

```cpp
// ai_worker_movel_controller_node.cpp:160-168
if (joint_name.find("arm_l_joint") != npos)  left_arm_joints_.push_back(...);
else if (joint_name.find("arm_r_joint") != npos)  right_arm_joints_.push_back(...);
else if (joint_name.find("lift_joint")  != npos)  lift_joint_ = joint_name;
```

이름 정렬 후, `lift_vel_bound_` 가 0이면 해당 인덱스의 관절속도 한계를 0으로 강제해 QP에서 자연스럽게 락된다.

### 1.3 콜백 분석

#### `jointStateCallback` (`ai_worker_movel_controller_node.cpp:219-242`)

- 첫 호출에서 메시지 내 조인트 순서 → `joint_index_map_` 저장 (캐시).
- `extractJointStates()` 로 `q_`, `qdot_` 갱신.
- **첫 수신** 또는 **타임아웃 복귀** 시 `syncCommandStateToFeedback()` 호출 → `q_desired_ = q_` 로 동기화하고 양쪽 MoveL 시작·목표 자세를 현재 FK 결과로 초기화. 점프(불연속) 방지의 핵심.

#### `rightMoveLCallback` / `leftMoveLCallback` (`:244-281`)

```cpp
// 가드: 조인트 상태 수신·미타임아웃·초기화 완료
if (!msg || !joint_state_received_ || jointStateTimedOut() || !q_desired_initialized_)
    return;

syncArmStateToFeedback(right_arm_joints_, q_desired_);   // 명령암만 측정값으로 재정렬
if (lift_joint_index_ >= 0) q_desired_[lift_joint_index_] = q_[lift_joint_index_];

kinematics_solver_->updateState(q_desired_, qdot_);
right_movel_start_pose_ = kinematics_solver_->getPose(r_gripper_name_);
right_movel_goal_pose_  = poseMsgToEigen(msg->pose);
right_active_motion_duration_ = commandDurationSeconds(msg->time_from_start);
right_motion_start_time_ = this->now();
right_movel_target_initialized_ = true;
right_movel_trajectory_active_ = right_active_motion_duration_ > -1.0;
```

**시작 자세 산정 방식**이 중요하다 — 측정 `q_` 가 아니라 **명령 `q_desired_`** 에 명령 쪽 팔만 측정으로 덮어쓴 상태에서 FK 를 푼다. 이렇게 하면:
- 명령 흐름이 끊기지 않고(다른 팔은 명령 상태 유지)
- 재명령 시 시작자세가 실제 추종 가능한 점에서 시작.

#### `controlLoopCallback` — 100 Hz 메인 루프 (`:319-463`)

순서:

1. 가드: `joint_state_received_ && q_desired_initialized_` && 타임아웃 아님.
2. **피드백 결합 상태 합성**:
   ```cpp
   Eigen::VectorXd q_feedback = q_desired_;
   if (lift_joint_index_ >= 0) q_feedback[lift_joint_index_] = q_[lift_joint_index_];
   ```
   → 팔 관절은 명령값(`q_desired_`), 리프트는 측정값. 리프트는 외부 제어이므로 피드백 그대로 사용.
3. `kinematics_solver_->updateState(q_feedback, qdot_)` → FK + 충돌거리 갱신, 양쪽 그리퍼 자세 publish.
4. 양팔 각각:

   - **궤적 활성 + 시간 미경과** → cubic spline 보간:
     ```cpp
     Vector3 linear_ref = cubicDotVector<3>(t, 0, T, p0, pf, 0, 0);   // 피드포워드 선속도
     Vector3 position_ref = cubicVector<3>(t, 0, T, p0, pf, 0, 0);    // 참조 위치
     Matrix3 rotation_ref = rotationCubic(t, 0, T, R0, Rf);           // SO(3) 보간
     Vector3 angular_ref  = rotationCubicDot(...);                    // 피드포워드 각속도
     pose_ref.translation() = position_ref;
     pose_ref.linear()      = rotation_ref;
     desired_vel = computeDesiredVelocity(current, pose_ref, linear_ref, angular_ref);
     ```
   - **시간 경과 또는 비활성** → 단순 목표점 P 제어:
     ```cpp
     desired_vel = computeDesiredVelocity(current, goal);  // 피드포워드 0
     ```

5. `computeDesiredVelocity` (`:299-317`):
   ```cpp
   pos_err = goal.translation() - current.translation();
   R_err   = goal.linear() * current.linear().transpose();
   ori_err = AngleAxis(R_err).axis() * AngleAxis(R_err).angle();   // 회전 벡터
   v.head<3>() = ff_linear  + kp_position    * pos_err;
   v.tail<3>() = ff_angular + kp_orientation * ori_err;
   ```
   즉 **PD with feedforward** 의 P 항 + 보간 미분 피드포워드 항으로 6D 작업공간 속도를 만든다.

6. QP 설정 + 풀이:
   ```cpp
   weights[r_link]   = [10,10,10, 1,1,1];     // 위치 우선
   weights[l_link]   = [10,10,10, 1,1,1];
   damping = 0.1·1_dof
   qp_controller_->setWeight(weights, damping);
   qp_controller_->setDesiredTaskVel({r_link: r_vel, l_link: l_vel});
   qp_controller_->getOptJointVel(opt_qdot);
   ```
7. **적분 + 출력**:
   ```cpp
   q_desired_ = q_feedback + opt_qdot * time_step_;
   publishTrajectory(q_desired_);
   ```
   `publishTrajectory` 는 좌/우 팔 인덱스만 골라 `trajectory_msgs::msg::JointTrajectory` 두 개를 따로 publish. 리프트는 `lift_vel_bound_ != 0` 인 경우에만 publish.

### 1.4 인터랙티브 마커 노드 (`eef_interactive_marker_node.cpp`)

- 시작 시 TF (`base_frame` → `controlled_link`) 가 들어오면 6-DoF 인터랙티브 마커 (CUBE + 3축 이동/회전 컨트롤) 를 RViz에 등록.
- 콜백:
  - `MOUSE_DOWN` → `dragging_ = true`
  - `POSE_UPDATE` + `dragging_ && publish_while_dragging_` → 드래그 중 매 프레임 `MoveL` publish
  - `MOUSE_UP` → 최종 자세 publish, `dragging_ = false`
- `MoveL.time_from_start = 0` (기본 Duration) 으로 publish → 노드 측에서 `duration > -1` 만 검사하므로 즉시 추종 (holding 모드와 동일하게 cubic 보간 미사용).

런치에서 두 개의 마커 노드를 시작하면:
- 오른쪽 마커 → `/r_goal_move` (빨강), 왼쪽 마커 → `/l_goal_move` (파랑).

---

## 2. MoveL 코어 QP 컨트롤러 — `VRController`

`AIWorkerMoveLController` 는 단순히 `VRController` 를 상속한 typedef 같은 클래스 (`ai_worker_movel_controller.cpp:31-36`). 모든 QP 로직은 `VRController` (`vr_controller.cpp`) 가 담당한다.

### 2.1 결정 변수 구성 (`vr_controller.cpp:40-72`)

`x ∈ ℝⁿˣ` 한 벡터에 다음을 차곡차곡 쌓는다:

| 블록 | 크기 | 의미 |
|---|---|---|
| `qdot` | `dof` | 최적 관절속도 (실제 출력) |
| `slack_q_min` | `dof` | 관절 하한 CBF 슬랙 (>= 0) |
| `slack_q_max` | `dof` | 관절 상한 CBF 슬랙 (>= 0) |
| `slack_sing` | `1` | 특이점 슬랙 (현 코드는 미사용; 비용·경계만 설정) |
| `slack_sel_col` | `collision_pair_count` | 충돌쌍별 CBF 슬랙 (>= 0) |

부등식 제약도 동일 순서: `q_min`, `q_max`, `sing`, `sel_col`.

### 2.2 비용 함수 `setCost()` (`vr_controller.cpp:113-148`)

각 추적 링크 `i` 에 대해 Jacobian `J_i` 로:

```
min  ½ q̇ᵀ [ 2 ΣᵢJᵢᵀ Wᵢ Jᵢ + 2 W_d ] q̇
     +  q̇ᵀ [ −2 ΣᵢJᵢᵀ Wᵢ ẋᵢ_des ]
     +  ρ·1ᵀ s_qmin + ρ·1ᵀ s_qmax + ρ·s_sing + ρ·1ᵀ s_sel_col
```

코드:
```cpp
P[qdot, qdot] += 2 * Jᵀ * diag(w_tracking) * J;
q[qdot]       += -2 * Jᵀ * diag(w_tracking) * xdot_desired;
P[qdot, qdot] += 2 * diag(w_damping);

q[slack_*] = slack_penalty * 1;   // L1 페널티 (>= 0 제약과 결합)
```

→ 작업공간 추종 + 관절속도 정규화 + 모든 슬랙 페널티화.

### 2.3 경계 제약 `setBoundConstraint()` (`:150-166`)

```
qdot_lb ≤ q̇    ≤ qdot_ub          (URDF 한계)
   0    ≤ s_*                       (모든 슬랙 ≥ 0)
```

### 2.4 부등식 제약 `setIneqConstraint()` — **CBF**

#### 관절 위치 한계 (Control Barrier Function)

- 하한: `+I·q̇ + I·s_qmin ≥ −α·(q − q_min)` → `q̇ ≥ −α·(q − q_min) − s_qmin`
- 상한: `−I·q̇ + I·s_qmax ≥ −α·(q_max − q)` → `q̇ ≤  α·(q_max − q) + s_qmax`

`α = cbf_alpha_`. 슬랙 `s_*` 는 `≥ 0` 이고 비용에서 큰 페널티가 걸려 있으므로 평소엔 ~0, 강제 시에만 활성.

#### 충돌 회피 (셀프 콜리전 페어별 CBF)

```cpp
for each collision pair i:
    A_ineq[con_sel_col+i, qdot] = grad_iᵀ;            // ∂d_i/∂q
    A_ineq[con_sel_col+i, slack_sel_col+i] = 1.0;
    if distance_i ≤ collision_buffer:
        l_ineq[con_sel_col+i] = -α·(d_i − d_safe);    // ≥ -α(d-d_safe)
```

즉 **거리가 buffer 안에 들어왔을 때만 CBF 활성**, 그 외엔 자연스럽게 `-∞` 로 비활성 (`l_ineq` 초깃값). 결과적으로:

```
∇d_iᵀ·q̇ + s_i  ≥  -α·(d_i - d_safe)
```

→ 거리의 시간변화율(`ḋ ≈ ∇d·q̇`)이 `−α·(d − d_safe)` 이상이 되어 `d → d_safe` 로 점근.

> `setEqConstraint` 는 비어 있음 (`neqc_ = 0`).

### 2.5 입력/출력 API

```cpp
setWeight(map<link, Vec6> w_tracking, VectorXd w_damping);
setDesiredTaskVel(map<link, Vec6> xdot_desired);
setControllerParams(slack_penalty, cbf_alpha, buffer, safe_distance);
getOptJointVel(VectorXd& opt_qdot);   // QPBase::solveQP → sol.head(dof)
```

---

## 3. MoveJ 데이터 흐름

### 3.1 토픽

```
   (운영자/리더 텔레옵)                         /joint_states
        │                                            │
        │  trajectory_msgs/msg/JointTrajectory       │
        ▼                                            ▼
 /leader/...broadcaster_{right,left}/raw_joint_trajectory   jointStateCallback()
        │                                                    │
        ▼                                                    ▼
 rightTrajectoryCallback / leftTrajectoryCallback ─── q_, qdot_(초기 동기화)
        │
        ├─ if duration > 0: syncArmStateToFeedback(arm)
        ├─ updateArmTargetFromTrajectory():
        │    msg.joint_names 있으면 그 인덱스로 그대로 매핑
        │    없으면 arm_joint_names 와 size가 같을 때만 사용 (fallback)
        ├─ updateGripperPositionFromTrajectory(): 그리퍼 값 보존
        ├─ right/left_movej_start_ = q_commanded_
        └─ right/left_movej_goal_  = target_q
                                                  ▼
                                  100Hz controlLoopCallback()
                                                  ▼
            q_ref = q_commanded_; 양팔 목표만 덮어씀
            desired_qdot = kp_joint · (q_ref − q_feedback)        (= q_commanded_)
            qp_filter_->setDesiredJointVel(desired_qdot)
            qp_filter_->setWeight(tracking, damping)
            qp_filter_->getOptJointVel(opt_qdot)
                                                  ▼
            q_commanded_ = q_feedback + opt_qdot · dt
                                                  ▼
            arm_l_pub_ / arm_r_pub_ 로 [arm joints + gripper] 필터링된 JointTrajectory publish
```

### 3.2 노드 클래스 (`ai_worker_movej_controller_node.{hpp,cpp}`)

#### 핵심 차이점 vs MoveL

- **Subscribe**: `raw_joint_trajectory` (입력) ↔ **Publish**: `joint_trajectory` (필터링 후)
- 입력과 출력이 같은 메시지 타입(`trajectory_msgs::msg::JointTrajectory`) → "필터" 역할이 분명.
- **컨트롤 목표는 관절공간**: 작업공간 게인 (`kp_position`) 대신 `kp_joint` 만 사용.
- 그리퍼 값은 **입력에서 추출 → 출력에 그대로 덧붙임**. QP 는 그리퍼 조인트를 만지지 않는다.

#### 입력 해석 (`updateArmTargetFromTrajectory`, `:231-277`)

```cpp
const auto& point = msg.points.front();   // 첫 포인트만 사용

if (!msg.joint_names.empty()) {
    // joint_names 가 있으면 이름으로 인덱스 매핑 (안전)
    for (i in msg.joint_names) {
        idx = model_joint_index_map_.find(msg.joint_names[i]);
        target_q[idx] = point.positions[i];
    }
    return true;
}

// fallback: positions.size() == arm_joint_names.size() 이면 순서대로
if (point.positions.size() == arm_joint_names.size()) {
    for (i in arm_joint_names) {
        target_q[model_joint_index_map_[arm_joint_names[i]]] = point.positions[i];
    }
    return true;
}

return false;
```

→ 호환성을 위해 두 방식 모두 지원하지만, joint_names 사용 권장.

#### `controlLoopCallback` (`:355-410`)

```cpp
q_feedback = q_commanded_;                  // 외부 피드백이 아닌 명령 누적값
kinematics_solver_->updateState(q_feedback, qdot_);

q_ref = q_feedback;
if (right_movej_target_initialized_) assignArmSegment(right_movej_goal_, right_arm, q_ref);
if (left_movej_target_initialized_)  assignArmSegment(left_movej_goal_,  left_arm,  q_ref);

desired_qdot = qdot_ref + kp_joint_ * (q_ref - q_feedback);   // qdot_ref = 0

qp_filter_->setDesiredJointVel(desired_qdot);
qp_filter_->setWeight(W·1, D·1);
qp_filter_->getOptJointVel(opt_qdot);

q_commanded_ = q_feedback + opt_qdot * time_step_;
publishTrajectory(q_commanded_);
qdot_ = opt_qdot;     // 다음 사이클 충돌 grad_dot 계산용
```

특이점:
- `q_feedback` 가 외부 피드백이 아니라 **자기 자신이 누적한 명령**(`q_commanded_`) 이다. 즉 노드는 자기 출력을 다시 입력해 닫힌 적분기로 동작하며, **충돌·관절한계만 QP가 강제하고 외부 피드백 점프는 절연**된다. 초기 동기화와 타임아웃 복귀에서만 외부 `q_` 로 리셋된다(`syncCommandStateToFeedback`).
- 결과적으로 MoveJ 노드는 **CBF 안전 필터** + **저역통과 효과** 의 결합.

#### 출력 (`createTrajectoryMsgWithGripper`, `:468-492`)

```
joint_names = [arm joints ...] + [gripper_joint_name]
positions   = [optimal_q (arm) ...] + [last_seen_gripper_position]
velocities  = 모두 0
time_from_start = trajectory_time_ (기본 0.05 s)
```

→ 한 포인트짜리 JointTrajectory. 다운스트림 컨트롤러가 0.05s에 도달하도록 보간.

### 3.3 MoveJ 코어 QP 컨트롤러 — `AIWorkerMoveJController`

VRController 와 **결정 변수·제약 구조는 동일**하다 (`ai_worker_movej_controller.cpp:33-81`). 차이는 **비용함수의 트래킹 항**:

```cpp
// MoveJ
P[qdot, qdot] += 2 * diag(w_tracking);                    //  q̇ᵀ·diag(w)·q̇
q[qdot]       += -2 * diag(w_tracking) * qdot_desired;    //  −2·w·q̇_des  → ‖q̇−q̇_des‖²
P[qdot, qdot] += 2 * diag(w_damping);                     //  추가 댐핑
```

VRController 의 `Jᵀ·W·J` 가 **단위행렬**로 바뀐 형태. 즉:
- VRController: `min ‖J·q̇ − ẋ_des‖²_W   + 댐핑 + 슬랙`
- MoveJ:        `min ‖q̇   − q̇_des‖²_W   + 댐핑 + 슬랙`

CBF (관절 위치한계, 충돌), 슬랙 처리, OSQP-Eigen 솔버 재사용 (희소 패턴 캐시) 은 동일 코드 흐름.

API:
```cpp
setDesiredJointVel(VectorXd qdot_desired);
setWeight(VectorXd w_tracking, VectorXd w_damping);
setControllerParams(slack_penalty, cbf_alpha, buffer, safe_distance);
getOptJointVel(VectorXd& opt_qdot);
```

---

## 4. 두 모드 비교 요약표

| 항목 | MoveL | MoveJ |
|---|---|---|
| 입력 메시지 | `robotis_interfaces/msg/MoveL` (PoseStamped + Duration) | `trajectory_msgs/msg/JointTrajectory` |
| 입력 토픽 | `/r_goal_move`, `/l_goal_move` | `/.../raw_joint_trajectory` |
| 입력 공간 | 작업공간 SE(3) | 관절공간 |
| 보간 | 100Hz 루프 내부에서 cubic + SO(3) 큐빅 | 보간 없음 (목표값을 그대로 P 제어) |
| 제어 게인 | `kp_position`, `kp_orientation` | `kp_joint` |
| QP 트래킹 항 | `‖J·q̇ − ẋ_des‖²_W` (Jacobian 사용) | `‖q̇ − q̇_des‖²_W` |
| CBF 충돌/한계 | 동일 (`VRController` 의 setIneqConstraint) | 동일 |
| 피드백 사용 | 매 사이클 `q_desired_` 누적 + 명령암만 측정으로 재정렬 | `q_commanded_` 누적 + 신규 명령 시 측정으로 재정렬 |
| 그리퍼 처리 | QP가 직접 제어 (조인트로 포함) | QP 미관여, 입력 그리퍼 값을 출력에 그대로 패치 |
| 출력 메시지 | 양팔 JointTrajectory (그리퍼 미포함) + (옵션)리프트 | 양팔 JointTrajectory (그리퍼 포함) |
| 출력 토픽 | `/.../joint_trajectory` | `/.../joint_trajectory` (필터링됨) |
| 부가 출력 | `r_gripper_pose`, `l_gripper_pose` (FK 결과) | 없음 |
| 리프트 제어 | `lift_vel_bound != 0` 일 때 별도 publish | 없음 |

---

## 5. 안전 기능 요약

두 노드 모두 다음 안전 메커니즘을 공유:

1. **조인트 상태 타임아웃**: `last_joint_state_time_` 이 `joint_state_timeout_` 이상 지나면 명령 송신 중단, 다음 수신 시 자동 재동기화. (`jointStateTimedOut()` + `syncCommandStateToFeedback()`)
2. **초기 동기화**: 첫 `/joint_states` 수신 전엔 어떤 명령도 무시 (`!q_desired_initialized_`).
3. **재명령 시 점프 방지**:
   - MoveL: 명령암만 측정으로 동기화 후 시작 자세 재산정.
   - MoveJ: `duration > 0` 일 때만 해당 팔을 측정으로 재정렬 (즉시 명령은 누적기에 합류).
4. **CBF 슬랙**: 정상 영역에서는 슬랙 0, 한계·충돌에 가까워지면 큰 페널티 ↔ 강제 제약 트레이드오프 → 솔버 infeasible 방지하면서도 실효 제약 유지.
5. **희소 패턴 캐시** (`qp_base.hpp:148-166`): 100Hz 실시간 보장을 위한 OSQP 재초기화 최소화.
6. **QP 실패 처리**:
   - MoveL: `controller_error_pub_` 로 `std_msgs/String` 에러 publish, 명령 중단.
   - MoveJ: `RCLCPP_WARN_THROTTLE` 1초 한 번 경고, 명령 중단.

---

## 6. 주요 라인 인덱스

### MoveL
- 노드 초기화: `ai_worker_movel_controller_node.cpp:23-140`
- 조인트 분류 (이름 패턴): `:147-184`
- 콜백 `rightMoveLCallback`/`leftMoveLCallback`: `:244-281`
- 메인 제어 루프: `:319-463`
- 보간 (cubicVector/rotationCubic/...): `common/type_define.hpp:66-273`
- 작업공간 PD 속도 계산: `:299-317`
- 코어 컨트롤러 (VR 상속): `ai_worker_movel_controller.cpp:31-36`
- VRController QP: `vr_controller.cpp:113-217`
- 인터랙티브 마커: `eef_interactive_marker_node.cpp`

### MoveJ
- 노드 초기화: `ai_worker_movej_controller_node.cpp:23-129`
- 입력 해석: `:231-277`
- 콜백: `:279-339`
- 메인 제어 루프: `:355-410`
- 그리퍼 패치 출력 메시지 빌드: `:468-492`
- 코어 컨트롤러 QP 설계: `ai_worker_movej_controller.cpp:33-222`
- 비용 항: `:123-147`
- 부등식 (CBF): `:167-216`

---

## 7. 정리

> **MoveL** 은 "작업공간 6D 목표 → cubic 보간 → Jacobian 기반 QP IK + CBF 안전 + 양팔 동시 트래킹" 의 풀-스택 IK 컨트롤러.
>
> **MoveJ** 는 "관절공간 raw 명령 → 동일한 QP 안에서 단위 Jacobian 으로 트래킹하되 CBF 로 충돌·관절한계 강제" 하는 **안전 필터** 성격이 강함. 리더팔/리타게팅 등 외부에서 산출된 관절 궤적을 그대로 흘려보내기 전에 한 번 정제하는 용도.
>
> 두 노드는 같은 `QPBase` 인프라(슬랙 변수 + CBF + 희소 패턴 캐시 + OSQP-Eigen) 위에서 **비용 함수만 다르게 정의**해 코드를 공유한다.
