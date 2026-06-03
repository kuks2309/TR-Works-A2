# cyclo_control QP+CBF MoveL 백엔드 통합

> **분석일**: 2026-06-03
> **패키지**: openarmx_pick

본 문서는 `openarmx_pick` 패키지가 외부 `cyclo_control` 모션 컨트롤러를 **MoveIt 대체 IK/모션 백엔드**로 사용하는 통합 구조를 코드 근거 기반으로 분석한다. 인용 경로는 별도 표기가 없으면 `openarmx_pick` 패키지 루트(`openarmx_ws/src/openarmx_pick/`) 기준 상대경로이고, `cyclo_control` 트리의 파일은 `cyclo_control/...` 절대 접두로 명시한다.

---

## 분석 범위

- `grasp_pose_node` 가 pre-grasp(사전 그랩) MoveL 명령을 어떤 토픽·메시지·frame 으로 publish 하는지.
- `cyclo_control` 패키지의 서브패키지 구조와 `omx_movel_controller_node` 의 소비 경로.
- MoveL goal → Pinocchio FK(Forward Kinematics, 순기구학)/Jacobian → OSQP(Operator Splitting Quadratic Program, 분할연산자 QP) → joint-limit / singularity / collision CBF(Control Barrier Function, 제어 장벽 함수) 제약 → `trajectory_msgs/JointTrajectory` 로 이어지는 내부 데이터 흐름.
- `openarmx_pick` 과 `cyclo_control` 사이의 **계약(contract)**: 토픽 이름, frame, 메시지 타입, 파라미터.
- 통합상의 사실적 위험요소(메시지 타입 네임스페이스 불일치, 미구현 CBF 항 등).

**범위 밖(다른 문서 소관)**: grasp 합성 기하/PCA 이론(`docs/03_grasp_synthesis_theory.md`), solver URDF 생성(`docs/04_solver_urdf.md`), launch 인자 상세(`docs/05_launch.md`), 검증 스크립트 절차(`docs/06_verification.md`), `grasp_pose_node` 내부 콜백 전체(`docs/02_grasp_pose_node.md`). 본 문서는 **cyclo 백엔드와의 경계면**에만 집중한다.

---

## 1. 통합 데이터 흐름 한눈에 보기

```
[box_plane 비전 파이프라인]                 (openarmx_pick 범위 밖)
   /box_plane/cloud (PointCloud2, camera_color_optical_frame)
          │
          ▼
grasp_pose_node                              (openarmx_pick/grasp_pose_node.py)
   tf2 → openarmx_body_link0 변환
   centroid + XY-PCA yaw + top-down 회전
          │  auto_send:=true 일 때만
          ▼  /openarmx/left/movel
   openarmx_scenario_player_msgs/MoveL  ◀── (publish 측 타입)
          │
          ╎  ※ 타입 네임스페이스 경계 (4절 참조)
          ▼  ~/movel  ← remap → /openarmx/left/movel
omx_movel_controller_node                    (cyclo_motion_controller_ros, robotis_interfaces/MoveL 구독)
   moveLCallback: poseMsgToEigen → goal Affine3d
   controlLoopCallback (100 Hz):
     KinematicsSolver(Pinocchio) FK/Jacobian
       → cubic Cartesian 보간 (start→goal)
       → 작업공간 desired twist (kp_position/kp_orientation)
       → OpenManipulatorMoveLController QP 구성
       → QPBase::solveQP (OSQP) → optimal qdot
       → q_commanded += qdot * time_step
          │
          ▼  /openarmx/left_arm/joint_trajectory
   trajectory_msgs/JointTrajectory  → (실제 로봇 forward controller, 미연결)
```

이 그림의 각 단계는 아래에서 코드 라인 단위로 검증한다.

---

## 2. `openarmx_pick` 측 송신 계약 (grasp_pose_node)

### 2.1 메시지 타입 import 와 publisher 생성

`grasp_pose_node` 는 MoveL 타입을 `openarmx_scenario_player_msgs` 에서 가져온다(`auto_send=False` 인 경우를 대비해 import 실패를 허용):

```
openarmx_pick/grasp_pose_node.py:38-42
    try:
        from openarmx_scenario_player_msgs.msg import MoveL
        _HAVE_MOVEL = True
    except Exception:  # MoveL only needed when auto_send=True
        _HAVE_MOVEL = False
```

publisher 는 `auto_send` 와 타입 가용성이 모두 참일 때만 생성되며, 토픽 기본값은 `/openarmx/left/movel` 이고 QoS(Quality of Service, 통신 품질) depth 는 10(기본 RELIABLE/VOLATILE)이다:

```
openarmx_pick/grasp_pose_node.py:107        self.declare_parameter("movel_topic", "/openarmx/left/movel")
openarmx_pick/grasp_pose_node.py:146-150
        self.movel_pub = None
        if self.auto_send and _HAVE_MOVEL:
            self.movel_pub = self.create_publisher(MoveL, gp("movel_topic").value, 10)
        elif self.auto_send:
            self.get_logger().warn("auto_send set but openarmx_scenario_player_msgs/MoveL unavailable.")
```

### 2.2 MoveL 메시지 채우기

`_send_movel` 는 base frame(`openarmx_body_link0`) 의 pre-grasp 위치/자세와 모션 시간을 채워 publish 한다:

```
openarmx_pick/grasp_pose_node.py:240-249
    def _send_movel(self, xyz, quat, stamp):
        m = MoveL()
        m.pose.header.frame_id = self.base_frame
        m.pose.header.stamp = stamp
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = map(float, xyz)
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = map(float, quat)
        m.time_from_start.sec = int(self.move_time)
        m.time_from_start.nanosec = int((self.move_time % 1.0) * 1e9)
        self.movel_pub.publish(m)
```

전달되는 `xyz` 는 grasp 점이 아니라 **pre-grasp(상공 호버) 점**이다. `_on_cloud` 에서 centroid 의 z 를 `pregrasp_height`(기본 0.10 m) 만큼 올린 값을 보낸다:

```
openarmx_pick/grasp_pose_node.py:194        pre_xyz = centroid.copy(); pre_xyz[2] += self.pregrasp_h
openarmx_pick/grasp_pose_node.py:198-199
        if self.movel_pub is not None and self._should_send(pre_xyz):
            self._send_movel(pre_xyz, quat, cloud.header.stamp)
```

즉 cyclo 백엔드로 가는 것은 **하강/그립/들어올리기 FSM(Finite State Machine, 유한상태기계) 이전의 상공 호버 1회 명령**뿐이다(README 의 "the descend/close/lift FSM is a later step", `grasp_pose_node.py:17` 와 일치).

### 2.3 디바운스(debounce) — 솔버 궤적 재시작 방지

매 카메라 프레임마다 MoveL 을 보내면 솔버가 매 사이클 궤적을 재시작해 팔이 기어가게 된다. `_should_send` 는 목표가 `send_min_delta`(기본 0.02 m) 이상 이동했거나 `send_min_interval`(기본 5.0 s) 쿨다운이 지났을 때만 재전송한다:

```
openarmx_pick/grasp_pose_node.py:226-238
    def _should_send(self, pre_xyz) -> bool:
        ...
        if moved > self.send_min_delta or elapsed > self.send_min_interval:
            self._last_sent_xyz, self._last_sent_t = np.asarray(pre_xyz), now
            return True
        return False
```

이 디바운스는 **cyclo 컨트롤러의 동작 특성(콜백마다 `motion_start_time_` 을 리셋하고 cubic 보간을 처음부터 다시 시작, 5.1절 참조)을 의식한 송신 측 보상**이다. 두 패키지의 계약이 토픽/메시지뿐 아니라 "재전송 빈도"라는 동적 규약까지 포함함을 보여준다.

---

## 3. `cyclo_control` 패키지 구조 (백엔드 소유 측)

`cyclo_control/README.md` 와 실제 디렉터리를 교차 확인한 결과, 통합에 관여하는 서브패키지는 다음과 같다.

| 서브패키지 | 역할 (통합 관점) | 본 통합에서 사용하는 핵심 산출물 |
|---|---|---|
| `cyclo_motion_controller_core` | 순수 C++ 코어: kinematics, controllers, optimization | `KinematicsSolver`(Pinocchio), `OpenManipulatorMoveLController`(QP), `QPBase`(OSQP), `math_utils`(cubic 보간) |
| `cyclo_motion_controller_ros` | ROS2 노드/launch/config 래퍼 | `omx_movel_controller_node` 실행파일, `config/omx_config.yaml` |
| `cyclo_motion_controller_models` | URDF/SRDF + 시각화 launch | (openarmx_pick 는 자체 URDF 사용 — 아래 4.3) |
| `cyclo_motion_controller_ros_py` | retargeting 파이썬 | 본 통합 미사용 |
| `osqp_eigen_vendor` | `osqp-eigen` 벤더 래핑 | QP 솔버 백엔드(빌드 의존) |

`openarmx_pick` 이 직접 호출하는 진입점은 **단 하나의 실행파일** `omx_movel_controller_node`(robot-agnostic, URDF·controlled_link 를 파라미터로 받음)이다. `openarmx_movel.launch.py:53-55` 가 이를 명시한다:

```
launch/openarmx_movel.launch.py:53-56
    movel = Node(
        package="cyclo_motion_controller_ros",
        executable="omx_movel_controller_node",
        name="openarmx_left_movel_controller",
```

> **워크스페이스 배치(검증됨)**: `cyclo_ws/src/cyclo_control` 은 `/home/openarmx/TR-Works/kkw/China/cyclo_control` 로의 **심볼릭 링크**이다(빌드된 소스와 분석한 소스가 동일 트리). `cyclo_control` 은 `cyclo_ws` 오버레이에서 빌드되고, `openarmx_pick` 과 MoveL 메시지는 `openarmx_ws` 에서 빌드된다(README "Build" 절, `README.md:48-68`).

---

## 4. 메시지 타입 계약 — 핵심 경계면

### 4.1 송신 타입과 수신 타입의 네임스페이스 불일치 (사실)

이 통합에서 가장 주의해야 할 사실은 **publish 측과 subscribe 측의 MoveL 타입 패키지 네임스페이스가 다르다**는 점이다.

- **송신(openarmx_pick)**: `openarmx_scenario_player_msgs/msg/MoveL`
  - `grasp_pose_node.py:39`, `scripts/verify_solver.py:19`
- **수신(cyclo omx 노드)**: `robotis_interfaces::msg::MoveL`
  - `cyclo_control/cyclo_motion_controller_ros/include/cyclo_motion_controller_ros/nodes/omx/omx_movel_controller_node.hpp:33` (`#include "robotis_interfaces/msg/move_l.hpp"`)
  - 같은 파일 `:96` (`rclcpp::Subscription<robotis_interfaces::msg::MoveL>::SharedPtr movel_sub_;`)
  - `cyclo_control/cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:84` (`create_subscription<robotis_interfaces::msg::MoveL>`)

두 `.msg` 정의는 **필드 레이아웃이 완전히 동일**하다(둘 다 `geometry_msgs/PoseStamped pose` + `builtin_interfaces/Duration time_from_start`):

| 출처 | 정의 |
|---|---|
| `../openarmx_ros2/openarmx_scenario_player_msgs/msg/MoveL.msg:5-6` | `geometry_msgs/PoseStamped pose` / `builtin_interfaces/Duration time_from_start` |
| `cyclo_ws/src/robotis_interfaces/msg/MoveL.msg:2-4` | `geometry_msgs/PoseStamped pose` / `builtin_interfaces/Duration time_from_start` |

`openarmx_scenario_player_msgs/MoveL.msg` 의 주석 자체가 출처를 밝힌다:

```
../openarmx_ros2/openarmx_scenario_player_msgs/msg/MoveL.msg:1-4
# Cartesian linear-motion target consumed by cyclo MoveL controller.
# Origin: cyclo_motion_controller_ros (forked from robotis_interfaces/MoveL).
# Defined locally to keep openarmx_scenario stack independent of any
# robot-vendor-specific message package.
```

### 4.2 왜 이것이 중요한가 (DDS 타입 매칭 관점)

ROS2 의 토픽 매칭은 **필드 구조가 아니라 정규화된 타입 이름(type name)** 으로 이루어진다. `openarmx_scenario_player_msgs/msg/MoveL` 와 `robotis_interfaces/msg/MoveL` 는 이름이 다르므로, 동일한 `/openarmx/left/movel` 토픽에 연결되더라도 **DDS(Data Distribution Service) 레벨에서 publisher↔subscriber 가 서로 호환 엔드포인트로 인식되지 않는다.** 따라서 다음이 **코드로 확인된 사실**이다.

- cyclo 의 `omx_movel_controller_node` 는 `robotis_interfaces::msg::MoveL` 만 구독한다(`omx_movel_controller_node.cpp:84`).
- `cyclo_control` 트리 전체에 `openarmx_scenario_player_msgs` 참조가 **하나도 없다**(`grep -rln openarmx_scenario_player_msgs cyclo_control` → 결과 0건, 검증됨).
- `openarmx_scenario_player_msgs` 는 `cyclo_ws/install` 에 빌드되어 있지 **않다**(존재하지 않음, 검증됨). `robotis_interfaces` 는 `cyclo_ws/install` 에 존재한다(검증됨).

> **결론(사실 + 평가)**: 현재 소스 상태 그대로라면 `grasp_pose_node(auto_send:=true)` 가 보내는 `openarmx_scenario_player_msgs/MoveL` 명령은 빌드된 `omx_movel_controller_node`(robotis_interfaces/MoveL 구독)에 **수신되지 않는다**. 이는 `README.md:50-52` 의 "the whole MoveL stack migrated to `openarmx_scenario_player_msgs`" 라는 서술과 **현재 cyclo 소스가 일치하지 않음**을 의미한다. 마이그레이션이 의도대로 동작하려면 다음 중 하나가 필요하다(아래는 검증되지 않은 **추정** 해결책):
> 1. cyclo 의 omx 노드 소스를 `openarmx_scenario_player_msgs::msg::MoveL` 구독으로 교체 후 재빌드(README 가 가정하는 상태로 보임 — **추정**), 또는
> 2. 두 타입 사이 브리지(타입 변환 relay) 노드 추가, 또는
> 3. grasp_pose_node 가 `robotis_interfaces/MoveL` 로 publish.
>
> 본 분석 시점의 트리에서는 위 1·2·3 어느 것도 적용되어 있지 않다. (검증 스크립트 `verify_e2e.py` 는 MoveL 토픽을 직접 echo/검사하지 않고 EE pose 수렴만 본다 — 5.4절. 즉 이 타입 불일치를 자동으로 잡아내지 못한다.)

### 4.3 frame 계약 — 추가 TF 불필요

타입 불일치와 별개로, **frame 계약은 정합한다**. grasp_pose_node 는 pose 를 `openarmx_body_link0` 에 표현하고(`grasp_pose_node.py:106`, `_send_movel` 의 `m.pose.header.frame_id = self.base_frame`), cyclo 솔버의 URDF 루트도 같은 링크다.

```
launch/openarmx_movel.launch.py:41-42
        DeclareLaunchArgument("base_frame", default_value="openarmx_body_link0"),
        DeclareLaunchArgument("controlled_link", default_value="openarmx_left_hand_tcp"),
```

README 가 명시하듯 `world → body_link0` 조인트가 identity 이므로 비전(grasp) frame 과 제어(solver root) frame 사이에 별도 TF 변환이 필요 없다(`README.md:24-26`). 단, **cyclo 노드는 들어온 pose 의 `header.frame_id` 를 검사하거나 TF 변환하지 않는다** — `poseMsgToEigen` 은 frame_id 를 무시하고 위치/쿼터니언을 그대로 Affine3d 로 읽는다:

```
cyclo_control/.../omx_movel_controller_node.cpp:268-283
Eigen::Affine3d OmxMoveLControllerNode::poseMsgToEigen(
  const geometry_msgs::msg::PoseStamped & pose_msg) const
{
  Eigen::Affine3d pose = Eigen::Affine3d::Identity();
  pose.translation() << pose_msg.pose.position.x, ... ;
  const Eigen::Quaterniond quat( ... );
  pose.linear() = quat.normalized().toRotationMatrix();
  return pose;
}
```

따라서 frame 정합은 **두 측이 동일 base frame 을 쓴다는 launch 규약에 전적으로 의존**한다(런타임 TF 안전망 없음). 송신 측이 다른 frame 의 pose 를 보내면 cyclo 는 그것을 base frame 좌표로 오해한다 — 이는 계약상 송신자가 반드시 base frame 으로 보내야 함을 뜻한다.

---

## 5. cyclo 내부 처리 경로 (MoveL → JointTrajectory)

### 5.1 MoveL 콜백 — goal 등록과 cubic 시작

`moveLCallback` 은 joint state 가 준비됐을 때만 동작하며, 현재 commanded 상태로부터 FK 로 시작 pose 를 잡고, MoveL 의 pose 를 goal 로, `time_from_start` 를 모션 지속시간으로 등록한다. **매 명령마다 `motion_start_time_` 을 now() 로 리셋**한다(이것이 2.3절 송신 디바운스의 근거):

```
cyclo_control/.../omx_movel_controller_node.cpp:245-266
void OmxMoveLControllerNode::moveLCallback(const robotis_interfaces::msg::MoveL::SharedPtr msg)
{
  ...
  const double requested_duration = commandDurationSeconds(msg->time_from_start);
  syncCommandStateToFeedback();
  kinematics_solver_->updateState(q_commanded_, qdot_);
  movel_start_pose_ = kinematics_solver_->getPose(controlled_link_);
  movel_goal_pose_ = poseMsgToEigen(msg->pose);
  active_motion_duration_ = requested_duration;
  motion_start_time_ = this->now();
  movel_target_initialized_ = true;
  movel_trajectory_active_ = requested_duration > -1.0;
}
```

### 5.2 제어 루프 — Pinocchio FK/Jacobian + cubic Cartesian 보간

제어 루프는 `control_frequency`(기본 100 Hz, `omx_config.yaml:3`) 타이머로 돈다. 매 tick:

1. `q_commanded_` 로 `KinematicsSolver::updateState` → Pinocchio `forwardKinematics` + `computeJointJacobians` + `updateFramePlacements`(`cyclo_control/.../kinematics_solver.cpp:149-156`).
2. `getPose(controlled_link)` 로 현재 EE Affine3d 획득 → `~/current_pose`(remap 후 `/openarmx/left_ee_pose`) 로 publish.
3. 경과시간 `elapsed < active_motion_duration_` 이면 **start→goal cubic 보간**으로 매 시점의 위치/회전 reference 와 feed-forward 속도를 생성:

```
cyclo_control/.../omx_movel_controller_node.cpp:337-371 (요약)
  position_ref = cubicVector<3>(elapsed, 0, dur, start.translation, goal.translation, 0, 0);
  rotation_ref = rotationCubic(elapsed, 0, dur, start.linear, goal.linear);
  linear_ref   = cubicDotVector<3>(...);   // feed-forward 선속도
  angular_ref  = rotationCubicDot(...);    // feed-forward 각속도
```

cubic 함수는 경계 위치/속도 조건을 만족하는 3차 다항식이고(`type_define.hpp:66-102`), `rotationCubic` 은 SO(3) 위에서 `R0 * exp(log(R0ᵀ Rf)·τ)` 형태의 보간을 수행한다(`type_define.hpp:230-248`). 보간이 끝나면(`elapsed >= dur`) goal 로 직접 수렴하는 모드로 전환한다(`omx_movel_controller_node.cpp:379-384`).

4. 작업공간 desired twist 는 feed-forward + P 제어(위치오차·자세오차)로 계산:

```
cyclo_control/.../omx_movel_controller_node.cpp:294-302
  position_error    = goal.translation - current.translation;
  rotation_error    = goal.linear * current.linear^T;  // AngleAxis → axis*angle
  desired_vel.head<3> = feedforward_linear  + kp_position_ * position_error;
  desired_vel.tail<3> = feedforward_angular + kp_orientation_ * orientation_error;
```

자세오차는 회전행렬 오차를 `Eigen::AngleAxisd` 로 풀어 `axis * angle` 로 표현한다(quaternion 이 아닌 axis-angle 잔차). `kp_position`/`kp_orientation` 기본값은 launch 에서 4.0 / 2.5(`openarmx_movel.launch.py:69-70`), config 기본은 50.0/50.0(`omx_config.yaml:7-8`)로 다르다 — **openarmx_pick launch 가 더 보수적인 게인으로 오버라이드**한다.

### 5.3 QP 구성 — OSQP + CBF 제약 (코어)

작업공간 desired twist 와 가중치는 `OpenManipulatorMoveLController` 로 전달되어 QP 로 풀린다:

```
cyclo_control/.../omx_movel_controller_node.cpp:393-408 (요약)
  qp_controller_->setDesiredTaskVel(desired_task_vel);
  qp_controller_->setWeights(task_weight, damping_weight);
  if (!qp_controller_->getOptJointVel(optimal_velocities)) { ...QP solve failed...; return; }
  q_commanded_ = q_feedback + optimal_velocities * time_step_;
  publishTrajectory(q_commanded_);
```

#### 5.3.1 결정변수 구성

QP 결정변수 `x` 는 관절속도 `qdot` 와 slack 변수들의 적층이다(`open_manipulator_movel_controller.cpp:54-90`):

| 블록 | 크기 | 의미 |
|---|---|---|
| `qdot` | `joint_dof`(=7, 좌팔) | 관절속도 |
| `slack_q_min` | `joint_dof` | 하한 joint-limit CBF 완화 |
| `slack_q_max` | `joint_dof` | 상한 joint-limit CBF 완화 |
| `slack_sing` | 1 | singularity CBF 완화(슬랙만 존재, 5.3.4 참조) |
| `slack_sel_col` | `collision pair 수` | self-collision CBF 완화 |

#### 5.3.2 비용함수 (task tracking + damping)

비용은 작업공간 추종 오차의 가중 2차형 + 관절속도 damping + slack penalty 이다:

```
cyclo_control/.../open_manipulator_movel_controller.cpp:141-172 (요약)
  J = robot_data_->getJacobian(controlled_link_);          // Pinocchio LOCAL_WORLD_ALIGNED 6×dof
  P += 2 * Jᵀ W_task J;   q += -2 * Jᵀ W_task xdot_desired;  // ‖J·qdot − xdot_desired‖²_W
  P += 2 * diag(damping_weight);                            // qdot 정규화(특이점 근처 폭주 억제)
  q[slack_*] = slack_penalty_;                              // 모든 slack 에 선형 penalty
```

즉 핵심 목적은 `min ‖J·qdot − xdot_desired‖²_W_task + ‖qdot‖²_damping + penalty·Σslack`. Jacobian 은 `KinematicsSolver::getJacobian` 이 Pinocchio `getFrameJacobian(LOCAL_WORLD_ALIGNED)` 로 제공한다(`cyclo_control/.../kinematics_solver.cpp:234-243`). damping 항이 **명시적 특이점(singularity) 완화 메커니즘**으로 작동한다(작은 가중치라도 `JᵀJ` 가 특이할 때 정칙화).

#### 5.3.3 제약 — joint-limit CBF (코드 확인)

bound 제약으로 관절속도 한계를 걸고(`open_manipulator_movel_controller.cpp:180-183`, `KinematicsSolver::getJointVelocityLimit`), 부등식 제약으로 **위치 limit CBF** 를 건다. CBF 형태는 `qdot ≥ -α(q - q_min)`, `-qdot ≥ -α(q_max - q)` 이고 slack 으로 완화된다:

```
cyclo_control/.../open_manipulator_movel_controller.cpp:216-217
  l_ineq[con_q_min] = -cbf_alpha_ * (q - q_min);     // 하한 접근 시 속도 제한
cyclo_control/.../open_manipulator_movel_controller.cpp:231-232
  l_ineq[con_q_max] = -cbf_alpha_ * (q_max - q);     // 상한 접근 시 속도 제한
```

`cbf_alpha`(기본 5.0)가 장벽의 수렴 속도를 정한다. README 의 "joints clamp exactly at limits → joint-limit CBF confirmed"(`README.md:124`) 가 이 항을 가리킨다.

#### 5.3.4 제약 — collision CBF (조건부) 와 singularity CBF (미구현 사실)

- **Self-collision CBF**: collision pair 가 있을 때만 활성. `KinematicsSolver::getCollisionPairDistances(true,...)` 가 거리와 그 관절-그래디언트를 Pinocchio FCL(거리/Jacobian)로 계산하고(`cyclo_control/.../kinematics_solver.cpp:250-362`), 거리 `≤ collision_buffer` 인 pair 만 CBF 하한을 설정한다:

```
cyclo_control/.../open_manipulator_movel_controller.cpp:234-253 (요약)
  A_ineq[con_sel_col, qdot] = res.grad^T;            // d(distance)/dq · qdot
  if (res.distance <= collision_buffer_)
      l_ineq[con_sel_col] = -cbf_alpha_ * (res.distance - collision_safe_distance_);
```

그러나 **현재 openarmx_pick 의 stage-1 URDF 는 collision 을 제거(stripped)했고 SRDF 가 비어 있어**(`README.md:34`, `openarmx_movel.launch.py:38-40`) collision pair 수가 0 이다. 따라서 `slack_sel_col_size = getCollisionPairCount() = 0` 이고 이 CBF 블록은 실질적으로 비활성이다(README "Stage-2 collision CBF" 가 향후 과제로 명시, `README.md:137-138`).

- **Singularity CBF (사실: 슬랙·인덱스만 할당, 부등식 미작성)**: 결정변수에 `slack_sing`(크기 1)과 부등식 인덱스 `con_sing`(크기 1)이 예약되어 있고 비용에 penalty 도 들어간다(`open_manipulator_movel_controller.cpp:57,61,168`). 그러나 `setIneqConstraint()`(같은 파일 `:194-255`)는 `con_q_min`, `con_q_max`, `con_sel_col` 블록만 채우고 **`con_sing_start` 행에 A 계수도, l/u 경계도 쓰지 않는다.** 결과적으로 singularity 부등식 행은 `[-OSQP_INFTY, +OSQP_INFTY]` 의 빈 제약으로 남아 **실효 제약이 없다**. 즉 현재 구현에서 특이점 회피는 (i) 위 damping 비용항과 (ii) 관절속도 bound 로만 간접 수행되고, **전용 singularity CBF 부등식은 미구현 상태**다(코드 확인된 사실; manipulability 지표 계산 호출도 없음).

#### 5.3.5 OSQP 솔버 (QPBase)

QP 는 `QPBase::solveQP` 가 `min ½xᵀPx + qᵀx s.t. l ≤ Ax ≤ u` 형태로 OSQP(`OsqpEigen`)에 넘긴다(`cyclo_control/.../optimization/qp_base.hpp:133-179`). bound/ineq/eq 를 한 행렬 `A` 로 적층하고, **희소 패턴이 바뀌지 않으면 warm-start 업데이트, 바뀌면 솔버 재초기화**하는 최적화가 있다(`qp_base.hpp:148-166`). 해의 `qdot` 블록만 추출해 반환한다(`open_manipulator_movel_controller.cpp:129-139`).

### 5.4 출력 — JointTrajectory

해 `qdot` 를 `time_step`(0.01 s) 적분해 `q_commanded_` 를 갱신하고, 단일 point JointTrajectory 로 publish 한다. joint 이름은 모델(Pinocchio) 조인트 순서이고, `time_from_start` 는 `trajectory_time`(launch 기본 0.05 s)이다:

```
cyclo_control/.../omx_movel_controller_node.cpp:194-209 (요약)
  traj_msg.joint_names = model_joint_names_;
  point.time_from_start = Duration::from_seconds(trajectory_time_);
  point.positions[i] = q_command[i]; point.velocities[i] = 0.0;
```

토픽은 launch 에서 `/openarmx/left_arm/joint_trajectory` 로 매핑된다(`openarmx_movel.launch.py:44-47`). README 는 이 토픽을 실제 로봇의 `forward_position_controller` 로 연결하는 작업이 **아직 미완**임을 명시한다(`README.md:139-141`).

> **현 단계의 폐루프는 시뮬레이션 self-loop**: `verify_e2e.py` 는 두 콜백으로 self-loop 를 구성한다. `_on_cmd`(`:69-75`)가 `/openarmx/left_arm/joint_trajectory` 를 받아 `self.q` 를 갱신하고, `_js_tick`(`:61-64`)이 10 ms 마다 `self.q` 를 `/joint_states` 로 publish 한다. 즉 cyclo 는 자신이 명령한 값을 피드백으로 받아 EE pose 수렴을 보인다. 실로봇 피드백 경로는 미연결(추정: forward controller 와이어링 후 완성).

---

## 6. 파라미터 계약 (openarmx_pick → cyclo)

`openarmx_movel.launch.py` 가 `omx_movel_controller_node` 에 넘기는 파라미터가 두 패키지 사이의 정량 계약이다. cyclo 의 기본 `omx_config.yaml` 과 다른 값(오버라이드)을 표시한다.

| 파라미터 | openarmx_pick launch 값 | cyclo omx_config.yaml 기본 | 비고 |
|---|---|---|---|
| `urdf_path` | `urdf/openarmx_left_solver.urdf` | `omx_f.urdf` | **openarmx 전용 7-DOF 좌팔 solver URDF** 사용 |
| `srdf_path` | `""`(빈 값) | `omx_f.srdf` | stage-1: collision pair 없음 |
| `base_frame` | `openarmx_body_link0` | `link0` | frame 계약(4.3) |
| `controlled_link` | `openarmx_left_hand_tcp` | `end_effector_link` | TCP 링크 |
| `joint_states_topic` | `/joint_states` | `/joint_states` | 동일 |
| `joint_command_topic` | `/openarmx/left_arm/joint_trajectory` | `/leader/joint_trajectory` | 출력 재매핑 |
| `movel_topic`(remap `~/movel`) | `/openarmx/left/movel` | `~/movel` | grasp_pose_node 와 합의된 토픽 |
| `control_frequency` | 100.0 | 100.0 | 동일 |
| `time_step` | 0.01 | 0.01 | 동일 |
| `trajectory_time` | 0.05 | 0.0 | point 의 time_from_start |
| `kp_position` | 4.0 | 50.0 | **보수적 오버라이드** |
| `kp_orientation` | 2.5 | 50.0 | **보수적 오버라이드** |
| `weight_task_position` | 10.0 | 10.0 | 동일 |
| `weight_task_orientation` | 1.0 | 1.0 | 동일 |
| `weight_damping` | 0.05 | 0.001 | **damping 강화**(특이점 안정성↑) |
| `slack_penalty` | 1000.0 | 1000.0 | 동일 |
| `cbf_alpha` | 5.0 | 5.0 | 동일 |
| `joint_state_timeout` | 0.5 | 0.5 (C++ 소스 기본값; yaml 미기재) | timeout 시 명령 hold |

(출처: `launch/openarmx_movel.launch.py:58-77`, `cyclo_control/cyclo_motion_controller_ros/config/omx_config.yaml:1-26`. `omx_config.yaml` 은 launch 의 dict 파라미터로 덮어쓰이므로 일부 값은 launch 가 우선.)

`joint_state_timeout` 안전장치: cyclo 는 joint state 가 0.5 s 이상 끊기면 모션을 비활성화하고 신선한 피드백이 올 때까지 명령을 hold 한다(`omx_movel_controller_node.cpp:316-325`, `jointStateTimedOut`). 복구 시 `syncCommandStateToFeedback` 로 commanded 상태를 현재 피드백에 재동기화한다(`:234-242`).

---

## 7. 토픽 계약 요약표

| 토픽 | 타입 | 방향(openarmx_pick 기준) | 상대(cyclo) | 비고 |
|---|---|---|---|---|
| `/openarmx/left/movel` | **송신**: `openarmx_scenario_player_msgs/MoveL` / **수신**: `robotis_interfaces/MoveL` | out → in | `omx_movel_controller_node` `~/movel` remap | **타입 네임스페이스 불일치(4.1)** |
| `/joint_states` | `sensor_msgs/JointState` | (외부 로봇/시뮬레이터) → in | cyclo 피드백 입력 | grasp_pose_node 는 미사용 |
| `/openarmx/left_arm/joint_trajectory` | `trajectory_msgs/JointTrajectory` | cyclo out → (실제 controller) | 명령 출력 | 실로봇 와이어링 미완(`README.md:139`) |
| `/openarmx/left_ee_pose` | `geometry_msgs/PoseStamped` | cyclo out | `~/current_pose` remap | 현재 EE pose(검증 스크립트가 수렴 확인용 구독) |
| `~/controller_error`(remap 없음) | `std_msgs/String` | cyclo out | QP 실패/루프 예외 보고 | `omx_movel_controller_node.cpp:211-220` |
| `/openarmx/grasp_pose` | `geometry_msgs/PoseStamped` | grasp out | (cyclo 미구독) | grasp 점(pre-grasp 아님). cyclo 입력 아님 |

---

## 8. 통합 관점의 결론과 위험요소

1. **계약의 본체는 단일 토픽 `/openarmx/left/movel` + base frame `openarmx_body_link0` + (PoseStamped, Duration) 페이로드**이다. frame·페이로드 구조·파라미터 계약은 정합한다.
2. **메시지 타입 네임스페이스 불일치(4절)는 현재 트리에서 실재하는 통합 결함이다.** 빌드된 cyclo `omx_movel_controller_node` 는 `robotis_interfaces/MoveL` 만 구독하는데, openarmx_pick 송신부와 검증 스크립트는 `openarmx_scenario_player_msgs/MoveL` 로 publish 한다. ROS2 타입명 매칭 규칙상 이 둘은 같은 토픽에서 자동 연결되지 않는다. README 가 주장하는 "MoveL stack migration" 이 cyclo 측 소스/빌드에는 반영되어 있지 않다(검증된 사실). 해결 방향은 4.2절의 1·2·3 중 하나(어느 것이 의도인지는 **추정** 영역).
3. **CBF 적용 범위(사실)**: 현재 활성 CBF 는 **joint-limit CBF 뿐**이다. collision CBF 는 stage-1 URDF 에 collision pair 가 없어 비활성, singularity CBF 는 슬랙/인덱스만 예약되고 부등식이 작성되지 않아 미구현이다. 특이점 회피는 비용함수의 damping 항과 관절속도 bound 로만 간접 수행된다.
4. **폐루프 검증은 시뮬레이션 self-loop 한정**: 실로봇 forward controller 와의 연결이 미완이라, 현재 통합 검증은 cyclo 가 자기 명령을 피드백으로 되받는 e2e 스크립트(`_on_cmd` + `_js_tick` self-loop) 범위에 머문다.
5. **동적 계약**: grasp_pose_node 의 송신 디바운스는 cyclo 가 매 MoveL 마다 cubic 보간을 재시작하는 특성에 대한 송신 측 보상이다. 즉 통합 계약은 정적 인터페이스뿐 아니라 "재전송 빈도" 라는 동적 규약을 포함한다.

> 본 문서의 모든 인용은 분석 시점(2026-06-03)의 소스 트리 기준이며, 코드로 직접 확인되지 않은 의도/해결책은 본문에서 '추정' 으로 명시했다.
