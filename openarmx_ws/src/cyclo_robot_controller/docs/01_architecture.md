# 01. 아키텍처 (Architecture)

[← 문서 목차](README.md)

---

## 1. 패키지 계층

```
cyclo_robot_controller/
├── cyclo_motion_controller_core/      # (A) ROS 비의존 코어 라이브러리
│   ├── kinematics/      KinematicsSolver        (Pinocchio FK / Jacobian / 충돌거리)
│   ├── optimization/    QPBase                  (OSQP 래퍼, min ½xᵀPx+qᵀx s.t. l≤Ax≤u)
│   ├── controllers/     OpenManipulatorMoveL/J, VRController, AIWorkerMoveL/J
│   └── common/          type_define.hpp         (Vector6d, cubic 보간, MinDistResult)
│
├── cyclo_motion_controller_ros/       # (B) ROS2 래퍼
│   ├── nodes/           omy/ omx/ ai_worker/    (코어 컨트롤러 ↔ ROS 토픽 연결)
│   ├── utils/           controller_params, pose_utils, trajectory_utils
│   │                    eef_interactive_marker_node, reference_checker_node
│   ├── launch/          omy_/omx_/ai_worker_controller.launch.py
│   └── config/          omy_/omx_/ai_worker_config.yaml
│
├── cyclo_motion_controller_ros_py/    # (C) 텔레오퍼레이션 리타게팅 (Python)
│   └── scripts/         arm_retargeting.py, teleop_retargeting.py
│
├── cyclo_motion_controller_models/    # (D) 로봇 모델 (URDF/SRDF/mesh/rviz)
│   └── models/          omx/ omy/ ai_worker/ hx5_d20/
│
└── osqp_eigen_vendor/                 # (E) OSQP-Eigen 솔버 vendoring
```

**의존 방향**: (B) ROS 노드 → (A) 코어 → (E) OSQP / Pinocchio / Eigen.
(A) 코어는 ROS 에 의존하지 않으므로 단독 테스트·재사용이 가능합니다.

---

## 2. 제어 데이터 흐름 (single-arm MoveL 기준)

```
 [목표 Pose]                                  ┌──────────── cyclo_motion_controller_ros ────────────┐
 MoveL msg ──► movel_sub ──► moveLCallback ──►│ start_pose = FK(controlled_link) @ q_commanded      │
 (pose,                                       │ goal_pose  = msg.pose,  duration = time_from_start  │
  time_from_start)                            └──────────────────────────┬──────────────────────────┘
                                                                         │
 /joint_states ──► jointStateCallback ──► q_, qdot_ (모델 관절 순서로 정렬)│
                                                                         ▼
                              control_timer (100 Hz) ──► controlLoopCallback()
                                                                         │
   ┌─────────────────────────────────────────────────────────────────────┐
   │ 1) kinematics_solver.updateState(q_commanded_, qdot_)   ← open-loop  │
   │ 2) current_pose = FK(controlled_link)                                │
   │ 3) cubic 보간: pose_ref(t), 피드포워드 속도(linear/angular_ref)       │
   │ 4) desired_task_vel = ff_vel + Kp · pose_error   (computeDesiredVelocity)
   │ 5) qp_controller.setDesiredTaskVel / setWeights                      │
   │ 6) qp_controller.getOptJointVel(qdot_opt)   ← QP 1회 풀이 (OSQP)      │
   │ 7) q_commanded_ += qdot_opt · time_step                             │
   └────────────────────────────────┬────────────────────────────────────┘
                                     ▼
   joint_command_pub ──► JointTrajectory(위치 1점) ──► (하위 trajectory 컨트롤러)
   ee_pose_pub       ──► PoseStamped (현재 EE pose)
   controller_error_pub ──► String (QP 실패 등)
```

### 핵심 설계 특징 (코드 검증됨)

1. **속도가 아닌 위치를 명령한다.**
   QP 해 `qdot_opt` 를 `q_commanded_ += qdot_opt * time_step` 로 적분하여
   **위치 1점짜리 `JointTrajectory`** 를 publish 합니다
   ([omy_movel_controller_node.cpp:346-347](../cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp#L346-L347)).

2. **Open-loop 적분.**
   제어 루프는 운동학 업데이트에 실측 위치(`q_`)가 아니라 **직전 명령값 `q_commanded_`**
   를 사용합니다([:263-264](../cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp#L263-L264)).
   실측 위치는 초기화 시점과 joint-state 타임아웃 복구 시에만 `q_commanded_` 에 동기화됩니다
   (`syncCommandStateToFeedback`). 즉 평상시에는 자체 적분 궤적을 따라갑니다.

3. **모션 종료 후 publish 중단.**
   `elapsed >= duration` 이면 명령을 더 내보내지 않고 마지막 명령 자세를 유지합니다.
   소스 주석에 따르면, 종료 후에도 `Kp·error` 를 계속 가하면 100 Hz 에서 ±0.01 rad 진동이
   발생해 이를 제거한 것입니다([:319-322](../cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp#L319-L322)).

4. **궤적 보간은 cubic(3차).**
   위치는 `cubicVector`, 자세는 SO(3) 위의 `rotationCubic`, 피드포워드 속도는
   `cubicDotVector`/`rotationCubicDot` 로 생성합니다
   ([type_define.hpp](../cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp)).

---

## 3. 제어 모드(컨트롤러 타입) 개요

| 타입 | 코어 클래스 | 입력 | 태스크 | 비고 |
| --- | --- | --- | --- | --- |
| `movel` | `OpenManipulatorMoveLController` | `MoveL`(pose) | 단일 EE 데카르트 추종 | OMX/OMY/AI Worker |
| `movej` | `OpenManipulatorMoveJController` | 관절 목표 | 관절 공간 추종 | OMX/OMY/AI Worker |
| `vr` | `VRController` | 좌/우 goal+elbow pose | **양팔** 동시 IK + 팔꿈치 | AI Worker 텔레오퍼레이션 |
| `leader` | (FK 전용) `LeaderController` + `VRController` | 리더 장치 관절 | 리더 관절→FK→goal pose 생성 | Leader-Follower |

> AI Worker 의 MoveL/MoveJ 컨트롤러는 양팔/리프트/그리퍼를 함께 다루는 별도 ROS 노드이며,
> 코어단에서는 `VRController`(다중 링크 태스크 맵) 정식화를 공유합니다. 자세한 차이는
> [03_controllers.md](03_controllers.md) 참고.

---

## 4. 외부 라이브러리 역할

| 라이브러리 | 사용처 | 무엇을 하는가 |
| --- | --- | --- |
| **Pinocchio** | `KinematicsSolver` | URDF 로딩, FK, 프레임 야코비안(`LOCAL_WORLD_ALIGNED`), HPP-FCL 충돌거리/그라디언트 |
| **OSQP** (via OsqpEigen) | `QPBase` | 희소 QP 풀이. warm-start + 희소 패턴 캐시로 재초기화 최소화 |
| **Eigen** | 전역 | 밀집/희소 선형대수 |
| **nlopt** | (ros_py 리타게팅) | 텔레오퍼레이션 손/팔 리타게팅 최적화 |

다음: [02. QP/CBF 정식화 →](02_qp_cbf_formulation.md)
