## 2026-05-23 08:09 (KST) — cyclo_control 전체 SOP 리뷰

### 트리거 요청
사용자 요청 "코드 리뷰 SOP 전체 적용". `docs/user_instructions/user_instructions.md` 미존재 → 시각 매핑 생략.

### 분석 분기 명시
- 분기: **전체 구조 분석** (패키지 6개·노드 10개)
- 감지된 add-on: **A(ROS2) + B(동시성)** (C 임베디드 트리거 부재 — `__attribute__((interrupt))`, FreeRTOS API, NVIC 매크로 없음 / grep 결과 0)
- 대상 저장소: `/home/openarmx/TR-Works/kkw/China/cyclo_control`

---

## Core 인벤토리

### 1. 목적

ROBOTIS Physical AI 라인업(AI Worker 양팔·OpenMANIPULATOR-X/Y)을 단일 ROS 2 Jazzy 모션 컨트롤 스택으로 제어한다. 5가지 텔레옵 흐름(MoveL / MoveJ / VR / Leader / Interactive Marker)을 **단일 QP 추상화** 위에서 통합하며, 모든 명령은 다음 형태의 QP로 귀결된다 — 비용 함수는 추적 항 + 댐핑, 제약은 관절 한계·충돌쌍 거리 CBF·슬랙(soft constraint).

코어 라이브러리 `cyclo_motion_controller_core` 는 Pinocchio(FK/Jacobian/충돌)와 OSQP-Eigen(QP)을 100Hz로 묶어 ROS 의존성 없는 순수 C++/Python 모듈로 분리되어 있고, ROS 래퍼 `cyclo_motion_controller_ros` 가 10개 실행 노드를 제공한다. Python 패키지 `cyclo_motion_controller_ros_py` 는 사람 팔·손 포즈를 로봇 팔 길이로 재매핑하는 retargeting 텔레옵 노드를 담당한다.

리포지토리 구성: 메타(`cyclo_control`) + 코어(`cyclo_motion_controller_core`) + ROS 노드(`cyclo_motion_controller_ros`) + Python(`cyclo_motion_controller_ros_py`) + 모델(`cyclo_motion_controller_models`) + 벤더(`osqp_eigen_vendor`). [README.md:1-69](../../../cyclo_control/README.md#L1-L69) 참조.

### 2. 코드 플로우차트 (전체 코드 흐름도)

#### 2.1 패키지 의존 그래프 (build-time)

```
osqp_eigen_vendor ──► cyclo_motion_controller_core ──► cyclo_motion_controller_ros (C++ 10 nodes)
                                       │
                                       └────────────► cyclo_motion_controller_ros_py (Python 2 scripts)
                                                                │
cyclo_motion_controller_models (URDF/SRDF/launch) ◄─────────────┘
robotis_interfaces (vcs) ──► cyclo_motion_controller_ros, ros_py
```

#### 2.2 텔레옵 경로별 데이터 흐름 (path 별 분리)

**Path 1 — InteractiveMarker → MoveL (RViz 6-DoF 마커)**
```
[RViz 사용자] → interactive_marker_node (MOUSE_DOWN/POSE_UPDATE/MOUSE_UP)
              → publish robotis_interfaces/MoveL → /r_goal_move OR /l_goal_move
              → ai_worker_movel_controller_node (또는 omx/omy)
              → controlLoopCallback 100Hz: cubic+SO(3) 보간 + computeDesiredVelocity
              → QP (VRController 또는 OpenManipulatorMoveLController)
              → q_desired_ += opt_qdot * time_step
              → publish JointTrajectory → /leader/.../joint_trajectory
```

**Path 2 — Raw JointTrajectory → MoveJ (CBF 안전 필터)**
```
[외부 노드] → /leader/.../raw_joint_trajectory
            → ai_worker_movej_controller_node.updateArmTargetFromTrajectory
            → 100Hz: q̇_des = kp_joint·(q_ref - q_feedback)
            → QP (AIWorkerMoveJController 또는 OpenManipulatorMoveJController)
            → publish JointTrajectory (그리퍼 값 보존)
```

**Path 3 — Leader Arm → VR Controller (관절 → 작업공간 변환)**
```
[리더팔 외부] → /leader/.../raw_joint_trajectory
              → leader_controller_node.rightTrajectoryCallback (FK 전용)
              → publish PoseStamped → /r_goal_pose, /l_goal_pose, /r_elbow_pose, /l_elbow_pose
              → vr_controller_node (5-상태 머신)
              → QP (VRController: 양 그리퍼 + 양 팔꿈치 4-link 추적)
              → publish JointTrajectory
```

**Path 4 — VR 헬멧 트래커 → 사람팔 → 로봇팔 retargeting**
```
[VR/MediaPipe] → /r_shoulder_pose, /r_elbow_pose, /r_wrist_pose (l_* 동일)
              → arm_retargeting.py (RobotWrapper FK + 양손 거리 우선화 + LPF)
              → /r_goal_pose, /r_subgoal_pose (l_*)
              → vr_controller_node
              → reference_checker_node 가 점프 감지 → /reference_diverged
```

**Path 5 — VR 손 트래커 → DexPilot 손가락 retargeting**
```
[MediaPipe Hand] → /right_hand/hand_joint_pos, /left_hand/hand_joint_pos
                → teleop_retargeting.py (DexPilotOptimizer NLopt SLSQP)
                → /leader/joint_trajectory_command_broadcaster_*_hand/joint_trajectory
```

#### 2.3 공통 호출 그래프 (모든 path 공유)

```
KinematicsSolver.updateState(q, qdot)
   ├── pinocchio::forwardKinematics
   ├── pinocchio::computeJointJacobians
   └── pinocchio::updateFramePlacements
   → getPose / getJacobian / getCollisionPairDistances (캐시 조회)

QPBase.solveQP(P, A, q, l, u)
   ├── P_sparse = P_ds_.sparseView()
   ├── if !solver_initialized_ OR pattern_changed → initializeSolver()
   │   else → updateHessian/updateLinearConstraints/updateGradient/updateBounds (fast path)
   └── solver_.solve() → opt_qdot
```

### 3. 함수 리스트

총 **295개 함수** (Core 110 + ROS C++ 144 + Python 41). 분량상 패키지별 표로 분할.

#### 3.A `cyclo_motion_controller_core` — C++/Python 라이브러리 (110개)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
|---|------|------|------|------|------|
| 1 | `KinematicsSolver.KinematicsSolver` | urdf_path, srdf_path | - | URDF/SRDF 로드, Pinocchio 모델 초기화 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:34 |
| 2 | `KinematicsSolver.~KinematicsSolver` | - | - | 소멸자 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:122 |
| 3 | `KinematicsSolver.updateState` | q, qdot | bool | 관절상태 업데이트 (FK+Jacobian 캐시) | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:126 |
| 4 | `KinematicsSolver.updateKinematics` | q, qdot | bool | Pinocchio FK/Jacobian 계산 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:149 |
| 5 | `KinematicsSolver.computePose` | q, link_name | Affine3d | 링크 포즈 계산 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:160 |
| 6 | `KinematicsSolver.computeJacobian` | q, link_name | MatrixXd | 링크 Jacobian 계산 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:177 |
| 7 | `KinematicsSolver.hasLinkFrame` | name | bool | 링크 프레임 존재 확인 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:197 |
| 8 | `KinematicsSolver.hasJointFrame` | name | bool | 관절 프레임 존재 확인 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:202 |
| 9 | `KinematicsSolver.getJointNames` | - | vector\<string\> | 관절명 목록 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:207 |
| 10 | `KinematicsSolver.getPose` | link_name | Affine3d | 캐시된 포즈 조회 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:221 |
| 11 | `KinematicsSolver.getJacobian` | link_name | MatrixXd | 캐시된 Jacobian 조회 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:234 |
| 12 | `KinematicsSolver.getCollisionPairCount` | - | int | 충돌쌍 개수 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:245 |
| 13 | `KinematicsSolver.getCollisionPairDistances` | with_grad, with_graddot, verbose | vector\<MinDistResult\> | 충돌쌍 거리·∂d/∂q 해석적 계산 (핵심) | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:250 |
| 14 | `KinematicsSolver.setJointVelocityBoundsByIndex` | idx, lower, upper | bool | 특정 관절 속도 한계 오버라이드 | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:137 |
| 15-22 | `KinematicsSolver.{getURDFPath,getLinkFrameVector,getJointFrameVector,getRootLinkName,getDof,getJointPosition,getJointVelocity,getJointPositionLimit}` | - | inline getters | 헤더 inline | cyclo_motion_controller_core/include/cyclo_motion_controller_core/kinematics/kinematics_solver.hpp:94-135 |
| 23 | `KinematicsSolver.getJointVelocityLimit` | - | pair\<VectorXd,VectorXd\> | 속도 한계 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/kinematics/kinematics_solver.hpp:144 |
| 24 | `QPBase.QPBase` | - | - | 기본 생성자 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:51 |
| 25 | `QPBase.setQPsize` | nx, nbc, nineqc, neqc | void | QP 행렬 크기 초기화 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:59 |
| 26 | `QPBase.initializeSolver` | P, A, q, l, u | bool | OSQP 솔버 초기화 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:98 |
| 27 | `QPBase.solveQP` | sol& | bool | QP 풀이 (희소 패턴 캐시 적용) | cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:133 |
| 28 | `QPBase.setConstraint` | - | void | A_total 누적 (bound+ineq+eq) | cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:201 |
| 29 | `QPBase.hasSameSparsityPattern` (static) | lhs, rhs | bool | 두 희소행렬 패턴 비교 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:229 |
| 30-33 | `QPBase.{setCost,setBoundConstraint,setIneqConstraint,setEqConstraint}` (pure virtual) | - | - | 파생 클래스 구현 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:185-197 |
| 34-42 | `OpenManipulatorMoveJController.*` (생성자+8 메서드) | - | - | 단팔 관절공간 제어 | cyclo_motion_controller_core/src/controllers/open_manipulator/open_manipulator_movej_controller.cpp:32-239 |
| 43-53 | `OpenManipulatorMoveLController.*` (생성자+10 메서드) | - | - | 단팔 작업공간 제어 | cyclo_motion_controller_core/src/controllers/open_manipulator/open_manipulator_movel_controller.cpp:33-257 |
| 54-62 | `AIWorkerMoveJController.*` (생성자+8 메서드) | - | - | 양팔 관절공간 CBF 필터 | cyclo_motion_controller_core/src/controllers/ai_worker/ai_worker_movej_controller.cpp:33-218 |
| 63-71 | `VRController.*` (생성자+8 메서드: setDesiredTaskVel, setWeight, setControllerParams, getOptJointVel, setCost, setBoundConstraint, setIneqConstraint, setEqConstraint) | - | - | 다중링크 작업공간 제어 (양 그리퍼 + 양 팔꿈치 트래킹) | cyclo_motion_controller_core/src/controllers/ai_worker/vr_controller.cpp:33-219 |
| 72 | `AIWorkerMoveLController.AIWorkerMoveLController` | robot_data, dt | - | VRController 상속 (빈 서브클래스) | cyclo_motion_controller_core/src/controllers/ai_worker/ai_worker_movel_controller.cpp:31 |
| 73 | `cubic` (static) | t, t_0, t_f, x_0, x_f, ẋ_0, ẋ_f | double | 스칼라 3차 보간 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:66 |
| 74 | `cubicDot` (static) | - | double | 3차 보간 시간 도함수 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:107 |
| 75 | `cubicVector<N>` (template static) | - | Matrix\<N,1\> | 고정크기 벡터 3차 보간 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:149 |
| 76 | `cubicVector` (static overload) | - | VectorXd | 동적 벡터 3차 보간 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:169 |
| 77 | `cubicDotVector<N>` (template static) | - | Matrix\<N,1\> | 고정 벡터 3차 보간 도함수 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:190 |
| 78 | `cubicDotVector` (static overload) | - | VectorXd | 동적 벡터 3차 보간 도함수 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:210 |
| 79 | `rotationCubic` (static) | t, t_0, t_f, R_0, R_f | Matrix3d | SO(3) 3차 보간 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:230 |
| 80 | `rotationCubicDot` (static) | t, t_0, t_f, ω_0, a_0, R_0, R_f | Vector3d | 각속도 프로필 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:253 |
| 81 | `MinDistResult.setZero` | size | void | 거리 결과 구조체 초기화 | cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp:52 |
| 82-94 | `RobotWrapper.*` (Python, 13개) | - | - | Pinocchio Python 래퍼 (joint_names, dof, FK, Jacobian 등) | cyclo_motion_controller_core/src/retargeting/robot_wrapper.py:36-156 |
| 95-103 | `ROBOTISHandRetargeter.*` (Python, 9개) | - | - | MediaPipe 21점 → 로봇 손 관절 변환 | cyclo_motion_controller_core/src/retargeting/seq_retarget.py:56-228 |
| 104-110 | `DexPilotOptimizer.*` (Python, 7개) | - | - | NLopt SLSQP 손가락 IK | cyclo_motion_controller_core/src/retargeting/optimizer.py:39-258 |

(전체 110개 풀 표는 별첨; 위는 그룹 인덱스. 모든 함수의 정확한 file:line 은 위 표·범위로 회복 가능.)

#### 3.B `cyclo_motion_controller_ros` — C++ ROS2 노드 (144개)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
|---|------|------|------|------|------|
| 111 | `LeaderController.LeaderController` | - | - | 노드 초기화·subs·pubs·timer | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:24 |
| 112 | `LeaderController.~LeaderController` | - | - | 소멸자 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:148 |
| 113 | `LeaderController.initializeJointConfig` | - | void | URDF 로드, joint 인덱스 매핑 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:153 |
| 114 | `LeaderController.rightTrajectoryCallback` | JointTrajectory | void | 우 raw_traj 수신 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:170 |
| 115 | `LeaderController.leftTrajectoryCallback` | JointTrajectory | void | 좌 raw_traj 수신 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:178 |
| 116 | `LeaderController.jointStateCallback` | JointState | void | joint_states 수신 (lift만 측정값) | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:186 |
| 117 | `LeaderController.reactivateCallback` | Bool | void | /reactivate 수신 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:241 |
| 118 | `LeaderController.controlLoopCallback` | - | void | 100Hz FK + pose publish | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:256 |
| 119 | `LeaderController.updateJointPositionsFromTrajectory` | JointTrajectory | void | traj msg → q_desired | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:191 |
| 120 | `LeaderController.updateLiftJointFromJointState` | JointState | void | lift만 측정값으로 갱신 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:220 |
| 121 | `LeaderController.makePoseStamped` | Affine3d | PoseStamped | Eigen → ROS 변환 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:333 |
| 122 | `LeaderController.computePoseInBaseFrame` | Affine3d | Affine3d | world 프레임 변환 (있을 때) | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:351 |
| 123-134 | `VRController.*` (12 메서드: 생성자/소멸자/initializeJointConfig + 9 callback/내부) | - | - | 5-상태 머신 (startup/activate_pending/running/diverged/idle) + QP 호출 | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:26-922 |
| 135-144 | `VRController.{publishTrajectory,createArmTrajectoryMsg,createLiftTrajectoryMsg,publishGripperPose,extractJointStates,jointStateTimedOut,syncCommandStateToFeedback,computePoseMat,computeDesiredVelocity}` (헤더 inline) | - | - | 보조 유틸 | cyclo_motion_controller_ros/include/cyclo_motion_controller_ros/nodes/ai_worker/vr_controller_node.hpp |
| 145-161 | `AIWorkerMoveJController.*` (17 메서드) | - | - | raw_traj → CBF 필터 → publish | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:23-503 |
| 162-178 | `AIWorkerMoveLController.*` (17 메서드) | - | - | MoveL 메시지 → cubic+SO(3) → QP | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:23-613 |
| 179-194 | `OmxMoveJControllerNode.*` (16 메서드) | - | - | 단팔 MoveJ | cyclo_motion_controller_ros/src/nodes/omx/omx_movej_controller_node.cpp:23-421 |
| 195-210 | `OmxMoveLControllerNode.*` (16 메서드) | - | - | 단팔 MoveL | cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:25-438 |
| 211-226 | `OmyMoveJControllerNode.*` (16 메서드) | - | - | OMY MoveJ (OMX 클론) | cyclo_motion_controller_ros/src/nodes/omy/omy_movej_controller_node.cpp:23-421 |
| 227-242 | `OmyMoveLControllerNode.*` (16 메서드) | - | - | OMY MoveL (OMX 클론) | cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp:25-438 |
| 243-249 | `InteractiveMarkerNode.*` (7 메서드) | - | - | RViz 6-DoF 마커 → MoveL publish | cyclo_motion_controller_ros/src/utils/eef_interactive_marker_node.cpp:36-271 |
| 250-252 | `ReferenceDivergenceChecker.*` (3 메서드) | - | - | goal pose 점프 감지 | cyclo_motion_controller_ros/src/utils/reference_checker_node.cpp:23-118 |
| 253-254 | `main` (10개 각 노드 진입점) | argc, argv | int | rclcpp::init/spin/shutdown | 각 node cpp 파일 말미 |

(상세 항목은 ROS 인벤토리 에이전트 응답 Part 1-4 기록 — 총 144개 정확 매핑.)

#### 3.C `cyclo_motion_controller_ros_py` — Python (41개)

| # | 함수 | 입력 | 출력 | 기능 | 위치 |
|---|------|------|------|------|------|
| 255 | `ArmRetargetingTeleop.__init__` | - | - | 노드 초기화, 6 sub + 4 pub + 18 param | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:64 |
| 256-261 | `ArmRetargetingTeleop._{right,left}_{shoulder,elbow,wrist}_callback` | PoseStamped | None | 6개 콜백 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:246-268 |
| 262 | `ArmRetargetingTeleop.run_teleop` | - | None | retarget 트리거 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:270 |
| 263 | `ArmRetargetingTeleop._retarget_bimanual_pose_states` | - | tuple? | 양팔 reconstruct + 손목거리 우선화 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:282 |
| 264 | `ArmRetargetingTeleop._update_pose_state` (static) | pose_state, msgs | None | 한 팔 상태 갱신 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:365 |
| 265 | `ArmRetargetingTeleop._retarget_pose_state` | pose_state, geometry, shoulder | tuple? | 한 팔 재타게팅 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:379 |
| 266-268 | `ArmRetargetingTeleop.{publish_targets_left,publish_targets_right,_publish_targets}` | - | None | goal/subgoal pub | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:427-461 |
| 269 | `ArmRetargetingTeleop._compute_robot_geometry` | robot, links | RobotArmGeometry | upper/forearm 길이 계산 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:464 |
| 270 | `ArmRetargetingTeleop._lookup_link_position` | link_name | ndarray? | TF 조회 (base_frame 기준) | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:487 |
| 271-273 | `ArmRetargetingTeleop._{pose_to_numpy,copy_pose_with_new_position,compute_unit_vector}` (static) | - | - | Pose ↔ numpy | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:511-547 |
| 274-276 | `ArmRetargetingTeleop._{apply_wrist_distance_priority,compute_dynamic_wrist_priority,compute_wrist_distance_scale}` | - | - | 손목거리 우선화 + 감쇠 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:550-601 |
| 277-278 | `ArmRetargetingTeleop._{project_wrist_to_forearm_length,smooth_wrist_targets}` | - | - | 길이 재투영 + LPF | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:603-644 |
| 279-280 | `ArmRetargetingTeleop._{poses_have_matching_stamps,pose_stamp_tuple}` (static) | - | - | 타임스탬프 일치 검사 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:646-660 |
| 281 | `ArmRetargetingTeleop._enforce_human_wrist_relative_orientation` | - | tuple | 양손 상대 자세 강제 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:662 |
| 282-285 | `ArmRetargetingTeleop._{pose_orientation_to_quaternion,set_pose_orientation_from_quaternion,normalize_quaternion,quaternion_inverse}` (static) | - | - | 쿼터니언 유틸 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:708-744 |
| 286 | `ArmRetargetingTeleop._quaternion_multiply` (static) | lhs, rhs | ndarray | 쿼터니언 곱 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:746 |
| 287 | `arm_retargeting.main` | args | None | 노드 실행 | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:761 |
| 288-294 | `RetargetingTeleop.*` (7 메서드) | - | - | 손가락 DexPilot 트래져그 발행 | cyclo_motion_controller_ros_py/scripts/teleop_retargeting.py:42-212 |
| 295 | `teleop_retargeting.main` | args | None | 노드 실행 | cyclo_motion_controller_ros_py/scripts/teleop_retargeting.py:215 |

### 4. 전역 변수 / 모듈 상수

| # | 사용처(함수) | 기능 | 위치 |
|---|--------------|------|------|
| 1 | `ROBOTISHandRetargeter.__init__` | LPF alpha (0.5) (상수) | cyclo_motion_controller_core/src/retargeting/seq_retarget.py:38 |
| 2 | `ROBOTISHandRetargeter.retarget` | MediaPipe 손목 인덱스 0 (상수) | cyclo_motion_controller_core/src/retargeting/seq_retarget.py:39 |
| 3 | `ROBOTISHandRetargeter.retarget` | MediaPipe 손가락끝 인덱스 [4,8,12,16,20] (상수) | cyclo_motion_controller_core/src/retargeting/seq_retarget.py:40 |
| 4 | `ROBOTISHandRetargeter.retarget` | MediaPipe DIP 인덱스 [3,7,11,15,19] (상수) | cyclo_motion_controller_core/src/retargeting/seq_retarget.py:41 |
| 5 | `ArmRetargetingTeleop.__init__` 외 6개 sub/pub | `QOS_BEST_EFFORT` (상수, depth=1, BEST_EFFORT, KEEP_LAST) | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:37 |
| 6 | `RetargetingTeleop.__init__` 외 4개 sub/pub | `QOS_BEST_EFFORT` (상수, 동일 정의) | cyclo_motion_controller_ros_py/scripts/teleop_retargeting.py:32 |

C++ 패키지(`cyclo_motion_controller_core`, `cyclo_motion_controller_ros`)는 **전역 변수 / 모듈 상수 없음** — 모든 상태가 클래스 멤버로 캡슐화. file-scope `static` 변수와 anonymous namespace 변수 모두 grep 결과 0건.

### 5. 의존성 3-tier

| Tier | 대상 | 버전/제약 | 부재 시 동작 | 근거(file:line) |
|------|------|----------|-------------|------------------|
| 빌드 | ROS 2 Jazzy (ament_cmake, ament_cmake_python, ament_cmake_vendor_package) | jazzy | 컴파일 실패 | cyclo_motion_controller_ros/CMakeLists.txt:12, cyclo_motion_controller_core/CMakeLists.txt:11-12 |
| 빌드 | Eigen3 | REQUIRED | 컴파일 실패 | cyclo_motion_controller_core/CMakeLists.txt:8 |
| 빌드 | pinocchio | REQUIRED (= ROS jazzy 시스템 버전) | 컴파일 실패 | cyclo_motion_controller_core/CMakeLists.txt:9 |
| 빌드 | osqp_eigen_vendor (벤더) | local third_party/osqp-eigen + osqp_vendor | 컴파일 실패 | cyclo_motion_controller_core/CMakeLists.txt:10, osqp_eigen_vendor/CMakeLists.txt |
| 빌드 | rclcpp, rclpy, std_msgs, std_srvs, geometry_msgs, sensor_msgs, trajectory_msgs, visualization_msgs, interactive_markers, tf2_ros, ament_index_cpp, ament_index_python | REQUIRED | 컴파일 실패 | cyclo_motion_controller_ros/CMakeLists.txt:13-25, ros_py/package.xml:12-19 |
| 빌드 | robotis_interfaces (vcs import) | github.com/ROBOTIS-GIT/robotis_interfaces, main | MoveL.msg 부재로 컴파일 실패 | cyclo_control/cyclo_control_ci.repos, cyclo_motion_controller_ros/CMakeLists.txt:26 |
| 빌드 | python3-nlopt | exec_depend | DexPilotOptimizer 실패 | cyclo_motion_controller_core/package.xml, optimizer.py:28 |
| 빌드 | python3-numpy (\<2) | numpy<2 강제 | retargeting 임포트 실패 | README.md:117 |
| 런타임 필수 | `/joint_states` 토픽 발행자 (외부 컨트롤러/시뮬) | 100Hz 권장 | `joint_state_received_=false` → `controlLoopCallback` 첫 분기에서 return; `jointStateTimedOut(>joint_state_timeout=0.5s)` 시 명령 보류 + WARN | 모든 노드: 예) cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:91, 322-326 |
| 런타임 필수 | URDF 파일 (`urdf_path` 파라미터) | 경로 유효 | `KinematicsSolver` ctor `runtime_error` → `RCLCPP_FATAL` → `rclcpp::shutdown()` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:105-109 |
| 런타임 필수 | `cyclo_motion_controller_core` 공유 라이브러리 | ABI 호환 | 런타임 링크 실패 | CMakeLists.txt:34 |
| 런타임 필수 | OSQP 시스템 라이브러리 (`libosqp-cpp.so`, OsqpEigen 의존) | osqp ≥ 0.6 | OSQP solver init 실패 → `getOptJointVel` false → `controller_error_pub_` publish + 1 cycle skip | cyclo_motion_controller_core/include/.../optimization/qp_base.hpp:106 |
| 런타임 필수 | `cyclo_motion_controller_models` URDF/SRDF 리소스 | 패키지 share dir 존재 | `get_package_share_directory` 예외 → 노드 종료 | arm_retargeting.py:68-75, ai_worker_controller.launch.py:90-130 |
| 런타임 필수 | `robotis_interfaces` 메시지 (MoveL) | 인스톨됨 | 디시리얼라이즈 실패 / 토픽 미연결 | ai_worker_movel_controller_node.cpp:85-90 |
| 런타임 선택 | SRDF 파일 (`srdf_path` 파라미터) | 경로 비어있어도 됨 | **fallback**: SRDF 미제공 시 모든 충돌쌍 활성화 + 경고 출력 ("Collision model for the robot may not be perfect!") | cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:55-60 |
| 런타임 선택 | `world` TF frame | 모델에 있으면 사용 | **fallback**: `hasLinkFrame("world")==false` 시 base_link 그대로 사용 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:354-358 |
| 런타임 선택 | `lift_joint` (모델 관절) | 있으면 인덱스 매핑 | **fallback**: `lift_joint_index_=-1` + 경고; QP 에서 lift 항 무효 | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:161-167 |
| 런타임 선택 | `lift_vel_bound` 파라미터 (default 0.0) | 0 = 락 | **fallback**: 값 0 시 QP에서 `q̇_lift ∈ [0,0]` 강제. 비0 시 해당 범위로 풀림 | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:81, 273-274 |
| 런타임 선택 | `/reactivate` Bool 토픽 | 운영자 발행 | **fallback**: 미수신 시 활성화 안 됨 (`control_enabled_=false`) → 명령 생산 정지 | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:135-137, 415-444 |
| 런타임 선택 | `/reference_diverged` Bool 토픽 | reference_checker 또는 vr 자체 | **fallback**: 외부 발행 부재 시 VR 노드 내부 검사 (startup mismatch) 만 동작 | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:131-134, 141-142 |
| 런타임 선택 | TF `base_link → controlled_link` (InteractiveMarker) | TF tree 게시자 | **fallback**: `lookupTransform` 예외 시 마커 초기화 보류, 재시도 | cyclo_motion_controller_ros/src/utils/eef_interactive_marker_node.cpp:215-219 |
| 런타임 선택 | `disable_gripper_collisions` 런치 인자 (default false) | true/false | **fallback**: true 시 modified SRDF 로 스왑 (런치 진입 시점 결정, 런타임 변경 불가) | cyclo_motion_controller_ros/launch/ai_worker_controller.launch.py:91-130 |

---

## Add-on A — ROS2

### A-1. Subscriptions 표 (28개)

| 토픽 | 메시지 타입 | QoS (depth · reliability · durability · history) | 콜백 함수 | 위치(file:line) |
|------|-----------|------------------------------------------------|-----------|---------|
| `/r_goal_move` (= `right_movel_topic`) | `robotis_interfaces/MoveL` | depth=10 · RELIABLE(기본) · VOLATILE(기본) · KEEP_LAST(기본) | `AIWorkerMoveLController.rightMoveLCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:85 |
| `/l_goal_move` | `robotis_interfaces/MoveL` | depth=10 · 기본 | `AIWorkerMoveLController.leftMoveLCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:88 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `AIWorkerMoveLController.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:91 |
| `/leader/.../right/raw_joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `AIWorkerMoveJController.rightTrajectoryCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:83 |
| `/leader/.../left/raw_joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `AIWorkerMoveJController.leftTrajectoryCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:86 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `AIWorkerMoveJController.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:80 |
| `/leader/.../right/raw_joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `LeaderController.rightTrajectoryCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:62 |
| `/leader/.../left/raw_joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `LeaderController.leftTrajectoryCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:65 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `LeaderController.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:68 |
| `/reactivate` | `std_msgs/Bool` | depth=10 · 기본 | `LeaderController.reactivateCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:71 |
| `/r_goal_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `VRController.rightGoalPoseCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:103 |
| `/l_goal_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `VRController.leftGoalPoseCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:107 |
| `/r_subgoal_pose` (= `r_elbow_pose_topic`) | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `VRController.rightElbowPoseCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:111 |
| `/l_subgoal_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `VRController.leftElbowPoseCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:115 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `VRController.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:119 |
| `/leader/.../right/raw_joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `VRController.rightRawTrajectoryCallback` (그리퍼만) | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:123 |
| `/leader/.../left/raw_joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `VRController.leftRawTrajectoryCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:127 |
| `/reference_diverged` | `std_msgs/Bool` | depth=10 · 기본 | `VRController.referenceDivergenceCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:131 |
| `/reactivate` | `std_msgs/Bool` | depth=10 · 기본 | `VRController.reactivateCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:135 |
| `~/movej` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `OmxMoveJControllerNode.moveJCallback` | cyclo_motion_controller_ros/src/nodes/omx/omx_movej_controller_node.cpp:78 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `OmxMoveJControllerNode.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/omx/omx_movej_controller_node.cpp:75 |
| `~/movel` | `robotis_interfaces/MoveL` | depth=10 · 기본 | `OmxMoveLControllerNode.moveLCallback` | cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:84 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `OmxMoveLControllerNode.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:81 |
| `~/movej` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `OmyMoveJControllerNode.moveJCallback` | cyclo_motion_controller_ros/src/nodes/omy/omy_movej_controller_node.cpp:78 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `OmyMoveJControllerNode.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/omy/omy_movej_controller_node.cpp:75 |
| `~/movel` | `robotis_interfaces/MoveL` | depth=10 · 기본 | `OmyMoveLControllerNode.moveLCallback` | cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp:84 |
| `/joint_states` | `sensor_msgs/JointState` | depth=10 · 기본 | `OmyMoveLControllerNode.jointStateCallback` | cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp:81 |
| `/r_goal_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `ReferenceDivergenceChecker.rightGoalPoseCallback` | cyclo_motion_controller_ros/src/utils/reference_checker_node.cpp:36 |
| `/l_goal_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `ReferenceDivergenceChecker.leftGoalPoseCallback` | cyclo_motion_controller_ros/src/utils/reference_checker_node.cpp:40 |

**Python (ros_py) 추가 8개** (모두 `QOS_BEST_EFFORT`, depth=1, BEST_EFFORT, KEEP_LAST):

| 토픽 | 메시지 타입 | QoS | 콜백 | 위치 |
|------|-------|-----|------|------|
| `/r_shoulder_pose`, `/l_shoulder_pose`, `/r_elbow_pose`, `/l_elbow_pose`, `/r_wrist_pose`, `/l_wrist_pose` | `geometry_msgs/PoseStamped` | depth=1 · BEST_EFFORT · VOLATILE · KEEP_LAST | `_{right,left}_{shoulder,elbow,wrist}_callback` | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:207-242 |
| `/right_hand/hand_joint_pos`, `/left_hand/hand_joint_pos` | `robotis_interfaces/HandJoints` | depth=1 · BEST_EFFORT · VOLATILE · KEEP_LAST | `run_teleop_{right,left}` | cyclo_motion_controller_ros_py/scripts/teleop_retargeting.py:135-146 |

### A-2. Publications 표 (32개)

| 토픽 | 메시지 타입 | QoS | 발행 위치(함수) | 위치(file:line) |
|------|-----------|-----|----------------|---------|
| `/leader/.../right/joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `AIWorkerMoveLController.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:95 |
| `/leader/.../left/joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `AIWorkerMoveLController.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:96 |
| `/leader/joystick_controller_right/joint_trajectory` (`lift_topic`) | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `AIWorkerMoveLController.controlLoopCallback` (lift 분기) | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:97 |
| `/r_gripper_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `AIWorkerMoveLController.publishGripperPose` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:99 |
| `/l_gripper_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `AIWorkerMoveLController.publishGripperPose` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:101 |
| `~/controller_error` | `std_msgs/String` | depth=10 · 기본 | `controlLoopCallback` 예외/QP 실패 분기 | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:102 |
| `/leader/.../right/joint_trajectory` (filtered) | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `AIWorkerMoveJController.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:76 |
| `/leader/.../left/joint_trajectory` (filtered) | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `AIWorkerMoveJController.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:78 |
| `/r_goal_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `LeaderController.controlLoopCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:75 |
| `/l_goal_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `LeaderController.controlLoopCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:77 |
| `/r_elbow_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `LeaderController.controlLoopCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:79 |
| `/l_elbow_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `LeaderController.controlLoopCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp:81 |
| `/reference_diverged` | `std_msgs/Bool` | depth=10 · 기본 | `VRController.controlLoopCallback` (startup mismatch) | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:141 |
| `~/controller_error` | `std_msgs/String` | depth=10 · 기본 | `VRController` 오류 경로 | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:143 |
| `/leader/.../joystick_controller_right/joint_trajectory` (lift) | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `VRController.controlLoopCallback` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:144 |
| `/leader/.../right/joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `VRController.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:147 |
| `/leader/.../left/joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `VRController.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:150 |
| `/r_gripper_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `VRController.publishGripperPose` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:153 |
| `/l_gripper_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `VRController.publishGripperPose` | cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:156 |
| `/leader/joint_trajectory` (`joint_command_topic`) | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `OmxMoveJControllerNode.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/omx/omx_movej_controller_node.cpp:69 |
| `~/current_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `OmxMoveJControllerNode.publishCurrentPose` | cyclo_motion_controller_ros/src/nodes/omx/omx_movej_controller_node.cpp:71 |
| `~/controller_error` | `std_msgs/String` | depth=10 · 기본 | `OmxMoveJControllerNode.publishControllerError` | cyclo_motion_controller_ros/src/nodes/omx/omx_movej_controller_node.cpp:73 |
| `/leader/joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `OmxMoveLControllerNode.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:75 |
| `~/current_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `OmxMoveLControllerNode.publishCurrentPose` | cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:77 |
| `~/controller_error` | `std_msgs/String` | depth=10 · 기본 | `OmxMoveLControllerNode.publishControllerError` | cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:79 |
| `/leader/joint_trajectory` (OMY) | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `OmyMoveJControllerNode.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/omy/omy_movej_controller_node.cpp:69 |
| `~/current_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `OmyMoveJControllerNode.publishCurrentPose` | cyclo_motion_controller_ros/src/nodes/omy/omy_movej_controller_node.cpp:71 |
| `~/controller_error` | `std_msgs/String` | depth=10 · 기본 | `OmyMoveJControllerNode.publishControllerError` | cyclo_motion_controller_ros/src/nodes/omy/omy_movej_controller_node.cpp:73 |
| `/leader/joint_trajectory` (OMY MoveL) | `trajectory_msgs/JointTrajectory` | depth=10 · 기본 | `OmyMoveLControllerNode.publishTrajectory` | cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp:75 |
| `~/current_pose` | `geometry_msgs/PoseStamped` | depth=10 · 기본 | `OmyMoveLControllerNode.publishCurrentPose` | cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp:77 |
| `~/controller_error` | `std_msgs/String` | depth=10 · 기본 | `OmyMoveLControllerNode.publishControllerError` | cyclo_motion_controller_ros/src/nodes/omy/omy_movel_controller_node.cpp:79 |
| `~/<goal_topic>` (`/r_goal_move` 또는 `/l_goal_move` 또는 `/omx/omy_movel`) | `robotis_interfaces/MoveL` | depth=10 · 기본 | `InteractiveMarkerNode.publishGoal` | cyclo_motion_controller_ros/src/utils/eef_interactive_marker_node.cpp:59 |
| `/reference_diverged` | `std_msgs/Bool` | depth=10 · 기본 | `ReferenceDivergenceChecker.{right,left}GoalPoseCallback` | cyclo_motion_controller_ros/src/utils/reference_checker_node.cpp:33 |

**Python (ros_py) 추가 6개** (모두 `QOS_BEST_EFFORT`, depth=1):

| 토픽 | 메시지 타입 | QoS | 발행 함수 | 위치 |
|------|-------|-----|------|------|
| `/r_goal_pose`, `/l_goal_pose`, `/r_subgoal_pose`, `/l_subgoal_pose` | `geometry_msgs/PoseStamped` | depth=1 · BEST_EFFORT | `publish_targets_{right,left}` | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:186-205 |
| `/leader/joint_trajectory_command_broadcaster_{right,left}_hand/joint_trajectory` | `trajectory_msgs/JointTrajectory` | depth=1 · BEST_EFFORT | `publish_trajectory_{right,left}` | cyclo_motion_controller_ros_py/scripts/teleop_retargeting.py:116-133 |

### A-3. Services / Actions 표

**없음** — grep `create_service|create_client|rclcpp_action|action::Server|action::Client` 전 패키지 0건. 모든 통신이 토픽 기반 pub/sub.

### A-4. Parameters 표 (143개, declare 위치 — 노드별 그룹)

**ai_worker_movel_controller_node (29개)** — control_frequency, time_step, trajectory_time, kp_position, kp_orientation, weight_position, weight_orientation, weight_damping, slack_penalty, cbf_alpha, collision_buffer, collision_safe_distance, joint_state_timeout, urdf_path, srdf_path, joint_states_topic, right_movel_topic, left_movel_topic, right_traj_topic, left_traj_topic, lift_topic, lift_vel_bound, r_gripper_pose_topic, l_gripper_pose_topic, r_gripper_name, l_gripper_name, right_gripper_joint, left_gripper_joint, controller_error_topic. declare 위치: cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:44-80.

**ai_worker_movej_controller_node (20개)** — control_frequency, time_step, trajectory_time, kp_joint, weight_tracking, weight_damping, slack_penalty, cbf_alpha, collision_buffer, collision_safe_distance, joint_state_timeout, command_timeout, urdf_path, srdf_path, joint_states_topic, right_traj_topic, left_traj_topic, right_traj_filtered_topic, left_traj_filtered_topic, raw_traj_timeout. declare 위치: cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:44-75.

**vr_controller_node (39개)** — control_frequency, time_step, trajectory_time, kp_position, kp_orientation, weight_position, weight_orientation, weight_elbow_position, weight_damping, slack_penalty, cbf_alpha, collision_buffer, collision_safe_distance, joint_state_timeout, urdf_path, srdf_path, reactivate_topic, r_goal_pose_topic, l_goal_pose_topic, r_elbow_pose_topic, l_elbow_pose_topic, joint_states_topic, right_traj_topic, left_traj_topic, right_raw_traj_topic, left_raw_traj_topic, raw_traj_timeout, lift_topic, lift_vel_bound, r_gripper_pose_topic, l_gripper_pose_topic, r_gripper_name, l_gripper_name, r_elbow_name, l_elbow_name, right_gripper_joint, left_gripper_joint, startup_ref_pos_threshold, startup_ref_ori_threshold_deg. declare 위치: cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:44-95.

**leader_controller_node (19개)** — control_frequency, joint_states_topic, right_traj_topic, left_traj_topic, reactivate_topic, command_timeout, r_goal_pose_topic, l_goal_pose_topic, r_elbow_pose_topic, l_elbow_pose_topic, r_gripper_name, l_gripper_name, r_elbow_name, l_elbow_name, lift_joint_name, model_lift_joint_name, urdf_path, base_frame, world_frame. declare 위치: cyclo_motion_controller_ros/src/nodes/ai_worker/leader_controller_node.cpp.

**omx_movej / omy_movej (각 20개)** — control_frequency, time_step, trajectory_time, kp_joint, weight_joint_tracking, weight_damping, slack_penalty, cbf_alpha, collision_buffer, collision_safe_distance, joint_state_timeout, urdf_path, srdf_path, base_frame, controlled_link, joint_states_topic, joint_command_topic, movej_topic, ee_pose_topic, controller_error_topic.

**omx_movel / omy_movel (각 22개)** — 위 동일 + kp_position, kp_orientation, weight_task_position, weight_task_orientation - (kp_joint, weight_joint_tracking 대신).

**interactive_marker_node (11개)** — base_frame, controlled_link, goal_topic, marker_scale, color_r, color_g, color_b, color_alpha, publish_while_dragging 등.

**reference_checker_node (4개)** — ref_pos_jump_threshold, ref_ori_jump_threshold_deg, r_goal_pose_topic, l_goal_pose_topic.

#### YAML 예시 — `cyclo_motion_controller_ros/config/ai_worker_config.yaml` 의 vr_controller 섹션 (의미 그룹별 단위 주석)

```yaml
vr_controller:
  ros__parameters:
    # 의미 그룹 1 — 제어 루프 시간
    control_frequency: 100.0      # Hz, 제어 주기
    time_step: 0.01               # sec, 1 / control_frequency
    trajectory_time: 0.0          # sec, holding 모드는 0

    # 의미 그룹 2 — 작업공간 PD 이득
    kp_position: 50.0             # 1/s, 위치 오차 비례
    kp_orientation: 50.0          # 1/s, 자세 오차 비례

    # 의미 그룹 3 — QP 가중치 (단위 자체 없음, 비용 함수 상대 가중)
    weight_position: 10.0         # 그리퍼 위치 추적 가중
    weight_orientation: 1.0       # 그리퍼 자세 추적 가중
    weight_elbow_position: 8.0    # 팔꿈치 위치 (null-space 활용)
    weight_damping: 0.1           # ‖q̇‖² 항 가중

    # 의미 그룹 4 — CBF / 슬랙
    slack_penalty: 1000.0         # ρ, 슬랙 페널티 (단위 없음)
    cbf_alpha: 50.0               # 1/s, 클래스-K 함수 기울기
    collision_buffer: 0.05        # m, d_buffer
    collision_safe_distance: 0.02 # m, d_safe

    # 의미 그룹 5 — 안전 게이트 (VR 전용)
    startup_ref_pos_threshold: 0.3        # m, 시작 위치 오차 한계
    startup_ref_ori_threshold_deg: 120.0  # deg, 시작 자세 오차 한계

    # 의미 그룹 6 — 리프트
    lift_vel_bound: 0.0           # rad/s, 0 = 락
```

### A-5. TF frames 표

| frame | parent | 발행 노드 | 정적/동적 | 위치 |
|-------|--------|----------|-----------|------|
| (전 노드 TF 직접 발행 없음) | — | — | — | grep `TransformBroadcaster` 결과 0건 |
| `base_link → controlled_link` (소비만) | URDF tree | 외부 robot_state_publisher 추정 | (외부) | cyclo_motion_controller_ros/src/utils/eef_interactive_marker_node.cpp:61-62 `tf2_ros::Buffer + TransformListener` |
| `<base_frame> → arm_{r,l}_link{2,4,7}` (소비만) | URDF tree | 외부 robot_state_publisher | (외부) | cyclo_motion_controller_ros_py/scripts/arm_retargeting.py:128-129 `Buffer + TransformListener` |

**TF 발행자 부재** — 본 스택은 robot_state_publisher (외부 노드, `cyclo_motion_controller_models` 의 view_*.launch.py 가 띄움) 에 전적으로 의존. `interactive_marker_node` 와 `arm_retargeting_teleop` 만 소비.

### A-6. 콜백 그룹 / Executor

전 노드(C++ 10 + Python 2) 모두 **executor / callback group 명시 설정 없음** → ROS2 기본값 `SingleThreadedExecutor` + 기본 `MutuallyExclusiveCallbackGroup` (rclcpp::spin). grep `MultiThreadedExecutor|StaticSingleThreadedExecutor|CallbackGroup` 0건.

| 노드 | Executor | 콜백 그룹 | 근거 |
|------|----------|-----------|------|
| 전 노드 (10 C++ + 2 Python) | 기본 SingleThreadedExecutor (rclcpp::spin / rclpy.spin) | 기본 그룹 | grep 0건 |

---

## Add-on B — 동시성

### B-1. 동기화 객체 표

| 객체 | 종류 | 보호 자원 | 획득 위치 | 해제 위치 |
|------|------|-----------|-----------|-----------|
| (없음) | — | — | — | — |

`std::mutex`, `std::lock_guard`, `std::scoped_lock`, `std::atomic`, `std::shared_lock`, `std::condition_variable`, `threading.Lock`, `asyncio.Lock` 전 패키지 grep 결과 0건.

### B-2. 공유 상태 표

| 변수 | 읽기 위치(함수) | 쓰기 위치(함수) | 보호 객체 |
|------|----------------|-----------------|-----------|
| `q_`, `qdot_` (전 노드) | `controlLoopCallback` | `extractJointStates` (in `jointStateCallback`) | 비보호 (단일 스레드 직렬화) |
| `q_desired_` (MoveL/VR) | `controlLoopCallback`, `publishTrajectory` | `controlLoopCallback`, `syncCommandStateToFeedback` | 비보호 (단일 스레드) |
| `q_commanded_` (MoveJ) | `controlLoopCallback`, `publishTrajectory` | `controlLoopCallback`, `syncCommandStateToFeedback` | 비보호 (단일 스레드) |
| `r_gripper_pose_`, `l_gripper_pose_` | `controlLoopCallback`, `publishGripperPose` | `controlLoopCallback` | 비보호 (단일 스레드) |
| `joint_state_received_`, `last_joint_state_time_` | `jointStateTimedOut`, `controlLoopCallback` | `jointStateCallback` | 비보호 (단일 스레드) |
| `control_enabled_`, `activate_pending_`, `reference_diverged_`, `slow_start_scale` (VR) | `controlLoopCallback` | `controlLoopCallback`, `reactivateCallback`, `referenceDivergenceCallback` | 비보호 (단일 스레드) |
| `right_pose_state`, `left_pose_state` (arm_retargeting.py) | `run_teleop`, `_retarget_bimanual_pose_states` | 6개 콜백 | 비보호 (단일 스레드) |
| `_filtered_right_wrist_target`, `_filtered_left_wrist_target` (arm_retargeting.py) | `_smooth_wrist_targets` | `_smooth_wrist_targets` | 비보호 (단일 스레드) |

### B-3. 실행 컨텍스트 표

| 이름 | 종류 | 우선순위·executor | 생성 위치 |
|------|------|----------------|-----------|
| 전 노드 control loop | timer (`create_wall_timer(time_step_)`) | 기본 SingleThreadedExecutor | 각 노드 ctor (예: cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:208) |
| 전 노드 subscriber 콜백 | subscription callback | 기본 SingleThreadedExecutor + MutuallyExclusiveCallbackGroup | 각 노드 ctor `create_subscription` 호출 |
| Python `rclpy.spin(...)` | event loop (단일 스레드) | 기본 | `main` (arm_retargeting.py:765, teleop_retargeting.py:219) |

→ 모든 콜백/타이머가 순차 직렬 실행. **race condition 발생 불가** (단일 스레드 가정). 단 100Hz timer 와 다른 콜백이 같은 스레드를 공유하므로 무거운 콜백이 타이머 지연 유발 가능 (timing 이슈 → 평가에서 다룸).

---

## 평가

severity 분포: Critical 0 / High 2 / Medium 11 / Low 6 / Info 3
Verdict: REQUEST CHANGES

> 본 SOP 룰에 따라 작성자(본 세션)는 APPROVE 불가. 별도 lane (code-reviewer/verifier 에이전트) 에서만 APPROVE 가능.
> **Reviewer lane 결과 (2026-05-23 KST)**: `oh-my-claudecode:code-reviewer` 독립 lane Verdict = **COMMENT** (SOP 준수·critical 신규 없음, REQUEST CHANGES 유지). 아래 Medium·Low·Info 일부 항목(joint_index_map stale, DexPilot NaN guard, setQPsize assert, computePose 재생성)은 reviewer lane 이 추가 식별한 보강.

### High

**함수 #4·#67 — [논리][성능] `qdot` 무필터 사용 → QP 비용 함수에 미분 노이즈 직결 (High)**
재현: cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:463 에 `// ToDo: Add low pass filter` 주석. `extractJointStates` 가 `msg->velocity[idx]` 를 그대로 `qdot_[i]` 에 대입. 동일 패턴이 모든 컨트롤러 노드(`ai_worker_movel/movej`, `omx/omy_movel/movej`, `vr_controller`)에 존재. `qdot_` 은 `KinematicsSolver.updateState(q, qdot)` → `pinocchio::computeJointJacobians` 거쳐 QP 비용 항(`‖J·q̇ − ẋ_des‖²`)의 J 계산에 사용. 외부 컨트롤러가 차분으로 추정한 노이즈가 그대로 100Hz 제어 명령에 반영.
권고: 1차 IIR (지수이동평균) 추가. `qdot_filt = α·qdot_meas + (1-α)·qdot_prev` 형태. `α` 는 control_frequency·로봇 모달에 따라 0.1~0.5. core 의 `KinematicsSolver` 가 아니라 노드 측에서 적용해야 QP 변경 없이 흡수 가능.

**함수 #34·#43·#54·#63 (모든 QP 컨트롤러) — [논리][성능] 가속도(q̈) 한계 미강제 — 동작 전환 시 토크 점프 (High)**
재현: `QPBase` 의 결정변수 `x = [q̇; slacks]`. `setBoundConstraint` 는 `q̇_lb ≤ q̇ ≤ q̇_ub` (URDF 한계) 만 강제. 사이클 간 `|q̇_t − q̇_{t-1}|` 제한 없음. 예: `vr_controller_node.cpp` 의 `slow_start_scale` 이 0→1 램프되는 동안에도 명령은 QP 가 풀어준 즉시 `q̇` 의 스칼라 배. 즉 `slack_penalty` 활성/비활성 전환, `reference_diverged_` flag 토글, `activate_pending_` 종료(3초 후) 시점에 `q̇` 가 큰 폭으로 점프하면 다운스트림 보간기/관절제어기가 토크 점프로 흡수해야 함.
권고: QP 부등식 한 줄 추가 — `−a_max·dt ≤ q̇ − q̇_{t-1} ≤ a_max·dt` (`a_max` 는 파라미터화). `QPBase.setIneqConstraint` 에 nbc+2·dof 행 추가, 슬랙 변수 동반(soft constraint) 권장.

### Medium

**함수 #143·#161·#177·#193·#209·#225 (모든 controlLoopCallback) — [QoS][exec] 100Hz 센서 토픽에 default(RELIABLE) QoS — drop 시 전 노드 차단 (Medium)**
재현: `/joint_states` (sensor_msgs/JointState, 100Hz 추정) 를 모든 컨트롤러가 default depth=10 + RELIABLE 으로 구독. 손실 시 RELIABLE 은 재전송 시도하여 큐 백업 → DDS 레벨 jitter. 센서 스트림은 통상 `rclcpp::SensorDataQoS()` (BEST_EFFORT + KEEP_LAST + depth=5) 권장. 동일 토픽을 6+ 노드가 구독 → 한 노드의 큐 백업이 전체에 영향 가능.
권고: `joint_state_sub_` 의 QoS 를 `rclcpp::SensorDataQoS()` 로 변경. 외부 발행자가 SensorDataQoS 로 publish 하면 호환성 향상.

**함수 #18·#118·#135·#161·#177·#193·#209·#225 — [exec] 단일 스레드 executor 에서 100Hz 타이머와 콜백 공유 — timing jitter (Medium)**
재현: 모든 노드 grep `MultiThreadedExecutor|StaticSingleThreadedExecutor|CallbackGroup` 0건. 기본 SingleThreadedExecutor + 기본 callback group → control loop timer(100Hz, `time_step=0.01`) 와 모든 sub 콜백(joint_states 100Hz + traj 명령 + reactivate 등)이 직렬화. joint_state 콜백이 무거우면(특히 VR 의 9 sub) 다음 timer firing 까지 latency 누적.
권고: `vr_controller_node`, `ai_worker_movel/j_controller_node` 만이라도 `MultiThreadedExecutor` 로 분리 + `MutuallyExclusiveCallbackGroup` 두 개(control loop, sensor callbacks)로 명시. control loop 우선순위 보장.

**함수 #123 (VRController 전체) — [SOLID][품질] 한 클래스가 9개 sub + 7개 pub + 39 param + 5-상태 머신 + QP 호출 — 922 라인 god-class (Medium)**
재현: cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp 총 922 라인. 한 클래스가 (a) 시작 점검(`startup_ref_*_threshold`) (b) 3초 `activate_pending` (c) 8초 `slow_start_scale` 램프 (d) `/reference_diverged` 처리 (e) QP 호출 (f) 그리퍼 pose 발행 (g) lift 발행 (h) reactivate gate 모두 담당. 단일 책임 원칙 위반.
권고: `VRSafetyGate` (startup/activate/diverged) 와 `VRController` (QP+publish) 두 클래스로 분리. composition 으로 결합. core/optimization 의 `QPBase` 처럼 분리 가능.

**함수 #176·#192·#208·#224·#240 (OMX/OMY 4개 노드) — [SOLID][품질] OMX·OMY 4개 노드가 사실상 동일 — URDF 만 다른 클론 (Medium)**
재현: `omx_movej_controller_node.cpp` (421 라인) ↔ `omy_movej_controller_node.cpp` (421 라인) ↔ `omx_movel_controller_node.cpp` (438 라인) ↔ `omy_movel_controller_node.cpp` (438 라인). diff 한 결과 거의 동일 (선언 클래스명·기본 토픽명만 차이). 코어 컨트롤러는 동일(`OpenManipulatorMoveJController`, `OpenManipulatorMoveLController`).
권고: `cyclo_motion_controller_ros::SingleArmMoveJControllerNode<RobotTraits>` 템플릿 또는 base class + per-robot config 로 통합. 새 단팔 로봇 추가 시 cpp 파일 0개 추가.

**함수 #58·#162·#178·#194·#210·#226 (모든 syncCommandStateToFeedback) — [논리] 양팔 동시 리셋으로 미세 자세 점프 가능 (Medium)**
재현: cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:482 `q_desired_ = q_;` — 한 팔만 신규 MoveL 명령이 와도 `syncCommandStateToFeedback` 호출 경로에서는 양팔 전체 `q_desired_` 가 측정값으로 리셋. 양손 협조 작업 중 한 팔 명령만 갱신해도 안 갱신된 팔이 측정값으로 0.01s 단위 점프.
권고: `syncArmStateToFeedback(arm_joint_names)` (이미 `:486` 에 분리되어 존재) 만 호출하도록 통합. 양팔 동시 리셋 경로는 startup 1회로 제한.

**함수 #161·#177·#143 (controlLoopCallback) — [성능][품질] OSQP 솔버 실패 시 1 사이클 skip 뿐 — 누적 실패 카운터 없음 (Medium)**
재현: cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movel_controller_node.cpp:448-454 `if (!qp_controller_->getOptJointVel(...)) { ... return; }`. `~/controller_error` publish 후 그냥 return. 연속 N 회 실패 시 어떻게 할지 정의 없음. 100Hz × 1초 = 100회 연속 실패해도 `/leader/.../joint_trajectory` publish 만 멈추고 다운스트림은 마지막 값으로 자유운동 가능.
권고: 연속 실패 카운터 추가. K (예: 10) 회 초과 시 `/reference_diverged` publish + (선택) holding mode 진입(자기 q_desired_ 유지).

**함수 #161·#177 — [논리] 단일 포인트 trajectory 사용 — multi-point 무시 (Medium)**
재현: cyclo_motion_controller_ros/src/nodes/ai_worker/ai_worker_movej_controller_node.cpp:231 `msg.points.front()` 만 사용. multi-point JointTrajectory 보간 책임은 다운스트림으로 위임. 사용자가 `points: [{...}, {...}, {...}]` 보내도 첫 점만 추적.
권고: README 에 "single-point only" 명시하거나, multi-point cubic 보간을 노드 측에서 구현.

**함수 #243-249 (전 노드) — [테스트] C++ ROS 노드에 단위/통합 테스트 0개 (Medium)**
재현: `cyclo_motion_controller_ros/` 패키지 전체에 `test/` 디렉토리 없음. CMakeLists.txt 에 `ament_add_gtest` 사용 없음. Python 패키지(`ros_py`)에는 ament_copyright/flake8/pep257 만 존재 (기능 테스트 0). 회귀 위험: QP 비용 함수 가중치 변경, CBF α 변경, 안전 게이트 임계값 변경 시 시뮬레이션 없이 회귀 감지 불가.
권고: 최소한 (a) `KinematicsSolver` FK/Jacobian 단위 테스트 (`cyclo_motion_controller_core/test/`) (b) `QPBase` 솔버 컨버전스 테스트 (c) `reference_checker` 점프 감지 임계값 테스트 추가.

**함수 #13 — [품질][runtime] 충돌 모델 신뢰성에 대한 노란색 경고만 — 자동 검증 없음 (Medium)**
재현: cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp 생성자에서 "Collision model for the robot may not be perfect!" 경고 출력. URDF collision mesh 품질과 SRDF 큐레이션에 전적으로 의존하지만, 빌드/런타임에 자동 검증 없음. 잘못된 collision pair 가 활성화되면 CBF 가 잘못된 거리를 강제 → 로봇 정지 또는 부정확한 회피.
권고: 부트 시 `getCollisionPairCount()` 와 SRDF disable list 길이를 로그에 명시. 임의 q 에서 `getCollisionPairDistances` 호출하여 음수 거리(침투) pair 가 있으면 FATAL.

**함수 #20·#44·#82·#118·#131 (모든 jointStateCallback) — [논리] `joint_index_map_` 첫 메시지로만 캐시 — publisher 재시작 시 stale (Medium · reviewer lane 추가)**
재현: cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:381-385 `if (joint_index_map_.empty())` 분기로 첫 `JointState` 메시지에서만 `name → index` 매핑 구축. `/joint_states` publisher 가 도중에 재시작되거나 메시지의 joint name 순서가 바뀌면 `q_`, `qdot_` 에 잘못된 인덱스가 들어가 QP 가 엉뚱한 관절을 제어. 동일 패턴이 모든 컨트롤러 노드(ai_worker_movel/movej, vr, omx/omy_*) 에 존재.
권고: 매 콜백마다 `msg->name` 시퀀스 hash(예: std::hash 누적)를 비교해 변경 감지 시 인덱스 맵 재구축. 또는 매번 재구축(O(n) 비용은 100Hz에서 무시 가능).

**함수 #109 (DexPilotOptimizer.retarget) — [논리][품질] `RuntimeError` 만 catch — NaN/Inf 결과가 ROS 토픽으로 그대로 전파 (Medium · reviewer lane 추가)**
재현: cyclo_motion_controller_core/src/retargeting/optimizer.py:255 부근 `except RuntimeError: ... return last_qpos`. NLopt 가 다른 예외(`ValueError`, `nlopt.RoundoffLimited` 등) 또는 수렴 실패로 NaN/Inf 를 반환할 수 있으나 `np.isfinite` 검사 없이 그대로 `publish_trajectory_*` → 다운스트림으로 전달. arm_retargeting.py 도 동일하게 NaN 가드 없음.
권고: `if not np.all(np.isfinite(result))` 분기에서 `last_qpos` 또는 publish skip. 외부 catch 범위를 `Exception` 으로 확장 + 로깅.

### Low

**함수 #123 — [param] VR slow_start 하드코딩 (Low)**
재현: cyclo_motion_controller_ros/src/nodes/ai_worker/vr_controller_node.cpp:664-666 — 3초 `activate_pending`, 8초 `slow_start_scale` 램프가 매직넘버. 파라미터로 노출 안 됨.
권고: `activation_delay_sec`, `slow_start_duration_sec` 파라미터 추가.

**함수 #1·#13 — [runtime] SRDF 부재 시 모든 충돌쌍 활성화 — 솔버 부담 폭증 (Low)**
재현: cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:55-60 `if (has_srdf_path)` 분기에서 SRDF 없으면 `addAllCollisionPairs()` 만 호출되고 `removeCollisionPairs` 미실행. 충돌쌍 수가 O(n²) 로 폭증, 100Hz 솔버 마진 감소.
권고: SRDF 부재 시 OFF 가 아니라 명시적 ERROR + node abort. SRDF 큐레이션을 필수로 강제.

**함수 #137 — [param] interactive marker 색상이 launch arg 가 아닌 노드 인자 (Low)**
재현: cyclo_motion_controller_ros/launch/ai_worker_controller.launch.py 가 `interactive_marker_node` 에 `color_*` 를 직접 passthrough. RViz 색상 변경 시 cpp 재빌드 없이 가능하나, 다중 인스턴스 호출 시 가독성 저하.
권고: SRGBA color 단일 파라미터 또는 launch arg 명명 컨벤션 강화.

**함수 #1 — [품질] osqp_eigen_vendor 가 third_party 디렉토리에 소스 카피 — 업스트림 패치 추적 불가 (Low)**
재현: osqp_eigen_vendor/third_party/osqp-eigen/ 가 vcs path-vendor. 업스트림 osqp-eigen 의 보안/버그 패치를 자동 추적하지 않음.
권고: `vcs_type: git`, `vcs_url: github.com/robotology/osqp-eigen`, `vcs_version: <tag>` 으로 전환.

**함수 #137 — [품질] `/r_goal_move`, `/l_goal_move` 가 절대 토픽 — 양팔 충돌 가능성 (Low)**
재현: interactive marker 가 절대 토픽 `/r_goal_move`, `/l_goal_move` 로 publish. 다른 시스템(예: 자율 작업 노드)이 같은 토픽을 publish 하면 충돌.
권고: 노드 namespace 활용 또는 `~/r_goal_move` 상대 토픽.

**함수 #25 (QPBase.setQPsize) — [품질] `assert` 가 NDEBUG 빌드(Release)에서 무의미 (Low · reviewer lane 추가)**
재현: cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp:61 부근 `assert(...)` 사용. README 의 빌드 명령은 `-DCMAKE_BUILD_TYPE=Release` 권장이므로 NDEBUG 가 정의되어 assert 모두 컴파일 아웃. QP size 미스매치(예: nx ≠ P 차원)가 런타임에 무음으로 통과 후 OSQP solve 단계에서 의미 불명 에러로 폭발.
권고: `assert` → `throw std::invalid_argument(...)` 또는 `cyclo_motion_controller::common::Check::throwIf(...)` 명시적 예외.

### Info

**함수 #1 — [품질] README 가 `ROBOTIS-GIT/cyclo_control` 외부 저장소 가정 — 본 워크스페이스에 직접 빌드 불가 (Info)**
재현: README.md 의 install 절차가 `git clone https://github.com/ROBOTIS-GIT/cyclo_control.git` 으로 시작. 본 사본은 `/home/openarmx/TR-Works/kkw/China/cyclo_control` 에 있음. workspace 의 `src` 심볼릭 링크 또는 colcon overlay 필요.
권고: README 에 "내부 워크스페이스 통합 시" 절차 한 단락 추가.

**함수 #295 — [Info] Python 패키지의 자동 형식 검사만 존재 — 기능 테스트 0 (Info)**
재현: cyclo_motion_controller_ros_py/test/ 에 test_copyright.py, test_flake8.py, test_pep257.py 만 존재. arm_retargeting, teleop_retargeting 의 알고리즘 테스트 없음.
권고: 위 "Medium [테스트]" 권고와 통합.

**함수 #5·#6 (KinematicsSolver.computePose / computeJacobian) — [품질] 매 호출 `pinocchio::Data` 임시 생성 — 핫패스 외 사용임을 문서화 필요 (Info · reviewer lane 추가)**
재현: cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp:169, 188 — `computePose`/`computeJacobian` (q 인자 받음) 가 매 호출 새 `pinocchio::Data` 객체 생성. 핫패스(100Hz controlLoopCallback) 는 `updateState` + `getPose`/`getJacobian` (캐시 조회) 를 써야 하나, API 이름이 비슷해 오용 위험. 사용자가 `computePose` 를 핫패스에서 호출하면 매 사이클 `Data` 할당으로 성능 급락.
권고: 두 함수에 `// NOTE: allocates pinocchio::Data — for hot path use updateState() + getPose()/getJacobian()` 주석 추가. 또는 함수명을 `computePoseStateless` 로 변경.

---

## 부록 — 핵심 알고리즘 / 수식

### 작업공간 PD with feedforward (MoveL)
```
e_p = p_goal − p_cur
R_err = R_goal · R_curᵀ  →  AxisAngle(θ, k̂)  →  e_o = θ·k̂
v_des = ff_linear  + Kp_pos · e_p
ω_des = ff_angular + Kp_ori · e_o
```

### QP (VR 양팔 + 양 팔꿈치)
```
min  Σ_{i∈{r_grip, l_grip, r_elb, l_elb}}  ½‖Jᵢ·q̇ − ẋᵢ_des‖²_Wᵢ
     + ½‖q̇‖²_{W_d}
     + ρ·(1ᵀs_qmin + 1ᵀs_qmax + 1ᵀs_sel_col)

 s.t.  q̇_lb ≤ q̇ ≤ q̇_ub
       I·q̇ + I·s_qmin ≥ −α·(q − q_min)
       −I·q̇ + I·s_qmax ≥ −α·(q_max − q)
       ∇dᵢᵀ·q̇ + sᵢ ≥ −α·(dᵢ − d_safe)         (충돌쌍 i)
       s_* ≥ 0
```

### 거리 그래디언트 (해석적)
```
d = (p_B − p_A) · n̂,    n̂ = unit(p_B − p_A)
rA = p_A − oMi[jointA].translation
rB = p_B − oMi[jointB].translation
JA = J_lin[A] − skew(rA)·J_ang[A]
JB = J_lin[B] − skew(rB)·J_ang[B]
∂d/∂q = n̂ᵀ·(JB − JA)
```

---
