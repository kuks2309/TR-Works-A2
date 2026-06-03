# 05. 파라미터 레퍼런스 (config)

[← 문서 목차](README.md) · [← 04. ROS 인터페이스](04_ros_interface.md)

config yaml 파일과 코드 기본값을 정리합니다. 출처:
[`config/omy_config.yaml`](../cyclo_motion_controller_ros/config/omy_config.yaml),
[`config/omx_config.yaml`](../cyclo_motion_controller_ros/config/omx_config.yaml),
[`config/ai_worker_config.yaml`](../cyclo_motion_controller_ros/config/ai_worker_config.yaml),
[`utils/controller_params.hpp`](../cyclo_motion_controller_ros/include/cyclo_motion_controller_ros/utils/controller_params.hpp).

---

## 1. 공통 파라미터 (`declareCommonControllerParams`)

6개 컨트롤러 노드가 공유. **코드 기본값**(config 미지정 시) 과 의미:

| 파라미터 | 코드 기본값 | 의미 |
| --- | --- | --- |
| `control_frequency` | `100.0` | 제어 루프 주기 [Hz] → 타이머 `round(1000/f)` ms |
| `time_step` | `0.01` | 적분 dt [s] (`q_cmd += qdot·dt`) |
| `trajectory_time` | `0.05` | publish 하는 JointTrajectory 점의 `time_from_start` [s] |
| `weight_damping` | `0.05` | 감쇠 가중치 D |
| `slack_penalty` | `1000.0` | 슬랙(제약 위반) 선형 페널티 |
| `cbf_alpha` | `5.0` | CBF 클래스-K 계수 α |
| `collision_buffer` | `0.05` | 충돌 CBF 활성 거리 [m] |
| `collision_safe_distance` | `0.02` | 유지 최소 안전거리 d_safe [m] |
| `joint_state_timeout` | `0.5` | joint_states 미수신 허용시간 [s] |
| `urdf_path` / `srdf_path` | `""` | 모델 경로(launch 주입). URDF 없으면 노드 종료 |
| `base_frame` | `link0` | 기준 프레임 |
| `controlled_link` | (노드별) | 태스크 링크 |
| `joint_states_topic` | `/joint_states` | 관절 상태 Sub |
| `joint_command_topic` | (노드별) | 관절 명령 Pub |
| `ee_pose_topic` | `~/current_pose` | 현재 EE pose Pub |
| `controller_error_topic` | `~/controller_error` | 에러 Pub |

> `time_step` 과 `control_frequency` 는 별도 파라미터입니다. 기본값(0.01s, 100Hz)은 정합하지만
> 둘을 따로 바꾸면 적분 dt 와 실제 루프 주기가 어긋날 수 있으니 함께 맞추세요.

### MoveL/MoveJ 전용 (각 노드에서 별도 declare)

| 파라미터 | 코드 기본값(OMY 노드) | 적용 |
| --- | --- | --- |
| `kp_position` | `4.0` | MoveL 위치 오차 게인 |
| `kp_orientation` | `2.5` | MoveL 자세 오차 게인 |
| `weight_task_position` | `10.0` | MoveL 위치 태스크 가중치 |
| `weight_task_orientation` | `1.0` | MoveL 자세 태스크 가중치 |
| `movel_topic` | `~/movel` | MoveL 입력 |
| `kp_joint` | (movej 노드) | MoveJ 관절 게인 |
| `weight_joint_tracking` | (movej 노드) | MoveJ 관절 추종 가중치 |
| `movej_topic` | `~/movej` | MoveJ 입력 |

> ⚠️ **config 가 코드 기본값을 덮어씁니다.** 예: OMY config 는 `kp_position/kp_orientation` 을
> `50.0/50.0`, `joint_command_topic` 을 `/leader/joint_trajectory`, `controlled_link` 를
> `end_effector_link` 로 지정. 실제 운용값은 **config 우선**.

---

## 2. OMY vs OMX 비교 (단일팔)

대부분 동일하고 **충돌 안전 마진과 감쇠/게인**만 다릅니다.

| 파라미터 | OMY (`omy_config`) | OMX (`omx_config`) |
| --- | --- | --- |
| `control_frequency` | 100.0 | 100.0 |
| `time_step` | 0.01 | 0.01 |
| `kp_position` / `kp_orientation` | 50.0 / 50.0 | 50.0 / 50.0 |
| `weight_task_position` / `_orientation` | 10.0 / 1.0 | 10.0 / 1.0 |
| `weight_damping` | 0.001 | 0.001 |
| `slack_penalty` | 1000.0 | 1000.0 |
| `cbf_alpha` | 5.0 | 5.0 |
| **`collision_buffer`** | **0.03** | **0.01** |
| **`collision_safe_distance`** | **0.01** | **0.005** |
| `base_frame` | `link0` | `link0` |
| `controlled_link` | `end_effector_link` | `end_effector_link` |
| `joint_command_topic` | `/leader/joint_trajectory` | `/leader/joint_trajectory` |
| `movel_topic` / `movej_topic` | `~/movel` / `~/movej` | `~/movel` / `~/movej` |

MoveJ 블록(`*_movej_controller`)은 `kp_joint=50.0`, `weight_joint_tracking=10.0`,
`weight_damping=0.001` 로 동일 구조.

---

## 3. AI Worker (`ai_worker_config.yaml`)

5개 컨트롤러 블록을 한 파일에 담습니다.

### 3.1 `ai_worker_movel_controller`

| 파라미터 | 값 | 비고 |
| --- | --- | --- |
| `control_frequency` / `time_step` | 100.0 / 0.01 | |
| `kp_position` / `kp_orientation` | 50.0 / 50.0 | |
| `weight_task_position` / `_orientation` | 10.0 / 1.0 | |
| `weight_damping` | **0.1** | 단일팔(0.001)보다 큼 |
| `collision_buffer` / `collision_safe_distance` | 0.05 / 0.02 | |
| `slack_penalty` / `cbf_alpha` | 1000.0 / **50.0** | α 가 단일팔의 10배(공격적 충돌제약) |
| `right_movel_topic` / `left_movel_topic` | `/r_goal_move` / `/l_goal_move` | |
| `right_traj_topic` / `left_traj_topic` | `…broadcaster_{right,left}/joint_trajectory` | 출력 |
| `lift_topic` | `…joystick_controller_right/joint_trajectory` | |
| `lift_vel_bound` | 0.0 | |
| `r/l_gripper_pose_topic` | `/r_gripper_pose` / `/l_gripper_pose` | |
| `r/l_gripper_name` | `arm_r_link7` / `arm_l_link7` | |
| `right/left_gripper_joint` | `gripper_r_joint1` / `gripper_l_joint1` | |

### 3.2 `ai_worker_movej_controller`

| 파라미터 | 값 |
| --- | --- |
| `kp_joint` / `weight_tracking` / `weight_damping` | 50.0 / 10.0 / 0.1 |
| `collision_buffer` / `safe` / `slack_penalty` / `cbf_alpha` | 0.05 / 0.02 / 1000.0 / 50.0 |
| `command_timeout` | 0.1 |
| `right/left_traj_topic` | `…broadcaster_{right,left}/raw_joint_trajectory` (입력) |
| `right/left_traj_filtered_topic` | `…broadcaster_{right,left}/joint_trajectory` (출력) |

### 3.3 `vr_controller`

| 파라미터 | 값 | 비고 |
| --- | --- | --- |
| `kp_position` / `kp_orientation` | 50.0 / 50.0 | |
| `weight_position` / `weight_orientation` | 10.0 / 1.0 | 그리퍼 태스크 |
| `weight_elbow_position` | 8.0 | 팔꿈치 위치 태스크 |
| `weight_damping` | 0.1 | |
| `collision_buffer` / `safe` / `slack_penalty` / `cbf_alpha` | 0.05 / 0.02 / 1000.0 / 50.0 | |
| `r/l_goal_pose_topic` | `/r_goal_pose` / `/l_goal_pose` | 그리퍼 목표 |
| `r/l_elbow_pose_topic` | `/r_subgoal_pose` / `/l_subgoal_pose` | 팔꿈치 목표 |
| `r/l_gripper_name` | `arm_{r,l}_link7` | |
| `r/l_elbow_name` | `arm_{r,l}_link4` | |
| `startup_ref_pos_threshold` | 0.3 | 시작 안전(위치) [m] |
| `startup_ref_ori_threshold_deg` | 120.0 | 시작 안전(자세) [deg] |
| `reactivate_topic` | `/reactivate` | |

### 3.4 `leader_controller`

| 파라미터 | 값 |
| --- | --- |
| `control_frequency` | 100.0 |
| `right/left_traj_topic` | `…broadcaster_{right,left}/raw_joint_trajectory` (입력) |
| `command_timeout` | 0.1 |
| `r/l_goal_pose_topic` | `/r_goal_pose` / `/l_goal_pose` (출력) |
| `r/l_elbow_pose_topic` | `/r_elbow_pose` / `/l_elbow_pose` (출력) |
| `r/l_gripper_name` / `r/l_elbow_name` | `arm_{r,l}_link7` / `arm_{r,l}_link4` |
| `lift_joint_name` / `model_lift_joint_name` | `lift_joint` / `joint` |

### 3.5 `reference_checker`

| 파라미터 | 값 |
| --- | --- |
| `ref_pos_jump_threshold` | 0.1 [m] |
| `ref_ori_jump_threshold_deg` | 30.0 [deg] |
| `r/l_goal_pose_topic` | `/r_goal_pose` / `/l_goal_pose` |

---

## 4. 튜닝 가이드 (요약)

| 증상 | 우선 조정 |
| --- | --- |
| 충돌 직전 너무 늦게 멈춤 | `cbf_alpha` ↓ 또는 `collision_buffer`/`safe_distance` ↑ |
| 충돌 제약이 과도해 못 움직임 | `cbf_alpha` ↑ 또는 SRDF 로 불필요한 쌍 비활성, `disable_gripper_collisions:=true`(AI Worker) |
| 추종이 출렁임/거침 | `weight_damping` ↑, `kp_*` ↓ |
| 추종이 느림/부정확 | `weight_task_*`/`weight_joint_tracking` ↑, `kp_*` ↑ |
| 한계 근처에서 떨림 | `cbf_alpha` ↓ |
| 제약을 거의 경성으로 | `slack_penalty` ↑ (단, QP 불능 위험 ↑) |

> CBF/슬랙/가중치의 수식적 의미는 [02_qp_cbf_formulation.md §7](02_qp_cbf_formulation.md) 참고.

---

## 5. 자기 충돌 쌍과 SRDF

충돌 CBF 제약 개수 `m` = `KinematicsSolver::getCollisionPairCount()` 이며, 이는
**URDF 의 모든 링크쌍 − SRDF 로 비활성화한 쌍** 입니다.

- SRDF 가 없으면 모든 쌍이 활성 → 제약/슬랙 수가 폭증하고 정상 자세에서도 충돌제약이 걸릴 수 있음.
- AI Worker 는 기본 SRDF(`*_default.srdf`)와 핸드 충돌을 끈 변형(`*_modified.srdf`)을 제공하며,
  `disable_gripper_collisions:=true` 로 후자를 선택(좌/우 `link7` 충돌 무시).
