# 04. ROS 인터페이스 (노드 · 토픽 · launch)

[← 문서 목차](README.md) · [← 03. 컨트롤러](03_controllers.md)

`cyclo_motion_controller_ros` 가 제공하는 실행파일·토픽·launch 를 정리합니다.
토픽 이름은 대부분 파라미터로 노출되며, 아래 값은 **config yaml 기본값** 기준입니다
(전체 파라미터는 [05_parameters.md](05_parameters.md)).

---

## 1. 실행파일 목록 (CMakeLists)

| 실행파일 | 소스 | 용도 |
| --- | --- | --- |
| `omy_movel_controller_node` | nodes/omy | OMY 단일팔 데카르트 |
| `omy_movej_controller_node` | nodes/omy | OMY 단일팔 관절 |
| `omx_movel_controller_node` | nodes/omx | OMX 단일팔 데카르트 |
| `omx_movej_controller_node` | nodes/omx | OMX 단일팔 관절 |
| `ai_worker_movel_controller_node` | nodes/ai_worker | AI Worker 양팔 데카르트(+리프트/그리퍼) |
| `ai_worker_movej_controller_node` | nodes/ai_worker | AI Worker 양팔 관절 |
| `vr_controller_node` | nodes/ai_worker | 양팔 VR 텔레오퍼레이션 IK |
| `leader_controller_node` | nodes/ai_worker | 리더 관절 → goal/elbow pose (FK) |
| `interactive_marker_node` | utils/eef_interactive_marker_node | RViz 6-DOF 마커 → MoveL goal |
| `reference_checker_node` | utils/reference_checker_node | reference pose 급변 감지 |

모두 `Eigen3 / pinocchio / OsqpEigen / cyclo_motion_controller_core` 에 링크됩니다.

---

## 2. OMY / OMX 단일팔 노드

OMX 와 OMY 는 **동일한 구조**이며 기본 모델/네임스페이스만 다릅니다.

### 2.1 `omy_movel_controller_node` (검증 기준 노드)

| 구분 | 토픽/타입 | 기본값 |
| --- | --- | --- |
| **Sub** | `joint_states_topic` · `sensor_msgs/JointState` | `/joint_states` |
| **Sub** | `movel_topic` · `openarmx_scenario_player_msgs/MoveL` | `~/movel` |
| **Pub** | `joint_command_topic` · `trajectory_msgs/JointTrajectory` | `/leader/joint_trajectory` |
| **Pub** | `ee_pose_topic` · `geometry_msgs/PoseStamped` | `~/current_pose` |
| **Pub** | `controller_error_topic` · `std_msgs/String` | `~/controller_error` |

동작:

- 제어 주기 = `round(1000/control_frequency)` ms (기본 100 Hz → 10 ms).
- `MoveL` 수신 시: 현재 명령자세를 시작점으로 잡고 `pose`/`time_from_start` 로 cubic 궤적 설정.
  `time_from_start > -1.0s` 일 때만 궤적 활성(`movel_trajectory_active_`).
- 매 주기: cubic 보간 자세 + 피드포워드 속도 → `computeDesiredVelocity`(ff + Kp·error)
  → QP 풀이 → `q_cmd += qdot·dt` → `JointTrajectory`(위치 1점) publish.
- **joint-state 타임아웃**(`joint_state_timeout`, 기본 0.5s) 동안 새 피드백이 없으면
  궤적 정지 + 명령 보류, 복구 시 명령상태를 실측으로 재동기화.
- 모션 종료(`elapsed ≥ duration`) 후에는 publish 중단(잔류 진동 방지).

> ⚠️ **노드 코드 기본값 vs config 값 불일치 주의**
> `omy_movel_controller_node` 의 `declare_parameter` 기본값은
> `kp_position=4.0`, `kp_orientation=2.5`, `joint_command_topic=/omy/joint_trajectory`,
> `controlled_link=link7` 이지만, `omy_config.yaml` 이 이를
> `50.0 / 50.0 / /leader/joint_trajectory / end_effector_link` 로 **덮어씁니다**.
> launch 로 config 를 항상 로드하므로 실제 운용값은 config 기준입니다.

### 2.2 `omy_movej_controller_node`

- **Sub**: `movej_topic`(기본 `~/movej`) — 관절 목표.
- 파라미터: `kp_joint`, `weight_joint_tracking`(MoveL 의 task 가중치 대체).
- 나머지 토픽/동작은 MoveL 노드와 동일, 보간만 관절 공간.

### 2.3 OMX 차이

- 기본 모델 `omx_f.urdf/srdf`, config `omx_config.yaml`.
- **충돌 파라미터가 더 타이트**: `collision_buffer=0.01`, `collision_safe_distance=0.005`
  (OMY 는 0.03 / 0.01). [05_parameters.md](05_parameters.md) 비교표 참고.

---

## 3. AI Worker 양팔 노드

`ai_worker_config.yaml` 의 파라미터로 좌/우 팔·리프트·그리퍼를 함께 다룹니다.
(아래 토픽은 config 기본값)

### 3.1 `ai_worker_movel_controller_node`

- 코어: `AIWorkerMoveLController`(= `VRController`) — 좌/우 그리퍼 데카르트 추종.
- **Sub**: `/joint_states`, `right_movel_topic`=`/r_goal_move`, `left_movel_topic`=`/l_goal_move`
  (`openarmx_scenario_player_msgs/MoveL`).
- **Pub**: 좌/우 trajectory
  (`/leader/joint_trajectory_command_broadcaster_{right,left}/joint_trajectory`),
  리프트(`/leader/joystick_controller_right/joint_trajectory`),
  `r_gripper_pose`/`l_gripper_pose`, `~/controller_error`.
- 충돌 파라미터: `cbf_alpha=50`(단일팔의 10배), `collision_buffer=0.05`, `safe=0.02`.

### 3.2 `ai_worker_movej_controller_node`

- 리더 raw 관절 trajectory 를 받아 필터링/명령으로 중계.
- **Sub**: `right/left_traj_topic`(`.../raw_joint_trajectory`),
  **Pub**: `right/left_traj_filtered_topic`(`.../joint_trajectory`).
- `command_timeout=0.1s`.

### 3.3 `vr_controller_node`

- 코어: `VRController` — **좌/우 그리퍼 + 좌/우 팔꿈치(elbow)** 를 한 QP 로 동시 IK.
- **Sub**: `r/l_goal_pose_topic`, `r/l_elbow_pose_topic`(=`/r,l_subgoal_pose`), `/joint_states`,
  리더 raw trajectory, `reactivate_topic`(`std_msgs/Bool`).
- **Pub**: 좌/우 arm trajectory, 리프트, `r/l_gripper_pose`, divergence/error.
- 가중치: `weight_position=10`, `weight_orientation=1`, `weight_elbow_position=8`, `weight_damping=0.1`.
- **시작 안전조건**: 현재 자세와 reference 자세 차이가 임계
  (`startup_ref_pos_threshold=0.3m`, `startup_ref_ori_threshold_deg=120°`) 이내일 때만 활성화.

### 3.4 `leader_controller_node`

- 리더 장치 관절 trajectory → **FK** 로 그리퍼/팔꿈치 goal pose 생성(QP 없음).
- **Sub**: 리더 raw trajectory, `/joint_states`(리프트), `reactivate_topic`.
- **Pub**: `r/l_goal_pose`, `r/l_elbow_pose`.
- 링크 매핑: 그리퍼 `arm_{r,l}_link7`, 팔꿈치 `arm_{r,l}_link4`,
  리프트 `lift_joint`↔모델 `joint`.

> **Leader-Follower 파이프라인**: `leader_controller`(리더 관절→pose) →
> `vr_controller`(pose→팔로워 양팔 IK). `controller_type:=leader` 는 두 노드를 함께 띄웁니다.

---

## 4. 유틸리티 노드

### 4.1 `interactive_marker_node` (utils/eef_interactive_marker_node)

- RViz 에 6-DOF 인터랙티브 마커를 띄워, 드래그 위치를 `MoveL` goal 로 publish.
- 파라미터: `base_frame`, `controlled_link`, `goal_topic`, `server_name`, `marker_name`,
  `marker_description`, `marker_scale`, `marker_color_{r,g,b}`.
- launch 에서 좌(파랑)/우(빨강)/OMY(주황) 마커를 색상으로 구분.

### 4.2 `reference_checker_node` (utils/reference_checker_node)

- `r/l_goal_pose` 의 **연속 프레임 간 급변** 을 감지해 발산 신호 publish.
- 임계값: `ref_pos_jump_threshold=0.1m`, `ref_ori_jump_threshold_deg=30°`.
- VR 텔레오퍼레이션의 글리치(teleport) 보호용. `controller_type:=vr` 일 때만 launch.

---

## 5. Launch 파일

모든 launch 는 `controller_type` 인자로 어떤 노드를 띄울지 분기합니다.

### 5.1 `omy_controller.launch.py` / `omx_controller.launch.py`

| 인자 | 기본값(OMY) | 설명 |
| --- | --- | --- |
| `controller_type` | `movel` | `movel` \| `movej` |
| `start_interactive_marker` | `false` | RViz 마커 노드 동반 (movel 한정) |
| `base_frame` | `link0` | 컨트롤러/마커 기준 프레임 |
| `urdf_path` / `srdf_path` | `omy_f3m.urdf/srdf` | 모델 경로 |
| `config_file` | `omy_config.yaml` | 파라미터 |
| `controlled_link` | `end_effector_link` | 태스크 링크 |
| `marker_goal_topic` | `/omy_movel_controller/movel` | 마커 → MoveL |
| `marker_scale` | `0.2` | 마커 크기 |

> OMX 는 기본값이 `omx_f.urdf/srdf` + `omx_config.yaml` + `omx_*` 노드명으로 동일 구조.

### 5.2 `ai_worker_controller.launch.py`

| 인자 | 기본값 | 설명 |
| --- | --- | --- |
| `controller_type` | `movel` | `movel` \| `movej` \| `leader` \| `vr` |
| `base_frame` | `base_link` | 마커/MoveL 프레임 |
| `follower_urdf_path` | `ffw_sg2_follower.urdf` | 팔로워 모델 |
| `default_srdf_path` | `ffw_sg2_follower_default.srdf` | 기본 SRDF |
| `modified_srdf_path` | `ffw_sg2_follower_modified.srdf` | 핸드 충돌 비활성 SRDF |
| `disable_gripper_collisions` | `false` | true 면 modified SRDF 사용(좌우 link7 충돌 무시) |
| `leader_urdf_path` | `ffw_lg2_leader.urdf` | 리더 모델 |
| `config_file` | `ai_worker_config.yaml` | 파라미터 |
| `reactivate_topic` | `/reactivate` | VR 토글 |
| `arm` / `hand` | `true` / `false` | (vr 전용) 팔/손 리타게팅 노드 동반 |
| `right/left_controlled_link` | `arm_{r,l}_link7` | 마커 제어 링크 |
| `right/left_movel_topic` | `/{r,l}_goal_move` | 마커 → MoveL |
| `start_interactive_marker`, `marker_scale` | `false`, `0.2` | 마커 옵션 |

`controller_type` 별 기동 노드:

| type | 기동되는 노드 |
| --- | --- |
| `movel` | `ai_worker_movel_controller_node` (+마커: start_interactive_marker=true) |
| `movej` | `ai_worker_movej_controller_node` |
| `vr` | `vr_controller_node` + `reference_checker_node` (+arm/hand 리타게팅) |
| `leader` | `leader_controller_node` + `vr_controller_node` |

리타게팅 노드(`cyclo_motion_controller_ros_py`): `arm_retargeting_teleop`, `retargeting_teleop`(손).

---

## 6. `MoveL` 메시지

`openarmx_scenario_player_msgs/msg/MoveL` (이 패키지 외부 정의). MoveL 노드/마커가 사용:

- `pose` (`geometry_msgs/Pose`) — 목표 EE 자세
- `time_from_start` (`builtin_interfaces/Duration`) — 목표 도달 시간(궤적 길이).
  `> -1.0s` 이면 cubic 궤적 활성, 그 외에는 마지막 자세 유지.

> 정확한 필드는 `openarmx_scenario_player_msgs` 패키지 정의를 확인하세요.

다음: [05. 파라미터 레퍼런스 →](05_parameters.md)
