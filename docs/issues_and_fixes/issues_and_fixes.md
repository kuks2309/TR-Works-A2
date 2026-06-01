# 이슈 / 수정 기록 (China 워크스페이스)

China 모노레포 전체의 이슈·원인·수정 누적 기록. 최신 항목이 위 (prepend).
항목 형식은 [README.md](README.md) 참조. 작업 흐름은 `.claude/skills/issue-fix/SKILL.md` 참조.

<!-- 새 항목은 아래 구분선 바로 다음 줄에 추가 (최신 위). -->

---

## 2026-06-01 23:07 (KST) — yolov8 DetectBox per-goal prompts 라벨 오류 (stale class_names)

### 증상
on-demand `DetectBox` 액션 goal 에 `prompts`(예: 테이프류) 를 줘도 결과 `class_name` 이 엉뚱한 COCO 클래스(예: `motorcycle`)로 나옴. prompts 가 적용되지 않는 것처럼 보임. (롤 테이프를 화면에 두고 탐지 시 재현)

### 원인
prompts 는 실제로 적용되고 있었음 — box 프롬프트(conf 0.05)→0건, 서술형 "round object"(conf 0.01)→테이프 1건@0.15 로 **goal 마다 vocab 이 바뀌는 차등 동작** 확인(만약 항상 COCO 였다면 0.05 goal 에서도 0.15 테이프가 잡혔어야 함). 진짜 버그는 라벨 매핑:
- `3d_detect_ws/.../yolov8_node.py:109` 가 `self._class_names = self._yolo.names` 를 **init 1회만** 캐시 (이때 prompts 없음 → 기본 COCO 80 클래스).
- per-goal `set_classes(prompts)`(`:331`) 가 `self._yolo.names` 를 새 vocab 으로 갱신하지만 `self._class_names` 는 그대로.
- 라벨 조회(`:230`) 가 stale `self._class_names`(COCO) 사용 → class_id 3 = 새 vocab "round object" 인데 COCO id 3 "motorcycle" 로 출력.
- 부수: 동일 프롬프트라도 매 goal 마다 `set_classes` 재호출 → CLIP 재임베딩으로 매번 60-90s 소요.

### 수정
`3d_detect_ws/src/yolov8_detection/yolov8_detection/yolov8_node.py` `_execute` per-goal 분기 (~4줄): 프롬프트가 **바뀔 때만** `set_classes` 호출하도록 `prompts != self._prompts` 가드 + `self._prompts` 추적 + 호출 직후 `self._class_names = self._yolo.names` 갱신. 라벨 정확성 + 중복 재임베딩 방지 동시 해결.

검증: symlink-install 이라 노드 재기동으로 반영. 동일 테이프 프롬프트(conf 0.01) 재탐지 → `class_id 3, class_name "round object"` (이전 "motorcycle") 로 **활성 vocab 정확 반영**, bbox_center (431,325) = 테이프 위치. `SUCCEEDED`.

### 재발 방지
`set_classes()` 로 vocab 변경 시 라벨 캐시(`self._class_names`) 도 반드시 동기 갱신한다. 고비용 vocab 재임베딩은 prompt 가 실제 변경될 때만 수행(매 goal 무조건 호출 금지).

## 2026-06-01 21:00 (KST) — cyclo 진동 근본 해결 + UI dedup refactor + MoveIt jog 지연 분석

### 증상
1. UI Cartesian Jog (cyclo backend) 명령 없는 idle 상태에서 robot joints 진동 (±0.01 rad, j2/j5).
2. UI 깨짐 — 탭 헤더 + 콤보 텍스트 잘림.
3. ee_leader_marker RViz 깜박임.
4. Marker 탭이 selected arm 한쪽만 표시 (Jog 양 arm 표시와 비대칭).
5. MoveIt jog 명령 후 ~0.5 초 motion start lag + 총 ~2 초 result lag.

### 원인
1. `omx_movel_controller_node.cpp:controlLoopCallback` 의 else 분기에서 `desired_vel = kp × (goal − current)` 영구 호출. `movel_goal_pose_` 가 한 번 set 후 clear 안 됨 + `q_feedback = q_commanded_` (open-loop) + kp=50 → joint-limit boundary 에서 100Hz chatter (10-lens workflow 합의 confidence 0.95).
2. Linear velocity row (`vrow`) 별도 추가로 가로 minimum width 증가 + window 부족 → squeeze.
3. 동일 `ee_leader_marker` 가 두 launch 동시 spawn (`scenario_player_with_ee_leader.launch.py` + 별도 `openarmx_scenario_workflow.launch.py`) → InteractiveMarkerServer sequence number 충돌.
4. `_build_ee_leader` 단일 `lblMarkerPos`/`lblMarkerRot` 만 생성 + `_on_marker_pose` 가 selected arm filter.
5. `_send_target` 의 `vel_scale=spnVel.value()` default=0.10 (10% velocity scale) → Pilz solver=0ms 인데 JTC trajectory execute 가 ×10 느림. 실제 motion-start lag (~300-500ms) 은 MoveIt ActionClient + `execute_trajectory` 중계의 본질적 overhead.

### 수정
1. `cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:379-389` — `else { active=false; return; }` (trajectory 종료 = publish 명시 중단). cyclo 단일 패키지 빌드 (1min 5s). 검증: idle 5s `/right_joint_trajectory_controller/joint_trajectory` 0 msg, joint_states byte-exact 동일.
2. `openarmx_scenario_ui/main_window.py:154-158` — `setMinimumSize(1962, 1365)` + resize 1962×1365.
3. 단일 launch 정책 — `scenario_player_with_ee_leader.launch.py spawn_workflow_rviz:=true` 만 사용. 별도 workflow launch 폐기.
4. `cartesian_control_tab.py:_build_ee_leader` — left/right 각자 QGroupBox + `_lbl_marker_pos`/`_lbl_marker_rot` dict. `_refresh_marker_display` 가 양 arm 모두 갱신.
5. `_send_target(..., vel_override=-1.0)` 인자 추가 + `_on_jog` MoveIt 분기에서 `cmbLinVel(mm/s)/MOVEIT_JOG_LIN_BASE_MPS(0.1)` → vel_scale 동적 매핑 (100mm/s 선택 시 vel_scale=1.0).

### UI dedup / 코드 정리 (6-lens workflow 후 Top 9 적용)
- `geometry_utils.py` 신규 — `rpy_to_quat`, `quat_to_rpy`, `pose_dict_to_se3_components`, `interp_pose` (slerp). `scenario_action_client.py` 의 inline 정의 22 줄 제거.
- `scenario_action_client.py` 에 `_lookup_transform_safe(dst, src, timeout)` private helper — `transform_pose` / `get_ee_pose` 중복 try/except 8-line 패턴 통합.
- `joint_data.py` 에 `ARM_SCALE`, `GRIP_SCALE`, `CYCLO_BASE_FRAME = "openarmx_body_link0"`, `_poses_dir()` 중앙화. `joint_control_tab.py` / `cartesian_control_tab.py` 의 중복 정의 제거.
- `cartesian_control_tab.py:486,519` — undefined `frame` 변수 참조 (`NameError` at runtime) → `user_frame` rename.
- `ik_check.py:_dict_to_se3` — quaternion (`qw,qx,qy,qz`) 또는 RPY (`roll,pitch,yaw`) 둘 다 수용 (ZYX intrinsic 변환).
- 헤더 `_build_current_pose` widget hide — Jog 양 arm joint angles 와 정보 비대칭 제거.

### cyclo C++ dedup refactor (별도 executor agent 진행, 빌드 통과)
- `include/cyclo_motion_controller_ros/utils/pose_utils.hpp` 신규 — `publishPoseStamped`, `publishStringMsg`.
- `include/cyclo_motion_controller_ros/utils/trajectory_utils.hpp` 신규 — `makeJointTrajectoryMsg`.
- `include/cyclo_motion_controller_ros/utils/controller_params.hpp` 신규 — `CommonControllerParams` struct + `declareCommonControllerParams(Node*)`.
- 6 controllers (`omx_movel/movej`, `omy_movel/movej`, `ai_worker_movel/movej`) constructor / publish 호출 helper 로 통일.

### MoveIt jog 지연 분석 결과 (계측)
| 단계 | 시간 | 비고 |
|---|---|---|
| entry→server_ready | 1 ms | |
| server_ready→goal_built | 2 ms | |
| send_goal_async | 1 ms | |
| **goal_response (DDS accept)** | **100-300 ms** | ActionClient handshake 본질 overhead |
| **accept→result (planning+execute)** | **2000+ ms (vel_scale 0.1)** | planning_time(solver)=0 ms; 전부 JTC execute |
| **planning_time(solver)** | **0 ms** | Pilz LIN 즉시 |
| **TOTAL** | **~2.5 s** | 본질적으로 ActionClient 2-단계 round-trip 큼 |

→ `vel_scale=1.0` (cmbLinVel=100mm/s) 적용해도 motion-start lag (~500ms) 은 MoveIt 아키텍처 본질. 빠른 jog 는 cyclo backend 가 architectural fit (~50ms publish-to-motion).

### 정책 / 메모리 추가
- [[feedback_kill_all_before_restart]] — 노드/UI 재시작 시 부분 kill 금지, `kill_all_ros2.sh` 로 전체 종료 후 재기동.
- [[feedback_rviz_must_always_spawn]] — stack/UI launch 시 RViz 무조건 함께. `--no-rviz` / `spawn_workflow_rviz=false` 금지.

### 관련 파일
- `cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp` (deadband fix + refactor)
- `cyclo_motion_controller_ros/include/cyclo_motion_controller_ros/utils/{pose,trajectory,controller_params}_utils.hpp` 신규
- `openarmx_scenario_ui/openarmx_scenario_ui/{cartesian_control_tab,scenario_action_client,joint_data,joint_control_tab,main_window,ik_check,geometry_utils}.py`
- `openarmx_pick/launch/openarmx_movel_bimanual.launch.py` (cyclo config — 원본 값 복원)
- `experiments/{test_cyclo_movel_velocity,test_ik_check}.py` 신규

### 미해결 / 후속
- MoveIt motion-start lag 추가 단축: `trajectory_execution_manager` 튜닝 또는 `compute_cartesian_path` service 사용 검토.
- cyclo `q_feedback = q_commanded_` (open-loop) → closed-loop 전환은 별도 작업.
- ee_leader_marker `onTick()` 의 10Hz 무조건 `applyChanges()` (auto_follow_link 기본 true) — pose-change guard 추가 시 RViz CPU 추가 절감 가능.

---

## 2026-06-01 19:28 (KST) — openarmx_pick MoveL 인터페이스 불일치 (robotis_interfaces → openarmx_scenario_player_msgs)

### 증상
`grasp_pose_node` 가 `auto_send:=true` 로 발행하는 MoveL 이 현재 구동 중인 cyclo MoveL 컨트롤러(`openarmx_left_movel_controller`)에 전혀 연결되지 않음. 비전 박스 픽업의 모션 단계(pre-grasp hover)가 동작 불가.

### 원인
MoveL 스택 전체가 `robotis_interfaces/MoveL`(토픽 `/openarmx/movel`) → `openarmx_scenario_player_msgs/MoveL`(토픽 `/openarmx/{left,right}/movel`)로 마이그레이션됨. cyclo 컨트롤러는 이미 전환 완료(`openarmx_ws/src/cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp:89` 가 `openarmx_scenario_player_msgs::msg::MoveL` 구독), 그러나 `openarmx_pick` 만 옛 타입·옛 토픽에 잔류. 두 msg 필드는 동일(`geometry_msgs/PoseStamped pose` + `builtin_interfaces/Duration time_from_start`)이라 타입/토픽 이름만 불일치 → 구독자 0.

### 수정
`openarmx_pick` 6개 파일을 새 인터페이스로 정렬 (로직 변경 없음, 필드 동일):
- `openarmx_pick/grasp_pose_node.py` — import `robotis_interfaces`→`openarmx_scenario_player_msgs`, `movel_topic` 기본값 `/openarmx/movel`→`/openarmx/left/movel`, warn 텍스트 (3줄)
- `package.xml` — `<exec_depend>` `robotis_interfaces`→`openarmx_scenario_player_msgs` (1줄)
- `launch/openarmx_pick.launch.py` — `movel_topic`→`/openarmx/left/movel` (1줄)
- `launch/openarmx_movel.launch.py` — `movel_topic` 기본값 + docstring (2줄)
- `scripts/verify_solver.py` — import + publish 토픽 (2줄)
- `README.md` — 토픽 테이블 + 빌드 오버레이 문구 (3곳)

검증: `colcon build --packages-select openarmx_pick` 성공. 런타임 `ros2 topic info /openarmx/left/movel -v` → `grasp_pose_node`(pub) + `openarmx_left_movel_controller`(sub) 모두 `openarmx_scenario_player_msgs/msg/MoveL` 로 타입 일치 확인. grasp 노드 기동 시 "MoveL unavailable" 경고 없음.

### 재발 방지
MoveL msg 타입은 `openarmx_scenario_player_msgs/MoveL` 로 단일화(cyclo + scenario_player + openarmx_pick). `robotis_interfaces` 는 cyclo_ws 에 잔존하나 미사용(vestigial). 새 MoveL 발행/소비 노드는 이 타입 + `/openarmx/{left,right}/movel` 토픽 규약을 따른다. 남은 통합 갭(GAP 2): descend→close→lift→place pick FSM + main-box filter 는 별도 작업.

## 2026-06-01 16:30 (KST) — UI Cartesian Jog (cyclo backend) 안 움직임 + 진동

### 증상
1. UI `Cartesian Control → Jog` 탭에서 cyclo backend 선택 후 +Z 50mm 클릭 → robot 거의 안 움직임 (50mm 명령 → 1.9mm = 3.8% 진행). 다른 방향은 부분 도달 (42-72%).
2. 어떤 명령도 보내지 않았는데 robot 진동 계속.
3. UI 클릭 후 status 영역에 `PRESS ...` 만 표시되고 그 다음 메시지 없이 robot 안 움직임 (Python 측 silent fail).

### 원인
1. `openarmx_pick/launch/openarmx_movel_bimanual.launch.py` 의 cyclo config 보수적: `trajectory_time=0.05, kp_position=20.0, kp_orientation=2.5`. 원본 `cyclo_control/cyclo_motion_controller_ros/config/omx_config.yaml` 은 `trajectory_time=0.0, kp_position=50.0, kp_orientation=50.0, collision_buffer=0.01, collision_safe_distance=0.005`.
2. cyclo는 첫 MoveL 받은 후 `movel_goal_pose_` 영구 저장. controlLoop 종료 후에도 `desired_vel = kp × (goal - current)` 로 publishTrajectory 영구 호출 (100Hz). robot이 도달 못 한 자세에서 추적 시도 → 진동. cyclo source [omx_movel_controller_node.cpp](openarmx_ws/src/cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp) 의 `q_feedback = q_commanded_` (open-loop) 라서 robot 실제 joint_states 무시.
3. cyclo는 unreachable target에 대해 `controller_error` 발행 안 함 — QP slack penalty로 항상 solve success 반환.
4. UI Jog 는 거리 step만 받고 속도 개념 없이 `duration_sec=2.0` 고정. Jog 본질은 속도 명령 (속도 × 제어시간 = 이동량) 인데 horizon 미지정.
5. `cartesian_control_tab.py:_apply_delta` 반환 dict `{x,y,z, roll,pitch,yaw}` 에 quaternion 없음. `transform_pose` 의 `src==dst` 분기에서 input 그대로 반환 → IK pre-check 의 `_dict_to_se3` 가 `pose["qw"]` 접근 시 KeyError 발생 → PyQt slot silent fail.

### 수정
1. `openarmx_pick/launch/openarmx_movel_bimanual.launch.py:46-58` — cyclo config 를 원본 cyclo_control 값으로 복원. 검증: Z+50mm 도달률 3.8% → 42.1% (11배). 도달 가능 방향 (-Y) 은 100%+ 도달.
2. `openarmx_scenario_ui/openarmx_scenario_ui/ik_check.py` 신규 — Pinocchio 기반 `LinearReachabilityChecker`. damped LS Newton-Raphson IK + 직선 경로 N등분 검증. unreachable waypoint detect → 사유 (`no_convergence` / `joint_limit`) + 실패 waypoint index 반환.
3. `cartesian_control_tab.py` — cyclo backend 분기에 IK pre-check 호출 추가. fail 시 publish 차단 + status `"UNREACHABLE: <reason> @ waypoint i/N"` 표시.
4. `cartesian_control_tab.py` — `LIN_STEPS_MM` 에 20mm 추가, `LIN_VELS_MM_S = [10,25,50,100]` / `ANG_VELS_DEG_S = [5,10,30,60]` 콤보 추가. `duration_sec = step / velocity` 자동 계산해서 cyclo MoveL publish.
5. `ik_check.py:_dict_to_se3` — quaternion (`qw,qx,qy,qz`) 또는 RPY (`roll,pitch,yaw`) 둘 다 수용 (ZYX intrinsic → quaternion 변환).

### 검증 결과
| 방향 | 명령 | 결과 (이전 config) | 결과 (원본 config) |
|---|---|---|---|
| +Z 50mm | (0,0,+50) | (+15.4, -4.8, **+28.8**) 57% | 도달 가능성 IK pre-check fail (joint4 limit) |
| -Z 20mm | (0,0,-20) | n/a | (-0.1, +0.2, **-20.4**) **102%** ✅ |
| -Y 20mm | (0,-20,0) | n/a | (+2.6, **-21.9**, -1.3) **109%** ✅ |

IK pre-check 단독 검증 (`experiments/test_ik_check.py`): +Z 발산 (waypoint 1/10, joint4 limit), -Z/-Y 10/10 통과.

### 미해결 / 후속
- **진동 근본 해결**: `omx_movel_controller_node.cpp:controlLoopCallback` 에 `active=false` + `‖goal − current‖ < threshold` 시 `publishTrajectory` 호출 중단 (deadband) 추가 필요. cyclo C++ 빌드 (j1, ~4분) 필요. 현재는 cyclo 노드 재시작으로 임시 대처.
- cyclo `q_feedback = q_commanded_` (open-loop) → 실제 joint_states 피드백으로 변경 필요 시 (closed-loop) cyclo 정공법 수정.

### 관련 코드 / 파일
- `experiments/test_cyclo_movel_velocity.py` — cyclo MoveL 단발 검증 스크립트
- `experiments/test_ik_check.py` — IK pre-check 단독 검증 스크립트
- `openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/ik_check.py` 신규
- `openarmx_pick/launch/openarmx_movel_bimanual.launch.py` (cyclo config)
- `cyclo_control/cyclo_motion_controller_ros/config/omx_config.yaml` (원본 ref)

### 재발 방지
- cyclo 노드 새 launch 작성 시 항상 원본 `omx_config.yaml` 값을 baseline 으로 사용. 다른 값 쓰면 코멘트로 사유 명시.
- UI Cartesian linear motion 명령은 publish 전 IK pre-check 통과 필수.
- Pose dict 핸들러는 quaternion / RPY 양쪽 형식 모두 수용.

---
