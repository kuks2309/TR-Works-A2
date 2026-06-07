# 이슈 / 수정 기록 (China 워크스페이스)

China 모노레포 전체의 이슈·원인·수정 누적 기록. 최신 항목이 위 (prepend).
항목 형식은 [README.md](README.md) 참조. 작업 흐름은 `.claude/skills/issue-fix/SKILL.md` 참조.

<!-- 새 항목은 아래 구분선 바로 다음 줄에 추가 (최신 위). -->

---

## 2026-06-07 16:00 (KST) — JTC(Joint Trajectory Controller) 명령 시 "덜덜덜" 떨림 근본원인 진단 (진단만)

### 증상
JTC(Joint Trajectory Controller)로 명령 시 관절이 고주파로 "덜덜덜" 떤다. 정지(hold) 중에도 지속.

### 원인
rosbag 분석(상세: [2026-06-07_jtc_tremor_diagnosis.md](2026-06-07_jtc_tremor_diagnosis.md)) 결과 2층위.
1. **(주원인) 모터단 MIT 임피던스 한계진동**: [v10_simple_hardware.cpp:572-591](../../openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp#L572-L591) `τ=KP·Δp+KD·(0−vel)+τ_ff`, KP/KD={50/2.5(j1-4),10/0.5(j5-7)}, 적분 없음. JTC 가 position-only → vel_cmd=0 이라 KD 가 **엔코더 양자화(0.022°) 속도노이즈를 증폭** → 토크 채터. hold 중 위치 p2p 0.022°인데 속도 ±6~13°/s·부호반전 ~38회/초.
2. **(악화) 호스트 루프 저속·지터**: [read():479-483](../../openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp#L479-L483) 의 동기 CAN(Controller Area Network) `recv_all()` 로 루프가 100Hz→**55~68Hz**(헤더 stamp dt 로 확정, 기록누락 아님), 지터 최대 수백 ms.
3. (별개) 무적분 정적 처짐 err_mean 2~5°. `open_loop_control:true` 는 "jog 풀림"만 막음.

### 수정
진단만 — 코드 변경 없음. 분석 스크립트 [analyze_jtc_jitter.py](../../experiments/analyze_jtc_jitter.py), [analyze_jointstates_tremor.py](../../experiments/analyze_jointstates_tremor.py) 추가. 해결 방향(게인 재튜닝/속도필터, 루프율 복구, velocity FF)은 사용자 지시 대기. [[motor_gain_tuning_plan]] [[motor_speed_limit_mit_mode]]

### 재발 방지
고주파 떨림은 controller_state(35Hz)로 못 본다 → `/joint_states` 속도/토크 + **헤더 stamp dt** 로 분석(저속 토픽 reversal=양자화 dither 혼동 주의).

---

## 2026-06-07 15:59 (KST) — ptp 전용 pick-and-place Qt UI 신규 패키지 `openarmx_ptp_ui` 생성 (표시·구동 분리) + `--symlink-install` 엔트리 실행비트 누락 수정

### 동기
ptp 묶음(`pick_and_place/ptp/`)에 모션 백엔드(C++ `openarmx_ptp_box_align`)와 액션 메시지만 있고 **전용 UI 부재**. ptp pick-and-place 구동에 하드웨어·컨트롤러·D435·원격검출/인지·정렬 백엔드·RViz 를 터미널에서 6개 수동 launch 후 `ros2 action send_goal` 직접 입력 필요. 직전 진단([2026-06-07 send_goal CLI 지연](2026-06-07_ui_detect_send_goal_cli_latency.md), [[ui_detect_cli_subprocess_bottleneck]])의 후속으로 **in-process 액션 클라이언트**를 채택한 전용 UI 를 신설.

### 구현 (신규 패키지 `openarmx_ptp_ui`, ament_cmake, scenario_ui 미러)
사용자 지시: UI 는 Qt(PyQt5), **표시와 구동의 분리** 준수, 가능한 코드는 C++(이미 `ptp_box_align_node`)·UI 는 Python, 계획참조 scenario. 사용자 결정 3건 = (1) Qt Designer `.ui`+분리 로직, (2) Launch 범위 전체 파이프라인, (3) AlignToBoxes 연동 in-process rclpy ActionClient.
- **표시(Display)**: [ui/ptp_pnp_ui.ui](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_ui/ui/ptp_pnp_ui.ui) — Qt Designer XML(launch 7행 + 골 파라미터 + Run/Cancel + 진행바 + 로그).
- 표시 결선: [main_window.py](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_ui/openarmx_ptp_ui/main_window.py) — `uic.loadUi` + 위젯↔구동 시그널 연결만.
- **구동·프로세스**: [managed_process.py](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_ui/openarmx_ptp_ui/managed_process.py) — scenario_ui 동일 파일 vendoring(시나리오 패키지 비결합, 백엔드 분리 정책과 동일).
- **구동·ROS**: [ptp_ros_bridge.py](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_ui/openarmx_ptp_ui/ptp_ros_bridge.py) — 백그라운드 스레드 rclpy `ActionClient`(`/openarmx/ptp_align_to_boxes`), `send_goal_async`/`cancel_goal_async`, feedback(phase/progress)·result→Qt 시그널.
- 엔트리/launch/문서: [scripts/ptp_pnp_ui.py](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_ui/scripts/ptp_pnp_ui.py), [launch/ptp_pnp_ui.launch.py](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_ui/launch/ptp_pnp_ui.launch.py), [README.md](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_ui/README.md).
- launch 7행 명령은 scenario_ui `launch_manager_tab.py`/`pick_and_place_tab.py` 의 검증된 것 그대로. RViz config 는 src 직접 로드(realpath walk-up, 설치/share 금지 규칙). ptp 는 MoveIt-free 라 move_group 불필요.

### 빌드/발견 gotcha (수정)
`colcon build --symlink-install` + `install(PROGRAMS …)` 환경에서 colcon 이 설치본을 src 스크립트로 **심볼릭 링크**한다. 이때 src `scripts/ptp_pnp_ui.py` 에 **실행비트(+x)가 없으면** 링크가 비실행 파일을 가리켜 `ros2 pkg executables` 가 빈 결과 → `ros2 run`/`ros2 launch` 가 엔트리를 못 찾음(`install(PROGRAMS)` 의 0755 권한이 심링크로 무력화됨). 비교: scenario_ui `scenario_ui.py` 는 `-rwxrwxr-x`. → `chmod +x src/…/scripts/ptp_pnp_ui.py` 로 해결(재빌드 불필요, 심링크가 src 직참조).

### 검증
- `colcon build`(msgs+ui) PASS. py_compile OK. `.ui` 오프스크린 로드 — 23개 위젯 전부 존재.
- 액션 Goal/Feedback/Result 필드 일치(`z/roll_deg/pitch_deg/yaw_deg/arms`, `phase/progress`, `success/message/detections_json/assignments_json`).
- `_SCENARIO_RVIZ` 가 **src 경로**로 해석(설치본 아님). 윈도우 생성·상태갱신·서버없을때 graceful(run→False)·종료(closeEvent→bridge.shutdown) 정상.
- **렌더 스크린샷 검증**(프로젝트 규칙): 레이아웃/색(초록 Start·빨강 Stop)/한글/진행바 60%/로그 정상, 외부 노드 감지(`● Running (external)`) 실동작 확인.
- `chmod +x` 후 `ros2 pkg executables openarmx_ptp_ui` → `ptp_pnp_ui.py`, launch 가 `LaunchDescription` 생성.
- **미실행**: 실로봇 스택 라이브 E2E(SIL→컨트롤러→ptp 백엔드 + `/detected_boxes` 발행 → Run/Cancel)는 외부 실행 노드 충돌 우려로 보류(사용자 파지 테스트 후 kill_all 클린 기동하여 진행 예정).

### 재발 방지
- `--symlink-install` 패키지의 `install(PROGRAMS)` 엔트리 스크립트는 **src 파일에 `chmod +x` 필수**(설치 권한이 심링크로 무효화됨). 신규 UI/노드 스크립트 추가 시 점검.
- 반복 호출 액션은 UI 에서 in-process ActionClient 사용(CLI send_goal spawn 금지) — [[ui_detect_cli_subprocess_bottleneck]].

---

## 2026-06-07 07:23 (KST) — LeRobot leader teleoperator 플러그인 entry-point 미등록 → lerobot-record 실패 (1줄 수정)

### 증상
`AI/scripts/vla_collect_gui.sh collect` 로 VLA 수집 기동 시 `lerobot-record --teleop.type=openarmx_leader_ros2` 가 leader 텔레오퍼레이터 플러그인을 해석하지 못해 인스턴스화 실패 → 수집 자체가 불가. (E2E 미실행 상태에서 TR-AI-Pick 타당성 조사 중 사전 발견.)

### 원인
`AI/lerobot_teleoperator_openarmx_leader_ros2/pyproject.toml` 에 `[project.entry-points."lerobot.teleoperators"]` 섹션이 **없음**. LeRobot 외부 플러그인은 entry-point 그룹으로 검색되는데, follower 는 등록돼 있고([lerobot_robot_openarmx_follower_ros2/pyproject.toml:42-43](../../openarmx_ws/src/pick_and_place/AI/lerobot_robot_openarmx_follower_ros2/pyproject.toml#L42-L43), `lerobot.robots`) leader 만 누락. 클래스 자체는 [config_openarmx_ros2.py:58-60](../../openarmx_ws/src/pick_and_place/AI/lerobot_teleoperator_openarmx_leader_ros2/lerobot_teleoperator_openarmx_leader_ros2/config_openarmx_ros2.py#L58-L60) 에 `OpenArmXRos2TeleopConfig` + `@TeleoperatorConfig.register_subclass("openarmx_leader_ros2")` 로 존재 → 코드는 있으나 **패키징 노출(entry-point)만 빠진** 상태.

### 수정
[lerobot_teleoperator_openarmx_leader_ros2/pyproject.toml](../../openarmx_ws/src/pick_and_place/AI/lerobot_teleoperator_openarmx_leader_ros2/pyproject.toml) 끝에 follower 형식을 미러링해 2줄 추가:

```toml
[project.entry-points."lerobot.teleoperators"]
openarmx_leader_ros2 = "lerobot_teleoperator_openarmx_leader_ros2.config_openarmx_ros2:OpenArmXRos2TeleopConfig"
```

### 추가 수정 — install_lerobot_plugins.sh 가 그대로는 설치 실패
검증을 위해 `~/lerobot-venv`(`python3 -m venv --system-site-packages`, LeRobot 0.4.4, numpy 2.2.6)에 설치를 시도하니 스크립트가 두 가지로 실패:
1. **행(hang)**: 플러그인 `dependencies = ["lerobot","rclpy"]` 중 `rclpy` 는 PyPI 에 없음(시스템 ROS2 제공) → 평범한 `pip install -e` 가 PyPI 에서 rclpy 를 풀려다 의존성 백트래킹에 갇힘(hf-transfer 경고 수십 회 반복).
2. **leader = "UNKNOWN-0.0.0"**: 빌드 격리 환경의 setuptools 가 leader `pyproject.toml` 의 `[project]` 를 못 읽어 이름 없는 패키지로 빌드 → entry-point 미등록. (leader 는 follower 와 달리 `setup.py` 없음.)

→ [scripts/install_lerobot_plugins.sh](../../openarmx_ws/src/pick_and_place/AI/scripts/install_lerobot_plugins.sh) 를 검증된 incantation 으로 수정:
`pip install -q -U "setuptools>=64" wheel setuptools_scm` 선행 후 `pip install --no-deps --no-build-isolation -e <plugin>`.
(`--no-deps`: rclpy/lerobot 이미 충족 → 백트래킹 회피. `--no-build-isolation`: env 의 setuptools 80 사용 → `[project]`/entry-points 정상 인식 + PEP 660 editable, UNKNOWN 회피.)

### 검증
- 정적: TOML 파싱 OK / 그룹 `lerobot.teleoperators` · 키 `openarmx_leader_ros2` · 타겟 경로 정합 / AST 로 `OpenArmXRos2TeleopConfig` 실재 + `register_subclass` 키 일치 / 동작 중인 follower 와 형식 대칭 확인.
- **런타임 PASS ✅**: 고친 스크립트 실행 결과 양 플러그인 editable 설치 + 검증기 출력 `lerobot.robots : ['openarmx_follower_ros2']` / `lerobot.teleoperators : ['openarmx_leader_ros2']`. → `--teleop.type=openarmx_leader_ros2` 해석 가능. (수정 전엔 teleoperators 빈 리스트.)

### 재발 방지
- LeRobot 플러그인 이식/추가 시 `@*.register_subclass(...)` 데코레이터와 `pyproject.toml` 의 `[project.entry-points."lerobot.<robots|teleoperators>"]` 등록을 **반드시 쌍으로** 확인.
- LeRobot 플러그인(rclpy 의존 + 로컬 경로) 은 항상 `pip install --no-deps --no-build-isolation -e` 로 설치. 평범한 `pip install -e` 는 rclpy PyPI 백트래킹으로 멈춘다.
- 설치 직후 `install_lerobot_plugins.sh` 의 entry-point 검증 출력에 robots/teleoperators 키가 모두 나오는지 점검.
- LeRobot venv 는 numpy 2.x 필수(cv_bridge 만 segfault, 플러그인은 회피) — [[lerobot_venv_numpy2_strategy]] 참조.

---

## 2026-06-07 (KST) — pick_and_place UI 검출/pick 지연: ros2 action send_goal CLI subprocess 콜드스타트 (진단만, 수정 보류)

### 증상
scenario_ui pick_and_place 탭에서 검출요청/pick 누르면 ~0.8~0.9초 소요. 사용자 의문 "UI 타이머 때문인가?". 반면 액션 서버 자체(이미지→Pi YOLO 분류 E2E)는 ~46ms.

### 원인
**UI 타이머 아님**(QTimer 는 상태표시 2s·자세 readout 0.5s 용, 검출/pick 경로와 분리). 근본 원인은 UI 가 검출요청·pick 을 매번 `ros2 action send_goal` **CLI 를 QProcess 로 새로 spawn** → 매회 콜드스타트(bash + 워크스페이스 source 0.2s + ros2 CLI 로드 + 새 rclpy 노드 + DDS discovery). 라이브 실측: CLI 방식 **~850ms** vs in-process 액션 클라이언트 **43.8ms** (약 19배). 근거 [pick_and_place_tab.py:561-584](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py#L561-L584)(검출요청), [:484-495](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py#L484-L495)(메인 pick).

### 수정
미적용. 진단·측정만 수행(코드 변경 없음). **전용 pick and place UI 생성 시 in-process 액션 클라이언트로 적용 예정**(사용자 결정). 상세·측정표 → [2026-06-07_ui_detect_send_goal_cli_latency.md](2026-06-07_ui_detect_send_goal_cli_latency.md).

### 재발 방지
반복 호출 액션/서비스는 UI 에서 `ros2 … send_goal` CLI spawn 금지(매회 ~0.8s). in-process 액션 클라이언트 사용. CLI spawn 은 일회성 launch 전용.

---

## 2026-06-07 05:02 (KST) — ptp 백엔드 모델 이중화 제거 리팩터 + 31mm xacro SSOT 수정 (라이브 검증완료)

### 증상 / 동기
- FK(모델)가 실제 팔보다 **상수 +31mm 높음**(5점 측정, 자세 무관 → arm-base 캘리브레이션 오차). 초기엔 (a) 명령 z에 +0.031 더하기, (b) 솔버 URDF base z 1.275→1.244 손편집으로 우회.
- 그 결과 모델값이 **3중 분기**(솔버 URDF 1.244 vs full URDF/TF 1.275 vs xacro 1.275) → "하나만 고치니 서로 다른 값으로 이상하게 움직임". 사용자: "비정상, full urdf에서 읽어서 필요한 것만 사용해야".

### 수정 1 — ptp 가 full 모델에서 런타임 축소 (이중화 제거)
- [ptp_box_align_node.cpp](openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_box_align/src/ptp_box_align_node.cpp): 솔버 URDF 파일 로드 제거 → `/robot_description`(std_msgs/String, transient_local) 구독 → `pinocchio::buildModelFromXML`(full) → 팔별 `buildReducedModel`(상대팔+손가락 관절 lock)로 7-DOF 단일팔 모델 런타임 생성. `onRobotDescription`/`buildArmFromFull` 신설.
- [launch](openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_box_align/launch/ptp_box_align.launch.py): `left/right_urdf_path` 인자 제거, `move_time` 기본 6.0→2.5. CMakeLists.txt/package.xml: `std_msgs` 의존 추가.

### 수정 2 — 31mm 를 xacro SSOT 한 곳에서만
- [v10.urdf.xacro:50,53](openarmx_ws/src/openarmx_description/urdf/robot/v10.urdf.xacro#L50): `right/left_arm_base_xyz` z `1.275 → 1.244`. 이게 full URDF/TF/RViz/MoveIt/모든 백엔드의 단일 근원. launch 가 arm_base 인자를 override 하지 않음(grep 확인) → 재기동 시 그대로 반영.
- 솔버 URDF(cyclo 전용, ptp 미사용)도 1.244 → 이제 xacro 와 정합. 명령 z +0.031 보정도 제거(모델이 맞으니 불필요).

### 적용 / 검증 (라이브)
- ptp colcon 재빌드 성공. **오프라인**: xacro 전개→pinocchio full 파싱 OK(nq=18, mimic/ros2_control 파싱에러 없음)→buildReducedModel 왼팔 nq=7, hand_tcp 프레임 O, neutral z=0.566(=옛 0.597−31mm).
- **라이브**: 재기동 후 `/robot_description` base z=**1.2440** 확인. 좌팔 (0.10,0.10,0.72) 액션 이동 후 **실제 엔코더 FK z=0.7063 = 사용자 실측 일치** → 1.244 모델 정확 확정.

### 별개 이슈 (모델 아님)
- 목표 0.72 대비 실제 0.706 = **−14mm 관절 추종 처짐**(MIT 모드 적분無, 팔꿈치 joint4 지령 85.7°→실제 81.8°). 게인/중력보상 영역. [[motor-gain-tuning-plan]]
- **토픽 단발 발행 유실**: 짧은 수명 노드가 `/<side>_joint_trajectory_controller/joint_trajectory` 토픽에 1회 발행 후 즉시 종료 → 컨트롤러 구독 매칭(discovery) 전이라 메시지 유실(팔 안 움직임). **FollowJointTrajectory 액션**(wait_for_server 핸드셰이크)으로 보내야 확실. 이 PC DDS 불안정 시 특히.

### 재발 방지
- 모델(기구학) 파라미터는 **xacro SSOT 한 곳**만 수정. 파생물(솔버 URDF, 명령 오프셋)에 band-aid 금지 → 분기→이상거동.
- 모델 검증 = **엔코더 FK vs 실측**(모델 오차)와 **지령 관절 vs 실제 관절**(추종 처짐)을 분리. 둘을 섞으면 오진.
- 실로봇 단발 모션은 토픽 발행 말고 액션 사용(discovery 유실 방지).

---

## 2026-06-07 04:59 (KST) — cyclo control: ① Stop 시 종료 안 됨 ② 요청 안 한 자동기동

### 증상
① Launch Manager 에서 cyclo MoveL 행 Stop 을 눌러도 cyclo 가 종료되지 않음("Running (external)" 유지). ② 사용자가 요청한 적 없는데 시작 시 cyclo control 이 자동 기동됨.

### 원인
- ① cyclo preset 의 stop 패턴이 `"openarmx_movel_bimanual"`(launch 파일명)뿐 → `ros2 launch` **부모 프로세스만** 매칭. 부모가 먼저 죽고 컨트롤러 노드(`omx_movel_controller_node …`, cmdline 에 그 문자열 없음)만 고아로 남으면 패턴이 아무것도 못 잡아 노드 생존. (좌/우 노드+부모는 같은 PGID 라 부모 살아있으면 그룹 kill 됐지만, 고아 시 누락.)
- ② cyclo 자동기동 설정 2곳: (a) [scenario_player_with_ee_leader.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/scenario_player_with_ee_leader.launch.py) `start_movel_controllers` **default `true`** → `openarmx_movel_bimanual` 자동 포함. (b) [main_window.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py) `ScenarioMainWindow` 기본 `follower="cyclo"` → cyclo_extras(cyclo_sim+vr_controller) 자동기동(잠복 불일치; `scenario_ui.py` 는 `--follower` 기본 `moveit`).
  - 참고: 정상 UI 경로(scenario_ui.launch.py→`--no-auto`, follower=moveit)는 cyclo 자동기동 안 함. `openarmx_scenario_workflow.launch.py` 도 cyclo 미포함.

### 수정
- ① [launch_manager_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/launch_manager_tab.py) cyclo preset 에 `"sweep": ["omx_movel_controller_node"]` 추가 → Stop 시 실제 노드 실행파일을 직접(그룹+PID) 종료, 고아 노드도 확실히 정리.
- ②(a) `start_movel_controllers` default `true`→`false` (opt-in: `:=true` 또는 Launch Manager cyclo 행 Start).
- ②(b) `ScenarioMainWindow` 기본 `follower` `"cyclo"`→`"moveit"` (cyclo_extras 는 `--follower cyclo` 명시 시만).

### 검증
- 3개 파일 py_compile OK. ①은 노드 실행파일명이 `omx_movel_controller_node`(좌/우 공통)임을 ps 캡처로 확인 — 패턴이 두 노드 직접 매칭. **라이브 재검증 대기**(cyclo 재기동 후 Stop → 노드 0 확인).
- ②는 정상 경로가 이미 moveit 라 회귀 없음, 기본 자동기동만 제거.

### 재발 방지
- cyclo 종료 패턴은 launch 파일명이 아니라 **노드 실행파일명**(고아 대비)으로 둔다.
- "Stop All (this tab)" 은 여전히 패턴 sweep 안 함 + L1 스포너(`--unload-on-kill`)까지 죽여 컨트롤러 통째 언로드 → 별도 결정 대기(A: Stop All 도 sweep / B: 스포너 제외).

---

## 2026-06-07 04:23 (KST) — UI 종료 시 팔이 토크 차단으로 낙하 → 종료 전 HOME 파크 추가

### 증상
scenario_ui 정상 종료(창 닫기 / Launch Manager "Stop All") 시 하드웨어·컨트롤러 노드가 즉시 죽으며 MIT(Mixed Impedance Torque) 모드 모터 토크가 끊겨 **팔이 중력으로 낙하**.

### 원인
[main_window.py:415](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py#L415) `closeEvent` 가 곧바로 `self._hw.stop()`, Launch Manager `_stop_all` 이 `proc.stop()` 으로 컨트롤러를 종료 → 토크 0 → 낙하. MIT 모드는 홀딩 브레이크 없음(홀딩도 토크 기반) → jog "풀림"([[2026-06-07 jog 풀림 항목]])과 동일 물리의 영구판.

### 수정
종료 직전 양팔을 HOME(전관절 0°)로 보내고 도착(±2°)/타임아웃까지 대기 후 하드웨어 종료.
- bridge `park_arms_home(speed_deg_s=25, tol_deg=2, tick=QApplication.processEvents)` 추가 — 컨트롤러 active(traj 구독>0)일 때만, **그리퍼는 미터치**(쥔 물체 낙하 방지). 도착확인은 spin-thread 가 갱신하는 `latest_positions()` 폴링(Qt 블록 중에도 갱신되는 구조 확인).
- 호출: `main_window.closeEvent` 최상단 + `launch_manager_tab._stop_all` 의 proc.stop 직전.
- 파일: `scenario_action_client.py`(메서드+스냅샷), `main_window.py`, `launch_manager_tab.py` (PyQt5 `QApplication.processEvents` 로 GUI 응답 유지).

### 검증
- py_compile + offscreen import 스모크 **통과** (park_arms_home/latest_positions 존재, HOME=0 확인).
- **라이브 동작 검증 대기**: UI 기동 후 HIL 상태에서 창닫기/Stop All → 팔이 HOME 이동 후 정지(낙하 없음) 확인 필요.

### 한계 / 재발 방지
- **그레이스풀 종료 한정.** kill_all_ros2.sh(즉시 SIGKILL)·크래시·전원차단·e-stop 엔 무력. 자세-무관 보호는 모터 홀딩 브레이크가 정석(현 HW 미탑재 추정).
- HOME(0°)가 중력 안정 자세라는 전제(사용자 선택). 0°가 수평 뻗음이면 효과 제한 → 추후 전용 안정 park 자세 검토 여지.

---

## 2026-06-07 04:09 (KST) — jog(Joint Control 탭) 각도 변경 시 조인트가 "풀렸다 다시 움직임"

### 증상
Joint Control 탭에서 관절 각도를 바꾸면(새 jog 명령), 해당 조인트가 **목표 반대 방향으로 한 번 처졌다가(홀딩 토크 상실=풀림) 다시 목표로 가는** 2단 동작. leader-follower(teleop)에서는 안 나타남.

### 원인 (2층위, rosbag 정량 확인)
rosbag `experiments/rosbags/jog_20260607_034347` 의 `/{arm}_joint_trajectory_controller/controller_state` desired/actual + `/joint_states` velocity 로 확인.
- **L1 — 직접 원인 (JTC(Joint Trajectory Controller) 설정):** JTC 에 `open_loop_control` 미설정 → 기본 `false`. 이 모드는 새 궤적을 **측정된(처진) actual 에서 시작**. 그래서 명령 수신 순간 출력 desired 가 직전 명령값에서 actual 로 **계단 하강** → 하드웨어 위치명령 하락 → 홀딩 토크≈0 → 조인트가 뒤로 처짐.
  - 증거 (CMD #6, 좌 joint1, +0.7954→+1.0472 명령): 명령직전 desired **0.8727** / actual 0.7954 (정적 추종오차 +0.077 rad≈4.4°). t=0.016s 에 desired→**0.7956** 계단하강 → actual 이 **0.7471 까지 역방향 처짐(−0.048 rad≈2.8°)** 후 반전·구동. 우 joint1(CMD #5) 도 부호 반대 동일.
- **L2 — 근본 원인 (하드웨어 게인):** JTC command_interface=`position` 단독, 실제 위치제어는 MIT(Mixed Impedance Torque) 모드 KP(비례게인)/KD(미분게인) 수행. **적분 없어** 정적 처짐 큼(홀딩 4.4°, +1.0472 위치 6.4°). 이 처짐 폭이 곧 L1 의 리셋 갭 → 처짐 클수록 풀림 큼. [[motor-gain-tuning-plan]] 와 동일 맥락.
- **교차검증:** leader-follower(teleop)는 `forward_position_controller`(ForwardCommandController, JTC 아님)로 **200Hz 연속 위치 스트리밍** → "측정상태 리셋" 로직 없음 + 처짐 갭 미누적 → 풀림 없음. ([teleop_bimanual_node.cpp:138-149](../../openarmx_ws/src/openarmx_teleop_bimanual/src/teleop_bimanual_node.cpp#L138-L149))

### 수정
JTC 양팔에 **`open_loop_control: true`** 추가 → 새 궤적을 마지막 명령값에서 시작(처진 actual 리셋 안 함) → desired 계단하강·토크 붕괴 제거. ros2_controllers 표준 파라미터(2.53.1).
- [openarmx_v10_bimanual_controllers.yaml](../../openarmx_ws/src/openarmx_ros2/openarmx_bringup/config/v10_controllers/openarmx_v10_bimanual_controllers.yaml) (활성 HIL config, 좌/우 2곳)
- [openarmx_bimanual_moveit_config/config/ros2_controllers.yaml](../../openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/config/ros2_controllers.yaml) (MoveIt 경로 일관성, 좌/우 2곳)
- symlink-install 이라 install 즉시 반영, 재빌드 불필요. YAML 파싱·4블록 True 확인 완료.

### 검증
- **YAML 유효성 + 파라미터 반영: 완료.**
- **라이브 동작 검증: 완료 (2026-06-07 04:36).** 재기동 후 양팔 JTC(Joint Trajectory Controller) `open_loop_control=True` 확인. rosbag `jog_verify_20260607_043459` (32개 jog 명령, joint1 −1.175 rad 대이동 포함)에서 명령 직후 역방향 처짐 **최대 0.0004 rad (0.02°)** — 수정 전 −0.048 rad (2.8°) 대비 사실상 0(노이즈 수준). 이동 크기와 무관하게 단조 추종. **풀림 제거 확인.** `open_loop_control` 은 read-only 라 적용엔 컨트롤러 재로드(클린 re-bringup) 필요했음.

### 재발 방지
- position-command JTC + 적분 없는(MIT 모드) 하드웨어 조합에서는 **`open_loop_control: true` 가 기본**이어야 함(정적오차 리셋 갭이 풀림으로 보임). 신규 JTC config 추가 시 체크.
- 근본적으로는 L2(정적 처짐) 축소 = 중력보상 피드포워드 또는 KP(비례게인)↑ ([[motor-gain-tuning-plan]]).

---

## 2026-06-06 23:09 (KST) — 팔 FK(Forward Kinematics) z 가 실제보다 상수 ~31mm 높음(엔코더 아래) + 관절 추종 처짐(게인 의존)

### 증상
ptp 로 왼팔을 명령 z 로 보내면 **실제 TCP(Tool Center Point) z 가 일정하게 낮음**. 명령 z=0.75 인데도 그리퍼가 바닥(실제 0.72)에 닿음.

### 원인 (z 오차가 2종)
1. **상수 ~31mm 바이어스 — FK(Forward Kinematics)(실측 관절각) vs 실제.** 5점 측정에서 FK(Forward Kinematics) z − 실측 z = +29~32mm (평균 +30.6, 편차 ±1.3), z 0.71~0.79 전 구간·자세 무관. 엔코더는 명령각에 와 있으므로 **관절각 오차가 아님** — 엔코더 아래 기구부 컴플라이언스(감속기/링크 휨) 또는 모델 캘리브(arm base z 1.275 의심). **게인과 무관.**
2. **관절 추종 처짐 — 명령각 vs 실측각.** e_ss = τ_load/KP(비례게인) (MIT(Mixed Impedance Torque) 모드 적분 없음). 자세·부하 의존(joint4 가장 큼). **KP(비례게인)↑로 감소.**

| 점 | FK(Forward Kinematics) z | 실제 z | 오프셋 |
|---|---|---|---|
| 1 | 0.741 | 0.710 | +31 |
| 2 | 0.763 | 0.734 | +29 |
| 3 | 0.786 | 0.754 | +32 |
| 4 | 0.757 | 0.725 | +32 |
| 5 | 0.743 | 0.714 | +29 |

### 검증
- 카메라-측정 바닥 0.724 ≈ 실제 바닥 0.72 → **카메라 체인은 ~4mm 로 정상**, 31mm 는 **팔 체인** 오차. 카메라 외부캘리브는 ChArUco 보드(body_link0 기준) 독립 수행([[d435_camera_extrinsic]])이라 팔 URDF(Unified Robot Description Format) 수정과 무관(별도 TF(Transform) 브랜치).

### 수정 (방향)
- **31mm**: 솔버 URDF(Unified Robot Description Format) 팔 base z `1.275 → 1.244`(−31mm) 정합 — 카메라 TF(Transform) 영향 없음, RViz/충돌/다른 백엔드까지 자동 정합. 임시로는 명령 z = 실제목표 + 0.031. (순수 z 평행인지 자세별 방향 변화인지 1~2자세 추가 확인 권장.)
  - **✅ 적용·검증 완료(2026-06-06):** 좌·우 솔버 URDF(Unified Robot Description Format) 4곳 모두 `1.275→1.244` 수정(`openarmx_pick/urdf/openarmx_{left,right}_solver.urdf`, install→build→src 심링크라 즉시 반영). 검증: **보정값 없이** 명령 z=0.750 → **실제 측정 0.745**(예측 0.746 일치). 이제 명령 z = 실제 목표(수동 +31mm 불필요). 남은 ~5mm는 관절 추종 처짐(별개).
  - **미적용:** full 로봇 URDF(Unified Robot Description Format)/xacro SSOT(Single Source of Truth)는 아직 base z=1.275 → TF(Transform)/RViz/readout 패널은 여전히 +31mm 높게 표시. ptp 기능은 정확하나 시각화·TF(Transform) 일관성 위해 xacro arm_base z도 −31mm 후 재생성 필요. **ptp 액션 노드는 재시작해야 새 모델 사용**(직접이동 스크립트는 즉시 반영).
- **추종 처짐**: 중력보상 토크 피드포워드(Pinocchio `computeGeneralizedGravity` → τ_ff) 가 정석. 차선 KP(비례게인)↑.

### 게인 조치 (이번 세션)
- 런타임 `kp_joint1~4` **50→100→150→200**(양팔, 동적 param, 재빌드 불필요). 육안 떨림 없음@200. joint4 추종오차 좌팔 ~1.6→0.9°, 우팔 5.3→3.9°. **31mm 는 그대로**(게인 무관), 추종 처짐만 ~7→4mm 감소. 참고 [[motor-gain-tuning-plan]].

---

## 2026-06-06 20:05 (KST) — HIL(Hardware In the Loop) RViz 로봇 망가짐: CAN(Controller Area Network) 미기동 → ros2_control_node SIGABRT → /joint_states 미발행 → TF(Transform) 깨짐

### 증상
HIL(Hardware In the Loop) 구동 시 RViz(ROS Visualization)에서 로봇이 망가져 보임(링크가 길게 늘어지고 카메라/포인트클라우드가 엉뚱한 위치). SIL(Software In the Loop, `use_fake_hardware:=true`)에서는 동일 URDF(Unified Robot Description Format)로 정상 렌더됨. 즉 SIL 정상 / HIL 만 깨짐.

### 원인
TF(Transform)/URDF(Unified Robot Description Format) 버그가 아니라 **CAN(Controller Area Network) 미기동의 연쇄**. 객관 증거:
1. `can2`(right)/`can3`(left) 인터페이스가 DOWN(`ip -details link show`: `state STOPPED`, bitrate 미설정).
2. [openarmx_hardware.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/openarmx_hardware.launch.py)는 CAN(Controller Area Network)을 올리지 않고 인터페이스 이름만 파라미터로 전달 → 다운된 소켓을 그대로 사용.
3. `v10_simple_hardware` on_init 의 소켓 생성([v10_simple_hardware.cpp:227](../../openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp#L227) `OpenArmX(can_interface_, can_fd_)`)에서 `std::runtime_error` throw → 미포착 → `std::terminate` → SIGABRT(Signal Abort). `~/.ros/log/.../launch.log` 에 `terminate called after throwing an instance of 'std::runtime_error' ... Aborted` 확인, `/var/crash` 에 apport 크래시 덤프 존재.
4. `ros2_control_node` 가 크래시·respawn 루프(생존 ~3초) → `controller_manager` 서비스 불안정 → `joint_state_broadcaster` spawner 가 무한 대기(600초+).
5. → `/joint_states` 퍼블리셔 0개 → `robot_state_publisher` 가 관절각 미수신 → 동적 TF(Transform) 미발행 → RViz(ROS Visualization) 로봇 망가짐.

SIL(Software In the Loop)은 mock 하드웨어가 CAN(Controller Area Network) 없이 초기화되므로 항상 정상. → SIL 만 멀쩡했던 이유.

### 수정
HW(Hardware) launch 가 HIL(Hardware In the Loop) 시 CAN(Controller Area Network)을 **자동 기동**하도록 통합(단일 근원: 스크립트 1개).
- 신규 [up_follower_can.sh](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/scripts/up_follower_can.sh): `can2`/`can3` 만(leader can0/can1 불간섭) bitrate 1000000(1 Mbps, classic CAN 2.0)로 멱등 기동. 비대화형 sudo(`SUDO_PASSWORD` 기본 `"ff"`, 기존 [run_bimanual_moveit_with_can2.0.sh](../../openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/run_bimanual_moveit_with_can2.0.sh) 관행 동일). 이미 UP 이면 skip.
- [openarmx_hardware.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/openarmx_hardware.launch.py): `use_fake_hardware=="false"` 일 때 `ros2_control_node` 생성 **전에** `_ensure_can_up([right_can, left_can])` 호출(위 스크립트를 멱등 실행). SIL 은 호출 안 함.
- [CMakeLists.txt](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/CMakeLists.txt): 스크립트를 `install(PROGRAMS ...)` 에 추가.

검증(E2E): `can2`/`can3` 강제 down → HIL launch → 로그에 `[up_follower_can] canX: UP ✓`, `ros2_control_node` 크래시 없이 생존(이전 `terminate/Aborted` 소멸), 하드웨어 컴포넌트 양팔 `active`, `joint_state_broadcaster`+JTC(Joint Trajectory Controller)+gripper 5종 활성화, `/joint_states` 퍼블리셔 1개+실제 관절각(예: -0.0926 rad, 모터 통신 정상), `/tf` ~18Hz 발행, 전 로봇 프레임 존재. 캡처: `experiments/capture/200240_rviz_TF_recovered.png`.

### 재발 방지
- HIL(Hardware In the Loop) 진입점([openarmx_hardware.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/openarmx_hardware.launch.py))이 CAN(Controller Area Network)을 보장하므로 "launch 전에 수동 `ip link`" 누락으로 인한 재발 차단.
- 단, `can_fd:=false` 전제(classic CAN 2.0, bitrate 1000000). CAN-FD 사용 시 스크립트/비트레이트 조정 필요.
- `up_follower_can.sh` 가 모든 대상 UP 시 0, 부재/실패 시 1 반환 → launch 는 실패해도 계속 진행(로그 경고)하므로, 그래도 안 뜨면 모터 전원/배선/CAN 어댑터 점검(패시브 candump 무트래픽은 MIT 모드 특성상 단정 불가).
- 참고: 정본 `en_all_can.py`(모든 CAN UP)는 현재 `openarmx_arm_driver` import 시 `placo.so` 심볼 오류(numpy/pinocchio ABI(Application Binary Interface) 충돌 계열)로 실패 가능 → 의존성 없는 `ip link` 경로 채택.

---

## 2026-06-06 19:06 (KST) — ptp box_align: IK 가 관절한계 무시 후 끝에서 1회 clamp → TCP(Tool Center Point) 65mm 오차

### 증상
신규 `openarmx_ptp_box_align`(Pinocchio DLS(Damped Least Squares) IK(Inverse Kinematics) → 단일 JTC(Joint Trajectory Controller) 끝점) 실로봇 첫 모션에서, 오른팔이 검출 박스 `(0.346, -0.116, 0.782)` 위로 갈 때 `ik_converged: true`(residual 9e-5)인데도 `err_mm: 65.0` 로 TCP(Tool Center Point)가 6.5cm 빗나감.

### 원인
`solveIK()` 의 DLS(Damped Least Squares) 루프가 매 스텝 관절한계를 강제하지 않고 수렴까지 자유 적분 → joint2 가 한계를 0.112 rad 초과한 해에 수렴(무클램프 FK(Forward Kinematics) 오차 0.1mm). 그 뒤 **루프 종료 후 1회만 clamp** 하니, 실제 명령되는 clamp 된 q 의 FK(Forward Kinematics) 가 타깃에서 65mm 벗어남. residual 은 clamp 이전 q 기준이라 false-positive 로 converged 보고.

### 수정
[ptp_box_align_node.cpp](../../openarmx_ws/src/pick_and_place/ptp/openarmx_ptp_box_align/src/ptp_box_align_node.cpp) `solveIK()` — `integrate` 직후 **매 스텝 `clampToLimits`** 추가(한계-인식 DLS(Damped Least Squares)). 한계 내에서 정확해로 수렴하고, 수렴 판정도 clamp 된 q 기준이라 도달불가 타깃은 `ik_converged=false` 로 정직하게 표시.
- 검증(오프라인): 동일 타깃에서 매스텝 clamp err 0.1mm / 끝1회 clamp 65mm. 랜덤재시작 30 best 0.0mm(한계 내 정확해 존재).
- 검증(실로봇): 재빌드 후 동일 모션 → `err_mm 0.05`, residual 1e-4. 전 파이프라인(검출→/detected_boxes→ptp IK(Inverse Kinematics)→JTC(Joint Trajectory Controller)→이동→그리퍼) 서브밀리미터 동작.

### 재발 방지
- 한계-제약 없는 미분 IK(Inverse Kinematics)(DLS(Damped Least Squares)/CLIK)는 **매 반복 한계 clamp** 필수(끝에서만 clamp 금지). 수렴 판정은 clamp 된 q 기준으로 해 false-positive 방지.
- ptp 는 의도적으로 CBF(Control Barrier Function)/충돌제약이 없는 경량 백엔드라 한계 처리는 clamp 가 유일 안전장치다.

---

## 2026-06-06 09:34 (KST) — Diagnostics(Pipe/Node Health) 표 컬럼 폭이 매 갱신마다 흔들림

### 증상
Scenario UI(User Interface) 의 Pipe Health / Node Health 탭 표가 1.5 초 주기 갱신마다 컬럼 폭이 미세하게 바뀌며 레이아웃이 흔들렸다. Value 칸이 살아있는 값(`68.0 Hz`)·`no pub`·`latched`·`idle`·`no data` 등으로 바뀔 때마다 폭이 달라져 그 오른쪽 Detail 경계까지 움직였다.

### 원인
[diagnostics_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/diagnostics_tab.py) 의 표 헤더에서 Category·Link·Value 3개 컬럼이 `QHeaderView.ResizeToContents` 였다. `_refresh()` 가 매 틱마다 Value 칸 텍스트를 갱신하는데, 그 픽셀 폭이 상태별로 제각각이라 ResizeToContents 가 매번 컬럼을 재계산 → 표 전체 레이아웃이 일정하지 않았다.

### 수정
컬럼 0~3 을 `QHeaderView.Fixed` 로 변경(Detail 컬럼 4만 `Stretch` 유지로 패널 채움). Category·Link 폭은 `QFontMetrics` 로 정적 spec 텍스트에서 빌드 시 1회 산출(탭별 최장 텍스트에 맞춤), Value 폭은 최장 표시값(`100.6 Hz`, `no data` 등)에 맞춘 고정값. 라이브 값과 무관하게 폭이 고정된다.

- 변경 파일: `diagnostics_tab.py` (import 1줄 + `_build_ui` 컬럼 사이징 블록). 코드 외 동작 변화 없음.
- 검증: 오프스크린 렌더로 동일 표를 라이브 상태(`68.0 Hz` 등) vs 전부 `no pub` 상태로 갱신 후 컬럼 폭 비교 → `[36, 118, 451, 100]` 으로 **양쪽 동일**(assert 통과). 스크린샷 `experiments/capture/20260606_091403_pipe_health_fixed.png` 에서 최장 Link(`/openarmx_left_movel_controller/controller_error`) 전체 표시·Value 고정 확인.

### 재발 방지
- 주기 갱신되는 셀이 든 표에서 그 컬럼에 `ResizeToContents` 를 쓰면 폭이 흔들린다 — 갱신 셀이 있는 컬럼은 `Fixed`(+필요 시 `QFontMetrics` 로 정적 폭 산출), 한 컬럼만 `Stretch` 로 채우는 패턴을 기본으로 한다.

---

## 2026-06-05 17:15 (KST) — a2-scenario(scenario_ui) pinocchio import segfault: numpy 1.x/2.x ABI 충돌

### 증상
`a2-scenario` 실행 시 scenario_ui 가 시작 중 죽음 (exit code -11, SIGSEGV). 트레이스:
`scenario_ui.py → main_window → cartesian_control_tab → ik_check.py:19 import pinocchio → /opt/ros/humble/lib/python3.10/site-packages/pinocchio/__init__.py:19 from .pinocchio_pywrap_default import * → AttributeError: _ARRAY_API not found` 후 segfault. 갑자기 발생.

### 원인
numpy 1.x/2.x ABI(Application Binary Interface) 충돌. apt `ros-humble-pinocchio` 3.9.0 은 numpy 1.x 로 빌드됐는데, `~/.local` 의 numpy 가 **2026-06-05 14:19 에 2.2.6 으로 (재)설치**되어 apt numpy 1.21.5 를 가림(에러 17:12 의 직접 원인). `/opt/ros` 가 `sys.path` 최상단이라 ROS source 여부와 무관하게 항상 numpy-1.x 빌드 pinocchio 가 먼저 import → segfault. **try/except 로 못 잡음**(C 레벨 SIGSEGV, 실측 exit 139). 실측: numpy<2 를 앞세우면 apt pinocchio import·`SE3` 변환·`buildModelFromUrdf` 정상.

- 추가 함정: 2026-06-02 설치된 pip cmeel 스택(pin 3.8.0 / eigenpy 3.12 / placo / cmeel-boost)은 `numpy>=2.2` 요구 → apt pinocchio(numpy<2)와 **단일 전역 numpy 로 동시 만족 불가**. 단 그 스택은 `undefined symbol: EIGENPY_ARRAY_API...` 로 단독 import 불가 + `/opt/ros` 에 가려 미사용 상태라 무시 가능.

### 수정
`python3 -m pip install --user "numpy<2"` → numpy 1.26.4 설치. **코드 변경 없음(환경만)**. launcher 환경(ROS humble + openarmx_ws install source)에서 import 체인 전체 통과 검증: pinocchio → ik_check → cartesian_control_tab → main_window (exit 0).

- 부수효과: numpy<2 라 `cv_bridge` numpy2 깨짐(imgmsg_to_cv2 bgr8)도 네이티브 해소될 가능성 — 코드는 미변경.
- 커밋 없음(사용자 지시 "기록만").

### 재발 방지

- `pip install --user` 가 numpy>=2 를 `~/.local` 에 다시 끌어오면 재발(이번이 그 경우). a2-scenario pinocchio segfault 시 **`python3 -c "import numpy; print(numpy.__version__)"` 부터 확인**, 2.x 면 `pip install --user "numpy<2"`.
- 정공법(우회 아님): apt ROS 스택은 numpy 1.x 전제이므로 전역 numpy 를 1.x 로 유지. numpy 2.x pinocchio 가 필요하면 별도 venv 로 격리(전역 혼용 금지).
- auto-memory 노트 동기화: `numpy-pinocchio-abi-crash`.

---

## 2026-06-05 10:25 (KST) — pick_and_place 목표 도달 시 gripper open + 고정 sleep → 완료/피드백 기반 대기

### 배경 / 동기
- 사용자 요청: `openarmx_ws/src/pick_and_place` 의 box_align(cyclo·pilz) 백엔드가 **목표 지점 도달 시 gripper 를 open** 하도록.
- 후속 논의로 두 백엔드가 이동 후 **고정 `time.sleep(예상시간)`** 으로 대기하는 구조의 비효율을 규명: 모션을 *토픽 발행(fire-and-forget)* 으로 명령해 **컨트롤러로부터 완료 신호가 돌아오지 않기 때문**. 도달 검증(`err_mm`)·gripper open 시점을 맞추려 운동 예상시간을 시간으로 때움.
  - cyclo MoveL 컨트롤러: 종료 시 내부 플래그만 끄고 완료 토픽/액션 없음 ([omx_movel_controller_node.cpp](../../openarmx_ws/src/cyclo_robot_controller/cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp) L326).
  - pilz: JTC(Joint Trajectory Controller) 의 `follow_joint_trajectory` 액션(완료 result 제공)이 아니라 `joint_trajectory` *토픽* 으로 발행 → 완료 신호 폐기.

### 수정
- **gripper open (cyclo·pilz 공통)**: 도달 직후 배정 팔의 GripperActionController(`control_msgs/GripperCommand`, `/{side}_gripper_controller/gripper_cmd`) 로 open. finger joint 범위 0.0(닫힘)~0.044m(완전 열림), 파라미터 `gripper_open_pos`(0.044)·`gripper_effort`(14.0), 결과는 `assignments_json` 에 기록. `control_msgs` 의존성 추가(두 package.xml).
- **pilz — 완료 신호 기반 대기** ([pilz_box_align_node.py](../../openarmx_ws/src/pick_and_place/pilz/openarmx_pilz_box_align/openarmx_pilz_box_align/pilz_box_align_node.py)): trajectory 를 `follow_joint_trajectory` **액션**으로 전송(`_send_traj`)하고 result 까지 대기 → 고정 sleep 제거. 액션 부재 시 `joint_trajectory` 토픽 + 시간 보정으로 안전 폴백.
- **cyclo — ee_pose 피드백 기반 도달 대기** ([box_align_node.py](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_cyclo_box_align/openarmx_cyclo_box_align/box_align_node.py)): 컨트롤러가 발행하는 `/openarmx/{side}/ee_pose`(엔드이펙터(end-effector) 포즈, base frame) 를 구독해 명령 `tcp_target` 과의 거리가 `arrive_tol_m`(기본 5mm) 안에 들면 도달로 판정(`_wait_until_arrived`) → 고정 sleep 제거. 미수신/미수렴 시 `move_time+1.0` 까지 대기 = 기존 동작 폴백. 파라미터 `arrive_tol_m`·`ee_pose_topic_template`.
- 명령 경로(cyclo MoveL 토픽 발행)는 불변 — "대기"만 피드백 기반으로 교체해 검증된 0mm 동작 보존.

### 검증
- `py_compile` 양호, flake8(max-line 99) 새 코드 위반 0. (cyclo 176–177 E501 2건은 기존 `move_arm` 좌표 대입 라인, 본 변경 아님.)
- **미완**: `colcon build` + 실기/시뮬 `AlignToBoxes` 실동작 검증(ee_pose 실수신·수렴, pilz 액션 result 도달) 대기.

### 재발 방지
- 모션을 *토픽*(fire-and-forget)으로 명령하면 완료 신호가 없어 고정 sleep 에 의존하게 됨. 완료 result 를 주는 **액션**(JTC(Joint Trajectory Controller) `follow_joint_trajectory`) 또는 컨트롤러 피드백(`ee_pose`)·TF(Transform) 수렴 판정으로 대기를 대체할 것.
- cyclo MoveL 컨트롤러에 완료 신호(액션/done 토픽)가 없는 한, 백엔드는 피드백 수렴으로 도달을 판정한다.

---

## 2026-06-05 07:20 (KST) — 시나리오 GUI 느림 + Teaching Capture 안 됨 종합 분석(py-spy) — 두 원인 규명

### 증상
- GUI 전반이 매우 느림(클릭/렌더 끊김). Teaching 탭 "Capture" 로 현재 포즈가 등록 안 됨("현재 포인트 등록이 안됩니다"). /joint_states·EE(End Effector) TF(Transform)(양팔 link0→link7)는 라이브 정상 — Capture 의 조기반환(조인트 미수신/TF(Transform) lookup 실패)은 원인 아님.

### 분석 (py-spy 정량, 10s/3998 samples)
`scenario_ui` 한 코어 ~87% 지속 점유. 느림은 **독립적인 두 원인**:

**원인 A — GUI(메인) 스레드가 블로킹 subprocess 로 주기적으로 얼어붙음** (2초 QTimer 2곳):
- [launch_manager_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/launch_manager_tab.py) `_refresh`: `ros2 node list`(timeout 4s, **샘플 1101개=최다**) + 프리셋 8개마다 `pgrep -f`.
- [pick_and_place_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py) `_refresh_status`: `pgrep -f` ×3 (`_proc_running_external`).
- 메인 스레드를 8회 샘플 시 4/8 이 `subprocess.communicate` 에서 frozen. GUI 가 매 2초 큰 비율 멈춤 → 클릭/모달 입력(Capture "Pose 이름") 유실.

**원인 B — executor(단일 rclpy 스레드)가 고빈도 토픽을 파이썬에서 역직렬화하며 한 코어 점유 → GIL 경합으로 GUI 굶음**:
- 프로파일 핫: `_take_subscription`(368, 역직렬화), `_wait_for_ready_callbacks`/`add_to_wait_set`(매 spin_once 마다 wait set 재구성, 구독 수에 비례), `control_msgs InterfaceValue` 디코드(`/dynamic_joint_states`).
- `_setup_diagnostics()` 가 `__init__` 에서 **항상** 14개 토픽 구독(Pipe Health 탭 안 봐도). 실측: **/joint_states 100 Hz**(피드백+diagnostics 로 **중복 구독** → ~200 cb/s), /dynamic_joint_states(역직렬화 비쌈, 레이트 표시 용도뿐), /tf 17 Hz, ee_pose ×2. (D435 미스트리밍이라 이미지 구독은 현재 병목 아님.)

### 수정
- **원인 A (완료, subprocess 를 GUI 스레드에서 제거)**: `_query_nodes` → 브리지 인프로세스 `live_node_names()`(rclpy 그래프 API). `_proc_running`/`_proc_running_external` → `/proc` 1회 스캔(`_running_cmdlines`, ~10ms) 재사용·부분문자열 매칭(`pgrep -f re.escape` 동치). [main_window.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py) 에서 `LaunchManagerTab(self._bridge,…)` 주입. 잔여 `subprocess.run` 은 Stop/kill 일회성(타이머 아님). symlink-install — **UI 재기동 시 적용**.
- **원인 B (완료, executor 부하 축소)**: [scenario_action_client.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/scenario_action_client.py) + [diagnostics_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/diagnostics_tab.py).
  - **diagnostics 구독 lazy 화**: `_setup_diagnostics` 가 더 이상 시작 시 14개 토픽을 구독하지 않음. `diag_start`/`diag_stop`(GUI 스레드는 의도 플래그만, 실제 create/destroy 는 spin 스레드 `_apply_diag_pending` 에서 — rclpy 엔티티를 spin_once 와 동시 변경 금지). Pipe Health 탭 `showEvent`/`hideEvent` 에서만 구독·해제 + 새로고침 타이머 start/stop(숨김 탭은 폴링도 안 함).
  - **/joint_states 중복 구독 제거**: diagnostics 가 /joint_states 를 두 번째로 구독하던 것 제거 → 기존 100Hz 피드백 콜백 `_on_joint_state` 에서 카운터만 증가(`_DIAG_PIGGYBACK`). 100Hz×2 역직렬화 → ×1.
  - 효과(예상): Pipe Health 안 볼 때 /dynamic_joint_states(무거운 InterfaceValue 디코드)·/tf(중복)·ee_pose×2 역직렬화 소멸 → executor ~87% CPU·GIL 경합 대폭 완화. **라이브 CPU 검증은 UI 재기동 후**(미실시).

### 적용 상태 / 재기동 토폴로지 (중요)
- 모든 수정은 **`scenario_ui` 프로세스 안에만** 존재(launch_manager_tab/pick_and_place_tab/diagnostics_tab/scenario_action_client/main_window). 다른 노드는 이 모듈 미사용 → **UI 프로세스만 재기동하면 100% 적용**, 제어 스택 무영향. (5개 파일 py_compile OK, symlink-install — 빌드 불필요.)
- **현 라이브 세션은 수동 조립 6개 launch**: ① UI(a2-scenario) ② RViz+EE(End Effector) 리더마커(`openarmx_scenario_workflow.launch.py`) ③ fake-HW+controller_manager+RSP(`openarmx_hardware.launch.py use_fake_hardware:=true`) ④ move_group(`move_group.launch.py`) ⑤ 컨트롤러 스폰(JSB·JTC×2·그리퍼×2) ⑥ cyclo MoveL×2(`omx_movel_controller_node`). **복합 bring-up 스크립트 없음**(`openarmx_ws/scripts/` 에 `kill_all_ros2.sh` 뿐) → `kill_all` 전체종료 시 ③~⑥ 수동 재조립 필요(순서 의존, 리스크).
- **재기동 미실시(사용자가 "기록 후 종료")**. 다음 세션에서 적용: UI-only 재기동 권장(제어스택 유지) 또는 규칙대로 kill_all 후 6종 순서 재기동.

### 재발 방지
- PyQt GUI 스레드(슬롯/QTimer 콜백)에서 블로킹 `subprocess.run` 금지(`ros2 node list`·`pgrep` 0.5~수초). ROS 그래프=인프로세스 rclpy API, 프로세스 조회=`/proc` 스캔, 불가피하면 백그라운드 QThread+시그널.
- 항상-구독은 꼭 필요한 토픽만. 고빈도(≥50Hz)·무거운 메시지(DynamicJointState/Image)는 보이는 탭에서만 구독하거나 별 executor/콜백그룹 분리 검토.

---

## 2026-06-05 06:57 (KST) — 시나리오 GUI HW bringup follower(can2/can3) 전환 + 박스 TF 잔재 제거 + box_align 문서 정리

### 증상 / 배경
- 시나리오 GUI의 HW bringup이 `can0/can1`(=leader, 텔레옵 입력)을 **고정 사용** → pick 시 leader 팔이 제어됨. 실제 작업팔(follower)은 `can2/can3`. (사용자가 실기에서 leader가 잡히는 것을 확인.)
- ③(인지/모션 분리)로 box TF 는 `box_perception_node` 가 발행하고 모션 백엔드는 더 이상 box TF 를 안 쓰는데, RViz "Box TFs" TF 디스플레이 + box_align README/package.xml 의 자체검출(DetectBox)·box TF 설명이 잔존.

### 수정
- **HW bringup CAN → follower can2/can3**: [launch_manager_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/launch_manager_tab.py) L0 "HW 하드웨어", [main_window.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py) `HW_LAUNCH_CMD` 둘 다 right=can2 / left=can3. (can0/can1 = leader.)
- **RViz "Box TFs" TF 디스플레이 삭제** ([openarmx_scenario.rviz](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/config/openarmx_scenario.rviz)). 박스는 MarkerArray("Detected Boxes")로만 표시. 좌표변환 프레임워크(`Transformation: TF`)는 필수라 유지.
- **box_align 문서 정리**(pilz/cyclo README + package.xml): 자체검출(DetectBox·N프레임·클러스터·box TF) 설명 → `/detected_boxes` 소비 방식으로. ③로 미사용이 된 의존성(cv_bridge/sensor_msgs/yolov8_detection_msgs) 제거.

### 재발 방지
- 시나리오 GUI HW bringup 은 follower(can2/can3) 기본. leader(can0/can1)는 텔레옵 입력용. (can 인터페이스: en_all_can.py / `ip link ... bitrate 1000000` 으로 별도 UP, sudo 필요.)
- 박스 시각화는 `/detected_boxes_markers`(MarkerArray). box `box_<i>` TF 방식은 폐기 — box TF/검출/3D 는 인지(`box_perception_node`) 단일 책임.

---

## 2026-06-05 01:50 (KST) — ③ 모션 백엔드(pilz·cyclo)를 /detected_boxes 소비로 전환 (인지·모션 완전 분리)

### 증상 / 배경
③ 전: pilz/cyclo box_align 이 *자체 검출*(DetectBox·depth·3D·클러스터)을 수행해 인지(`/detected_boxes`, 마커는 정상)와 별개로 동작.
- pilz: `detect_boxes` 가 0개 → "no boxes detected" abort.
- cyclo: depth 를 RELIABLE 로 구독 → `d435_qos.yaml` 의 BEST_EFFORT depth 와 비호환(데이터 0).
- 백엔드 라디오를 바꿔도 정렬 백엔드가 재기동되지 않아(선택 cyclo인데 pilz 프로세스가 떠 있음) `/openarmx/align_to_boxes` 서버 부재 → "Waiting for an action server...".

### 수정
- [pilz_box_align_node.py](../../openarmx_ws/src/pick_and_place/pilz/openarmx_pilz_box_align/openarmx_pilz_box_align/pilz_box_align_node.py) / [cyclo box_align_node.py](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_cyclo_box_align/openarmx_cyclo_box_align/box_align_node.py): 자체 검출/depth/3D/박스TF 전부 제거, **`/detected_boxes`(PoseArray, base) 구독**으로 교체. 최신값(max_box_age 10s)으로 assign→move, 없으면 abort("먼저 '검출요청'").
- 두 launch([pilz](../../openarmx_ws/src/pick_and_place/pilz/openarmx_pilz_box_align/launch/pilz_box_align.launch.py)/[cyclo](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_cyclo_box_align/launch/box_align.launch.py))에서 검출 파라미터(n_frames/cluster_radius/min_hits/default_confidence) 제거.
- [pick_and_place_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py) `_on_backend_changed`: 백엔드 변경 시 떠있는 정렬 백엔드 자동 종료(불일치·"Waiting" 방지).
- 효과: **depth QoS 충돌 소멸**(depth 소비자는 box_perception_node 하나), pilz "no boxes" 우회, 백엔드 전환 정합. 빌드 exit 0.

### 흐름 / 재발 방지
검출요청(인지 `/detected_boxes`+마커) → 모션 컨트롤러(cyclo MoveL / MoveIt Pilz move_group) 기동 → 정렬 백엔드 Start(선택) → Run AlignToBoxes(좌표 소비·이동). 모션은 검출하지 않음 — 검출/3D/QoS 는 인지(box_perception_node) 단일 책임. 실로봇 AlignToBoxes 이동 라이브 검증 대기.

---

## 2026-06-05 01:25 (KST) — 인지/모션 분리: box_perception_node 신설(검출→3D 좌표+마커), 모션 백엔드 box TF 발행 제거

### 증상 / 배경
- "검출(원격 seg)"만으로는 RViz 에 박스 위치가 안 떴음. box TF 는 모션 백엔드(pilz/cyclo box_align)가 AlignToBoxes(모션) **액션 안에서만** 발행 → 모션을 돌려야만 박스가 보이는 구조(인지·모션 결합).
- 사용자 요구: **인지(검출+위치)와 모션을 명확히 분리**. 박스 3D 위치는 검출 시점에 YOLOv8 결과와 함께 나와야 하고(정석), 모션은 그 뒤. 또한 TF 프레임보다 **3D 좌표 토픽 + 마커**가 검출 객체 표현의 표준.

### 수정
- **신규 인지 노드** [box_perception_node.py](../../3d_detect_ws/src/yolov8_detection/yolov8_detection/box_perception_node.py) — `/yolov8_node/detections`(2D) + depth + camera_info + TF(camera→base, 내부 계산용) → 3D box 좌표 → `/detected_boxes`(PoseArray, base) + `/detected_boxes_markers`(MarkerArray, 클래스 색/텍스트 라벨). 워크스페이스 필터로 상단 슬리버 제거. **box TF 는 발행하지 않음**(좌표 토픽+마커가 표준, 모션은 PoseArray 만 구독하면 TF lookup 없이 이동 가능).
- [yolo_remote.launch.py](../../3d_detect_ws/src/yolov8_detection/launch/yolo_remote.launch.py)에 통합 → "원격검출 브리지(Pi) Start" 시 동반 기동. setup.py 엔트리 추가.
- [pilz_box_align_node.py](../../openarmx_ws/src/pick_and_place/pilz/openarmx_pilz_box_align/openarmx_pilz_box_align/pilz_box_align_node.py): `publish_box_tfs` 및 관련 import/메서드 제거(인지로 이관, 이중발행 해소).
- [openarmx_scenario.rviz](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/config/openarmx_scenario.rviz): MarkerArray "Detected Boxes"(`/detected_boxes_markers`) 디스플레이 추가.
- PnP 탭([pick_and_place_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py)): 검출 버튼 라벨 "원격검출 브리지(Pi)" 로 명확화, "검출요청(1회)" 버튼(모션 없이 검출 검증), "로그 지우기" 버튼, confidence 기본 0.5.
- 빌드 exit 0. **라이브 검증**: 검출요청 → RViz 에 클래스별 색 박스 마커 + 라벨이 3D 위치에 표시 확인.

### 재발 방지 / 후속
- 인지(검출+box 좌표/마커)와 모션은 분리 유지 — box TF/마커 발행은 인지(box_perception_node) 책임.
- 후속(③): 모션 백엔드가 `/detected_boxes`(PoseArray)를 consume 하여 타깃 결정(자체 검출 제거) = 완전 분리. cyclo box_align 도 box TF 제거 동일 적용 필요.

---

## 2026-06-05 00:33 (KST) — a2-scenario 종료 시 카메라/TF 브리지 잔존: pnp shutdown + SIGINT graceful 정리

### 증상
a2-scenario 종료(창 X 닫기 또는 터미널 Ctrl+C) 후 `/camera/camera`, `/d435_center_to_camera_link_tf`(및 검출/정렬 노드)가 종료되지 않고 잔존.

### 원인
1. [pick_and_place_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py) `shutdown()`이 카메라/검출/백엔드 ManagedProcess 를 **의도적으로 종료하지 않음**(UI 재시작 간 prereq 유지 설계). closeEvent 가 돌아도 카메라가 살아남음.
2. [scenario_ui.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/scripts/scenario_ui.py) 의 `signal.SIGINT = SIG_DFL` → 터미널 Ctrl+C 시 즉시 종료되어 `closeEvent`(정리) 자체가 미실행. ManagedProcess 는 `start_new_session`(detached)이라 부모 사망에도 생존.

### 수정
1. `pick_and_place_tab.py` `shutdown()` 에서 `_cam_proc/_det_proc/_backend_proc.stop()`(killpg) 호출 추가 → 창 닫기 시 정리.
2. `scenario_ui.py` `SIGINT` 를 graceful 핸들러(`win.close()` + `app.quit()`)로 교체 + Qt 이벤트루프가 시그널 처리하도록 `QTimer(200ms)` 추가 → 터미널 Ctrl+C 도 `closeEvent` 경유 정리.
- py_compile OK, 빌드 exit 0. 창 닫기/Ctrl+C 양쪽 모두 카메라+브리지+검출/정렬 종료.

### 재발 방지
- UI 가 띄운 prereq(카메라/검출/백엔드)는 UI 종료 시 함께 종료가 기본. 기존 잔존 노드는 [[kill_all_ros2_canonical]] `kill_all_ros2.sh` 로 정리 후 재기동.

---

## 2026-06-05 00:30 (KST) — RViz fps 30→10 저하 (포인트클라우드 발행 시): 5-에이전트 진단, PointCloud2 렌더 부하 확정

### 증상
카메라 포인트클라우드 발행 또는 RViz 'RGBD Cloud'(PointCloud2) 디스플레이 체크 시 RViz fps 가 30→10 으로 저하.

### 원인 (5-에이전트 병렬 진단, 가설 2개 측정 기각)
- **주원인**: PointCloud2 'RGBD Cloud' 디스플레이의 대용량 점(~146k/frame, 2.93MB/msg, ~27.6MB/s, `Style=Flat Squares`=점당 빌보드 쿼드 ~6배 정점부하)이 RViz **단일 OGRE 렌더 스레드 포화**. 측정: `top -H` rviz2 메인 스레드만 60~80%, 나머지 40스레드 ≤6.7%.
- **증폭기**: 시스템 CPU 포화 — load avg ~11(8코어), run-queue 17, realsense 96% + scenario_ui 85% + anydesk/rustdesk 동시.
- **기각**: 소프트웨어 렌더(iris_dri HW가속 확인·swrast 0건), TF 부하(카메라 TF 전량 static/latched, RViz TF 스레드 6.7%).
- 사용자 차분 테스트('RGBD Cloud' 체크→10fps 재현)로 주원인 확정. 이후 **10→15fps 부분 개선** 관측(원인 미확정 — QoS BEST_EFFORT 적용/부하 프로세스 감소 추정).

### 수정
권장안 제시, **사용자 승인 대기로 미적용**. 권장(RViz 표시 전용 · 검출 파이프라인 무영향):
`openarmx_scenario_player/config/openarmx_scenario.rviz` — line249 `Style` Flat Squares→Points, line246 `Selectable`→false, line247 `Size(Pixels)` 3→2. (`decimation_filter` 는 검출 depth 에도 영향이라 제외.)

### 재발 방지
- 약한 iGPU + 단일 PC 에 realsense+RViz+UI+원격데스크톱 집중 회피. 대용량 클라우드는 Points 스타일/다운샘플/RViz 분리.
- 1분 차분 테스트(디스플레이 체크 토글)로 렌더 부하 여부 즉시 판별.

---

## 2026-06-05 00:25 (KST) — D435 센서 QoS RELIABLE→BEST_EFFORT(SENSOR_DATA) 정렬

### 증상
진단 중 "QoS 불일치"로 보고됨. 그러나 라이브 확인 결과 **연결을 끊는 불일치는 없음** — 퍼블리셔(클라우드)가 RELIABLE 이고 RELIABLE 은 모든 구독과 호환(클라우드 정상 렌더가 증거). 보고된 "RELIABLE 구독 0건"은 **CPU 포화로 인한 진단 CLI starvation 의 오인**(다른 에이전트의 exit 124/143 타임아웃과 동일 정황).

### 원인
realsense2_camera QoS 기본값 `DEFAULT` 가 RELIABLE/KEEP_LAST 로 매핑됨. 고속 센서 스트림(27.6MB/s 클라우드)엔 BEST_EFFORT(sensor_data)가 ROS2 관례 — RELIABLE 은 불필요한 재전송/버퍼 오버헤드.

### 수정
센서 스트림을 SENSOR_DATA(BEST_EFFORT)로 정렬:
1. 새 파일 [d435_qos.yaml](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/config/d435_qos.yaml) — `pointcloud.pointcloud_qos / depth_qos / color_qos = SENSOR_DATA`.
2. [d435_camera.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/d435_camera.launch.py) 가 rs_launch 의 `config_file` 메커니즘(`parameters=[params, params_from_file]`, rs_launch.py:159)으로 주입. rs_launch 가 qos 를 launch 인자로 미노출(`--show-args` 0건)하므로 config_file 경유.
- `SENSOR_DATA` 는 realsense 라이브러리 유효 enum(strings 확인) + 런타임 set "successful". 빌드 exit 0. **적용은 카메라 재기동 후** (`ros2 topic info /camera/camera/depth/color/points -v` → `Reliability: BEST_EFFORT` 확인).

### 재발 방지
- 센서 스트림 QoS 는 BEST_EFFORT 기본. QoS '불일치' 의심 시 `ros2 topic info -v` 로 pub 의 offered QoS 를 먼저 확인 — CPU starvation 으로 인한 CLI 무수신을 QoS 불일치로 오인하지 말 것.

---

## 2026-06-05 00:05 (KST) — 카메라 Start 해도 RViz에 3D 포인트클라우드/박스 미표시: d435_center_link→camera_link TF 브리지 누락

### 증상
- Pick and Place 탭에서 "카메라 Start"(D435) 해도 RViz에 **3D 포인트클라우드가 안 보임**. 박스 검출 결과(box TF)도 안 보임.
- depth/color/camera_info 토픽·포인트클라우드 토픽은 정상 발행(10Hz, 640×480), RViz config엔 PointCloud2 디스플레이도 존재.

### 원인
카메라 TF 체인 단절. 라이브 tf2_echo 세그먼트 검사:
- `openarmx_body_link0 → d435_center_link` (정적 extrinsic, RSP) : **OK**
- `d435_center_link → camera_link` (브리지) : **끊김** ← 근본 원인
→ 카메라 서브트리가 base에서 끊겨 `/camera/camera` 클라우드/박스 TF가 RViz fixed frame(`openarmx_body_link0`)으로 해결되지 않음.
이 브리지(static_transform_publisher `d435_center_to_camera_link_tf`)는 [scenario_player_with_ee_leader.launch.py:112-120](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/scenario_player_with_ee_leader.launch.py#L112-L120)에만 있었는데, a2-scenario는 [openarmx_scenario_workflow.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/openarmx_scenario_workflow.launch.py)(RViz + ee_leader 마커만)를 띄워 브리지가 안 떴고, 카메라 Start의 D435_CMD도 realsense만 띄우고 브리지를 포함하지 않았음.

### 수정
사용자 지시: "카메라 구동 시 실행되게, 기존 launch 재사용 말고 새로 만들어 사용".
1. **새 launch** [d435_camera.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/d435_camera.launch.py) 생성 — realsense2_camera(컬러+깊이+depth→color 정렬+포인트클라우드) + `d435_center_link→camera_link` 정적 TF 브리지를 한 LaunchDescription으로.
2. [pick_and_place_tab.py:62-67](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py#L62-L67) `D435_CMD`를 `ros2 launch openarmx_scenario_player d435_camera.launch.py`로 교체. ManagedProcess가 `start_new_session`+`killpg`라 카메라 Stop 시 realsense·브리지가 한 프로세스그룹으로 함께 종료.
3. 빌드(openarmx_scenario_player ament_cmake — 새 launch 설치 위해 필수 + openarmx_scenario_ui) exit 0, 설치·`--show-args` 파싱 검증 완료.
- 체인 완성: `body_link0 →extrinsic→ d435_center_link →브리지→ camera_link →realsense→ *_optical_frame`.

### 재발 방지
- 카메라를 띄우는 경로는 항상 브리지를 포함한 전용 launch(`d435_camera.launch.py`) 사용. RViz fixed frame=`openarmx_body_link0` 기준 외부 `/camera/camera` 클라우드는 `d435_center_link→camera_link` 엣지가 전제.
- 박스(box_0…) TF는 정렬 백엔드(box_align 노드)가 AlignToBoxes 액션 실행 중에만 브로드캐스트하는 on-demand 방식 → 박스를 RViz에서 보려면 정렬 백엔드 Start + Run AlignToBoxes 필요(클라우드 자체는 카메라 Start만으로 표시).

---

## 2026-06-04 23:45 (KST) — a2-scenario 시작 동작 정리: RViz 2중 spawn + Hardware/Player auto_start 자동실행 + tm_task_manager 유령 노드

### 증상
- `a2-scenario` (bashrc alias) 실행 시 **RViz 창이 2개** 뜸 — 제목줄 `openarmx_scenario.rviz` 하나 + `openarmx_description/.../bimanual.rviz` 하나.
- 사용자가 누르지 않았는데 **Hardware Bringup / Scenario Player 가 자동으로 Running** (UI 상단 상태 패널).
- `ros2 node list` 에 OpenArmX 와 무관한 `/tm_task_manager_node` 가 보임.

### 원인
1. **RViz 2중**: scenario_player workflow launch 가 `openarmx_scenario.rviz` 를 띄우고(의도된 단일 소유), HW bringup `openarmx.bimanual.launch.py` 의 `start_rviz` 기본값이 `true` 라 `bimanual.rviz` RViz 를 **추가로** 띄움. `HW_LAUNCH_CMD` 가 `start_rviz:=false` 를 전달하지 않음. 근거: [main_window.py:29-37](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py#L29-L37), [openarmx.bimanual.launch.py:267-273](../../openarmx_ws/src/openarmx_ros2/openarmx_bringup/launch/openarmx.bimanual.launch.py#L267-L273).
2. **자동 실행**: `scenario_ui.launch.py` 가 노드를 인자 없이 실행 → `auto_start = not no_auto = True` → main_window 의 auto-start 시퀀스가 HW(+0.5s) / Player(+5s) / Refresh(+10s) 를 자동 Start. 근거: [scenario_ui.py:40](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/scripts/scenario_ui.py#L40), [main_window.py:177-179](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py#L177-L179).
3. **유령 노드**: `/tm_task_manager_node` = Techman(TM) 협동로봇 드라이버 노드 (`tm_msgs`/`tm_driver`/`connect_tmsvr`/`techman_image`). 이 PC 에 패키지 미설치·프로세스 없음. `ROS_LOCALHOST_ONLY=0` + `ROS_DOMAIN_ID` 미설정(기본 0) 으로 동일 LAN(192.168.1.x) 의 다른 PC 노드가 한때 디스커버리됨 → ros2 CLI 데몬의 stale 그래프 캐시. 신선한 `--no-daemon` 스캔엔 없음, `ros2 daemon stop` 후 사라짐 → **a2-scenario 와 무관**.

### 수정
1. [main_window.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py) `HW_LAUNCH_CMD` 에 `start_rviz:=false` 추가 → RViz 는 scenario_player workflow 의 `openarmx_scenario.rviz` 1개만.
2. [scenario_ui.launch.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/launch/scenario_ui.launch.py) Node 에 `arguments=["--no-auto"]` 추가 → `a2-scenario` 는 UI + RViz 만 뜨고 Hardware Bringup / Scenario Player 는 **수동 Start** (사용자 지시 2026-06-04 "auto 실행 금지").
3. (유령 노드) 코드 변경 없음 — `ros2 daemon stop` 으로 stale 캐시 정리.
- `--symlink-install` 환경이라 두 수정 모두 **colcon 빌드 불필요·즉시 반영** (설치본 main_window.py / scenario_ui.launch.py 가 src 심볼릭 링크임을 확인).

### 재발 방지
- HW bringup 을 외부 launch 와 함께 띄울 땐 항상 `start_rviz:=false`. RViz 단일 소유 = scenario_player workflow (해당 launch arg description 에 이미 명시).
- a2-scenario 자동 실행 정책: **금지** (UI + RViz 만). 자동 시작이 다시 필요해지면 `scenario_ui.launch.py` 의 `--no-auto` 를 제거.
- 같은 네트워크에 타 로봇(Techman 등) 존재 시 `ROS_DOMAIN_ID` 격리 권장. 재기동은 `kill_all_ros2.sh` (FastDDS shm + 데몬 정리 포함) 사용.

---

## 2026-06-04 — Camera 탭(D435+Pi YOLOv8 오버레이) 신설 + cv2/PyQt5 xcb 크래시 + Teaching 2테이블 컬럼 정렬 + 클라우드 TF 브리지

### 증상
- Camera 탭 추가 직후 GUI 가 startup 에서 즉시 코어덤프: `Could not load the Qt platform plugin "xcb" in ".../cv2/qt/plugins"`.
- Teaching 탭 상/하 두 QTableWidget 의 컬럼 폭이 서로 다름(name 컬럼이 한쪽은 좁고 한쪽은 넓음).
- RViz 에서 D435 포인트클라우드 가 안 보임(데이터는 발행 중). box 검출 TF(box_<i>) 도 안 보임.

### 원인
1. **cv2/PyQt5 충돌**: pip `opencv-python` 이 자체 Qt xcb 플러그인을 번들. `camera_tab.py` 가 모듈 최상위에서 `import cv2` → QApplication 생성 *전* 에 cv2 가 `QT_QPA_PLATFORM_PLUGIN_PATH` 를 가로채 PyQt5 의 xcb 로드 실패 → 코어덤프. (main_window 가 startup 에 camera_tab 을 import 하므로 치명적)
2. **컬럼 폭 불일치**: 두 테이블 모두 name 컬럼을 `Stretch` 로 둠 → 17열/13열 표의 남는 폭 배분이 달라 name 폭이 크게 어긋남.
3. **클라우드 안 보임**: `openarmx_body_link0→d435_center_link` 는 URDF(RSP)에 있으나, realsense 는 `camera_link` 루트로 발행 → `d435_center_link→camera_link` 연결이 없어 카메라 TF 트리가 로봇 트리와 분리. 이 identity 브리지는 `scenario_player_with_ee_leader.launch.py`(노드 `d435_center_to_camera_link_tf`)가 발행하는데, 그 스택 미기동 시 누락.
4. **box TF 안 보임**: 검출 노드 `/yolov8_node`(Pi Hailo 브리지) 미실행 → 박스 인식 자체가 안 됨. (`box_align` 은 인식 후에만, 그것도 AlignToBoxes 액션 중에만 `box_<i>` 를 /tf_static 에 발행)

### 수정
1. **신규 [camera_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/camera_tab.py)** — D435 라이브 영상(브리지 `sig_image` 구독, show/hide 시 구독/해제) + Pi(10.42.0.2) HTTP POST 워커스레드로 YOLOv8 검출 오버레이(`view_remote_seg.py` 의 draw/request_remote 포팅). `scenario_action_client.py` 에 `sig_image`+`set_image_topic`/`stop_image`/`_on_image`(cv_bridge 지연 import) 추가. main_window 3곳. package.xml 에 cv_bridge/python3-opencv.
2. **cv2 는 반드시 지연 import**: camera_tab 모듈 최상위 `import cv2` 제거, draw/_render/_worker_loop 내부로 이동(QApplication 이후 import → xcb 이미 로드돼 안전). 브리지의 cv2/cv_bridge 도 지연 import.
3. **Teaching 컬럼**: name·value 컬럼 모두 두 표 **동일 고정폭** + 끝에 Stretch 스페이서 컬럼(남는 폭 흡수) → 데이터 컬럼 정렬 일치. 선택색은 테이블 레벨 `selection-background-color`.
4. **클라우드**: `d435_center_link→camera_link` identity 정적 TF 발행하면 체인 복구(검증: body_link0→camera_depth_optical_frame 해석됨). 정식 경로는 scenario 스택.

### 재발 방지
- PyQt5 앱에서 `import cv2`(opencv-python)는 **QApplication 생성 후** 에만(지연 import). 또는 opencv-python-headless 사용.
- 두 테이블 정렬은 고정폭+스페이서. name 컬럼 Stretch 는 열 수가 다르면 어긋남.
- 카메라 RGBD/클라우드가 RViz 에 안 보이면 먼저 `d435_center_link→camera_link` TF 연결(=scenario 스택 또는 정적 TF) 확인. box TF 는 `/yolov8_node` 검출 노드 실행이 전제.

---

## 2026-06-03 — GUI에서 kill_all 기반 "STOP ALL" 제거 (shm 정리가 살아있는 GUI의 DDS를 깨뜨림)

### 증상
스택을 한 번 재시작한 뒤 GUI가 /joint_states 를 못 받음(Joint Control Actual "---"·회색, Teaching Capture "/joint_states 미수신", Cartesian Jog 조인트 "---"). EE TF 는 정상. fresh GUI 로 재기동하면 즉시 복구. 사용자: "잘 되었는데 (kill_all 돌린) 수정후에 안됨".

### 원인
`kill_all_ros2.sh` 는 `/dev/shm/fastrtps_*`(FastDDS 공유메모리) 를 정리한다(ghost participant 제거 목적). 그런데 이 스크립트는 GUI(rclpy, `__node:=` 리맵 없음)를 **죽이지 않고 살려둔다**. 그 결과 **살아남은 GUI 의 shm 기반 DDS 엔드포인트가 그 자리에서 깨지고**, 이후 새로 뜬 publisher(joint_state_broadcaster)와 재매칭에 실패 → 데이터 전달 두절. TF 는 다른 경로라 살아있어 더 헷갈림. (FastDDS 2.6.11; shm 비활성 env 미지원 → XML 프로파일 필요)

### 수정 (사용자 결정: kill_all 은 GUI 에서 빠져야 한다)
- `ui/scenario_ui.ui` — Scenario Player 탭의 `btnStopAll`("STOP ALL (kill_all_ros2.sh)") 위젯 제거.
- `main_window.py` — `_stop_all`(kill_all 실행) 메서드 + 시그널 연결 + `_resolve_kill_all_script`/`KILL_ALL_SCRIPT` 죽은 코드 제거.
- GUI 종료(`closeEvent`)는 기존대로 **GUI 가 띄운 managed 노드/런치만 중단**(각 탭 shutdown→proc.stop, `_hw`/`_player`/workflow/cyclo_extras stop, `ros2 daemon stop`(비파괴)). 전역 kill_all 호출 없음.
- Launch Manager 의 "Stop All (this tab)" 은 managed `proc.stop()` 만 하므로 유지(kill_all 아님).

### 재발 방지
- GUI 가 살아있는 채로 **kill_all_ros2.sh(또는 /dev/shm/fastrtps_* 정리)를 실행하지 말 것** — 살아남은 GUI 의 DDS 가 깨진다. 정리는 per-target `pkill`(shm 미정리) 또는 GUI 까지 함께 종료 후 재기동.
- 노드 종료는 "관련(=내가 띄운) 노드/런치만" 이 원칙. 전역 kill 은 GUI 책임이 아님.

---

## 2026-06-03 — 홈 자세에서 GUI가 /joint_states 못 읽음 (stale self-echo 필터) + Teaching 신규 탭 + Cartesian Jog 좌표표시

### 증상
- 컨트롤러(JTC) 실행 중인데 **홈/영 자세**에서 GUI 조인트 표시가 "---", Teaching Capture 가 "/joint_states 미수신" 으로 실패. EE TF 는 정상(초록), /joint_states 만 회색. 로봇이 한 번 움직이면 정상화됨(값이 달라져서).
- (부수) Cartesian Control Jog 탭에 현재 카테시안 좌표 미표시. Teaching 표 선택 행 글자가 흰색이라 안 보임.

### 원인 (근본)
`joint_control_tab._on_sil_tick` 는 JTC 활성 시 SIL 발행을 건너뛰지만(`traj_subscriber_count()>0`), 브리지의 **self-echo 필터 상태(`_last_self_publish`)를 클리어하지 않음**. 순서: ①GUI 기동(스택 전, JTC 없음) → SIL 이 /joint_states(영) 발행 → `_last_self_publish={0}` ②스택 기동(JTC 활성) → SIL 발행은 멈추나 `_last_self_publish` 잔존 ③joint_state_broadcaster 가 홈(영) 발행 → `_on_joint_state` 가 stale `{0}` 과 일치 → **자기 에코로 오판해 실제 피드백을 전부 드롭**. 즉 SIL 과 JTC 가 /joint_states 를 두고 충돌(이중 소스). 사용자 지적대로 **JTC 실행 시 SIL 은 분리되어야** 함.

### 수정
1. `joint_control_tab._on_sil_tick` — JTC 활성으로 SIL 발행을 건너뛸 때 `self._bridge.clear_self_echo()` 호출 → stale 필터 제거, 컨트롤러 피드백 항상 통과. (홈에서도 조인트 표시·Teaching Capture 정상 — xwd 캡처로 /joint_states 점 초록·`pose_0` 캡처 +0.0 확인)
2. **신규 [teaching_tab.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/teaching_tab.py)** — 상하 QSplitter 2 테이블(상: 조인트16+그리퍼, 하: 카테시안), 같은 웨이포인트 2표현, 인라인 편집(관절 clamp·RPY→quat), 자체완결 시나리오 Save(movej/movel+gripper)/Load, Go-to-Pose. `main_window.py` 3곳. 테이블 셀 글자색: 선택행 흰글자 문제 → `color:#222` + `:selected{background:#cfe3fb}`.
3. `cartesian_control_tab.py` — Jog 탭 per-arm **Cartesian pose 표시 + link7/TCP point 셀렉터**(500ms TF 갱신, 프레임 콤보 변경 시 재계산).

### 재발 방지
- /joint_states 는 단일 소스. ros2_control(JTC/broadcaster) 실행 중이면 GUI SIL 발행 **금지 + self-echo 필터 클리어**. self-echo 필터는 시간제한 없이 두면 우연히 값이 일치하는 실제 피드백을 삼킨다.
- "조인트 안 읽힘" 진단 시 EE TF(get_ee_pose) 와 /joint_states(sig_joint_state) 를 분리해 확인 — TF 만 되고 joint 안 되면 self-echo/이중발행 의심.

---

## 2026-06-03 — Pipe Health 전 스트리밍 토픽 0.0 Hz (Diagnostics 2탭 분리 후 카운터 이중 소비) + cyclo 경로 토픽 누락

### 증상
- Pipe Health 탭에서 `/joint_states`·`/tf` 가 **0.0 Hz(warn)** 로 표시. `/robot_description`(latched)·event 토픽은 정상. 사용자: "토픽이 모두 맞는지?"
- 실제로는 데이터가 정상 흐름: `ros2 topic echo` 카운트로 `/joint_states` ≈37 Hz(111건/3s), `/tf` 스트리밍, `/dynamic_joint_states` ≈19 Hz, `/openarmx/left/ee_pose` ≈35 Hz 확인. (주의: 이 PC 의 `ros2 topic hz` 는 QoS 무관 항상 0 — 도구 오작동, `echo` 로 검증할 것.)

### 원인
`scenario_action_client.py:diag_consume_rates()` 는 **읽으면서 per-topic 카운터를 0으로 리셋**(reset-on-read). Diagnostics 를 Node Health(show="node") + Pipe Health(show="topic") **2개 DiagnosticsTab 인스턴스로 분리**하면서, 양쪽 `_refresh` 가 각자 1.5s 타이머로 `diag_consume_rates()` 를 호출 → Node Health 가 rate 를 표시도 안 하면서 카운터를 리셋해 Pipe Health 몫을 0 으로 만듦. "rate" 종류(count 기반)만 영향, latched/event(age 기반)는 무영향 — 증상과 정확히 일치.

### 수정
1. `diagnostics_tab.py:_refresh` — `diag_consume_rates()` 를 **rate 를 표시하는 탭(show in {both,topic})만** 호출. Node Health 는 소비 안 함(리셋-온-리드 카운터는 단일 소비자 원칙).
2. `diagnostics_spec.py` TOPIC_SPECS 를 **활성 cyclo MoveL 백엔드 기준**으로 재구성: JTC `/…/joint_trajectory` 2행 제거(액션 구동이라 항상 idle·cyclo 미사용), 추가 = `/dynamic_joint_states`, `/tf_static`, `/openarmx/{l,r}/movel`(MoveL), `/openarmx/{l,r}/ee_pose`(PoseStamped), `/openarmx_{l,r}_movel_controller/controller_error`(String). QoS: 스트리밍/이벤트=best_effort(모든 pub 호환), tf_static=transient_local.
   - (Pilz/JTC 백엔드 전환 시 joint_trajectory 또는 `/<ctrl>/controller_state` 재추가 — 주석에 명시.)

### 재발 방지
- reset-on-read 카운터는 **소비자 1개**만. 동일 위젯을 여러 인스턴스로 띄울 때 공유 상태의 파괴적 읽기 주의.
- 토픽 발행률 진단은 `ros2 topic hz`(이 PC 오작동) 대신 `ros2 topic echo … --field … | 카운트` 로 확인.
- 모니터링 토픽 SET 은 **현재 활성 모션 백엔드**에 맞춘다(cyclo↔Pilz 경로 상이).

---

## 2026-06-03 — [코드리뷰/H3 · 수정 보류] ai_worker_config.yaml elbow 토픽 불일치 + 이 프로젝트(OpenArmX)는 해당 yaml 미사용

> 20-에이전트 코드리뷰(cyclo_robot_controller) 확정 결함 중 H3. **사용자 지시로 수정 보류 — vr_controller 미사용, VR 은 기본 구동 이후 진행.**

### 증상
`ai_worker_config.yaml` 에서 `leader_controller` 는 elbow 를 `/r_elbow_pose`,`/l_elbow_pose` 로 발행하고 `vr_controller` 는 `/r_subgoal_pose`,`/l_subgoal_pose` 로 구독 → `leader` 모드에서 elbow 기준값이 전달되지 않아 팔꿈치 리타게팅 불능(손목 추종은 정상).

### 원인 / 핵심 발견
- 토픽 이름 불일치: `vr_controller`(`:88-89`) vs `leader_controller`(`:115-116`).
- **그러나 이 프로젝트(OpenArmX)의 실제 VR 통합 `openarmx_motion/launch/openarmx_vr_bimanual.launch.py` 는 `ai_worker_config.yaml` 을 로드하지 않음.** `vr_controller_node` 를 노드명 `openarmx_vr_bimanual_controller` 로 띄우고 모든 파라미터를 **인라인**(OpenArmX 토픽 `/openarmx/{right,left}/elbow_pose`, 링크 `openarmx_*_link4/7`, **`weight_elbow_position: 0.0`=elbow 태스크 비활성**)으로 넘김. ROS2 파라미터는 **노드명 매칭**이라 yaml `vr_controller:` 섹션은 적용되지도 않음.
- 결론: `ai_worker_config.yaml` 은 ROBOTIS AI Worker **레퍼런스 잔재**. H3 불일치는 현재 OpenArmX 구동에 **무영향**(elbow weight=0, yaml 미로드).

### 수정
**보류(코드 변경 없음).** 사유: `vr_controller` 는 아직 사용 안 함(기본 구동 우선). VR 작업 재개 시 `ai_worker_config.yaml` 을 OpenArmX 값으로 정렬(또는 레퍼런스 섹션 제거 + openarmx 전용 config 분리) 여부를 결정.

### 재발 방지
- `ai_worker_config.yaml` 값은 ROBOTIS AI Worker 기준(이 프로젝트 무관)임을 인지. OpenArmX VR 의 실제 SSOT 는 `openarmx_vr_bimanual.launch.py` 인라인 파라미터.
- "yaml 값이 틀렸다"고 런타임 디버깅하지 말 것 — 런치가 yaml 을 로드하지 않음.

---

## 2026-06-03 — [코드리뷰/H2 · 수정 보류] AI Worker VR/leader 텔레오퍼레이션: 팔로워 그리퍼 명령 미발행 (passthrough 미구현)

> 20-에이전트 코드리뷰(cyclo_robot_controller) 확정 결함 중 H2. **사용자 지시로 수정 보류 — VR 구동 작업 시 함께 처리.**

### 증상
`controller_type:=vr`(또는 `leader`)로 AI Worker 텔레오퍼레이션 시, 리더 장치의 그리퍼 개폐가 팔로워 그리퍼로 전달되지 않음. 팔/리프트는 동작하나 grab(파지) 불가.

### 원인 (file:line 근거)
- `vr_controller_node.cpp` 가 리더 raw 궤적에서 그리퍼 위치를 추출해 `right_raw_gripper_position_`/`left_raw_gripper_position_`(콜백 `:346`, `:369`)에 **저장만** 하고, 어떤 발행 경로로도 내보내지 않음 (grep: 읽기 0건).
- `publishTrajectory` 는 팔(`arm_*_joint`)+lift 만 발행하며 그리퍼 조인트는 **의도적으로 제외**(`:813`,`:820` 주석 "without gripper joint").
- `r/l_gripper_pose_pub_` 는 측정 FK 자세(레퍼런스 정렬용)일 뿐 명령이 아님. `raw_traj_timeout_` 파라미터(`:78`)도 미사용.
- 전용 파라미터(`right/left_gripper_joint`)·구독·멤버·timeout 이 모두 갖춰져 있어, 원저자가 passthrough 를 의도하다 **미완성**한 정황. (적대적 검증 통과 — 활성 결함 확정)
- 참고: 워크스페이스의 그리퍼 명령 경로(`control_msgs/GripperCommand` HIL / `/joint_states` SIL, `scenario_action_client.py`)는 **시나리오 플레이어 경로**로 VR 텔레오퍼레이션과 별개.

### 수정
**보류(코드 변경 없음).** 사유: VR 구동 자체를 추후 진행 예정이며, 그리퍼 명령 인터페이스(토픽/메시지 타입, 별도 퍼블리셔 vs 팔 궤적 병합)를 함께 설계해야 함.
정리 일정: **VR 구동 작업 재개 시** (a) 그리퍼 passthrough 구현 또는 (b) 죽은 멤버/콜백 제거 중 택1.

### 재발 방지
- VR 그리퍼는 현재 동작하지 않음을 전제로 둘 것. "`gripper_pose` 토픽이 있으니 그리퍼가 제어된다"는 오해 금지(그건 측정 FK 관측값).
- VR 구동 재개 시 본 항목을 먼저 참조.

---

## 2026-06-03 — [코드리뷰/H1 · 수정] OMY MoveJ 노드: 다중포인트 명령 trailing point 재발행 (안전적분 우회)

> 20-에이전트 코드리뷰(cyclo_robot_controller) 확정 결함 중 H1. 수정·빌드 검증 완료.

### 증상
`omy_movej_controller` 입력 `JointTrajectory` 가 2개 이상 point 를 담으면, 2번째 이후 point(원본 raw 목표+원본 `time_from_start`)가 출력에 그대로 남아 100Hz 로 재발행됨. 하류 Joint Trajectory 컨트롤러가 `point[0]`(QP 적분 스텝)→`point[1..]`(raw 목표)를 보간해 QP/CBF 안전적분을 우회·목표로 점프, time 비단조 시 거부/오동작 가능.
- 단, 현재 워크스페이스엔 cyclo `~/movej` 로 다중포인트를 발행하는 소스가 없어 **잠재(latent)** — "단일포인트 입력 계약"을 코드로 강제하지 않은 견고성/일관성 갭. (제어 루프는 `movej_goal_`=`points.front()` 단일 목표만 추종하므로 출력의 trailing point 와 불일치)

### 원인
`makeOutputTrajectory` 가 `latest_movej_command_` 전체를 복사 후 `points.front()` 만 갱신(`omy_movej_controller_node.cpp`). MoveL 은 `trajectory_utils::makeJointTrajectoryMsg(model_joint_names_, …)` 로 매번 깨끗한 단일 포인트를 새로 만드는 것과 불일치(MoveJ 만 입력 메시지 재사용).

### 수정
`publishTrajectory` 를 MoveL 과 동일하게 `trajectory_utils::makeJointTrajectoryMsg(model_joint_names_, trajectory_time_, q_command)` 로 교체 → **항상 모델 전체 관절·단일 포인트** 발행. `makeOutputTrajectory` 및 죽은 멤버(`latest_movej_command_`, `latest_movej_command_received_`) 제거. (hpp 2곳, cpp 3곳)
검증: cyclo 스택 클린 재빌드 통과(6 pkg, exit 0), `omy_movej_controller_node` 실행파일 재생성. 부수로 재배치(`cyclo_robot_controller/` 이동)로 깨진 stale `build/`·`install/` 트리 복구(5개 cyclo 패키지).

### 재발 방지
- 노드 출력 궤적은 항상 컨트롤러 내부 상태(`q_commanded_`)로 `model_joint_names_` 전체를 단일 포인트로 새로 구성. 입력 명령 메시지를 출력 베이스로 재사용 금지.
- 동일 패턴이 OMX MoveJ(`omx_movej_controller_node`)에도 존재(리뷰 L21) → 같은 패치 적용 예정.

---

## 2026-06-03 — Launch Manager cyclo 타깃 "package 'cyclo_motion_controller_ros' not found" + RViz 노드명 불일치

### 증상
- Launch Manager 의 L2 cyclo MoveL 타깃 실행 시 `package 'cyclo_motion_controller_ros' not found` 로 실패.
- "rviz2 실행해도 모니터링 못함" — Launch Manager 상태/Node Health 가 RViz 를 안 떠 있음으로 표시.

### 원인
1. **cyclo**: GUI(scenario_ui) 프로세스가 15:31:41 기동, `cyclo_motion_controller_ros` 는 15:33:59 빌드 — 패키지가 GUI 기동 *후* 빌드되어 GUI 프로세스의 `AMENT_PREFIX_PATH` 에 해당 prefix 없음. Launch Manager 타깃은 GUI 환경을 상속하므로 동일 누락 → not found. (패키지는 openarmx_ws 안에 정상 존재; cyclo_ws 불필요 — 커밋 e1568f2 에서 `cyclo_robot_controller/` 로 이동됨.)
2. **rviz**: `launch_manager_tab.py` 의 scenario_rviz 타깃 cmd 가 bare `rviz2` 라 노드명이 `/rviz` 인데, 상태감시용 `nodes` 와 `diagnostics_spec.py` 는 `/openarmx_scenario_rviz` / `/rviz2` 를 기대 → 항상 불일치 → "안 떠 있음".

### 수정
1. **cyclo**: GUI 를 openarmx_ws/install 재소싱하여 재기동 (`AMENT_PREFIX_PATH` 에 cyclo_motion_controller_ros 포함 확인). 빌드 후 GUI 무재시작 시 환경 stale 됨에 유의.
2. **rviz 노드명 정합화 (3곳)**:
   - `launch_manager_tab.py` scenario_rviz cmd 에 `--ros-args -r __node:=openarmx_scenario_rviz` 추가 → 실제 노드명 `/openarmx_scenario_rviz`.
   - `diagnostics_spec.py` NODE_SPECS `/rviz2` → `/openarmx_scenario_rviz`, OPTIONAL_NODES 동일 치환.
   - 효과: Launch Manager 상태·Node Health 정상 표시, kill_all_ros2.sh 가 `__node:=` 리맵으로 PID 매핑 가능(종료 가능).

### 재발 방지
- "package not found" 는 소스(`find package.xml`)·빌드물(`install/`)·git log(이동/리네임) 를 먼저 확인. 메모리 옛 위치 기록 맹신 금지.
- 패키지를 새로 빌드하면 그 패키지를 쓰는 **실행 중 프로세스(GUI 등)는 재소싱(재기동)** 해야 인식. `/proc/<pid>/environ` 의 AMENT_PREFIX_PATH 와 빌드 시각(stat) 비교로 진단.
- RViz 등 도구 노드는 launch 시 `__node:=` 로 명시적 노드명 부여 → 상태감시/종료 스크립트와 정합.

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

## 2026-06-03 22:35 (KST) — Launch Manager "EE Leader 마커" 가 RViz 에 안 뜸 (프레임 이름 불일치)

### 증상
Launch Manager 탭에서 "EE Leader 마커 — RViz 6-DoF 드래그 티칭" Start → 상태는 `Running (this tab)` 이지만 RViz InteractiveMarkers 디스플레이에 마커(빨강/파랑 구체)가 전혀 안 나타남. RViz Displays 패널엔 EE Leader 항목이 `Status: Ok` 로 보임. ("TF 가 존재하는데 왜 안 뜨냐" 는 질문 동반.)

### 원인
`ee_leader_marker_bimanual.launch.py` 는 의도적으로 robot-agnostic 이라 launch arg 기본값이 placeholder (`base_frame=base_link`, `left/right_controlled_link=left/right_end_effector`, `goal_topic=/ee_leader/<arm>/goal_pose`) 이다 (docstring 에 "포팅 시 변경" 명시). Launch Manager 의 `ee_markers` preset 이 이 기본값을 **override 없이** 그대로 실행 → openarmx TF 트리엔 그 이름의 프레임이 **없다**.

- 마커 노드 [eef_interactive_marker_node.cpp:240-259](openarmx_ws/src/ee_leader_marker/src/eef_interactive_marker_node.cpp#L240-L259) 는 `base_frame → controlled_link` TF lookup **성공 후에만** `create6DofMarker()` 로 InteractiveMarker 를 server 에 insert. 프레임이 없으면 `lookupPose` 가 영구 실패 → 마커가 한 번도 insert 안 됨 → RViz 엔 디스플레이만 있고 마커 없음.
- 노드 로그(`/tmp/openarmx_scenario_ui_launch_ee_markers.log`)에 `Base frame: base_link` / `Controlled link: left_end_effector` 만 찍히고 `EE Leader Marker initialized from link transform` 줄은 **없음** → init 전 단계에서 멈춘 직접 증거.
- "TF 존재" 와 "이 프레임 이름 존재" 는 별개. openarmx TF 트리(`openarmx_*`, `d435_*`)는 정상 생존(그래서 Teaching 탭 `EE TF` 점이 녹색이고 Cartesian 포즈가 채워짐). 죽은 건 TF 가 아니라 **잘못된 프레임 이름**. 라이브 검증:
  - `tf2_echo base_link left_end_effector` → `Terminated` (영구 미해결, 프레임 없음)
  - `tf2_echo openarmx_body_link0 openarmx_left_link7` → `Translation: [0.181, 0.170, 0.312]` 해결됨 (프레임 존재)

### 수정
- [launch_manager_tab.py](openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/launch_manager_tab.py) `ee_markers` preset cmd 에 openarmx 전용 인자 추가 — 이미 검증된 SSOT [scenario_player_with_ee_leader.launch.py:138-142](openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/scenario_player_with_ee_leader.launch.py#L138-L142) 와 동일:
  - `base_frame:=openarmx_body_link0`
  - `left_controlled_link:=openarmx_left_link7`, `right_controlled_link:=openarmx_right_link7`
  - `left_goal_topic:=/openarmx/left/ee_leader/goal_pose`, `right_goal_topic:=/openarmx/right/ee_leader/goal_pose`
- standalone launch 의 generic 기본값은 그대로 둠(의도적 robot-agnostic, 특화는 caller 책임).

### 적용
symlink-install 이라 colcon 재빌드 불필요. 단, 실행 중인 UI 는 preset 을 메모리에 이미 로드했으므로 `kill_all_ros2.sh` 전체 종료 후 UI 재기동해야 새 인자 반영. 이후 노드 로그에 `EE Leader Marker initialized from link transform` 출력 + 손목에 빨강(우)/파랑(좌) 6-DoF 구체 표시.

### 관련 코드 / 파일
- `openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/launch_manager_tab.py` (`ee_markers` preset)
- `openarmx_ws/src/ee_leader_marker/launch/ee_leader_marker_bimanual.launch.py` (robot-agnostic 기본값)
- `openarmx_ws/src/ee_leader_marker/src/eef_interactive_marker_node.cpp` (init 시 TF lookup 의존)
- `openarmx_ws/src/openarmx_ros2/openarmx_scenario_player/launch/scenario_player_with_ee_leader.launch.py` (SSOT 인자값)

### 재발 방지
- robot-agnostic launch (placeholder 기본값) 를 Launch Manager preset 으로 감쌀 때는 반드시 openarmx 전용 프레임/토픽 인자를 명시 override. 기본값(`base_link` 등) 그대로 실행 금지.
- "마커/디스플레이 안 뜸" 진단 시 노드 로그의 init 완료 줄 유무 + `tf2_echo <base> <child>` 로 **그 프레임 이름이 실제 존재하는지** 확인 (TF 트리 생존 여부와 구분).

### 추가 수정 (2026-06-04 00:23 KST — 구 자체가 안 보임)
프레임 수정 후 마커는 떴으나 **중심 grab 구(sphere)가 안 보임**. 원인: 구 크기가 `marker.scale * 0.3` (=0.045 m)이라 link7 손목 메쉬 안에 파묻힘 (링/화살표 6-DoF 컨트롤만 보임).
- 수정: [eef_interactive_marker_node.cpp:117-124](openarmx_ws/src/ee_leader_marker/src/eef_interactive_marker_node.cpp#L117-L124) — 구 `0.3 → 0.8배` (=0.12 m, 링 반경 ~= 손목보다 큼) + `alpha 0.8 → 0.6` (반투명, 링/로봇 비침).
- **이 노드는 C++ 라 symlink 자동반영 안 됨** → `colcon build --packages-select ee_leader_marker` (25 s) 후 마커 노드 재기동 필요. (Python UI 와의 핵심 차이 — 재발 방지)
- 검증: 양 손목 link7 에 빨강(우)/파랑(좌) 반투명 구 또렷이 표시 (사용자 확인).

---

## 2026-06-04 23:41 (KST) — 세션 정리: HW 브링업 중복 RViz 제거 + IK 솔버 KDL→LMA

### 1. HW 하드웨어 브링업 시 RViz 2개 (중복)
- 증상: HW 하드웨어 브링업하면 RViz 창이 2개 뜸 (a2-scenario).
- 원인: scenario_player workflow launch 가 이미 `openarmx_scenario.rviz` 를 소유하는데, HW 브링업(`HW_LAUNCH_CMD`)도 자체 `bimanual.rviz` 를 spawn.
- 수정: [main_window.py](openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/main_window.py) `HW_LAUNCH_CMD` 에 `start_rviz:=false` 추가. RViz 는 scenario workflow 가 단독 소유 (RViz 자체는 반드시 뜨되 중복만 금지 — [[feedback_rviz_must_always_spawn]] 와 충돌 아님).

### 2. MoveIt IK 솔버 KDL → LMA + timeout 확대
- 변경: [kinematics.yaml](openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/config/kinematics.yaml) 양팔(`left_arm`/`right_arm`) `kdl_kinematics_plugin/KDLKinematicsPlugin → lma_kinematics_plugin/LMAKinematicsPlugin`, `kinematics_solver_timeout 0.005 → 0.05` (10배).
- 효과: LMA(Levenberg-Marquardt)는 KDL 대비 일부 자세에서 수렴이 안정적 + timeout 10배 확대로 IK 실패율 감소.

---
