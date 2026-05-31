# 패키지 상세 분석

## 1. 코어 패키지 (`src/openarmx_ros2/`)

### 1.1 openarmx (메타패키지)
- **빌드:** ament_cmake
- **역할:** description + bringup + hardware 집계만 수행
- **노드/런치:** 없음 (순수 메타패키지)

### 1.2 openarmx_description
- **위치:** `src/openarmx_description/`
- **목적:** URDF/Xacro 로봇 모델, 메쉬, 기구학 파라미터
- **메쉬:** 31개 (충돌 STL 20 + 시각화 DAE 11)
- **xacro 파일:**
  - `openarmx_robot.xacro` (최상위)
  - `openarmx_arm.xacro` / `openarmx_macro.xacro`
  - `openarmx_body.xacro` / `openarmx_body_macro.xacro`
  - `openarmx_hand.xacro` / `openarmx_hand_macro.xacro`
  - `openarmx.ros2_control.xacro` / `openarmx.bimanual.ros2_control.xacro`
- **지원 변형:** 단팔 / 양팔 / 그리퍼 옵션
- **런치:** `display_openarmx.launch.py` — RViz + robot_state_publisher + joint_state_publisher_gui
- **RViz 구성:** `arm_only.rviz`, `bimanual.rviz`
- **설정:** v10 기구학 12개 YAML (조인트 한계, 링크 오프셋, 관성)

### 1.3 openarmx_hardware
- **빌드:** ament_cmake (C++, v0.3.0)
- **플러그인:** `openarmx_hardware/OpenArmX_v10HW` (ros2_control SystemInterface)
- **통신:** CAN / CAN-FD (기본 `can0`, 양팔은 `can0` + `can1`)
- **모터:** 7-DOF + 그리퍼 (총 8개) — RS04/RS03/RS00
- **CAN ID:** 0x01-0x07 (팔), 0x08 (그리퍼)
- **제어 모드:** MIT (기본) / CSP (Compliant Stiffness Position)
- **그리퍼:** 스톨 감지 (위치 오차 임계값), 릴리즈 언락
- **런타임 파라미터:** KP/KD 동적 조정 (전용 executor 스레드)

### 1.4 openarmx_bringup
- **런치:** `openarmx.bimanual.launch.py` (양팔, 100 Hz)
- **설정:**
  - `openarmx_v10_controllers.yaml` (단팔)
  - `openarmx_v10_bimanual_controllers.yaml` (양팔)
  - `openarmx_v10_bimanual_controllers_namespaced.yaml`
- **컨트롤러:**
  - `joint_state_broadcaster`
  - `joint_trajectory_controller` (좌/우)
  - `forward_position_controller` (8-DOF, 그리퍼 포함)
  - `forward_velocity_controller` (7-DOF)
  - `gripper_action_controller` (GripperActionController, 스톨 감지)
  - `forward_effort_controller` (중력 보상 피드포워드)
- **URDF 인자:** `arm_type`, `use_fake_hardware`, `can_fd`, `right_can_interface`, `left_can_interface`, `control_mode` (mit/csp), `robot_controller`, `arm_prefix`, `enable_forward_effort`

### 1.5 openarmx_preview_bringup
- **bringup과 차이:** RViz + JointSliderPanel 필수 (시각화 중심)
- **런치 4개:** `openarmx.bimanual` × 2 + `openarm.bimanual` × 2 (사실상 동일)
- **하드웨어:** `use_fake_hardware` 토글 (기본 false). Gazebo 없음, xacro 조건부.
- **RViz:** `bimanual_with_panel.rviz` (RobotModel + JointSliderPanel)
- **용도:** 실 하드웨어 없이 안전한 오프라인 테스트

### 1.6 openarmx_gravity_comp
- **빌드:** ament_cmake (C++17)
- **노드:** `gravity_comp_node`
- **알고리즘:** Orocos KDL, `ChainDynParam::JntToGravity()`
- **입력:** `/joint_states` (sensor_msgs/JointState)
- **출력:** `/left_forward_effort_controller/commands`, `/right_forward_effort_controller/commands` (Float64MultiArray)
- **파라미터:** `urdf_path`, `g_scale` (기본 1.05, 런타임 조정), `enable_left/right`, `verbose`
- **안전 한계:** 어깨 20 Nm, 손목 2 Nm
- **양팔:** 좌/우 KDL 체인 독립 처리

### 1.7 openarmx_bimanual_moveit_config
- **MoveIt:** MoveIt2
- **플래닝 그룹 (SRDF):** `left_arm` (7), `right_arm` (7), `left_gripper` (1), `right_gripper` (1)
- **사전 정의 자세:** `home`, `hands_up`, gripper: `closed`/`half_closed`/`open` (0/22/44 mm)
- **IK 솔버:** `kdl_kinematics_plugin/KDLKinematicsPlugin` (search res 0.005, timeout 0.005 s)
- **플래너:** Pilz Industrial Motion Planner (직선 모션 1.0 m/s, 1.57 rad/s)
- **런치:** `demo.launch.py` (전체), `move_group.launch.py`, `moveit_rviz.launch.py`, `demo_sim.launch.py`
- **컨트롤러:** `{left,right}_joint_trajectory_controller`, `{left,right}_gripper_controller`, `joint_state_broadcaster` (100 Hz)

## 2. 텔레오퍼레이션 패키지

### 2.1 openarmx_teleop_bimanual
- **빌드:** C++17 (주) + Python3 (보조), ament_cmake
- **방식:** leader-follower (양팔 leader가 직접 CAN 통신, 양팔 follower는 ROS2 토픽 수신)
- **노드:**
  - `teleop_bimanual_node.cpp` — 기본, 200 Hz
  - `teleop_bimanual_with_gravitycomp_single.cpp` — 중력 보상 (KDL), 300 Hz
  - `teleop_unilateral_single.py` — 단팔 프로토타입 (Kp=100, Kd=5.5)
  - `teleop_unilateral_bimanual.py` — 위 노드 2개 런처
- **발행:** `/{left,right}_forward_position_controller/commands` (8-DOF)
- **구독:** 없음 (leader 모터 CAN 직접 읽음)
- **그리퍼 변환:** 모터 rad → 조인트 m, `0.044 m × (motor_rad / 1.0472 rad)` 선형
- **방향 반전:** 조인트/그리퍼 sign 플립 옵션 (미러 설치 대응)
- **중력 보상 옵션:** `g_scale=0.9`, hold 임계값 0.02 rad/s, settling 300 ms

### 2.2 openarmx_teleop_vr (Python)
- **빌드:** ament_python
- **노드:** `openarmx_teleop_vr_node.py` (60-100 Hz)
- **IK:** `openarmx_arm_driver.OpenArmTeleopController` (외부 바이너리)
- **VR 시스템:** OpenXR (HMD 비종속)
- **구독:**
  - `/vr_{left,right}_controller/pose` (PoseStamped)
  - `/vr_{left,right}_controller/{trigger,grip,rate}` (Float32)
  - `/vr_{left,right}_controller/button_{a,b,x,y}` (Bool)
- **모드:** safe (1°/cycle) / fast (12°/cycle), `grip ≥ 0.5` 데드맨
- **constraint_mode:** `"joint"` 또는 `"link"` (link4 위치 제약)

### 2.3 openarmx_teleop_bridge_vr (C++)
- **빌드:** ament_cmake
- **프로토콜:** UDP (포트 5100)
- **소스:** `openarmx_teleop_bridge_vr_node.cpp` (510줄, 단일 실행 파일)
- **역할:** VR 헤드셋 APK가 보내는 UDP 데이터그램을 ROS2 토픽으로 변환
- **UDP 포맷:** `[HAND] [pos_x pos_y pos_z] [qx qy qz qw] [trigger grip] [button_a b x y] [rate] [timestamp_ns]`
- **TF 발행:** 옵션 (기본 비활성)

### 2.4 openarmx_teleop_vr_apk
- **ROS 패키지 아님** (바이너리 배포 폴더, package.xml 없음)
- **APK:**
  - `openarmx-vr-quest.apk` (12.2 MB)
  - `openarmx-vr-pico.apk` (62.2 MB)
- **설치:** `adb install openarmx-vr-{device}.apk` (개발자 모드 + USB 디버깅)

## 3. Tools 패키지 (`src/openarmx_tools/`)

### 3.1 openarmx_joint_slider_panel
- **빌드:** ament_cmake + Qt5 + C++17
- **플러그인:** `openarmx_joint_slider_panel/JointSliderPanel` (rviz_common::Panel)
- **제어:** 양팔 14 조인트 (±π rad) + 그리퍼 2개 (0-44 mm)
- **발행:** `/{left,right}_forward_position_controller/commands`
- **구독:** `/joint_states`, `/robot_description`
- **루프:** 20 ms 스텝 (1-200 mrad/조인트, 0.1-10 mm/그리퍼)
- **기능:** 실시간 FK 미리보기 (TF2), mimic joint 지원, Home 버튼

### 3.2 openarmx_gripper_panel
- **빌드:** ament_cmake + Qt5
- **플러그인:** `openarmx_gripper_panel/GripperPanel`
- **액션:** `/{left,right}_gripper_controller/gripper_cmd` (control_msgs/GripperCommand)
- **프리셋:** 0 mm (closed), 22 mm (half), 44 mm (open)
- **양팔 동기:** 우측 → 좌측, 5 ms 시차

### 3.3 openarmx_kp_kd_panel
- **빌드:** ament_cmake + Qt5 + C++17
- **플러그인:** `openarmx_kp_kd_panel::KpKdPanel`
- **파라미터:** `/openarmx_{left,right}_hardware_params` 노드의 `kp_joint1..8`, `kd_joint1..8`
- **모터별 한계:** RS04 (J1-2: KP 0-5000, KD 0-100), RS03 (J3-4: 동일), RS00 (J5-8: KP 0-500, KD 0-5)
- **UI:** 팔 선택 (R/L/Both) + 슬라이더 + Reset/Apply

### 3.4 openarmx_teach
- **빌드:** ament_python
- **노드:**
  - `record_joint_states_always` — `/joint_states` 캡처 (기본 10 Hz, SPACE 토글, w 저장, q 종료)
  - `play_joint_trajectory` — YAML 궤적 재생 (액션 + 그리퍼 동기)
- **저장 포맷:** YAML (`joint_names`, `points[{positions, time_from_start}]`)
- **파일명:** 자동 타임스탬프 (`joint_states_stream_YYYYMMDD_HHMMSS.yaml`)
- **런치:** 없음 (CLI 진입점만)

## 4. VLA 패키지 (`src/openarmx_vla/`)

### 4.1 openarmx_lerobot
- **빌드:** ament_python (Python 3.8+)
- **통합:** HuggingFace LeRobot (ACT 정책)
- **3개 동반 패키지:**
  - `openarmx_lerobot` (인프라/런치)
  - `lerobot_robot_openarmx_follower_ros2` (`openarmx_follower_ros2` 등록)
  - `lerobot_teleoperator_openarmx_leader_ros2` (`openarmx_leader_ros2` 등록)
- **모듈:**
  - `openarmx_ros2.py` — LeRobot Robot 상속
  - `ros2_interface_openarmx.py` — pub/sub 레이어
  - `ros2_camera.py` — 이미지 토픽 구독
  - `config_openarmx_ros2.py` — 양팔 16-DOF 설정
- **카메라 토픽:** `/cam_{left,right,head}/{color,depth}/image`
- **VLA:** ACT (Action Chunking Transformer)
- **데이터:** `lerobot-record` 파이프라인, `~/.cache/huggingface/lerobot/local`
- **단축키:** → 저장, ← 폐기, Esc 종료
- **의존성:** torch, lerobot, transformers, datasets, accelerate, deepspeed, opencv-python, rclpy, numpy<2.0

## 5. 모터 매니저 (단독 실행, `src/openarmx_motor_manager/`)

- **ROS 패키지 아님** (PySide6 데스크톱 앱)
- **언어:** Python 3
- **GUI:** `GUI_MultiRobot.py` (탭 기반 다중 로봇)
- **모듈:** RobotPage, SingleMotorTestDialog, SettingsDialog, ConfigDialog, RobotWorker, can_detector, config_manager
- **CLI 스크립트:** 14개 (en/dis_all_motors, set_motor_zero, control_motor_gohome, check_motor_status, test_motor_all_random, ...)
- **기능:** CAN 자동 감지, 모터 enable/disable 일괄, 영점 보정, Go-home, MIT 모드 단일 테스트, 상태 모니터링
- **다국어:** EN/CN/JP/RU

## 패키지 의존성 매트릭스 (요약)

```
openarmx (metapackage)
  ├─ openarmx_description
  ├─ openarmx_hardware ─── 외부 .deb (openarmx-can)
  └─ openarmx_bringup
       ├─ openarmx_description
       ├─ openarmx_hardware
       └─ openarmx_gravity_comp

openarmx_bimanual_moveit_config
  ├─ openarmx_description
  └─ openarmx_bringup

openarmx_teleop_bimanual ─── 외부 .deb (openarmx-can)

openarmx_teleop_vr (Python)
  └─ openarmx_teleop_bridge_vr (UDP)
       ↑ APK from openarmx_teleop_vr_apk

openarmx_lerobot
  ├─ openarmx_bringup
  └─ lerobot_robot_openarmx_follower_ros2

tools (panels) ─── RViz / ros2_control 의존
```
