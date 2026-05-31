# openarmx_teleop_vr

이 디렉터리는 OpenArmX의 **VR 텔레오퍼레이션 체인**으로, 두 개의 ROS 2 패키지를 포함합니다. 이들을 조합하면 다음을 구현할 수 있습니다.

`VR 컨트롤러 UDP 데이터 -> ROS 2 토픽 -> 듀얼암 관절 제어 명령`

> ⚠️ "로봇 하위 계층 -> 브릿지 노드 -> 텔레오퍼레이션 노드" 순서로 엄격히 실행하실 것을 권장합니다. 토픽이 준비되지 않아 제어가 실패하는 상황을 방지할 수 있습니다.

## ✅ 사전 조건

본 패키지의 ROS 2 체인을 시작하기 전에, VR 디바이스에 먼저 VR 텔레오퍼레이션 App을 설치해 주시기 바랍니다.  
해당 App의 APK 소스 저장소는 다음과 같습니다.

- `https://github.com/openarmx/openarmx_teleop_vr_apk.git`

> ⚠️ VR 디바이스에 해당 App이 설치되어 정상 동작하지 않으면, 브릿지 노드가 유효한 컨트롤러 UDP 데이터를 수신할 수 없습니다.

## 🗂️ 디렉터리 구조

```text
openarmx_teleop_vr/
├── openarmx_teleop_bridge_vr/   # C++ 브릿지 패키지: UDP -> ROS 2 토픽/TF
├── openarmx_teleop_vr/          # Python 텔레오퍼레이션 패키지: 토픽 입력 -> IK -> 제어 명령
├── LICENSE                           # 라이선스
├── README_CN.md                      # 본 소개 (중국어)
└── README_EN.md                      # This overview (English)
```

## 📦 두 서브 패키지의 역할

1. `openarmx_teleop_bridge_vr`
- VR/OpenXR 측의 UDP 데이터를 수신합니다 (기본 수신 포트 `5100`).
- 컨트롤러 포즈, 트리거, 그립, 속도 등의 ROS 2 토픽을 퍼블리시합니다 (TF 퍼블리시 선택 가능).

2. `openarmx_teleop_vr`
- 브릿지 패키지가 퍼블리시한 토픽과 `/joint_states`를 구독합니다.
- IK 계산과 제약 조건 처리를 수행합니다.
- 듀얼암 제어 명령을 다음 토픽으로 퍼블리시합니다.
  - `/left_forward_position_controller/commands`
  - `/right_forward_position_controller/commands`

## 🚀 최단 사용 체인 (권장 순서)

1. 로봇 하위 계층 실행 (`forward_position_controller`가 사용 가능한지 확인)  
2. 브릿지 노드 실행  
3. VR 텔레오퍼레이션 노드 실행

> ✅ 세 단계를 모두 실행한 후 컨트롤러를 조작하시면 더 안정적입니다.

예시:

```bash
cd <워크스페이스 경로>
colcon build --packages-select openarmx_teleop_bridge_vr openarmx_teleop_vr
source install/setup.bash

# 터미널 1: 로봇 하위 계층 (예시)
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false

# 터미널 2: VR 브릿지
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node

# 터미널 3: VR 텔레오퍼레이션 실행 노드
ros2 launch openarmx_teleop_vr teleop_vr.launch.py
```

## 🔎 빠른 점검

- 브릿지 입력이 정상인지 확인:
  - `ros2 topic echo /vr_left_controller/pose`
  - `ros2 topic echo /vr_right_controller/pose`
- 텔레오퍼레이션 출력이 정상인지 확인:
  - `ros2 topic echo /left_forward_position_controller/commands`
  - `ros2 topic echo /right_forward_position_controller/commands`

> ⚠️ 입력 데이터는 있는데 출력 명령이 없는 경우, 로봇 컨트롤러와 노드 실행 순서를 우선 확인해 주시기 바랍니다.

## 🧩 주요 의존성 (요약)

- 브릿지 패키지: `rclcpp`, `geometry_msgs`, `tf2_ros`
- 텔레오퍼레이션 패키지: `rclpy`, `geometry_msgs`, `sensor_msgs`, `std_msgs`, `tf2_ros`, `xacro`, `openarmx_description`
- 실행 측에 `openarmx_arm_driver`도 필요합니다 (자세한 내용은 서브 패키지 문서 참조)

## 📚 상세 문서 진입점

- 브릿지 패키지 한국어 설명: `openarmx_teleop_bridge_vr/README_kr.md`
- 텔레오퍼레이션 패키지 한국어 설명: `openarmx_teleop_vr/README_kr.md`
- 텔레오퍼레이션 launch 파일: `openarmx_teleop_vr/launch/teleop_vr.launch.py`
