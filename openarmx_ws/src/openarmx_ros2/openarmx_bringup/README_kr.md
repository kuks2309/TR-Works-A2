# OpenArmX Bringup

[English](README.md) | [中文](README_CN.md) | [한국어](#개요)

---

### 개요

본 패키지는 OpenArmX 듀얼암 로봇 시스템을 위한 실행 설정을 제공합니다. launch 파일과 컨트롤러 설정을 포함하며, ros2_control을 사용해 다양한 제어 모드와 하드웨어 인터페이스로 로봇을 실행하는 것을 지원합니다.

### 기능 특성

- 듀얼암 7 자유도 설정 (left_arm 및 right_arm)
- 듀얼 그리퍼 제어 (8 자유도 모드, 그리퍼 통합)
- 시뮬레이션 모드(가상 하드웨어) 및 실제 하드웨어 모드 지원
- CAN 2.0 및 CAN-FD 통신 지원
- MIT(모션 컨트롤) 및 CSP(위치) 제어 모드 지원
- 다양한 컨트롤러 타입: 궤적 컨트롤러와 전방향 위치 컨트롤러
- 네임스페이스 지원으로 다중 로봇 구성에 적합
- RViz 시각화

### 패키지 구조

```
openarmx_bringup/
├── config/
│   └── v10_controllers/
│       ├── openarmx_v10_bimanual_controllers.yaml          # 메인 듀얼암 컨트롤러 설정
│       ├── openarmx_v10_bimanual_controllers_namespaced.yaml # 네임스페이스 포함 컨트롤러 설정
│       └── openarmx_v10_controllers.yaml                   # 단일 팔 컨트롤러 설정
├── launch/
│   └── openarmx.bimanual.launch.py    # 메인 듀얼암 launch 파일
├── rviz/
│   └── bimanual.rviz                 # RViz 설정
├── GRIPPER_CONTROL_GUIDE.md          # 그리퍼 제어 상세 문서
├── CMakeLists.txt
├── package.xml
├── README.md
└── README_CN.md
```

### 의존성

- ROS 2 Humble
- ros2_control
- controller_manager
- joint_state_broadcaster
- joint_trajectory_controller
- forward_command_controller (선택 사항, 설치 명령: `sudo apt-get install ros-humble-forward-command-controller`)
- openarmx_description
- openarmx_hardware

### 설치

1. 워크스페이스에 모든 OpenArmX 패키지가 구성되어 있는지 확인합니다
2. 본 패키지를 빌드합니다:

```bash
colcon build --packages-select openarmx_bringup
source install/setup.bash
```

### 사용 방법

#### 단일 로봇 실행 (기본 설정)

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py
```

#### 단일 로봇 실행 (MIT 제어 모드, 텔레오퍼레이션 권장)

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py control_mode:=mit
```

#### 듀얼 로봇 구성

**마스터 측 팔:**
```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit
```

**팔로워 측 팔:**
```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can2 \
    left_can_interface:=can3 \
    control_mode:=mit
```

#### 시뮬레이션 모드 (실제 하드웨어 불필요)

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    use_fake_hardware:=true
```

### 실행 파라미터

#### 핵심 파라미터

| 파라미터 | 타입 | 기본값 | 선택 가능 값 | 설명 |
|------|------|--------|--------|------|
| `robot_controller` | string | `joint_trajectory_controller` | `joint_trajectory_controller`, `forward_position_controller` | 로봇 컨트롤러 타입 |
| `control_mode` | string | `mit` | `mit`, `csp` | 하위 계층 모터 제어 모드 |
| `use_fake_hardware` | bool | `false` | `true`, `false` | 시뮬레이션 하드웨어 활성화 |

#### `robot_controller` 상세

**1. `joint_trajectory_controller` (기본값, 모션 플래닝 권장)**
- 부드러운 궤적 보간 지원
- 적용 시나리오: MoveIt 모션 플래닝, 사전 정의된 궤적 실행
- 인터페이스 타입: Action (`control_msgs/action/FollowJointTrajectory`)
- 실행되는 컨트롤러:
  - `left_joint_trajectory_controller`
  - `right_joint_trajectory_controller`
  - `left_gripper_controller`
  - `right_gripper_controller`

**2. `forward_position_controller` (텔레오퍼레이션 권장)**
- 직접 위치 명령 컨트롤러, 실시간 반응이 빠름
- 적용 시나리오: 텔레오퍼레이션, 티칭, 실시간 제어
- 인터페이스 타입: Topic (`std_msgs/msg/Float64MultiArray`)
- 실행되는 컨트롤러:
  - `left_forward_position_controller` (그리퍼를 8번째 관절로 포함)
  - `right_forward_position_controller` (그리퍼를 8번째 관절로 포함)

#### `control_mode` 상세

| 모드 | 설명 | 적용 시나리오 | 특징 |
|------|------|----------|------|
| `mit` | MIT 모션 컨트롤 모드 | 힘 제어, 컴플라이언스 제어, 텔레오퍼레이션 | 토크 제어, 저임피던스, 드래그 티칭 지원 |
| `csp` | CSP 위치 모드 | 고정밀 위치 제어 | 고강성, 정확한 위치 결정 |

**주의**: 시스템 강성을 변경하려면 `openarmx_hardware/include/openarmx_hardware/v10_simple_hardware.hpp`의 KP/KD 파라미터를 조정합니다.

#### 하드웨어 인터페이스 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `right_can_interface` | string | `can0` | 우측 팔 CAN 인터페이스 |
| `left_can_interface` | string | `can1` | 좌측 팔 CAN 인터페이스 |
| `can_fd` | string | `false` | CAN-FD 활성화 (true) 또는 클래식 CAN 사용 (false) |

**일반적인 CAN 인터페이스 설정:**
- 단일 로봇: `can0` (우측 팔), `can1` (좌측 팔)
- 듀얼 로봇 마스터 측: `can0` (우측 팔), `can1` (좌측 팔)
- 듀얼 로봇 팔로워 측: `can2` (우측 팔), `can3` (좌측 팔)

#### 고급 파라미터

| 파라미터 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `description_package` | string | `openarmx_description` | URDF/xacro 파일이 위치한 패키지 |
| `description_file` | string | `v10.urdf.xacro` | 로봇 기술 파일명 |
| `arm_type` | string | `v10` | 로봇 팔 타입 |
| `runtime_config_package` | string | `openarmx_bringup` | 컨트롤러 설정 파일이 위치한 패키지 |
| `controllers_file` | string | `openarmx_v10_bimanual_controllers.yaml` | 컨트롤러 설정 파일명 |
| `arm_prefix` | string | `` (빈 문자열) | 토픽 네임스페이스 접두사 |

### 컨트롤러 설명

#### 자동으로 실행되는 컨트롤러

`robot_controller` 선택과 관계없이 다음 컨트롤러는 항상 실행됩니다.

1. **joint_state_broadcaster**: 관절 상태 퍼블리시
2. **그리퍼 컨트롤러** (`joint_trajectory_controller` 사용 시):
   - `left_gripper_controller`
   - `right_gripper_controller`
3. **팔 컨트롤러** (`robot_controller` 파라미터에 따라 선택)

#### 활성 컨트롤러 확인

```bash
sudo apt-get install ros-humble-ros2controlcli
ros2 control list_controllers
```

### 토픽 및 인터페이스

#### 관절 상태 브로드캐스트

- `/joint_states` - 모든 관절의 상태

#### 궤적 컨트롤러 모드

- `/left_joint_trajectory_controller/follow_joint_trajectory` (Action)
- `/right_joint_trajectory_controller/follow_joint_trajectory` (Action)
- `/left_gripper_controller/gripper_cmd` (Action)
- `/right_gripper_controller/gripper_cmd` (Action)

#### 전방향 위치 컨트롤러 모드

- `/left_forward_position_controller/commands` (Topic: `std_msgs/msg/Float64MultiArray`)
- `/right_forward_position_controller/commands` (Topic: `std_msgs/msg/Float64MultiArray`)

### 그리퍼 제어

그리퍼 제어에 관한 상세 정보 (다음 내용 포함):
- GripperActionController와 ForwardCommandController 비교
- 텔레오퍼레이션 설정
- Python 및 C++ 코드 예시

[GRIPPER_CONTROL_GUIDE_kr.md](GRIPPER_CONTROL_GUIDE_kr.md)를 참조하시기 바랍니다.

### 사용 예시

#### 예시 1: 궤적 컨트롤러로 실행 (MoveIt)

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    robot_controller:=joint_trajectory_controller \
    control_mode:=mit
```

#### 예시 2: 위치 컨트롤러로 실행 (텔레오퍼레이션 팔로워 측)

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    robot_controller:=forward_position_controller \
    control_mode:=mit \
    right_can_interface:=can2 \
    left_can_interface:=can3
```

#### 예시 3: CSP 위치 모드 사용

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    control_mode:=csp
```

#### 예시 4: 위치 명령 전송 (전방향 위치 컨트롤러)

```bash
# 좌측 팔에 위치 전송 (관절 7개 + 그리퍼 1개)
ros2 topic pub /left_forward_position_controller/commands \
    std_msgs/msg/Float64MultiArray \
    "data: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02]" --once
```

---

## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Zhang Li** (张力)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

**현재 버전**: 1.0.0

---

## 📞 문의

### Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
**Chengdu Changshu Robotics Co., Ltd.**

| 연락처 | 정보 |
|---------|------|
| 📧 이메일 | openarmrobot@gmail.com |
| 📱 전화/WeChat | +86-17746530375 |
| 🌐 공식 웹사이트 | <https://openarmx.com/> |
| 📍 주소 | 천진 경제기술개발구 서구 신예팔가 11호 화성기계공장 |
| 👤 담당자 | Mr. Wang |
