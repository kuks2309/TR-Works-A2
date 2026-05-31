# OpenArmX Bimanual MoveIt Config

[English](#english) | [中文](README_CN.md) | [한국어](#한국어)

---

### 개요

본 패키지는 OpenArmX 듀얼암 로봇 시스템을 위한 MoveIt 2 설정을 제공합니다. MoveIt 모션 플래닝 프레임워크를 사용해 듀얼암과 그리퍼의 모션 플래닝 및 제어를 구현하기 위한 모든 설정 파일, launch 파일, 스크립트가 포함되어 있습니다.

### 기능 특성

- 듀얼암 7 자유도 설정 (left_arm 및 right_arm)
- 듀얼 그리퍼 제어 (left_gripper 및 right_gripper)
- 시뮬레이션 모드(가상 하드웨어) 및 실제 하드웨어 모드 지원
- CAN 2.0 및 CAN-FD 통신 지원
- MIT 및 CSP 제어 모드 지원
- 사전 설정된 모션 플래닝 그룹과 상태
- KDL 키네매틱스 솔버 통합
- RViz 시각화 및 MoveIt 플러그인

### 패키지 구조

```
openarmx_bimanual_moveit_config/
├── config/
│   ├── openarmx_bimanual.srdf          # 시맨틱 로봇 기술 파일
│   ├── openarmx_bimanual.urdf.xacro    # 로봇 URDF (Xacro)
│   ├── joint_limits.yaml              # 관절 속도/가속도 제한
│   ├── kinematics.yaml                # 키네매틱스 솔버 설정
│   ├── moveit_controllers.yaml        # MoveIt 컨트롤러 설정
│   ├── ros2_controllers.yaml          # ros2_control 컨트롤러 설정
│   ├── initial_positions.yaml         # 기본 관절 위치
│   ├── pilz_cartesian_limits.yaml     # Pilz 플래너 카르테시안 제한
│   ├── sensors_3d.yaml                # 3D 센서 설정
│   └── moveit.rviz                    # RViz 설정
├── launch/
│   ├── demo.launch.py                 # 메인 데모 launch 파일 (실제 하드웨어)
│   ├── demo_sim.launch.py             # 시뮬레이션 모드 데모 launch 파일
│   ├── move_group.launch.py           # MoveIt move_group 노드
│   ├── moveit_rviz.launch.py          # MoveIt 플러그인 포함 RViz
│   ├── spawn_controllers.launch.py    # 컨트롤러 스포너
│   └── static_virtual_joint_tfs.launch.py
├── run_bimanual_moveit_sim.sh         # 빠른 실행 스크립트 (시뮬레이션)
├── run_bimanual_moveit_with_can2.0.sh # 빠른 실행 스크립트 (실제 하드웨어)
├── CMakeLists.txt
├── package.xml
└── README.md
```

### 의존성

- ROS 2 Humble
- MoveIt 2
- ros2_control
- openarmx_description
- openarmx_bringup
- openarmx_hardware
- openarmx_arm_driver

### 설치

1. 워크스페이스에 모든 OpenArmX 패키지가 구성되어 있는지 확인합니다
2. 본 패키지를 빌드합니다:

```bash
colcon build --packages-select openarmx_bimanual_moveit_config
source install/setup.bash
```

### 사용 방법

#### 시뮬레이션 모드 (하드웨어 불필요)

실제 하드웨어 없이 테스트하는 경우:

```bash
# 편의 스크립트 사용
./run_bimanual_moveit_sim.sh

# 또는 ros2 launch 직접 사용
ros2 launch openarmx_bimanual_moveit_config demo_sim.launch.py
```

#### 실제 하드웨어 모드

**중요**: 실제 하드웨어를 실행하기 전에 다음을 확인하시기 바랍니다.
1. CAN 인터페이스(can0, can1)가 올바르게 설정되어 있는지
2. 매니퓰레이터에 전원이 인가되어 있고 영점 부근에 위치하는지 (편차 30도 이내)

##### 방법 1: 편의 스크립트로 원클릭 실행

```bash
# 편의 스크립트 사용 (CAN 자동 설정)
./run_bimanual_moveit_with_can2.0.sh
```

##### 방법 2: 수동으로 CAN을 설정한 후 실행

```bash
# 또는 CAN을 수동으로 설정한 후 실행
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

ros2 launch openarmx_bimanual_moveit_config demo.launch.py
```

### 실행 파라미터

| 파라미터 | 기본값 | 설명 |
|------|--------|------|
| `description_package` | `openarmx_description` | URDF 파일을 포함하는 패키지 |
| `description_file` | `v10.urdf.xacro` | URDF/Xacro 파일명 |
| `arm_type` | `v10` | 매니퓰레이터 타입 |
| `use_fake_hardware` | `false` | 시뮬레이션 모드 활성화 |
| `robot_controller` | `joint_trajectory_controller` | 컨트롤러 타입 (`forward_position_controller` 또는 `joint_trajectory_controller`) |
| `control_mode` | `mit` | 모터 제어 모드 (`mit` 또는 `csp`) |
| `right_can_interface` | `can0` | 우측 팔 CAN 인터페이스 |
| `left_can_interface` | `can1` | 좌측 팔 CAN 인터페이스 |

커스텀 파라미터 예시:

```bash
ros2 launch openarmx_bimanual_moveit_config demo.launch.py control_mode:=mit
```

### 모션 플래닝 그룹

| 그룹명 | 관절 | 설명 |
|------|------|------|
| `left_arm` | joint1-7 | 좌측 팔 (7 자유도) |
| `right_arm` | joint1-7 | 우측 팔 (7 자유도) |
| `left_gripper` | finger_joint1 | 좌측 그리퍼 |
| `right_gripper` | finger_joint1 | 우측 그리퍼 |

### 사전 정의된 그룹 상태

| 상태명 | 그룹 | 설명 |
|--------|----|----|
| `home` | left_arm / right_arm | 모든 관절이 0 위치 |
| `hands_up` | left_arm / right_arm | joint4가 2 rad, 나머지는 0 |
| `closed` | left_gripper / right_gripper | 그리퍼 완전 닫힘 (0) |
| `half_closed` | left_gripper / right_gripper | 그리퍼 반쯤 닫힘 (0.022) |
| `open` | left_gripper / right_gripper | 그리퍼 완전 열림 (0.044) |

---

## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Author

- **Zhang Li** (张力)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

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
