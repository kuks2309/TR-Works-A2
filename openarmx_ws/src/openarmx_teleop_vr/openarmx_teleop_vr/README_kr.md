# openarmx_teleop_vr 사용 설명 (한국어)

## 1. 패키지 포지셔닝

`openarmx_teleop_vr`은 OpenArmX의 VR 텔레오퍼레이션 실행 노드입니다.
이 노드는 VR 컨트롤러의 포즈, 트리거, 그립 등 ROS 토픽을 구독하고, 듀얼암 컨트롤러로 관절 명령을 퍼블리시하여 "VR 컨트롤러 -> 로봇 듀얼암"의 온라인 텔레오퍼레이션을 구현합니다.

VR 디바이스 텔레오퍼레이션에 관한 그림 및 텍스트 튜토리얼은 공식 문서를 참조해 주시기 바랍니다: <http://docs.openarmx.com>

## 2. 패키지 구조

```text
openarmx_teleop_vr/
├── README_CN.md
├── README.md
├── launch/
│   └── teleop_vr.launch.py         # 실행 진입점
├── openarmx_teleop_vr/
│   └── openarmx_teleop_vr_node.py  # 메인 노드
├── package.xml
└── setup.py
```

## 3. 시스템 체인

전체 데이터 흐름은 다음과 같습니다.

1. VR 디바이스가 브릿지 패키지(`openarmx_teleop_bridge_vr`)를 통해 컨트롤러 데이터를 퍼블리시합니다.
2. 본 노드가 입력 토픽을 구독하여 좌/우 팔의 관절 제어 명령을 계산합니다.
3. 제어 명령이 `forward_position_controller`로 퍼블리시되어 시뮬레이션 또는 실기를 구동합니다.

## 4. 주요 기능

1. 양손 컨트롤러 포즈를 사용해 듀얼암 말단 모션을 제어합니다.
2. 검지 트리거를 사용해 좌/우 그리퍼 개폐를 제어합니다.
3. 그립을 데드맨 스위치(deadman switch)로 사용하여 오조작 위험을 낮춥니다.
4. 속도 모드 전환 지원 (저속/고속)으로 큰 폭의 점프로 인한 안전 위험을 줄입니다.
5. 시각화 TF 퍼블리시 선택 가능하여 RViz 디버깅에 편리합니다.

## 5. 빠른 실행

### 사전 조건

1. 로봇 하위 계층(시뮬레이션 또는 실기)이 실행되어 있고 `forward_position_controller`를 사용할 수 있어야 합니다.
2. VR 브릿지 노드가 실행되어 있고 컨트롤러 토픽을 지속적으로 퍼블리시 중이어야 합니다.
3. 실행 환경에서 `openarmx_arm_driver`를 임포트할 수 있어야 합니다 (본 노드의 필수 의존성).

### 일반적인 실행 순서

1. 터미널 1: 로봇 실행

```bash
cd <워크스페이스 경로>
source install/setup.bash

# 시뮬레이션 모드
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=true

# 실기 모드: 먼저 CAN 채널 활성화
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

2. 터미널 2: VR 브릿지 실행

```bash
cd <워크스페이스 경로>
source install/setup.bash

ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

3. 터미널 3: 본 패키지 실행

```bash
cd <워크스페이스 경로>
source install/setup.bash

ros2 launch openarmx_teleop_vr teleop_vr.launch.py
```

## 6. 입력 및 출력 토픽

### 입력 토픽 (기본)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/vr_left_controller/pose` | `geometry_msgs/PoseStamped` | 좌측 컨트롤러 포즈 입력 |
| `/vr_right_controller/pose` | `geometry_msgs/PoseStamped` | 우측 컨트롤러 포즈 입력 |
| `/vr_left_controller/trigger` | `std_msgs/Float32` | 좌측 트리거 (그리퍼) |
| `/vr_right_controller/trigger` | `std_msgs/Float32` | 우측 트리거 (그리퍼) |
| `/vr_left_controller/grip` | `std_msgs/Float32` | 좌측 그립 (활성화) |
| `/vr_right_controller/grip` | `std_msgs/Float32` | 우측 그립 (활성화) |
| `/vr_right_controller/rate` | `std_msgs/Float32` | 속도 모드 입력 (0.1/1.0) |
| `/joint_states` | `sensor_msgs/JointState` | 현재 관절 상태 피드백 |

### 출력 토픽 (기본)

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 좌측 팔 관절 명령 |
| `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 우측 팔 관절 명령 |

## 7. 자주 사용하는 파라미터 (애플리케이션 계층)

| 파라미터 | 기본값 | 설명 |
|------|--------|------|
| `control_rate` | `100.0` | 제어 루프 주파수 (Hz) |
| `rate_topic` | `/vr_right_controller/rate` | 속도 모드 입력 토픽 |
| `slow_max_step_deg` | `1.0` | 저속 모드의 사이클당 최대 관절 스텝 |
| `fast_max_step_deg` | `12.0` | 고속 모드의 사이클당 최대 관절 스텝 |
| `ik_iterations` | `3` | 사이클당 역기구학 반복 횟수 |
| `grip_threshold` | `0.5` | 그립 활성화 임계값 |
| `left_grip_topic` | `/vr_left_controller/grip` | 좌측 그립 토픽 |
| `right_grip_topic` | `/vr_right_controller/grip` | 우측 그립 토픽 |
| `publish_visualization_tf` | `true` | 시각화 TF 퍼블리시 여부 |
| `print_performance` | `false` | 성능 로그 출력 여부 |
| `use_xacro` | `true` | 런타임에 xacro로 URDF 생성할지 여부 |
| `use_link4_ext` | `true` | 확장 제약 프레임 설정 활성화 여부 |
| `constraint_mode` | `joint` | 제약 모드: `joint` 또는 `link` |

예시:

```bash
ros2 launch openarmx_teleop_vr teleop_vr.launch.py \
  constraint_mode:=link
```

주의: 매니퓰레이터가 작업 경계에 근접하거나 초과할 때 떨림이 발생할 수 있습니다. 이는 솔버가 도달 불가능한 목표에 계속 근사하려 하기 때문입니다. 안전을 위해 극한 포즈 조작은 피해 주시기 바랍니다.

`constraint_mode` 모드 설명:

1. `joint` (기본값): 반응이 더 빠르지만 역기구학에 추가 자세 제약이 없어, 직관에 맞지 않는 관절 자세가 나올 수 있습니다.
2. `link`: 두 번째와 네 번째 관절에 추가 제약을 도입하여, 팔 자세가 본체 중심에 더 가까워지고 가슴 앞 작업 영역에 더 적합합니다.

태스크 요구사항에 따라 두 모드를 전환해 사용하시기 바랍니다.

## 8. 자주 묻는 질문

1. 손목 관절의 반응이 느린 경우

기본 파라미터는 매니퓰레이터의 급변으로 인한 위험을 줄이기 위해 관절 스텝을 비교적 엄격하게 제한하고 있습니다. `slow_max_step_deg`와 `fast_max_step_deg`를 적절히 늘려 반응 속도를 향상시킬 수 있지만, 안전 위험은 직접 평가하셔야 합니다.

2. 노드는 실행되었으나 매니퓰레이터가 움직이지 않는 경우

먼저 브릿지 토픽에 데이터가 있는지 확인합니다.

```bash
ros2 topic echo /vr_left_controller/pose
ros2 topic echo /vr_right_controller/pose
```

3. 컨트롤러 데이터는 있는데 제어 명령이 없는 경우

컨트롤러 명령 토픽을 확인합니다.

```bash
ros2 topic echo /left_forward_position_controller/commands
ros2 topic echo /right_forward_position_controller/commands
```

4. 오류: `openarmx_arm_driver` 임포트 실패

현재 환경에 해당 Python 의존성이 누락된 것입니다. 먼저 설치하고 환경이 올바르게 `source` 되었는지 확인하시기 바랍니다.

5. 그리퍼가 반응하지 않는 경우

트리거 토픽 데이터와 그립 임계값(`grip_threshold`)이 실제 조작 습관과 일치하는지 확인해 주시기 바랍니다.

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
