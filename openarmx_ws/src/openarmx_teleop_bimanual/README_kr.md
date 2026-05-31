# OpenArm 텔레오퍼레이션 패키지 (openarmx_teleop_bimanual)

[English](README.md) | [简体中文](README_CN.md) | 한국어

OpenArmX 매니퓰레이터의 텔레오퍼레이션 기능을 구현하기 위한 ROS2 패키지입니다. 두 가지 모드를 포함합니다.

1.  **중력 보상 없음 모드**: 듀얼암 텔레오퍼레이션을 지원합니다. 마스터 측 팔의 모터는 비활성화되어 있어 직접 손으로 드래그할 수 있습니다.
2.  **중력 보상 포함 모드**: 단일 팔 텔레오퍼레이션을 지원합니다. 마스터 측 팔이 능동적으로 자체 중력을 보상하여 "무중력" 드래그 경험을 구현합니다.

---

## 모드 1: 듀얼암 텔레오퍼레이션 (중력 보상 없음)

이 모드에서는 좌우 두 매니퓰레이터를 동시에 제어할 수 있습니다. 마스터 측 모터는 비활성화되므로 손으로 쉽게 드래그할 수 있으며, 팔로워 측이 실시간으로 모션을 따라옵니다.

### 기능 특성

✅ **듀얼암 동시 제어**
✅ **높은 실시간성**: 200Hz 제어 주기
✅ **수동 드래그**: 마스터 측 모터 비활성화, 저항 없음
✅ **ROS2 통합**
✅ **8 자유도**: 7개 관절 + 1개 그리퍼 제어

### 시스템 아키텍처
```
마스터 측 (Leader)                  팔로워 측 (Follower)
┌─────────────────┐                ┌──────────────────┐
│  우측 팔 (can0)  │                │  우측 팔 (can2)   │
│  7관절+그리퍼     │  ─────ROS2────→│  8DOF 컨트롤러    │
└─────────────────┘                └──────────────────┘
┌─────────────────┐                ┌──────────────────┐
│  좌측 팔 (can1)  │                │  좌측 팔 (can3)   │
│  7관절+그리퍼     │  ─────ROS2────→│  8DOF 컨트롤러    │
└─────────────────┘                └──────────────────┘
```

### 사용 방법

#### 1단계: 팔로워 측 로봇 실행

**중요: 반드시 팔로워 측을 먼저 실행해야 합니다!**

팔로워 측은 `forward_position_controller` 모드로 실행해야 합니다.
```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can2 \
    left_can_interface:=can3 \
    control_mode:=mit \
    robot_controller:=forward_position_controller
```

#### 2단계: 텔레오퍼레이션 노드 실행

다른 터미널에서 텔레오퍼레이션 노드를 실행합니다.
```bash
source ~/openarmx_ws/install/setup.bash
ros2 launch openarmx_teleop_bimanual teleop_bimanual.launch.py
```

**커스텀 파라미터로 실행:**
```bash
ros2 launch openarmx_teleop_bimanual teleop_bimanual.launch.py \
    leader_right_can:=can0 \
    leader_left_can:=can1 \
    follower_right_prefix:=right \
    follower_left_prefix:=left \
    control_rate_hz:=200
```

---

## 모드 2: 단일/듀얼암 텔레오퍼레이션 (중력 보상 포함)

이 모드에서 마스터 측 매니퓰레이터는 모터를 활성화하고, URDF 모델 기반으로 자체 중력 토크를 실시간으로 계산해 보상합니다. 이를 통해 매니퓰레이터를 드래그할 때 자체 무게를 느끼지 않게 되어, 조작의 부드러움과 정밀성이 크게 향상됩니다.

### 기능 특성

✅ **중력 보상**: "무중력" 드래그 경험 구현
✅ **능동 댐핑**: 안정성을 높이도록 댐핑 설정 가능
✅ **위치 유지**: 팔이 정지하면 위치를 잠글 수 있음
✅ **URDF 기반**: 정확한 로봇 모델 기반 계산

### 사용 방법

#### 1단계: 팔로워 측 로봇 실행

팔로워 측은 `forward_position_controller` 모드로 실행해야 합니다.
```bash
# 우측 팔을 팔로워 측으로 실행
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can2 \
    left_can_interface:=can3 \
    control_mode:=mit \
    robot_controller:=forward_position_controller
```

#### 2단계: 중력 보상 텔레오퍼레이션 노드 실행

다른 터미널에서 텔레오퍼레이션 노드를 실행합니다. **주의:** 이 모드에는 URDF 파일이 필요합니다.

### 먼저 urdf 파일 생성
다음 명령을 실행합니다.
```
cd {워크스페이스 경로}
xacro ./src/openarmx_description/urdf/robot/v10.urdf.xacro  arm_type:=v10 bimanual:=true > /tmp/v10_bimanual.urdf
```

- **듀얼암 실행 (중력 보상 포함, 실험적)**
간단한 실행 명령
```bash
source ~/openarmx_ws/install/setup.bash
ros2 launch openarmx_teleop_bimanual teleop_bimanual_with_gravitycomp.launch.py
```

커스텀 실행 파라미터
```bash
source ~/openarmx_ws/install/setup.bash
ros2 launch openarmx_teleop_bimanual teleop_bimanual_with_gravitycomp.launch.py mode:=bimanual leader_urdf_path:="/tmp/v10_bimanual.urdf"
```

### 중력 보상 실행 파라미터 설명

| 파라미터명 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `mode` | string | `bimanual` | 제어 모드: `bimanual`, `left_only`, `right_only` |
| `leader_urdf_path` | string | `/tmp/v10_bimanual.urdf` | **필수**, 마스터 측 URDF 파일 경로 |
| `g_scale` | double | `0.9` | 중력 보상 스케일 계수. <1.0이면 가볍게 느껴지고, >1.0이면 위로 떠오르는 느낌이 됨 |
| `kd_damp` | double | `0.0` | 댐핑 계수, 진동 억제를 위해 증가시킴 |
| `kp_hold` | double | `0.0` | 위치 유지 스티프니스, 정지 시 위치 잠금을 위해 증가시킴 |
| `vel_hold_thresh` | double | `0.02` | 위치 유지를 트리거하는 속도 임계값 (rad/s) |
| `gdir` | array | `[0.0, -9.81, 0.0]` | 중력 벡터 |
| `verbose` | bool | `false` | 상세 토크 및 관절 정보 출력 여부 |


---

## 일반 정보

### 빌드
```bash
cd ~/openarmx_ws
colcon build --packages-select openarmx_teleop_bimanual
source install/setup.bash
```

### 의존성
- ROS2 Humble
- rclcpp, std_msgs
- openarmx_can
- openarmx_bringup (팔로워 측)

### 토픽 설명

#### 퍼블리시 토픽
| 토픽명 | 메시지 타입 | 설명 |
|---|---|---|
| `/right_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 팔로워 측 우측 팔 위치 명령 (8DOF) |
| `/left_forward_position_controller/commands` | `std_msgs/Float64MultiArray` | 팔로워 측 좌측 팔 위치 명령 (8DOF) |

### 안전 주의사항
⚠️ **사용 전 주의사항:**
1. **작업 공간**: 마스터 측과 팔로워 측 모두 충분한 작업 공간이 있는지 확인합니다.
2. **비상 정지 버튼**: 언제든지 비상 정지 버튼을 누를 수 있도록 준비합니다.
3. **팔로워 측 우선 실행**: 반드시 팔로워 측을 먼저 실행한 후 텔레오퍼레이션 노드를 실행합니다.
4. **천천히 이동**: 초기 테스트 시 마스터 측을 천천히 움직입니다.

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
