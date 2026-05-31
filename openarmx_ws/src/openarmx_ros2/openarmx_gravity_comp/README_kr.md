# OpenArmX 중력 피드포워드 보상

[English](README.md) | [中文](README_CN.md) | [한국어](#개요)

---

### 개요

본 패키지는 OpenArmX 듀얼암 로봇을 위한 실시간 중력 피드포워드 보상 기능을 제공합니다. MIT 모션 컨트롤 모드에서 순수 PD 제어는 중력으로 인해 약 6°의 정상 상태 위치 오차를 발생시키는데, 본 패키지는 모터의 `τ_ff`(피드포워드 토크)에 KDL로 실시간 계산된 중력 보상 토크를 주입하여 오차를 1° 이내로 줄입니다.

### 구현 원리

#### 문제 배경

MIT 모드에서 모터 제어 공식은 다음과 같습니다.

```
τ_output = kp × (pos_cmd - pos_actual) + kd × (vel_cmd - vel_actual) + τ_ff
```

`τ_ff = 0` (순수 PD 제어)일 때, 중력이 정상 상태 오차를 유발합니다.

```
정상 상태 오차 = τ_gravity / kp ≈ 5.3 / 50 ≈ 0.106 rad ≈ 6°
```

#### 해결 방안

`gravity_comp_node`는 `/joint_states`를 구독하고, 관절 각도를 수신할 때마다 KDL 재귀 뉴턴-오일러 알고리즘을 사용해 각 관절의 중력 보상 토크를 실시간으로 계산한 후, `forward_command_controller`를 통해 하드웨어의 `τ_ff`에 작성합니다.

**피드포워드 토크는 고정값이 아니며**, 매니퓰레이터의 자세에 따라 실시간으로 변화합니다. 관절이 펼쳐졌을 때 중력 모멘트 암이 가장 길어 토크가 최대가 되고, 관절이 접혔을 때 토크가 감소합니다.

#### 전체 데이터 체인

```
/joint_states  (sensor_msgs/JointState, 14개 관절 위치 포함)
        │
        ▼
gravity_comp_node
  ├─ 이름으로 좌측 팔 7 관절 각도 q[7] 검색
  │       ↓
  │   Dynamics::GetGravity(q, tau_g)
  │     └─ KDL::ChainDynParam::JntToGravity()  ← 재귀 뉴턴-오일러 알고리즘
  │       ↓
  │   tau_out[j] = clamp(g_scale × tau_g[j], ±TAU_LIMITS[j])
  │       ↓
  │   /left_forward_effort_controller/commands  (Float64MultiArray)
  │       ↓
  │   left_forward_effort_controller
  │     └─ effort command interface에 작성 → tau_commands_[i]
  │       ↓
  │   v10_simple_hardware.cpp  write()
  │     └─ param.torque = tau_commands_[i] × direction_multipliers[i]
  │       ↓
  │   MIT CAN 패킷 → 좌측 팔 모터 τ_ff
  │
  └─ 우측 팔 동일 → /right_forward_effort_controller/commands → 우측 팔 모터
```

#### 중력 방향 규약

매니퓰레이터 base는 X축을 중심으로 ±90° 회전하여 장착되므로, 월드 좌표계의 중력 `[0, 0, -9.81]`이 회전 후 각 팔의 link0 좌표계에서 Y 방향이 됩니다.

| 팔 | link0에서의 실제 중력 벡터 | 코드 설정값 |
|----|---------------------|-----------|
| 우측 팔 | `[0, +9.81, 0]` | `RIGHT_ARM_GY = -9.81` |
| 좌측 팔 | `[0, -9.81, 0]` | `LEFT_ARM_GY  = +9.81` |

코드 설정값이 실제 방향과 반대인 이유는 하드웨어 `write()`가 일괄적으로 `-1`을 곱하므로, 두 번의 부호 반전을 통해 최종적으로 방향이 올바르게 되기 때문입니다. 노드 내부에서 별도의 방향 보정은 필요하지 않습니다.

#### 안전 클리핑 (TAU_LIMITS)

이상 자세에서 피드포워드 출력이 과도해지는 것을 방지합니다.

| 관절 | 모터 모델 | 현재 상한값 |
|------|---------|---------|
| joint1, 2 | RS04 | 20 Nm |
| joint3, 4 | RS03 | 7 Nm |
| joint5, 6, 7 | RS00 | 2 Nm |

---

### 패키지 구조

```
openarmx_gravity_comp/
├── include/
│   └── dynamics.hpp          # Dynamics 클래스 선언 (KDL 다이내믹스 래핑)
├── src/
│   ├── dynamics.cpp          # KDL 다이내믹스 구현 (중력, 코리올리, 관성, 자코비안)
│   └── gravity_comp_node.cpp # ROS2 노드 본체, 관절 상태 구독 및 피드포워드 토크 퍼블리시
├── CMakeLists.txt
├── package.xml
├── GRAVITY_COMP_NOTES.md     # 상세 설계 설명 (부호 규약, 오차 분석 등)
└── README_CN.md
```

**Dynamics 클래스 주요 인터페이스:**

| 메서드 | 설명 |
|------|------|
| `Init()` | URDF를 파싱하고 KDL 모션 체인을 구축하며, 다이내믹스 솔버를 생성 |
| `SetGravityVector(gx, gy, gz)` | 중력 벡터 설정 (link0 좌표계 기준) |
| `GetGravity(q, tau_g)` | 각 관절의 중력 보상 토크 계산 |
| `GetColiori(q, q_dot, tau_c)` | 코리올리 토크 계산 |
| `GetMassMatrixDiagonal(q, diag)` | 관절 공간 관성 행렬의 대각 요소 획득 |
| `GetJacobian(q, J)` | 자코비안 행렬 계산 |
| `GetNullSpace(q, N)` | 영공간 투영 행렬 계산 |
| `GetEECordinate(q, R, p)` | 전방 키네매틱스, 말단 포즈 획득 |

---

### 의존성

- ROS 2 Humble
- `orocos_kdl`
- `kdl_parser`
- `urdf` / `urdfdom`
- `Eigen3`
- `forward_command_controller` (`openarmx_bringup` 또는 `openarmx_bimanual_moveit_config`에서 실행)

---

### 설치 및 빌드

```bash
colcon build --packages-select openarmx_gravity_comp openarmx_bringup
source install/setup.bash
```

---

### 사용 방법

본 패키지의 노드는 bringup 또는 MoveIt demo launch 파일에서 통합 관리되며, `enable_forward_effort` 파라미터로 활성화 여부를 제어합니다. **기본값은 비활성화입니다.**

#### 방법 1: Bringup을 통한 실행

텔레오퍼레이션, 티칭 등 MoveIt이 필요하지 않은 시나리오에 적합합니다.

**중력 보상 비활성화 (기본값):**

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller
```

**중력 보상 활성화:**

```bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    right_can_interface:=can0 \
    left_can_interface:=can1 \
    control_mode:=mit \
    robot_controller:=forward_position_controller \
    enable_forward_effort:=true
```

#### 방법 2: MoveIt Demo를 통한 실행

MoveIt으로 모션 플래닝을 수행하는 시나리오에 적합합니다.

**중력 보상 비활성화 (기본값):**

```bash
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit
```

**중력 보상 활성화:**

```bash
ros2 launch openarmx_bimanual_moveit_config demo.launch.py \
    control_mode:=mit \
    enable_forward_effort:=true
```

#### 정상 동작 확인

```bash
# effort controller가 활성화되었는지 확인
ros2 control list_controllers | grep effort

# 실시간 피드포워드 토크 출력 확인
ros2 topic echo /left_forward_effort_controller/commands
ros2 topic echo /right_forward_effort_controller/commands
```

정상 출력 예시 (영점 부근):

```
data: [16.1, 15.9, 0.0, 5.0, 0.0, 0.0, 0.0]
```

---

### 주요 파라미터

#### g_scale (런타임 조정 가능)

KDL 계산은 URDF 관성 파라미터를 기반으로 하며 실물과 편차가 존재합니다. `g_scale`은 전체적인 스케일링에 사용됩니다.

| 값 | 효과 |
|----|------|
| < 1.0 | 과소 보상, 관절이 여전히 아래로 편향됨 |
| 1.0 | 이론적인 완전 보상 |
| > 1.0 | 과보상, 관절이 위로 떠오름 |

**현재 최적값: `1.05`, 오차 < 1°.**

런타임에 노드 재시작 없이 조정 가능합니다.

```bash
ros2 param set /gravity_comp_node g_scale 1.05
ros2 param get /gravity_comp_node g_scale
```

#### 노드 파라미터 요약

| 파라미터 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `urdf_path` | string | 필수 | URDF 파일 경로 (launch 파일이 자동으로 `/tmp/v10_bimanual_gravity.urdf`에 작성) |
| `g_scale` | double | `1.05` | 중력 토크 전체 스케일 계수 |
| `enable_left` | bool | `true` | 좌측 팔 보상 활성화 여부 |
| `enable_right` | bool | `true` | 우측 팔 보상 활성화 여부 |
| `verbose` | bool | `false` | 각 관절의 실시간 토크 출력 여부 (1초 throttle) |

---

### 잔여 오차 분석

| 단계 | 오차 |
|------|------|
| 중력 보상 없음 (순수 PD) | ~6.4° (0.111 rad) |
| g_scale = 0.975 | ~3.5° |
| g_scale = 1.05 | < 1° |

잔여 오차의 주요 원인:

1. **URDF 관성 부정확**: `inertials.yaml`의 질량과 질량 중심 위치는 설계값입니다. `g_scale`은 전체 스케일링만 가능하며, 각 관절의 상대 오차를 보정할 수는 없습니다.
2. **정상 상태 오차 공식**: `정상 상태 오차 = τ_residual / kp`. 오차를 2°에서 1°로 줄이려면 `τ_residual < 0.87 Nm`이 필요합니다.

추가 오차 감소 방향:
- `g_scale` 계속 미세 조정
- `kp` 증가 (진동 방지를 위해 `kd`도 함께 증가)
- `inertials.yaml`의 관성 파라미터 캘리브레이션 (가장 근본적)

---

## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Li Yongqi** (李永旗)
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
