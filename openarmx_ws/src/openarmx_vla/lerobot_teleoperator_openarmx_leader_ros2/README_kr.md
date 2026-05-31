# lerobot_teleoperator_openarmx_leader_ros2 한국어 설명

## 1. 패키지 소개

`lerobot_teleoperator_openarmx_leader_ros2`는 LeRobot의 OpenArmX 리더(Leader) 측 텔레오퍼레이션 플러그인입니다.  
OpenArmX VR 체인 내의 ROS2 관절 명령 토픽을 구독하여, LeRobot 표준 action 딕셔너리(`*.pos`)로 변환합니다.

핵심 기능:

1. 좌/우 팔의 `Float64MultiArray` 명령 입력을 구독합니다.
2. 관절 순서에 따라 LeRobot action 키로 매핑합니다 (예: `openarmx_left_joint1.pos`).
3. LeRobot 제어 루프에서 사용할 `get_action()`을 제공합니다. 직접 로봇 제어 명령을 전송하지는 않습니다.

## 2. 패키지 구조

```text
lerobot_teleoperator_openarmx_leader_ros2/
├── README.md
├── README_CN.md
├── pyproject.toml
└── lerobot_teleoperator_openarmx_leader_ros2/
    ├── __init__.py
    ├── config_openarmx_ros2.py      # 설정 정의 (ROS2 토픽 + 관절 순서)
    ├── openarmx_ros2.py             # LeRobot Teleoperator 구현
    └── ros2_interface_openarmx.py   # ROS2 구독 인터페이스 (명령/관절 상태/grip)
```

## 3. 데이터 흐름

1. 상류의 teleop 노드가 좌/우 팔 목표 관절 배열을 퍼블리시합니다 (일반적으로 `*_commands_original`).
2. 본 패키지가 좌/우 토픽을 구독하고 최신 명령을 캐시합니다.
3. `get_action()` 호출 시 LeRobot 포맷의 액션 딕셔너리를 출력합니다.
4. 명령이 수신되지 않은 경우, `/joint_states`의 현재 자세를 액션으로 폴백 사용합니다 (정지 상태 유지).
5. 동시에 `/pico_left_controller/grip`, `/pico_right_controller/grip`을 구독하여 사람 개입 이벤트를 감지합니다.

## 4. 설치

워크스페이스 소스 디렉터리에서 설치합니다 (편집 가능 모드):

```bash
# lerobot 가상 환경 활성화
lerobot-env   # 이 명령은 lerobot 가상 환경을 활성화하기 위해 설정한 단축 명령입니다. conda가 로컬 python과 충돌하여 빌드가 실패할 수 있으므로, 기본적으로 conda를 초기화하지 않고 사용 시에만 단축 명령으로 활성화하도록 했습니다!

cd <워크스페이스 경로>/src/openarmx_vla/lerobot_teleoperator_openarmx_leader_ros2
pip install -e . --no-deps
```

## 5. 주요 수정 가능 파라미터

## Teleoperator 전체 설정 `OpenArmXRos2TeleopConfig`

| 파라미터 | 기본값 | 설명 |
|------|--------|------|
| `ros2` | `OpenArmXRos2TeleopInterfaceConfig(...)` | ROS2 구독 인터페이스 파라미터 |

## ROS2 구독 인터페이스 설정 `OpenArmXRos2TeleopInterfaceConfig`

| 파라미터 | 기본값 | 설명 |
|------|--------|------|
| `namespace` | `""` | ROS2 네임스페이스 |
| `left_command_topic` | `/left_forward_position_controller/commands_original` | 좌측 팔 입력 명령 토픽 |
| `right_command_topic` | `/right_forward_position_controller/commands_original` | 우측 팔 입력 명령 토픽 |
| `left_joint_names` | 좌측 팔 8 관절명 | 좌측 팔 입력 배열 순서 (퍼블리시 측과 반드시 일치해야 함) |
| `right_joint_names` | 우측 팔 8 관절명 | 우측 팔 입력 배열 순서 (퍼블리시 측과 반드시 일치해야 함) |

주의: `left_joint_names`/`right_joint_names`의 순서는 action의 각 관절 매핑 관계를 직접 결정하므로, 상류 퍼블리시 순서와 일치하지 않으면 관절이 어긋날 수 있습니다.

## 고정 구독 토픽 (현재 구현)

다음 토픽은 현재 코드에서 고정값으로 설정되어 있습니다 (설정 파라미터 아님):

1. `/joint_states`: 명령이 누락된 경우의 폴백 데이터 소스.
2. `/pico_left_controller/grip`: 좌측 grip, 개입 감지에 사용.
3. `/pico_right_controller/grip`: 우측 grip, 개입 감지에 사용.

## 6. 최소 사용 예시

```python
from lerobot.teleoperators import make_teleoperator

teleop = make_teleoperator(
    {
        "type": "openarmx_ros2",
        "ros2": {
            "left_command_topic": "/left_forward_position_controller/commands_original",
            "right_command_topic": "/right_forward_position_controller/commands_original",
        },
    }
)

teleop.connect()
action = teleop.get_action()  # dict: { "<joint>.pos": value, ... }
events = teleop.get_teleop_events()  # 사람 개입 여부 포함
teleop.disconnect()
```

## 7. 자주 묻는 질문

1. `get_action()`이 데이터 없음 오류를 보고하는 경우  
먼저 상류에서 `left/right_command_topic`을 퍼블리시하고 있는지 확인합니다. 퍼블리시되지 않는 경우, `/joint_states`의 존재 여부를 다시 확인합니다.

2. 액션 관절이 잘못 대응되는 경우  
`left_joint_names`/`right_joint_names`의 순서가 상류 배열 순서와 완전히 일치하는지 확인합니다.

3. 개입 감지가 계속 False인 경우  
`/pico_left_controller/grip`과 `/pico_right_controller/grip`에 데이터가 있는지 확인합니다. 임계값 로직은 `> 0.5`로 개입을 판정합니다.


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
