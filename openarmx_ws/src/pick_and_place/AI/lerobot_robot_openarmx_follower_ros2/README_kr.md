# lerobot_robot_openarmx_follower_ros2 한국어 설명

## 1. 패키지 소개

`lerobot_robot_openarmx_follower_ros2`는 LeRobot의 OpenArmX ROS2 어댑터 플러그인입니다.  
역할은 OpenArmX의 ROS2 토픽 인터페이스를 LeRobot의 `Robot` 인터페이스로 캡슐화하여, 데이터 수집, 재생, 정책 추론을 손쉽게 수행하도록 하는 것입니다.

핵심 기능:

1. `/joint_states`에서 듀얼암 관절 상태를 읽어 관측값으로 사용합니다.
2. 좌/우 컨트롤러 명령 토픽으로 `Float64MultiArray` 관절 목표값을 퍼블리시합니다.
3. ROS2 이미지 토픽을 통해 다중 카메라(RGB/Depth) 연동을 지원합니다.

## 2. 패키지 구조

```text
lerobot_robot_openarmx_follower_ros2/
├── README.md
├── README_CN.md
├── pyproject.toml
├── setup.py
└── lerobot_robot_openarmx_follower_ros2/
    ├── __init__.py
    ├── config_openarmx_ros2.py        # 설정 정의 (로봇 + ROS2 + 카메라)
    ├── openarmx_ros2.py               # LeRobot Robot 구현
    ├── ros2_interface_openarmx.py     # ROS2 관절 상태 구독/명령 퍼블리시 인터페이스
    └── ros2_camera.py                 # ROS2 이미지 토픽 카메라 구현
```

## 3. 데이터 흐름

1. ROS2 제어 시스템이 `/joint_states`를 퍼블리시합니다.
2. 본 패키지가 관절 상태를 읽어 LeRobot 관측값을 생성합니다.
3. LeRobot이 액션(각 관절의 목표값)을 제공합니다.
4. 본 패키지가 액션을 다음 토픽으로 퍼블리시합니다.
   - `/left_forward_position_controller/commands`
   - `/right_forward_position_controller/commands`
5. 선택 사항: `/cam_*/color/image`, `/cam_*/depth/image`에서 이미지를 읽어 비주얼 관측값으로 사용합니다.

## 4. 설치

워크스페이스 소스 디렉터리에서 설치합니다 (편집 가능 모드):

```bash
# lerobot 가상 환경 활성화
lerobot-env   # 이 명령은 lerobot 가상 환경을 활성화하기 위해 설정한 단축 명령입니다. conda가 로컬 python과 충돌하여 빌드가 실패할 수 있으므로, 기본적으로 conda를 초기화하지 않고 사용 시에만 단축 명령으로 활성화하도록 했습니다!

cd <워크스페이스 경로>/src/openarmx_vla/lerobot_robot_openarmx_follower_ros2
pip install -e . --no-deps
```

## 5. 주요 수정 가능 파라미터

## 로봇 전체 설정 `OpenArmXRos2Config`

| 파라미터 | 기본값 | 설명 |
|------|--------|------|
| `skip_send_action` | `True` | `True`인 경우 수집만 하고 액션을 전송하지 않습니다. `False`인 경우 실제로 제어 명령을 전송합니다 |
| `max_relative_target` | `None` | 한 스텝당 관절 변화 폭을 제한합니다 (안전 클리핑) |
| `ros2` | `OpenArmXRos2InterfaceConfig(...)` | ROS2 관절 인터페이스 파라미터 |
| `cameras` | 3채널 ROS2 카메라 | 카메라 설정 딕셔너리 (우측/좌측/헤드) |

## ROS2 관절 인터페이스 설정 `OpenArmXRos2InterfaceConfig`

| 파라미터 | 기본값 | 설명 |
|------|--------|------|
| `namespace` | `""` | ROS2 네임스페이스 |
| `joint_states_topic` | `/joint_states` | 관절 상태 입력 토픽 |
| `left_command_topic` | `/left_forward_position_controller/commands` | 좌측 팔 명령 토픽 |
| `right_command_topic` | `/right_forward_position_controller/commands` | 우측 팔 명령 토픽 |
| `left_joint_names` | 좌측 팔 8 관절명 | 좌측 팔 명령 벡터 순서 (컨트롤러와 반드시 일치해야 함) |
| `right_joint_names` | 우측 팔 8 관절명 | 우측 팔 명령 벡터 순서 (컨트롤러와 반드시 일치해야 함) |

주의: `left_joint_names`/`right_joint_names`의 순서는 전송되는 배열의 의미를 직접 결정하므로, 컨트롤러 설정과 일치하지 않으면 액션이 어긋날 수 있습니다.

## 카메라 설정 `Ros2CameraConfig`

| 파라미터 | 기본값 | 설명 |
|------|--------|------|
| `image_topic` | `/camera/color/image_raw` | RGB 이미지 토픽 |
| `use_depth` | `False` | 깊이 영상 활성화 여부 |
| `depth_topic` | `/camera/depth/image_raw` | 깊이 영상 토픽 |
| `fps` | 설정에서 지정 | 목표 프레임 레이트 (LeRobot 피처 기술용) |
| `width`/`height` | 설정에서 지정 | 이미지 해상도 |
| `color_mode` | `RGB` | 출력 색상 포맷 |
| `rotation` | `NO_ROTATION` | 이미지 회전 |
| `qos_reliability` | `best_effort` | QoS 신뢰성 (저지연) |
| `queue_size` | `1` | 버퍼 큐 길이 (최신 프레임 유지) |


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
