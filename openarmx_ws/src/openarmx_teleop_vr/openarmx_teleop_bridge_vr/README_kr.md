# openarmx_teleop_bridge_vr

## 1. 이 패키지는 무엇을 합니까

`openarmx_teleop_bridge_vr`은 ROS 2 브릿지 패키지입니다.
VR 컨트롤러가 UDP로 전송한 데이터를 수신하여 ROS 2 토픽으로 퍼블리시합니다 (TF 퍼블리시 선택 가능). 이를 통해 하류의 텔레오퍼레이션이나 제어 노드가 직접 사용할 수 있습니다.

한 문장으로: **VR 컨트롤러 데이터를 ROS 2로 끌어옵니다.**

## 2. 패키지 구조

```text
openarmx_teleop_bridge_vr/
├── README_CN.md
├── README.md
├── CMakeLists.txt
├── package.xml
└── src/
    └── openarmx_teleop_bridge_vr_node.cpp
```

## 3. 애플리케이션 계층 데이터 흐름

VR/OpenXR 송신 측
-> UDP 데이터 (기본 포트 `5100`)
-> `openarmx_teleop_bridge_vr_node`  
-> ROS 2 토픽 (pose/trigger/grip/button/rate)  
-> 사용자 teleop 또는 제어 노드

## 4. 빠른 사용법

### 빌드

```bash
cd <워크스페이스 경로>
colcon build --packages-select openarmx_teleop_bridge_vr
```

### 실행

```bash
source install/setup.bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node
```

### 데이터 수신 확인

```bash
ros2 topic echo /vr_left_controller/pose
ros2 topic echo /vr_right_controller/pose
```

## 5. 기본 퍼블리시 토픽

| 번호 | 종류 | 토픽명 |
|------|------|--------|
| 1 | 포즈 | `/vr_left_controller/pose` |
| 2 | 포즈 | `/vr_right_controller/pose` |
| 3 | 트리거 | `/vr_left_controller/trigger` |
| 4 | 트리거 | `/vr_right_controller/trigger` |
| 5 | 그립 | `/vr_left_controller/grip` |
| 6 | 그립 | `/vr_right_controller/grip` |
| 7 | 비율 | `/vr_left_controller/rate` |
| 8 | 비율 | `/vr_right_controller/rate` |
| 9 | 버튼 | `vr_right_controller/button_a` |
| 10 | 버튼 | `vr_right_controller/button_b` |
| 11 | 버튼 | `vr_left_controller/button_x` |
| 12 | 버튼 | `vr_left_controller/button_y` |

## 6. 자주 사용하는 파라미터 (애플리케이션 계층)

| 파라미터명 | 설명 | 기본값 |
|--------|------|--------|
| `listen_address` | 수신 주소 | `0.0.0.0` |
| `listen_port` | 수신 포트 | `5100` |
| `publish_tf` | TF 퍼블리시 여부 | `false` |
| `frame_id` | 포즈 퍼블리시 시 부모 좌표계 | `vr_hmd` |

예시 (포트를 변경하고 TF를 활성화):

```bash
ros2 run openarmx_teleop_bridge_vr openarmx_teleop_bridge_vr_node \
  --ros-args -p listen_port:=5101 -p publish_tf:=true
```

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
