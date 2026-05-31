# OpenArm 그리퍼 제어 패널

[中文](README_CN.md) | [English](README.md) | **한국어**

## 개요

`openarmx_gripper_panel`은 OpenArm 로봇 시스템을 위한 RViz2 플러그인으로, 단일 또는 듀얼 그리퍼를 제어합니다. RViz에서 직관적인 그래픽 인터페이스를 제공하며, ROS2 action 명령을 통해 그리퍼 위치를 제어합니다.

## 기능 특성

- **듀얼 그리퍼 지원**: 좌측 그리퍼, 우측 그리퍼 또는 듀얼 그리퍼 동시 제어 가능
- **직관적인 그래픽 인터페이스**: 슬라이더 기반 위치 제어와 프리셋 단축 버튼 제공
- **실시간 피드백**: 연결 상태와 명령 상태의 시각적 인디케이터
- **RViz 통합**: RViz 패널로 매끄럽게 통합
- **설정 영속화**: 그리퍼 선택 환경 설정 저장

## 시스템 아키텍처

### 컴포넌트

- **RViz 플러그인**: Qt5 기반 GUI 패널, RViz2에 통합
- **Action 클라이언트**: `control_msgs/action/GripperCommand`를 통해 그리퍼 컨트롤러와 통신
- **컨트롤러 인터페이스**:
  - 좌측 그리퍼: `/left_gripper_controller/gripper_cmd`
  - 우측 그리퍼: `/right_gripper_controller/gripper_cmd`

### 제어 범위

- **위치 범위**: 0-44mm
- **프리셋 값**:
  - 닫힘: 0mm
  - 반쯤 열림: 22mm
  - 열림: 44mm
- **최대 힘**: 10.0N

## 설치

### 의존성

본 소프트웨어 패키지는 다음 의존성을 필요로 합니다.

- ROS2 (Humble 이상)
- `rclcpp`
- `rclcpp_action`
- `control_msgs`
- `rviz_common`
- `rviz_default_plugins`
- `pluginlib`
- Qt5 (Core, Widgets)

### 빌드

```bash
# 워크스페이스로 이동
cd ~/openarmx_ws

# 패키지 빌드
colcon build --packages-select openarmx_gripper_panel

# 워크스페이스 로드
source install/setup.bash
```

## 사용 방법

### 1. RViz 실행

패키지가 빌드되고 로드된 후, 그리퍼 패널이 RViz에서 자동으로 사용 가능해집니다.

```bash
# RViz 실행
ros2 run rviz2 rviz2

# 또는 moveit으로 실행
ros2 launch openarmx_bimanual_moveit_config demo.launch.py can_fd:=false
```

### 2. RViz에 패널 추가

1. RViz에서 **Panels** → **Add New Panel** 클릭
2. **openarmx_gripper_panel/GripperPanel** 선택
3. 패널이 RViz 창에 나타납니다

### 3. 그리퍼 제어

1. **대상 선택**: 드롭다운 메뉴에서 선택합니다.
   - 좌측 그리퍼
   - 우측 그리퍼
   - 듀얼 그리퍼 (동기)

2. **위치 설정**:
   - 슬라이더를 사용해 위치 조정 (0-44mm)
   - 또는 단축 프리셋 버튼을 클릭합니다.
     - **닫힘 (0mm)**: 완전 닫힌 위치
     - **반쯤 열림 (22mm)**: 반쯤 열린 위치
     - **열림 (44mm)**: 완전 열린 위치

3. **명령 실행**:
   - 녹색 **"적용 - 명령 실행"** 버튼 클릭
   - 상태 라벨에 명령 피드백이 표시됩니다

### 4. 상태 인디케이터

패널은 실시간 상태 정보를 표시합니다.

- 녹색: 명령 전송 성공 / 컨트롤러 준비됨
- 노란색: 하나의 그리퍼 컨트롤러가 연결되지 않음
- 빨간색: 모든 그리퍼 컨트롤러가 연결되지 않음
- 회색: 준비/유휴 상태

## 기술 세부사항

### ROS2 인터페이스

**Action 타입**: `control_msgs/action/GripperCommand`

**Action 목표 구조**:
```cpp
goal_msg.command.position = position;  // 목표 위치 (미터) (0.0-0.044)
goal_msg.command.max_effort = 10.0;    // 최대 힘 (뉴턴)
```

### 토픽 및 서비스

플러그인은 다음 action 서버에 연결됩니다.
- `/left_gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand)
- `/right_gripper_controller/gripper_cmd` (control_msgs/action/GripperCommand)

### 동기 제어

"듀얼 그리퍼 (동기)"를 선택한 경우:
1. 먼저 우측 그리퍼로 명령 전송
2. 5밀리초 지연
3. 좌측 그리퍼로 명령 전송
4. 이를 통해 시간 오프셋을 최소화하면서 거의 동시 실행을 보장합니다

### 설정

패널 설정은 RViz 설정 파일(.rviz)에 자동으로 저장됩니다.
- 그리퍼 선택 환경 설정
- 패널 위치 및 크기

## 플러그인 등록

플러그인은 pluginlib 메커니즘을 통해 RViz에 등록됩니다.

```xml
<library path="openarmx_gripper_panel">
  <class name="openarmx_gripper_panel/GripperPanel"
         type="openarmx_gripper_panel::GripperPanel"
         base_class_type="rviz_common::Panel">
    <description>
      OpenArm 그리퍼 제어 패널 - RViz에서 단일 또는 듀얼 그리퍼의 개폐 제어
    </description>
  </class>
</library>
```

## 트러블슈팅

### RViz에 패널이 나타나지 않는 경우

```bash
# 패키지가 빌드되어 있는지 확인
colcon build --packages-select openarmx_gripper_panel

# 워크스페이스 로드
source install/setup.bash

# 플러그인 등록 확인
ros2 pkg prefix openarmx_gripper_panel

# 플러그인 기술 파일 검증
cat install/openarmx_gripper_panel/share/openarmx_gripper_panel/plugins/plugin_description.xml
```

### 컨트롤러 연결 실패

1. 그리퍼 컨트롤러가 실행 중인지 확인:
```bash
ros2 action list | grep gripper
```

예상 출력:
```
/left_gripper_controller/gripper_cmd
/right_gripper_controller/gripper_cmd
```

2. 컨트롤러 상태 확인:
```bash
ros2 action info /left_gripper_controller/gripper_cmd
ros2 action info /right_gripper_controller/gripper_cmd
```

3. action 수동 테스트:
```bash
ros2 action send_goal /left_gripper_controller/gripper_cmd \
  control_msgs/action/GripperCommand \
  "{command: {position: 0.022, max_effort: 10.0}}"
```

### 패널에 경고 메시지가 표시되는 경우

- **"경고: 그리퍼 컨트롤러가 연결되지 않음"**: 두 컨트롤러 모두 사용 불가
  - 로봇 하드웨어가 연결되어 있는지 확인
  - 컨트롤러가 실행되었는지 확인

- **"경고: 좌측 그리퍼 컨트롤러가 연결되지 않음"**: 좌측 컨트롤러만 사용 불가
  - 우측 그리퍼는 여전히 제어 가능

- **"경고: 우측 그리퍼 컨트롤러가 연결되지 않음"**: 우측 컨트롤러만 사용 불가
  - 좌측 그리퍼는 여전히 제어 가능

## 개발

### 파일 구조

```
openarmx_gripper_panel/
├── CMakeLists.txt              # 빌드 설정
├── package.xml                 # 소프트웨어 패키지 메타데이터
├── README.md                   # 영문 문서
├── README_zh.md                # 중국어 문서
├── include/
│   └── openarmx_gripper_panel/
│       └── gripper_panel.hpp   # 헤더 파일
├── src/
│   └── gripper_panel.cpp       # 구현 파일
├── plugins/
│   └── plugin_description.xml  # 플러그인 등록
└── resource/
    └── openarmx_gripper_panel  # 리소스 마커
```

### 주요 클래스

**GripperPanel**: 메인 패널 클래스, `rviz_common::Panel` 상속
- 사용자 상호작용을 위한 Qt 슬롯 함수
- 그리퍼 제어를 위한 ROS2 action 클라이언트
- 설정 저장/로드 기능

### 소스 코드에서 빌드

```bash
# 저장소 클론 (워크스페이스에 없는 경우)
cd ~/openarmx_ws/src/openarmx_tools/

# 의존성 설치
rosdep install --from-paths . --ignore-src -r -y

# 빌드
cd ~/openarmx_ws
colcon build --packages-select openarmx_gripper_panel --cmake-args -DCMAKE_BUILD_TYPE=Release

# 로드 및 테스트
source install/setup.bash
ros2 run rviz2 rviz2
```

## 라이선스

본 저작물은 Creative Commons 저작자표시-비영리-동일조건변경허락 4.0 국제 라이선스 (CC BY-NC-SA 4.0)에 따라 이용 허락됩니다.

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)

자세한 내용은 [LICENSE_kr.md](LICENSE) 파일을 참조하시거나 다음 링크를 방문하시기 바랍니다: http://creativecommons.org/licenses/by-nc-sa/4.0/

## 작성자

- **Wei Lindong** (魏林栋)
- 회사: Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
- 웹사이트: https://openarmx.com/

## 버전

1.0.0

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
