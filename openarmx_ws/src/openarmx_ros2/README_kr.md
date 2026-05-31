# OpenArmX ROS 2 코어 라이브러리

[English](README.md) | [简体中文](README_CN.md) | 한국어

OpenArmX의 기반 ROS 2 패키지입니다. `openarmx_description`(URDF/xacro/mesh)과 함께 사용하면 매니퓰레이터의 기본 모델링과 모션 제어를 구현할 수 있습니다.

## 포함 내용
- `openarmx`: 메타 패키지로, 코어 컴포넌트를 통합합니다.
- `openarmx_hardware`: `ros2_control` 하드웨어 플러그인 `openarmx_hardware/OpenArmX_v10HW`로, ` `을 통해 매니퓰레이터와 그리퍼를 구동합니다.
- `openarmx_bringup`: launch 파일, RViz 설정, 그리퍼 조작 가이드.
- `openarmx_bimanual_moveit_config`: 듀얼암 MoveIt 설정으로, `openarmx_description`에 의존합니다.
- `openarmx_preview_bringup`: 로봇 관절 모션 제어 패키지.
- 추가 저장소: `openarmx_minimal.repos`(설명 패키지만 포함) 또는 `openarmx.repos`(텔레오퍼레이션, 도구, 파라미터 매니저 포함).
- `openarmx-can_1.0.0_amd64.deb`: 모터 드라이버. 빌드 전에 모터 드라이버를 먼저 설치해야 합니다.

## 환경 요구사항
- Ubuntu 22.04, ROS 2 Humble.
- 빌드: `colcon`, `ament_cmake`, C++17 툴체인.
- ROS 의존성: `rclcpp`, `pluginlib`, `hardware_interface`/`ros2_control`, 그리고 MoveIt(MoveIt 설정용).
- 시스템: SocketCAN 지원 (`can-utils` 설치 권장, 스크립트용으로 `python-can` 선택 가능).
- 실기: Robstride 모터가 CAN(기본값 `can0`)을 통해 접근 가능해야 합니다.

## 워크스페이스 준비
```bash
# vcs 도구 설치
sudo apt-get install python3-vcstool -y

mkdir -p ~/openarmx_ws/src && cd ~/openarmx_ws/src
git clone https://github.com/openarmx/openarmx_ros2.git
# 필수 설명 패키지 또는 전체 옵션 패키지 가져오기
vcs import < openarmx_ros2/openarmx_minimal.repos
# 또는: vcs import < openarmx_ros2/openarmx.repos
rosdep install --from-paths . --ignore-src -r -y
```

## OpenArmX Can 설치
```bash
sudo dpkg -i openarmx-can_1.0.0_amd64.deb
```

## 빌드
```bash
cd ~/openarmx_ws
colcon build
source install/setup.bash
```

## 실행 예시
```bash
# 실기 실행, 원클릭 실행 스크립트
/home/openarmx/openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/run_bimanual_moveit_with_can2.0.sh
# 시뮬레이션 모드
/home/openarmx/openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/run_bimanual_moveit_sim.sh
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

**현재 버전**: 6.0.0

## 감사의 말

본 패키지는 OpenArmX 로봇 플랫폼 생태계의 일부이며, 협동 로봇 분야의 연구 및 산업 응용을 위해 개발되었습니다.

---

## 📞 문의

### Chengdu Changshu Robot Co., Ltd. (成都长数机器人有限公司)
**Chengdu Changshu Robotics Co., Ltd.**

| 연락처 | 정보 |
|---------|------|
| 📧 이메일 | openarmrobot@gmail.com |
| 📱 전화/WeChat | +86-17746530375 |
| 🌐 공식 웹사이트 | <https://openarmx.com/> |
| 🌐 문서 | <http://docs.openarmx.com/> |
| 📍 주소 | 천진 경제기술개발구 서구 신예팔가 11호 화성기계공장 |
| 👤 담당자 | Mr. Wang |
