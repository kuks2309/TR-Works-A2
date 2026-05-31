# 빌드 상태 & 배포

## 빌드 결과 (2026-04-23 기준)

| 항목 | 값 |
|---|---|
| 마지막 빌드 일자 | 2026-04-23 15:30:40 |
| 분석 시점 경과 | 약 21일 (재빌드 권장) |
| ROS2 배포판 | Humble |
| 빌드 결과 | 16/16 패키지 성공, 에러 0 |
| 워크스페이스 클린 상태 | ✓ |

## 빌드된 패키지 (16개)

```
lerobot_robot_openarmx_follower_ros2
openarmx
openarmx_bimanual_moveit_config
openarmx_bringup
openarmx_description
openarmx_gravity_comp
openarmx_gripper_panel
openarmx_hardware
openarmx_joint_slider_panel
openarmx_kp_kd_panel
openarmx_lerobot
openarmx_preview_bringup
openarmx_teach
openarmx_teleop_bimanual
openarmx_teleop_bridge_vr
openarmx_teleop_vr
```

## 소스 19개 vs 빌드 16개 차이

`src/` 디렉토리에는 19개의 package.xml이 있으나 install에는 16개만 존재. 차이는:
- `openarmx_teleop_vr_apk` — ROS 패키지 아님 (APK 바이너리 폴더)
- `openarmx_motor_manager` — PySide6 데스크톱 앱 (ROS 아님)
- `lerobot_teleoperator_openarmx_leader_ros2` — VLA에서 참조하나 위치 불명 (소스 vs 외부 의존성 확인 필요)

## 빌드 로그 위치

- 가장 최근: `openarmx_ws/log/build_2026-04-23_15-30-40/`
- 이전: `openarmx_ws/log/build_2026-04-23_15-30-06/`

## 배포 프로파일 (.repos)

모든 .repos 파일은 `version: 6.0_basic`, `type: git` (GitHub 호스팅).

### openarmx_minimal.repos
- `openarmx_description`
- **용도:** 최소 설정, 로봇 모델 시각화

### openarmx.repos
- `openarmx_description`
- `openarmx_teleop_bimanual` — leader-follower 텔레오프
- `openarmx_tools` — RViz 패널 + teach
- `openarmx_motor_manager` — 모터 파라미터 설정
- **용도:** 표준 풀 셋업

### openarmx_vr.repos
- 위의 모든 것 +
- `openarmx_teleop_vr`
- `openarmx_teleop_vr_apk`
- **용도:** VR 텔레오프 추가

### openarmx_vla.repos
- 위의 모든 것 +
- `openarmx_vla`
- **용도:** VLA (Vision-Language-Action) 풀 스택

## CAN 드라이버 (DEB 패키지)

**파일:**
- `openarmx-can_1.0.0_amd64.deb`
- `openarmx-can_1.0.0_arm64.deb`

**역할:** OpenArmX 모터(Robstride RS04/RS03/RS00) 전용 CAN 드라이버.
**중요:** **colcon build 전에 반드시 설치** 필요.

```bash
sudo dpkg -i openarmx-can_1.0.0_amd64.deb   # x86_64
sudo dpkg -i openarmx-can_1.0.0_arm64.deb   # ARM (Jetson 등)
```

## 표준 설치 절차

```bash
# 1. vcs 도구 설치
sudo apt-get install python3-vcstool -y

# 2. 워크스페이스 생성
mkdir -p ~/openarmx_ws/src && cd ~/openarmx_ws/src

# 3. 코어 리포 클론
git clone https://github.com/openarmx/openarmx_ros2.git

# 4. 의존성 임포트 (프로파일 선택)
vcs import < openarmx_ros2/openarmx_minimal.repos    # 또는
vcs import < openarmx_ros2/openarmx.repos            # 또는
vcs import < openarmx_ros2/openarmx_vr.repos         # 또는
vcs import < openarmx_ros2/openarmx_vla.repos

# 5. rosdep 의존성
rosdep install --from-paths . --ignore-src -r -y

# 6. CAN 드라이버 설치 (필수!)
sudo dpkg -i openarmx_ros2/openarmx-can_1.0.0_amd64.deb

# 7. 빌드
cd ~/openarmx_ws && colcon build

# 8. 환경 변수
source install/setup.bash
```

## 빠른 실행 명령

### 실제 로봇 (양팔)
```bash
~/openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/run_bimanual_moveit_with_can2.0.sh
```

### 시뮬레이션
```bash
~/openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/run_bimanual_moveit_sim.sh
# 또는 use_fake_hardware:=true
```

### 특정 도구만 빌드
```bash
colcon build --packages-select \
  openarmx_joint_slider_panel \
  openarmx_gripper_panel \
  openarmx_kp_kd_panel \
  openarmx_teach
```
