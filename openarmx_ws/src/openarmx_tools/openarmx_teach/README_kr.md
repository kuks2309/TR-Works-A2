# openarmx_teach | 궤적 녹화 및 재생 도구

[English](README.md) | [简体中文](README_CN.md) | 한국어

본 패키지는 **녹화**와 **재생** 두 종류의 스크립트를 제공하며, OpenArmX 듀얼암 + 듀얼 그리퍼 시나리오를 대상으로 합니다. 녹화 단계에서는 `/joint_states`에서 샘플링하여 통합 YAML을 생성합니다. 재생 단계에서는 관절 명명에 따라 자동으로 분할하여 좌우 팔의 `FollowJointTrajectory`와 그리퍼의 `GripperCommand` Action으로 각각 전송하며, 피드백 기반의 그리퍼 동기화도 선택할 수 있습니다.

---

## 기능 개요
- **장시간 녹화**: `record_joint_states_always`가 정해진 주기로 `/joint_states`를 샘플링하며, 수동으로 시작/일시정지/저장이 가능합니다.
- **다중 컨트롤러 병렬 재생**: `play_joint_trajectory`가 좌우 팔과 좌우 그리퍼를 자동으로 그룹핑하여 동시에 실행합니다.
- **관절 필터링**: 지정 관절 리스트, 좌측 팔, 우측 팔, 듀얼암 또는 전체 관절을 지원합니다.
- **속도 스케일링**: `--rate-scale`로 궤적 전체를 가속/감속할 수 있습니다.
- **그리퍼 지능형 스케줄링**: 위치 평균값을 스칼라로 매핑하고, 미세 변화에 대한 노이즈 제거/압축을 수행합니다. 사용 가능한 팔의 피드백 시간으로 그리퍼 동작을 트리거할 수 있습니다 (`--sync-feedback`/`--sync-margin`).
- **명명 규칙 자동 인식**: 기본적으로 `openarmx_left_joint*` / `openarmx_right_joint*` / `openarmx_left_finger*` / `openarmx_right_finger*`에 의존합니다.

## 의존성 및 빌드
워크스페이스에서 실행합니다.
```bash
colcon build --packages-select openarmx_teach
source install/setup.bash
```
필요한 의존성: `rclpy`, `control_msgs`, `trajectory_msgs`, `PyYAML` 등 (ROS 2/의존 패키지와 함께 설치됨).

## 일반적인 워크플로우
1. 하드웨어/시뮬레이션과 해당 컨트롤러를 실행합니다 (예: bringup 또는 moveit).
2. 새 터미널을 열어 녹화를 진행합니다: 동작 진입 → `SPACE` 시작 → `SPACE` 일시정지 → `w` 저장.
3. 재생: 먼저 `--all-joints`로 검증한 후, 필요에 따라 단일 팔이나 단일 그리퍼로 필터링합니다. 필요 시 `--sync-feedback`을 추가해 그리퍼 동기화를 개선합니다.
4. "Warning: Joint 'xxx' not found"가 발생하면, YAML의 `joint_names`가 현재 컨트롤러의 관절명과 일치하는지 확인합니다.

## 1단계: Moveit 또는 bringup 실행
```bash
# can 채널 활성화
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

# Moveit 실행
ros2 launch openarmx_bimanual_moveit_config demo.launch.py

# bringup 실행
ros2 launch openarmx_bringup openarmx.bimanual.launch.py
```

## 2단계: pid 조정

본 패키지의 KP KD 조정 패널을 사용하시면 로봇의 pid를 손쉽게 조정하여 강성을 낮춰 후속 녹화를 편리하게 할 수 있습니다.

버튼 클릭 **Panels** --> **Add New Panel** --> **KPKDPanel** --> **OK**

모든 관절의 KP KD를 0으로 조정합니다 (기본값이 0이므로 별도 조작이 필요 없을 수 있습니다)

**모든 관절에 KP/KD 적용** 버튼(녹색 버튼)을 클릭하면 pid 설정이 완료됩니다

## 3단계: 녹화 — record_joint_states_always
명령 예시:
```bash
ros2 run openarmx_teach record_joint_states_always --rate 20
# 출력 파일명 커스텀:
ros2 run openarmx_teach record_joint_states_always --rate 10 --outfile demo.yaml
```
기본 출력 파일명: `joint_states_stream_YYYYMMDD_HHMMSS.yaml`. 샘플링 주파수 `--rate` (Hz)가 `time_from_start`의 증가 스텝을 결정합니다 (`(i+1)*dt`).

키보드 조작:
- `SPACE` / `p`: 시작/일시정지
- `c`: 현재 캐시 초기화 (확인 필요)
- `w`: 저장 후 종료 (확인 필요)
- `q`: 저장 없이 종료

주의: `joint_names`는 첫 메시지의 순서를 따르며, 이후 각 프레임은 해당 순서로 정렬됩니다.

## 4단계: pid 조정

녹화 단계를 완료한 상태라면 다음 재생 단계로 진행할 수 있습니다. 다만 이전 단계에서 pid를 0으로 조정했으므로, 그대로 재생하면 로봇이 아무런 동작을 하지 않습니다 (모터에 힘이 없음).

따라서 pid를 기본 상태나 더 높은 값으로 복원해야 합니다 (강성이 높을수록 로봇이 녹화된 경로에 더 가깝게 동작합니다).

모든 **기본값 복원** 버튼을 클릭한 다음 **모든 관절에 KP/KD 적용** 버튼(녹색 버튼)을 클릭하면 pid 설정이 완료되며, 이후 재생을 진행할 수 있습니다.

**주의: pid가 작을 때는 (기본 kp kd 포함. 안전을 위해 로봇의 기본 kp kd는 비교적 작게 설정되어 있습니다) 로봇이 녹화된 경로를 매우 정확하게 추종하지는 않습니다. 모터의 힘이 충분치 않기 때문입니다. pid를 키우거나, 위치 모드(보다 정밀한 모드이지만 로봇의 하중이 낮음(약 3kg). 하중 운동 시 모터가 손상될 수 있음)를 직접 사용해 보시기 바랍니다.**

## 재생: play_joint_trajectory

다중 컨트롤러 (기본값):
```bash
ros2 run openarmx_teach play_joint_trajectory <record.yaml> --all-joints --rate-scale 0.5
```
기본 사용 action 이름:
- 좌측 팔: `/left_joint_trajectory_controller/follow_joint_trajectory`
- 우측 팔: `/right_joint_trajectory_controller/follow_joint_trajectory`
- 좌측 그리퍼: `/left_gripper_controller/gripper_cmd`
- 우측 그리퍼: `/right_gripper_controller/gripper_cmd`

자주 사용하는 필터/스케줄링 파라미터:
- `--left-arm` / `--right-arm` / `--both-arms` / `--all-joints`
- `--joints <list>`: 커스텀 관절 서브셋
- `--rate-scale f`: 시간 스케일링 (>1 가속, <1 감속)
- `--sync-feedback`: 팔 피드백 시간으로 그리퍼 스케줄링 구동
- `--sync-margin m`: 피드백 시간 + m ≥ 목표 시간일 때 그리퍼 트리거, 약간 앞당기는 효과
- `--action <name>`: 단일 컨트롤러 모드. 이름에 `gripper`가 포함되면 `GripperCommand`를 전송

단일 컨트롤러 예시 (좌측 그리퍼만):
```bash
ros2 run openarmx_teach play_joint_trajectory <record.yaml> \
  --action /left_gripper_controller/gripper_cmd \
  --joints openarmx_left_finger_joint1 openarmx_left_finger_joint2
```

## YAML 포맷 예시
```yaml
joint_names: [openarmx_left_joint1, openarmx_left_joint2, ...]
points:
  - positions: [0.1, 0.2, ...]
    time_from_start: 0.1   # 초, 녹화 주파수에 따라 증가
```

## 사용 주의사항
- 녹화된 `time_from_start`는 샘플링 주파수에서 유도되며, 실제 실행 타임스탬프가 아닙니다. 컨트롤러 시작/지연이 큰 경우 `--sync-margin`을 적절히 늘리거나 `--rate-scale`을 조정해야 합니다.
- 녹화 주파수가 너무 낮으면 궤적이 희소해지고, 너무 높으면 파일이 커지면서 그리퍼에는 큰 의미가 없습니다.
- 그리퍼는 전송 전에 미세하거나 과밀한 변화를 압축하므로, 명확한 개폐 동작이 더 잘 재현됩니다.
- 재생 전에 action server가 실행 중이고 이름이 일치하는지 확인합니다. 그렇지 않으면 바로 실패합니다.

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
