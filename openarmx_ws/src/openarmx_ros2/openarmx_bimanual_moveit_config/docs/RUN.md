# openarmx_bimanual_moveit_config 실행 명령

MoveIt 2 + Pilz 산업용 모션 플래너 (직선 보간 LIN / 점대점 PTP / 원호 CIRC) 가 활성화된 양팔 OpenArmX 설정.

## 시뮬레이션 기동 (mock 컨트롤러, 하드웨어 불필요)

```bash
source /opt/ros/humble/setup.bash && source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash && ros2 launch openarmx_bimanual_moveit_config demo_sim.launch.py
```

- `use_fake_hardware:=true` 가 기본값
- RViz + MoveIt `move_group` + 양팔 관절 궤적 제어기 + 그리퍼 제어기 + 두 planning pipeline (`ompl`, `pilz_industrial_motion_planner`) 모두 자동 기동
- 약 5초 안에 RViz 가 떠오며 양팔 모델이 보임

## 실 하드웨어 기동 (CAN 통신 + 모터 연결 필수)

```bash
source /opt/ros/humble/setup.bash && source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash && ros2 launch openarmx_bimanual_moveit_config demo.launch.py
```

- 내부 `check_motor_status()` 가 모터 각도 검사. 영점에서 ±30° 이상 벗어나면 abort
- CAN 인터페이스 인자: `right_can_interface:=can0 left_can_interface:=can1` (기본값)
- 다른 옵션: `arm_type:=v10`, `control_mode:=mit`, `can_fd:=false`, `enable_forward_effort:=false`

## RViz 에서 Pilz LIN (직선 보간) 사용 절차

기동된 RViz 좌측 **MotionPlanning** 패널:

1. **Context** 탭 → **Planning Pipeline** dropdown → `pilz_industrial_motion_planner`
2. **Context** 탭 → **Planner ID** dropdown → `LIN` (직선) / `PTP` (관절 보간) / `CIRC` (원호)
3. **Planning** 탭 → **Planning Group** dropdown → `left_arm` 또는 `right_arm`
4. RViz 안 6 자유도 인터랙티브 마커 (적·녹·청 링과 화살표) 드래그 → goal 자세 설정
5. **Plan** → 직선 궤적 미리보기 → **Execute**

## 양팔 동시 운영

Pilz 는 단일 그룹 planning 만 지원합니다. 양팔 동시 LIN 은 불가. 시나리오:

- **순차**: 오른팔 plan/execute → 왼팔 plan/execute (RViz 에서 그룹만 바꿔가며)
- **비동기 독립**: 각 팔별 thread 에서 `MoveGroupCommander` 인스턴스 분리. 두 팔이 자기 페이스로 동작 (작업 공간 분리 가정)

## 프로그래밍 호출 예시 (Python `moveit_commander`)

```python
from moveit_commander import MoveGroupCommander

mg = MoveGroupCommander("right_arm")
mg.set_planning_pipeline_id("pilz_industrial_motion_planner")
mg.set_planner_id("LIN")
mg.set_pose_target(target_pose)  # geometry_msgs/Pose, frame_id = openarmx_right_link0 권장
mg.go(wait=True)
```

## 운영 시 주의

- LIN 의 goal 자세 frame_id 는 **arm base** (`openarmx_right_link0` 또는 `openarmx_left_link0`) 권장. `world` 사용 시 KDL 역기구학이 자주 `NO_IK_SOLUTION (-31)` 반환
- 짧은 이동 (수 cm) 부터 테스트. 큰 이동은 경로 중간 충돌·관절 한계 검사로 실패 가능
- Pilz 는 가속도 한계 필수 — `config/joint_limits.yaml` 의 `has_acceleration_limits: true` 가 모든 가동 관절에 설정되어 있어야 함 (이미 설정됨)
- RViz 의 default 가 OMPL — Pilz 쓰려면 매번 dropdown 으로 전환 필요
- 첫 기동 시 `Semantic description is not specified for the same robot as the URDF` 경고는 무해. 동작에 영향 없음
