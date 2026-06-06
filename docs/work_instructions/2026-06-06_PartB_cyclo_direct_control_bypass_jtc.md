# 작업 지시서 — Part B: cyclo MoveL 직접 제어 (JTC 미사용)

> 대상 워크스페이스: **China 전용** (`/home/openarmx/TR-Works/kkw/China`). jsy 절대 금지.
> 작성: 2026-06-06 세션. 실행: 다른 세션.

---

## 1. 목적 / 근거

**cyclo MoveL 컨트롤러는 JTC(Joint Trajectory Controller)를 거치지 않고 하드웨어를 직접(position passthrough) 제어하도록 전환한다.**

근거 (단 하나, 구조적):
- cyclo(`omx_movel_controller_node`)는 제어루프에서 **매 틱(100Hz)마다 새 관절 위치를 계산해 명령하는 위치 제어 컨트롤러**다.
- 이렇게 매 틱 계산한 위치 명령은 **가공·재해석 없이 하드웨어 position에 그대로 전달돼야** 한다.
- 그런데 JTC(Joint Trajectory Controller)는 *완성된 궤적을 받아 시간에 걸쳐 실행하는 트래젝토리 실행기*다. 매 틱 위치 명령을 JTC에 넣는 것은 **용도 불일치**다.
- 따라서 cyclo는 JTC를 거치지 않고 **하드웨어 position을 직접 제어**한다.

> 참고: 이번 세션의 임시책 A(`batch_trajectory=true`)는 MoveL을 endpoint 1점으로만 발행해 매-틱 위치제어 자체를 버린 우회책이다(cyclo의 실시간 Cartesian 제어를 잃음). B(직접 제어)가 정석이며, B 적용 시 `batch_trajectory=false`로 매-틱 위치제어를 복원한다.

---

## 2. 방식

ros2_control에서 외부 노드(cyclo)는 하드웨어 명령 인터페이스에 직접 못 쓰므로, **위치 passthrough 컨트롤러**를 다리로 둔다. JTC 대신 이 컨트롤러가 cyclo의 매-틱 위치를 재계획 없이 하드웨어에 그대로 쓴다.

- 컨트롤러: `position_controllers/JointGroupPositionController` (또는 `forward_command_controller/ForwardCommandController`, interface=position). 받은 값을 매 update마다 그대로 명령 인터페이스에 write. **궤적 해석·재계획 없음.**
- cyclo 출력: `JointTrajectory`→JTC 대신 `std_msgs/Float64MultiArray`(7관절 위치)를 이 컨트롤러의 `/commands`로 발행.

---

## 3. 구현 단계

### 3-1. 위치 passthrough 컨트롤러 추가
`openarmx_ws/src/openarmx_ros2/openarmx_bimanual_moveit_config/config/ros2_controllers.yaml`:
```yaml
controller_manager:
  ros__parameters:
    left_arm_position_controller:
      type: position_controllers/JointGroupPositionController
    right_arm_position_controller:
      type: position_controllers/JointGroupPositionController

left_arm_position_controller:
  ros__parameters:
    joints: [openarmx_left_joint1, ..., openarmx_left_joint7]
    interface_name: position
right_arm_position_controller:
  ros__parameters:
    joints: [openarmx_right_joint1, ..., openarmx_right_joint7]
    interface_name: position
```
- 하드웨어 launch(컨트롤러 스폰)에 추가. position 인터페이스를 JTC와 공유 못 하므로 **둘 중 하나만 active**(아래 3-3).

### 3-2. cyclo 출력 전환
`omx_movel_controller_node.{hpp,cpp}`:
- 파라미터 `output_mode`(`"jtc"`|`"forward"`, 기본 `"forward"`), `forward_command_topic`(`/{side}_arm_position_controller/commands`).
- `output_mode=="forward"`이면 매 틱 `publishTrajectory(q)`가 `std_msgs/Float64MultiArray`(7관절 위치 q)를 `forward_command_topic`으로 발행. (`<std_msgs/msg/float64_multi_array.hpp>` 추가, 퍼블리셔 타입 분기.)
- `batch_trajectory=false`로 두어 매-틱 스트리밍 복원(직접 제어라 그대로 전달됨). batch(A)는 jtc 모드 fallback으로 유지.
- cyclo launch(`openarmx_movel_bimanual.launch.py`)에서 `output_mode:=forward`, `forward_command_topic` 주입.

### 3-3. JTC와의 공존 (position 인터페이스 1개뿐)
position 명령 인터페이스를 JTC와 passthrough가 동시 점유할 수 없다. cyclo·pilz·joint_control·MoveIt이 컨트롤러를 공유하는 구조이므로:
- **cyclo(Cartesian) 사용 시**: `{side}_arm_position_controller` active / `{side}_joint_trajectory_controller` inactive.
- **joint_control·MoveIt·pilz 사용 시**: 반대로.
- 백엔드 선택부(scenario_player/UI의 backend 전환)에서 `controller_manager/switch_controller`(STRICT)로 전환. 전환 실패 시 롤백.
- (단순화 옵션: cyclo 전용 운용이면 passthrough만 active로 두고 JTC는 미사용. 단 MoveIt/pilz를 쓰려면 전환 필요.)

### 3-4. 빌드
```bash
cd /home/openarmx/TR-Works/kkw/China/openarmx_ws
colcon build --packages-select cyclo_motion_controller_ros openarmx_bimanual_moveit_config --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## 4. 검증
SIL(`use_fake_hardware:=true`) 선검증 후 HIL.
1. cyclo MoveL 실행 시 **매-틱 위치 명령이 그대로 반영돼** 매끄럽고 정확하게 이동.
2. **매-틱 Cartesian 직선 경로 유지**(A의 관절공간 직선과 달리). 큰 이동(예: 5cm)에서 rosbag FK로 EE 경로가 직선인지 확인.
3. 컨트롤러 전환(cyclo↔JTC) 정상, idle 시 팔 hold.
4. 회귀: joint_control·MoveIt·pilz가 JTC로 정상 동작.

> 게인은 팩토리 A2 유지(j1~4 KP50/KD2.5, j5~7 KP10/KD0.5).

---

## 5. 주의 / 완료기준
- position 인터페이스 **동시 점유 금지**(전환 누락=충돌). STRICT 전환 + 실패 롤백.
- HIL 하드웨어 launch 재기동은 모터 재인에이블 동반 → 팔 근처 안전 확인. SIL 선검증.
- A(`batch_trajectory`)와 B(`output_mode=forward`)는 상호 배타: B 쓰면 batch=false.

완료기준:
- [ ] passthrough 컨트롤러 2개 추가·스폰
- [ ] cyclo `output_mode=forward`로 매-틱 위치 직결 발행
- [ ] cyclo↔JTC 백엔드 전환 동작
- [ ] HIL MoveL: 매-틱 위치명령 그대로 반영 + Cartesian 직선 유지
- [ ] joint_control·MoveIt·pilz 회귀 정상
- [ ] docs/issues_and_fixes 기록, 커밋은 사용자 지시 후

---

## 부록 — Part A 메모 (별도)
pick_and_place 새 패키지(YOLOv8 위치추정 → Cartesian 최종위치 MoveL). **pick 자세 ≠ place 자세**(각각 구분해 발행). 입력 `/detected_boxes`(PoseArray, base), 출력 `/openarmx/{side}/movel`. 참조: `openarmx_cyclo_box_align/box_align_node.py` `move_arm()`.
