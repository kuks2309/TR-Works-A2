# 2026-06-06 — raised-arm(+0.54 m) 재캘리브 미전파 연쇄 + cyclo MoveL 과도응답

2026-06-05 raised-arm 재캘리브(arm_base z 0.735→1.275, +0.54 m)가 **여러 다운스트림 산출물에 전파되지 않아** 연쇄 증상이 발생. 일부는 해결, cyclo MoveL '미친 모션'은 조사 진행 중(이 세션은 사용자 퇴근으로 SIL 자율 진행 인계).

> 작업 워크스페이스: **China(`/home/openarmx/TR-Works/kkw/China`) 전용.** 실행 스택이 jsy 워크스페이스로 떠 있던 적이 있으나(혼합 환경), 수정은 China 에만 적용. jsy 를 임시 수정했다가 전량 원복함(claude-mistake 2026-06-06 참조).

---

## 2026-06-06 00:20 (KST) — 검출됐는데 RViz 박스 마커 안 보임

### 증상
원격 Hailo 검출은 정상(노란 박스 2개)인데 RViz `/detected_boxes_markers` 에 아무것도 안 뜸. 노드 로그는 매 검출마다 `detected 0 box(es)`.

### 원인
`box_perception_node` 의 워크스페이스 z-필터가 모든 박스를 버림. 오늘 raised-arm +0.54 m 로 카메라/작업면이 올라가 박스 윗면이 base 기준 z≈0.78 m 인데, 필터 `ws_z` 는 옛 설정 `[0.10, 0.32]` 그대로라 전량 REJECT → `DELETEALL` 마커만 발행. 라이브 재현(depth+TF+camera_info)으로 box0 z=0.782, box1 z=0.779 확인, x·|y|는 통과·z만 실패.

### 수정
[3d_detect_ws/.../box_perception_node.py](../../3d_detect_ws/src/yolov8_detection/yolov8_detection/box_perception_node.py) `ws_z` 기본값 `[0.10, 0.32]` → `[0.64, 0.86]` (이전값 +0.54 m). ws_x/ws_y_abs 는 수평이라 불변. 빌드 후 `detected 22 box(es)` 확인, RViz 큐브 표시 확인(스크린샷). **커밋 보류.**

### 재발 방지
raised/lowered 구성 변경 시 +0.54 m 영향 받는 모든 z-기준 파라미터를 동시 점검(체크리스트 하단).

---

## 2026-06-06 00:20 (KST) — cyclo Cartesian jog 가 "UNREACHABLE: no_convergence" 로 안 움직임

### 증상
Cartesian Control → backend=cyclo → +Z 등 jog 시 UI 상태줄 `cyclo ... UNREACHABLE: no_convergence @ waypoint 0/10`, 명령이 cyclo 로 발행조차 안 됨. 사용자: "충분히 도달 가능한 거리/위치임".

### 원인
UI 의 사전 도달성 체크([ik_check.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/ik_check.py) `LinearReachabilityChecker`)가 **stale 솔버 URDF** 로 IK 를 풀어 모든 라이브 포즈에서 수렴 실패. 격리 검증: 솔버 모델 FK(tcp) z@q=0 = **0.057**, 라이브 TF = **0.597** → **정확히 0.540 m(=raise) 차이**. 즉 사전체커 모델이 0.54 m 낮음. (pinocchio/numpy/IK 알고리즘은 자기일관 IK 0회 수렴으로 정상 확인 — 모델만 틀림.) 게이트가 결과기반 차단(fail-closed)이라 jog 봉쇄. 솔버 URDF 출처: `body_link0→link0` joint origin z=0.735 (xacro SSOT `v10.urdf.xacro` arm_base default 는 1.275).

### 수정
1. **솔버 URDF 재생성(정석):** 현재 xacro(`ros2_control:=false bimanual:=true`) expand → `gen_solver_urdf.py --no-collision` → [openarmx_{left,right}_solver.urdf](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_pick/urdf/) base z 0.735→**1.275**. FK z@q=0 0.057→0.597 라이브 일치 확인. (생성기 입력이던 옛 expansion `openarmx_motion/urdf/openarmx_bimanual_solver.urdf`, `openarmx_description/.../openarmx_robot.urdf` 도 0.735 stale — 본 수정은 openarmx_pick 솔버만, 나머지는 별도 점검 대상.)
2. **ik_check seed clamp:** [ik_check.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/ik_check.py) `numerical_ik` 진입 시 `q = np.clip(q, q_lo, q_hi)`. 라이브 joint4 가 캘리브상 하한 0.0 을 미세하게 밑돌(-0.002) 때 seed-한계위반으로 전체 모션을 헛막던 것 방지(현재 로봇자세는 정의상 유효). in-range/경계 시드 reachable=True 확인, +Z ±50/100 mm 전부 통과.
**둘 다 커밋 보류.**

### 재발 방지
- 솔버 URDF 는 xacro SSOT 에서 `gen_solver_urdf.py` 로 재생성(손수 z 박지 말 것). raise/lower 시 재생성 필수.
- 사전체커는 모델≠라이브면 가짜 UNREACHABLE 을 낼 수 있음 → 게이트 fail-open 정책 재검토 여지(설계 주석도 "사전체크가 jog 를 막으면 절대 안 된다" 명시).

---

## 2026-06-06 01:30 (KST) — cyclo MoveL '미친 모션' (✅ 해결·SIL 검증)

### 증상
SIL/실하드웨어 모두에서, cyclo MoveL(jog 또는 AlignToBoxes) 시 팔이 **목표와 무관하게 크게 휘둘렸다가** 목표 부근으로 수렴. AlignToBoxes 는 `success:true` 지만 `wait:timeout`·`err_mm`~1000 (참고: [box_align_node.py:315](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_cyclo_box_align/openarmx_cyclo_box_align/box_align_node.py#L315) 가 timeout 도 success=True 로 보고 — 별도 결함).

### 원인 (SIL 컨트롤러 로깅으로 측정·확정)
**bimanual launch의 cyclo 게인이 공격적(고 kp·저 damping)이라 velocity-resolved QP가 qdot를 폭주시킴.**

- [openarmx_movel_bimanual.launch.py](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_pick/launch/openarmx_movel_bimanual.launch.py) (실행본)은 **upstream cyclo 기본값** `kp_position=50, kp_orientation=50, weight_damping=0.001, trajectory_time=0` 사용. 반면 단일팔 [openarmx_movel.launch.py](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_pick/launch/openarmx_movel.launch.py)는 안정화 튜닝 `kp=4, kp_ori=2.5, damping=0.05, traj_time=0.05`.
- 측정(SIL, 컨트롤러 per-tick 로깅, 사용자 시작 config [36.6,−31.7,33,96.2,−35.6,31.9,38.3]에서 원본 MoveL 재생): 거의-제로 델타(5 mm)인데 **dvel 0.35→4.5 상승하는 동안 qdot가 6.4 rad/s까지 폭주**, cz가 goal 반대로 갔다(perr 증가) 복귀 → **EE 102 mm 이탈**(= fly). 저-manipulability 방향 증폭(1/min_sv≈7×) × kp=50 × damping≈0 = 정규화 없는 폭주. (관절한계 근처 config(INIT)에선 한계 CBF가 이 폭주를 throttle → 반대로 undershoot. 같은 뿌리.)
- code 자체는 upstream과 동일(분기 버그 아님). **순수 게인 튜닝 문제.**
- 진단 중 발견한 함정: 컨트롤러 재기동 누적으로 controller_manager의 JTC가 언로드되면 cyclo 명령이 실행 안 됨 → arm/cyclo decouple로 오인하기 쉬움. `ros2 control list_controllers`로 JTC active 먼저 확인할 것. (복구: `ros2 run controller_manager spawner joint_state_broadcaster left/right_joint_trajectory_controller -c /controller_manager`)

### 수정 (✅ SIL 검증)
[openarmx_movel_bimanual.launch.py](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_pick/launch/openarmx_movel_bimanual.launch.py) 좌/우 게인을 단일팔 튜닝값과 일치: `kp_position 50→4, kp_orientation 50→2.5, weight_damping 0.001→0.05, trajectory_time 0→0.05`. **검증(동일 config·동일 명령): EE 이탈 102 mm→8 mm, 최대 qdot 6.4→2.26, fly 소멸. 정상 −Z 100 mm 추종도 정상(91% 부드럽게).** 컨트롤러 코드는 진단용 임시 로깅만 추가했다 제거(코드 변경 없음). **커밋 보류.**

### 재발 방지
- cyclo 게인은 단일팔/양팔 launch가 **동일 튜닝값을 공유**하도록(중복 정의 금지·하드코딩 주의). upstream 기본값 회귀 금지.
- AlignToBoxes 결과의 success 판정을 `wait==arrived`로 보정(별도 결함).
- pilz backend는 cyclo와 다른 경로(MoveIt→JTC)라 이 게인 이슈와 무관할 것이나, 동일 과도 여부는 별도 확인 권장.

### 재발 방지
- AlignToBoxes 결과의 success 판정을 `wait==arrived`로 보정(별도 결함).
- 검증은 INIT 포즈에서 다양한 MoveL 로.

---

## 부록 — raise(+0.54 m) 영향 z-기준 산출물 체크리스트
- [x] box_perception `ws_z` (검출 마커)
- [x] 솔버 URDF `body_link0→link0` z (cyclo 컨트롤러 + ik_check 공용)
- [x] ik_check seed clamp
- [ ] AlignToBoxes `z` 파라미터 기본값 0.4 (→ ~0.94 추정, 미적용)
- [ ] `openarmx_bimanual_solver.urdf`, `openarmx_robot.urdf` 옛 expansion(0.735) 정리
- [x] cyclo MoveL 과도응답 (bimanual launch 게인 튜닝, SIL 검증 — fly 102→8 mm)
- [ ] pilz MoveL 동일 여부

---

## 2026-06-06 (오후) — cyclo MoveL '미친 모션' 진짜 근본원인: 왼팔 j1 관절한계 −120° offset 버그 (✅ 실측·SIL 검증)

### 정정
앞 절(01:30)에서 '게인 문제'로 결론냈으나, 그것은 SIL(Software In the Loop, mock=완벽추종) 한정 증상이었다. **하드웨어에서 +Z jog 시 팔이 미러 자세로 폭주**하는 진짜 원인은 따로 있었다.

### 증상
하드웨어에서 cyclo(cyclo MoveL backend) +Z jog 시, 명령 궤적부터 틀렸다. INIT(왼 j1=+50°)에서 +Z인데 cyclo가 **j1을 −48°(거의 −현재값, 미러)로 명령** → 팔이 휘둘리고 종료 후 한계진동/joint7 wind-up.

### 원인 (관절별 cmd vs act 시계열 + 솔버 URDF 대조로 확정)
- 관절별 명령 분석: cyclo의 **target joint 명령 자체가 미러 config**(j1 −48 vs 실제 +48, j4 +50 vs +99). 추종 문제가 아니라 명령 생성 오류.
- 솔버 URDF(Unified Robot Description Format) FK/Jacobian/IK는 정상(±Z 오프라인 시뮬 깨끗). → 미러는 **관절한계 CBF(Control Barrier Function)** 에서 발생.
- 실측(모터 disable 후 손 sweep, `/dynamic_joint_states`): 실제 왼 j1 가동 = **−193.5°~+86.6°**. 그런데 솔버 URDF 한계 = **[−191.6, +51.9°]** → **상한 52°가 실제 87°보다 35° 좁음.** INIT(50°)이 상한 코앞이라 +Z(j1 증가 필요)를 한계 CBF가 막고 QP(Quadratic Programming)가 미러 해로 튕김.
- **근본:** [openarmx_arm.xacro:35](../../openarmx_ws/src/openarmx_description/urdf/arm/openarmx_arm.xacro#L35) 가 왼팔 joint1 한계에만 **하드코딩 offset `-2.094396`(−120°)** 를 적용 → canonical [−1.25,3.0]을 [−3.344,0.906](상한 52°)로 시프트. 현재 하드웨어는 이 offset 없는 canonical 가동범위(90°+)를 쓰는데 솔버만 −120° 시프트되어 불일치. joint control은 URDF 한계를 안 쓰므로(MIT 직접) 정상, cyclo CBF만 걸림.

### 수정 (정석: SSOT 수정 → 재생성)
실측을 10°단위·안전 inward 반올림해 SSOT(Single Source of Truth) 수정:
1. [joint_limits.yaml](../../openarmx_ws/src/openarmx_description/config/arm/v10/joint_limits.yaml) canonical 갱신 (j1 [−80,+190], j2 ±90, j3 [−90,+100], j4 [0,+110], j5 ±90, j6 ±40, j7 ±80 deg).
2. [openarmx_arm.xacro:35](../../openarmx_ws/src/openarmx_description/urdf/arm/openarmx_arm.xacro#L35) 왼 j1 offset **−2.094396 → −1.9198622**(−120°→−110°).
3. v10.urdf.xacro(bimanual:=true, arm_base 1.275) 전개 → `gen_solver_urdf.py`로 [openarmx_{left,right}_solver.urdf](../../openarmx_ws/src/pick_and_place/cyclo/openarmx_pick/urdf/) 재생성. 좌우 14개 한계 목표와 정확 일치, base z=1.275 유지 확인.
4. **검증(격리 도메인 88 SIL, 실 cyclo QP+CBF + mock):** INIT에서 +Z MoveL → **j1 명령 +48.7~+53.8°로 정상 증가(미러 없음)**, j4 99→108°. 이전 미러(−48°) 폭주 소멸.

병행: 모터 MIT 모드 **KP 170→160** 하향([v10_simple_hardware.cpp:177](../../openarmx_ws/src/openarmx_ros2/openarmx_hardware/src/v10_simple_hardware.cpp#L177), 사용자 요청, 영구).

### 측정 방법(재사용)
모터 토크 off = `/openarmx_{left,right}_hardware_params` 의 `kp_joint1~8`/`kd_joint1~8` 동적 파라미터를 0으로 set(엔코더 read 유지, 손 sweep 가능). 복원은 kp 160·kd 2.5. 실측은 `/joint_states`(UI 가짜) 아닌 **`/dynamic_joint_states`**(broadcaster=실 엔코더)로. TM 로봇 도메인 오염 시 TM을 다른 ROS_DOMAIN_ID로 분리.

### 재발 방지
- 좌우 비대칭 관절한계(offset/reflect)는 **실제 하드웨어 가동범위와 반드시 대조**. 솔버 CBF는 URDF 한계를 강제하므로 한계가 좁으면 정상 jog가 미러로 튄다.
- 하드웨어 검증 전 **격리 도메인 SIL**(mock+실 cyclo)로 명령 궤적 먼저 검증.
- 잔여: 우 j6는 sweep 부족(+24°만)이라 미러값 +40 사용 — 추후 재측정 권장. pilz도 동일 한계 사용하니 영향 확인.

---

## 2026-06-06 (오후 2차) — cyclo 하드웨어 wild/퀀텀점프의 진짜 근본: UI의 /joint_states dual-publish → cyclo 관절매핑 손상 (✅ 정석 수정)

### 정정·확장
앞 절(한계 offset)은 실재 결함이고 SIL(Software In the Loop)에서 미러를 해소했으나, **하드웨어에서만 나는 wild/퀀텀점프**의 진짜 원인은 따로였다. 한계는 2차 요인.

### 증상
하드웨어에서 +Z jog 시 cyclo가 **첫 명령부터 INIT과 무관한 wild config**(예: 오른팔 INIT j4=100°인데 명령 j4≈0°, j5≈47°, j6≈−49°)를 내고, 모터가 그 큰 점프를 실행하다 안전차단(퀀텀점프·모터해제). SIL(Software In the Loop)에선 정상.

### 원인 (SIL vs 하드웨어 bag 비교 + 라이브로 확정)
- 관절별 cmd 비교: **SIL(Software In the Loop) 명령은 INIT 유지(j1=−50,j4=100,j7=−50), 하드웨어 명령은 첫 틱부터 wild.** 같은 cyclo 코드·같은 솔버 한계인데 다름 → 입력/내부상태 차이.
- 입력은 정상: 하드웨어 `/joint_states`·`/dynamic_joint_states` 둘 다 INIT(라이브 j4=98.3° 일정, 오염값 없음).
- 그런데 **cyclo 내부 ee_pose가 FK(wild)에 frozen**(라이브 179mm 어긋남). cyclo만 재기동하면 **0mm 추종**(누적/latch 손상 확정).
- 발행자 확인: 하드웨어 `/joint_states` 발행자가 **joint_state_broadcaster + scenario_ui(UI) 둘.** UI는 SIL(Software In the Loop) 시각화용으로 발행하는데(`publish_joint_states`/`_on_sil_tick` 20Hz), 하드웨어에선 런타임 가드(`traj_subscriber_count()>0`)에만 의존해 **기동 초기(JTC 구독자 등록 전) 창에서 발행이 샌다.**
- **결정 메커니즘:** cyclo `jointStateCallback`은 **첫 /joint_states 메시지로 `joint_index_map_`(이름→인덱스)를 단 한 번만 latch.** UI 발행 순서(`left1..7,right1..7` 순차)와 broadcaster 순서(`right1,right3,left_finger1,left7,left2...` 뒤섞임)가 **다름.** cyclo가 기동 시 UI 메시지를 먼저 latch하면, 이후 broadcaster 메시지를 잘못된 인덱스로 읽어 **q_가 엉뚱한 관절값으로 손상** → 시작자세 틀림 → wild 명령. SIL(Software In the Loop)은 발행자 1개(순서 일정)라 안 남.

### 수정 (정석: UI 분리 원칙)
UI는 view/명령 계층이어야 하고, **로봇 상태(/joint_states)는 SIL(Software In the Loop)/HIL(Hardware In the Loop) 모두 joint_state_broadcaster(SIL=fake_components, HIL=실하드웨어)가 단일 출처**여야 한다(차이는 하드웨어 플러그인 하나뿐).
1. [scenario_action_client.py](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/scenario_action_client.py): `/joint_states` 발행자 `_js_pub` **삭제**, `publish_joint_states` **no-op**. self-echo 필터는 `_last_self_publish`=None으로 자동 무력.
2. SIL(Software In the Loop)도 **broadcaster(L1 컨트롤러 스폰 preset)를 항상 띄워** /joint_states를 broadcaster가 내게 한다. (install은 src 심링크라 빌드 불필요, UI 프로세스만 재시작.)
3. cyclo의 `joint_index_map_` latch-once는 잠재적 취약점 — 다중 발행자/순서변화에 견고하도록 **매 메시지 `msg->name`으로 인덱싱**하게 고치면 근본적으로 더 안전(차후).

### 검증
재시작 후 `ros2 topic info /joint_states -v` 발행자가 **joint_state_broadcaster 하나만**이면 성공. cyclo 내부 ee_pose가 실제 팔 FK와 0mm 추종.

### 재발 방지
- **UI는 절대 로봇 상태(/joint_states 등)를 발행하지 않는다.** 상태는 ros2_control(fake/real)+broadcaster 단일 출처.
- /joint_states 발행자는 항상 1개여야 한다(다중발행자 = 구독자 매핑 손상 위험).
- SIL(Software In the Loop) bring-up은 L0(fake_components)+broadcaster를 함께 올린다.

---

## 2026-06-06 16:00 (KST) — cyclo MoveL '점프' 근본원인 확정: 100Hz 스트리밍 → JTC 재계획 자유낙하 (게인 무죄)

### 결론 한 줄
cyclo MoveL '미친 모션/점프'의 진짜 원인은 **cyclo가 100Hz로 단발(single-point) JointTrajectory 를 JTC(Joint Trajectory Controller) 에 스트리밍**하는 것. JTC 는 매 수신마다 **(중력에 처지는) 실측 위치에서 궤적을 재계획**하므로, 명령이 떨어지는 팔을 따라 내려가 **자유낙하 + 명령 종료 시 튕김(snap-back)** 이 발생. **모터 게인(KP/KD) 은 원인이 아님.**

### 실험적 증명 (HIL, 좌팔 −Z 10mm, rosbag /tmp/cyclo_*)
1. **cyclo 무죄(replay)**: 녹화된 cyclo→JTC 명령을 cyclo 없이 그대로 replay → 동일하게 점프. 명령 자체는 매끄러움(j4 97→91° 완만). 실행단 문제 확정.
2. **완료이벤트 무죄(truncate)**: 명령을 0.6s 에서 끊어도 그 지점에서 점프 → 점프는 '명령 멈춤 지점'을 따라옴.
3. **속도/대역폭 무죄(감속)**: 2배·4배 느리게 해도 점프(4배는 점프↓·심한 진동). 감쇠는 속도 무관.
4. **게인 무죄 + 방향 반증**: 팩토리 A2 게인(KP 50/10)으로 원복하니 오히려 더 악화(−Z 에서 j4 가 명령보다 32°/72mm 더 자유낙하 후 복귀). 낮은 KP일수록 중력에 더 떨어짐.
5. **결정타(전달방식 대조, 동일 팩토리 게인)**:
   - 100Hz 스트리밍(100개) → j4 가 **60°까지 자유낙하** 후 91 로 튕김.
   - **단일 명령 1개**(dur 1s) → j4 **87.6°로 매끄럽게 정착, 자유낙하 없음**.
   - 다중점 궤적 1개(전체 100점) → 동일하게 매끄러움.

### 메커니즘
JTC 는 **트래젝토리 실행기**(완성된 궤적을 받아 시간에 걸쳐 실행)이지 고율 setpoint sink 가 아님. 단발 궤적을 100Hz 로 흘리면 매번 활성 궤적을 교체·재계획하고, (open-loop 가 아닌) 기본 동작상 **실측 상태에서 출발**하므로, 중력에 처진 실측을 따라 명령이 내려감 → 자유낙하. 단일/다중점 1회 발행은 계획대로 끝점까지 끌어주므로 자유낙하 없음.

### 수정 A (구현·기본 적용) — cyclo 가 endpoint 1점만 발행
[omx_movel_controller_node.cpp](../../openarmx_ws/src/cyclo_robot_controller/cyclo_motion_controller_ros/src/nodes/omx/omx_movel_controller_node.cpp)
- 파라미터 `batch_trajectory`(기본 true) 추가. true 면 `moveLCallback` 에서 `precomputeAndPublishTrajectory()` 호출:
  - KinematicsSolver 에 역기구학(IK) 이 없으므로, control loop 와 동일한 cubic+QP(Quadratic Programming) 를 open-loop 로 목표까지 적분해 **endpoint(목표 관절각)** 를 구함(경로점은 버림).
  - `makeJointTrajectoryMsg(names, duration, q_endpoint)` 로 **단일점**(time_from_start=duration) 1회 발행. JTC 가 현재→endpoint 를 duration 에 걸쳐 보간.
  - `movel_trajectory_active_=false` 로 매-틱 스트리밍을 끔(원본 코드는 보존, false 면 기존 100Hz 동작).
- 한계: 단일 endpoint 보간이라 **관절공간 직선**(Cartesian 직선 아님). 박스 pick 의 작은 이동엔 무해. 정확한 Cartesian 직선이 필요하면 다중점 1회 발행으로 확장 가능.

### 수정 B (보류·기록 — 원칙적 해법)
cyclo 는 QP(Quadratic Programming)+CBF(Control Barrier Function) 를 매 틱 푸는 **실시간 스트리밍 컨트롤러**이므로, 트래젝토리 실행기 JTC 가 아니라 **forward_command_controller(위치 직결)** 에 setpoint 를 흘리는 것이 ros2_control 정석.
- `ros2_controllers.yaml` 에 `{left,right}_arm_forward_position_controller`(`forward_command_controller/ForwardCommandController`, interface=position, 7 팔관절) 추가. 받은 명령을 재계획 없이 매 틱 그대로 하드웨어 position 에 씀 → 스트리밍해도 자유낙하 없음.
- cyclo 출력을 `Float64MultiArray`→`/{arm}_arm_forward_position_controller/commands` 로 전환.
- **컨트롤러 스위칭 필요**: cyclo·pilz·joint 가 JTC 를 공유하는 현 설계와 position 인터페이스 충돌 → cyclo MoveL 시 forward 활성/JTC 비활성, 완료·타 백엔드 시 JTC 복귀(`switch_controller`). 시나리오 백엔드 선택부가 오케스트레이션.
- 장점: cyclo 의 실시간 반응성(장애물 회피 등) 보존. 단점: 컨트롤러 스위칭 오케스트레이션 추가.

### 참고 — 모터 게인은 별개 이슈
스트리밍 수정과 무관하게, 팩토리 게인(KP 50/10) 은 중력보상(tau feedforward) 부재 시 정적 sag 가 큼(j1~4 ~5°). sag 가 문제되면 중력보상 토크 또는 KP 상향은 **별도** 튜닝(점프와 무관).

---

## 2026-06-07 — cyclo MoveL solver(OSQP) null-space contort 근본원인 + 수정 (HIL 라이브검증 보류)

### 증상
B(직접제어, forward_command_controller) 검증 후에도 cyclo MoveL −Z 10mm 가 손목/어깨(j2,j3,j5,j6)를 ±9~16° contort + EE 과도/드리프트(예: X +30mm). forward 컨트롤러는 받은 명령을 그대로 실행만 하므로 **cyclo solver(QP)가 생성한 명령 자체가 문제**.

### 진단 (소거법 + 오프라인 OSQP repro)
1. 비용함수=가중 damped LS, 자코비안 프레임=LOCAL_WORLD_ALIGNED(=desired_vel 프레임), 가중치(pos10/ori1) — 전부 정상.
2. **오프라인 분석해**(cyclo 가중비용 그대로, 무제약): INIT·비-INIT 둘 다 −Z를 **j4 −3°만**으로 깨끗(contort 0, drift 0).
3. 제약: 관절한계 CBF 비구속(한계 멀어), 특이점 제약 con_sing **A행렬 전부 0=trivial**, 충돌 **0쌍**(솔버 URDF에 collision 없음), 속도한계 느슨 → 전부 비구속.
4. **python OSQP로 cyclo QP 동일 재현**(슬랙+관절한계+기본설정): 기본(eps 1e-3, polish off) → j2=−9.1,j3=7.5,j5=−8.2,j6=8.5 **contort**(과제 EE는 −10mm 정확). **eps 1e-6 + polish on → j2=j3=j5=j6=0 분석해 일치(깨끗)**.

→ **근본원인: OSQP 기본 tolerance(1e-3)가 느슨해 7DOF↔6D 여유자유도의 damping(‖qdot‖) 최소화를 마무리 못 함** → 과제(EE)는 만족하나 null-space에 군더더기 모션(contort). 볼록 QP라 최적해는 분석해(깨끗)인데 OSQP가 비최적해를 "Solved"로 반환.

### 수정 (적용·재빌드 완료, 오프라인 검증)
[qp_base.hpp](../../openarmx_ws/src/cyclo_robot_controller/cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp) `initializeSolver`에:
```
setAbsoluteTolerance(1e-6); setRelativeTolerance(1e-6); setMaxIteration(10000); setPolish(true);
```
cyclo 재빌드(core+ros) 완료. 오프라인 OSQP repro로 contort 제거 확인. **HIL 라이브 재검증은 개발 일정상 보류** — 재개 시 스택 기동 후 좌 −Z MoveL 로 j4 −3°만(contort 0) 확인하면 됨.

### 이번 세션 cyclo 변경 요약 (전부 미커밋)
- **A**: `omx_movel_controller_node`(.hpp/.cpp) `batch_trajectory`(endpoint 1점) — HIL 검증(free-fall 제거).
- **B**: bringup yaml `{left,right}_arm_position_controller`(7관절 forward passthrough) + cyclo `output_mode`/`forward_command_topic` + `openarmx_movel_bimanual.launch.py` 인자 — HIL 검증(cyclo→JTC 0건, 직접제어).
- **solver**: qp_base.hpp OSQP eps+polish — 오프라인 검증(contort 제거), HIL 보류.
- JTC `open_loop_control: true`(both yaml, jog 풀림) + 모터게인 팩토리 복원.
