# openarmx_pick 분석 문서 종합 개요 & 인덱스

분석일: 2026-06-03 · 패키지: openarmx_pick

---

## 분석 범위

본 문서는 `openarmx_pick` 패키지에 대한 12편의 심층 분석 문서(`01`~`12`)의 **종합 인덱스이자 진입점(entry point)**이다. 각 문서로 가는 링크, 패키지의 한 문단 개요, 12편에서 도출된 '핵심 발견'(아키텍처 골격·검증 현황·주요 공백/리스크 Top 5), 그리고 빌드/실행을 위한 빠른 시작 포인터를 제공한다.

- 본 문서는 **분석 전용(ANALYSIS-ONLY)**이며 어떤 소스·launch·urdf·config·setup 파일도 수정하지 않는다. 유일한 쓰기 대상은 이 파일(`docs/00_overview.md`)이다.
- 개별 알고리즘·수식·코드 라인 단위 근거는 각 전담 문서에 있으며, 본 개요는 이를 **요약·연결**할 뿐 중복 서술하지 않는다. 모든 사실 주장의 1차 근거는 해당 전담 문서가 인용한 소스 코드다.
- 약어는 첫 등장 시 풀어쓴다: ROS2(Robot Operating System 2, 로봇 운영체제 2), QP(Quadratic Program, 이차 계획법), CBF(Control Barrier Function, 제어 장벽 함수), PCA(Principal Component Analysis, 주성분 분석), TF(Transform, 좌표 변환), FK(Forward Kinematics, 순기구학), DoF(Degree of Freedom, 자유도), TCP(Tool Center Point, 공구 중심점), QoS(Quality of Service, 통신 품질), FSM(Finite State Machine, 유한 상태 기계), SRDF(Semantic Robot Description Format, 의미 기반 로봇 기술 형식), URDF(Unified Robot Description Format, 통합 로봇 기술 형식), EE(End-Effector, 말단 장치).

---

## 1. 패키지 한 문단 개요

`openarmx_pick`은 OpenArmX 이족(bimanual) 로봇의 **비전(perception) 기반 단일 팔(single-arm) 박스 픽(pick) 파이프라인**을 MoveIt 없이(MoveIt-free) 구현하는 ROS2 패키지다. 인식 스택(`3d_detect_ws`의 카메라 + YOLO-World + `box_plane` RANSAC)이 발행하는 박스 상면 인라이어 포인트 클라우드(`/box_plane/cloud`)를 입력으로 받아, 핵심 노드 `grasp_pose_node`가 TF 변환·centroid·XY 평면 PCA·고정 top-down approach 세 가지 기하 휴리스틱만으로 학습 기반 grasp 네트워크 없이 6-DoF top-down grasp pose를 해석적으로 합성한다. 합성된 pose는 `openarmx_body_link0` 프레임에서 표현되며, 동일 프레임을 root로 쓰는 **cyclo_control의 QP+CBF MoveL 솔버(`omx_movel_controller_node`)를 IK/모션 백엔드로** 활용해(MoveIt 대체) 관절 궤적으로 변환된다. 패키지는 인식 측과 제어 측을 ROS2 토픽으로만 잇는 **글루 레이어(glue layer)**이며, 자체적으로는 단일 팔 7-DoF solver URDF 두 종(left/right), 3개 launch 진입점, 3개 검증 스크립트를 관리한다. 현재 구현은 박스 상면 위 pre-grasp hover까지만 자동화하며 descend→close→lift FSM은 향후 과제다.

---

## 2. 문서 인덱스

| 문서 | 한 줄 설명 |
|---|---|
| [01_architecture.md](01_architecture.md) | 패키지의 전체 시스템 맥락, 엔드투엔드 데이터 파이프라인, ROS2 노드 그래프·토픽 배선, frame 전략, Stage A/B 단계 구분 |
| [02_grasp_pose_node.md](02_grasp_pose_node.md) | 핵심 노드 `grasp_pose_node.py`(278줄)의 메서드 단위 코드 심층 분석 — import guard, 14개 파라미터, 모든 콜백·헬퍼의 제어 흐름·예외 처리 |
| [03_grasp_synthesis_theory.md](03_grasp_synthesis_theory.md) | Top-down grasp 합성 이론 — centroid 하강, 고정 approach, XY PCA yaw, Gram-Schmidt `R=B·Tᵀ` 유도, trace 4-case quaternion 변환, 수치 검증 |
| [04_solver_urdf.md](04_solver_urdf.md) | reduced 7-DoF solver URDF(left/right) 실측 분석과 `gen_solver_urdf.py` joint freeze·collision strip 생성 스크립트, left/right 미러 관계 |
| [05_launch.md](05_launch.md) | 3개 launch 파일(단팔/양팔/pick) 분석 — LaunchArgument, 하드코딩 QP+CBF 튜닝값, 토픽 remapping, 노드 토폴로지, 알려진 불일치 |
| [06_verification.md](06_verification.md) | 3개 검증 스크립트(solver/grasp/e2e) 동작 원리·assert·실행법, 공식 PASS 결과, live camera 현황, 테스트 커버리지 공백 7건 |
| [07_cyclo_integration.md](07_cyclo_integration.md) | cyclo_control QP+CBF MoveL 백엔드 통합 — 송수신 계약, cyclo 내부 처리(FK/Jacobian→OSQP→CBF→JointTrajectory), 메시지 타입 네임스페이스 불일치 |
| [08_api_reference.md](08_api_reference.md) | 토픽·프레임·파라미터 완전 레퍼런스 — QoS·기본값·방향, README 표와 코드 기본값 교차 검증, solver launch 파라미터 |
| [09_dependencies_build.md](09_dependencies_build.md) | 런타임/테스트 의존성, `ament_python` 빌드 시스템, 설치 레이아웃, 3-overlay 빌드 절차(cyclo_ws `-j1` 필수), apt 의존성 |
| [10_coordinate_frames_tf.md](10_coordinate_frames_tf.md) | 좌표 프레임·TF 체계 — TF 트리, `_tf_to_Rt` quaternion↔matrix 변환, world→body_link0 identity, 캘리브레이션 의존성, tool-frame convention |
| [11_gaps_risks_todos.md](11_gaps_risks_todos.md) | 공백·운영/안전 리스크·코드 리뷰 결함 15건(R1~R15)을 심각도별로 종합, 우선순위 부여된 잔여 TODO 목록 |
| [12_performance.md](12_performance.md) | 런타임 성능 — `_read_xyz` Python 루프 병목, `cloud_stride` 완화, PCA 비용, latched QoS, MoveL debounce, 벡터화 개선 방향 |

---

## 3. 핵심 발견 (Key Findings)

### 3.1 아키텍처 골격

전체 파이프라인은 세 워크스페이스가 ROS2 토픽으로만 결합되는 3계층 구조다(상세: [01](01_architecture.md), [07](07_cyclo_integration.md)).

```
3d_detect_ws (외부)          openarmx_pick (본 패키지)            cyclo_ws (백엔드)
카메라+YOLO-World+box_plane → grasp_pose_node                  → omx_movel_controller_node
  /box_plane/cloud             TF변환·centroid·XY PCA·top-down     QP+CBF MoveL 솔버
                               /openarmx/left/movel               /openarmx/left_arm/joint_trajectory
```

- **두 단계 분리**: Stage A는 solver port(QP+CBF MoveL 검증, `grasp_pose_node` 미사용), Stage B는 grasp synthesis(`grasp_pose_node`가 클라우드→grasp pose 합성). [01 §5](01_architecture.md), [06](06_verification.md).
- **frame 단일화**: 인식 출력 frame과 solver URDF root가 모두 `openarmx_body_link0`이고 `world→body_link0` joint가 identity이므로, 인식↔제어 경계에 추가 TF 변환이 없다. [01 §4](01_architecture.md), [10 §3](10_coordinate_frames_tf.md).
- **grasp 합성 = 해석적 휴리스틱**: 학습 네트워크 없이 centroid(위치) + 고정 approach `(0,0,-1)`(2 DoF) + XY PCA principal axis(yaw 1 DoF)로 6-DoF를 완전 결정. [03](03_grasp_synthesis_theory.md).
- **MoveL 백엔드**: MoveIt 대신 cyclo의 Pinocchio FK/Jacobian → OSQP QP → joint-limit/singularity/collision CBF 솔버를 IK 엔진으로 사용. [07 §5](07_cyclo_integration.md).

### 3.2 검증 현황 (PASS/미완)

공식 검증 기록(2026-05-31)은 모두 PASS이나, **검증 범위가 시뮬레이션·pose 산출까지로 한정**된다는 점이 핵심이다. 상세: [06](06_verification.md).

| 검증 항목 | 상태 | 비고 |
|---|---|---|
| Stage A — QP 솔버 MoveL 수렴 + joint-limit CBF 클램프 | PASS | 정성적 로그 확인, 자동 수치 assert 없음 |
| Stage B — 합성 박스 grasp 위치 오차 ≈ 2 mm, approach `(0,0,-1)`, opening 단축 정렬 | PASS | 자동 assert 3종(`ok_pos`/`ok_down`/`ok_open`) |
| E2E — EE XY 수렴 ≈ 6–12 mm (임계 30 mm) | PASS | self-loop 시뮬레이션, z축 수렴은 측정만·판정 없음 |
| Live camera — D435 + 골판지 박스 top-down grasp pose | PASS | **"Robot not moved"** — pose 산출만, 실 팔 동작 미검증 |
| 실 팔 구동 통합 / Pick FSM / Stage-2 collision CBF / 다중 박스 선택 | 미완료 | 차기 과제 |

### 3.3 주요 공백·리스크 Top 5

[11](11_gaps_risks_todos.md)이 정리한 15건(R1~R15) 중 심각도·차단성 기준 상위 5건. (괄호는 [11]의 항목 번호.)

1. **메시지 타입 네임스페이스 불일치 (통합 결함, [07 §4](07_cyclo_integration.md))** — 송신부(`grasp_pose_node`)는 `openarmx_scenario_player_msgs/MoveL`로 publish하지만, 빌드된 cyclo `omx_movel_controller_node`는 `robotis_interfaces/MoveL`만 구독한다. ROS2 타입명 매칭 규칙상 동일 토픽이라도 DDS 레벨에서 자동 연결되지 않는다 — 현재 트리에서 실재하는 결함. README의 "MoveL stack migration"이 cyclo 소스/빌드에 반영되지 않음(검증된 사실).
2. **`auto_send=true` 안전 게이트 부재 (R1, High, [11 §3.1](11_gaps_risks_todos.md))** — 박스 인식 즉시 디바운스만 통과하면 실 MoveL이 발행된다. E-stop·작업영역 한계·사람 승인 인터록이 전혀 없어, 다른 결함(R2/R4/R6/R13)이 곧바로 실로봇 모션으로 전파된다.
3. **tool axis 가정 미검증 (R4, High, [11 §3.2](11_gaps_risks_todos.md), [10 §8](10_coordinate_frames_tf.md))** — `tool_approach_axis=[0,0,1]`, `tool_opening_axis=[1,0,0]`이 실측 FK로 검증되지 않았다. 실제 TCP 축이 다르면 quaternion이 틀어져 손목이 박스로 비스듬히/거꾸로 돌진할 수 있다.
4. **main-box 필터 부재 + Pick FSM 부재 (R2·R3, High, [11 §2.1-2.2](11_gaps_risks_todos.md))** — `box_plane`이 최대 3개 상면 후보를 발행하면 grasp pose가 후보 간 점프한다. 또한 노드는 pre-grasp hover 1점만 명령하며 descend→gripper close→lift 상태 전이가 없다(실제 집기 미구현).
5. **CBF 적용 범위 한정 + 실 컨트롤러 배선 미완 (R7·R8, Med, [07 §5.3.4](07_cyclo_integration.md), [11 §2.3-2.4](11_gaps_risks_todos.md))** — 현재 활성 CBF는 joint-limit뿐이다. collision CBF는 stage-1 URDF에 collision pair가 없어 비활성, singularity CBF는 슬랙/인덱스만 예약되고 부등식이 미작성되어 사실상 미구현이다. `joint_command`→`forward_position_controller` 배선·타입 정합도 미검증.

---

## 4. 빠른 시작 포인터

빌드·실행·검증의 **정식 절차는 패키지 README**(`../README.md`)를 참조하라. 본 개요는 진입점만 안내한다.

- **빌드 절차 상세**: 3-overlay(base → openarmx_ws → cyclo_ws) 빌드 순서, cyclo_ws의 `-j1` 단일 스레드 빌드 필수성(Pinocchio OOM 회피, ~21분), apt 의존성 → [09_dependencies_build.md](09_dependencies_build.md).
- **실행 진입점(launch)** → [05_launch.md](05_launch.md):
  - 단일 팔 solver만: `ros2 launch openarmx_pick openarmx_movel.launch.py`
  - 전체 pick 파이프라인(solver + grasp_pose_node): `ros2 launch openarmx_pick openarmx_pick.launch.py` (자동 모션은 `auto_send:=true`)
  - 양팔 solver: `ros2 launch openarmx_pick openarmx_movel_bimanual.launch.py`
- **검증 스크립트 실행법** → [06_verification.md](06_verification.md): `verify_solver.py`(Stage A), `verify_grasp.py`(Stage B), `verify_e2e.py`(통합).
- **토픽/파라미터 레퍼런스** → [08_api_reference.md](08_api_reference.md).

> 주의: `openarmx_pick.launch.py`는 카메라-로봇 외부 교정 TF를 발행하지 않으므로, `grasp_pose_node`의 TF 조회가 성공하려면 별도 스택에서 해당 TF가 발행되어 있어야 한다([05 §7](05_launch.md), [10 §9](10_coordinate_frames_tf.md)).

---

## 5. 검증 현황 메모 (문서 정확도)

12편의 분석 문서는 모두 별도 검증자 패스를 거쳐 `accurate=true` 판정을 받았다. 검증 과정에서 주로 **행 번호 인용 오류**가 발견·수정되었고, 일부 사실 오류도 교정되었다.

| 문서 | 판정 | 수정 내역 요약 |
|---|---|---|
| 01_architecture.md | accurate=true | 행 번호 인용 6건 수정 |
| 02_grasp_pose_node.md | accurate=true | 파라미터 수 "11개"→"14개" 정정(declare_parameter 14회) |
| 03_grasp_synthesis_theory.md | accurate=true | `_quat_from_matrix` 분기 판정 2곳 수정(else/z 최대 → case 3/y 최대) |
| 04_solver_urdf.md | accurate=true | 행 번호 인용 6건 수정 |
| 05_launch.md | accurate=true | 주석 라인 번호 1건 수정(:67→:66) |
| 06_verification.md | accurate=true | 행 번호 인용 3건 수정 |
| 07_cyclo_integration.md | accurate=true | 3건 수정(qp_base.hpp 라인, verify_e2e 함수명·라인, joint_state_timeout yaml 미기재 사실) |
| 08_api_reference.md | accurate=true | 행 번호 오인용 5건 수정 |
| 09_dependencies_build.md | accurate=true | 줄 번호 1건 수정(setup.py:32→:26) |
| 10_coordinate_frames_tf.md | accurate=true | 수정 불필요 |
| 11_gaps_risks_todos.md | accurate=true | 수정 불필요 |
| 12_performance.md | accurate=true | 존재하지 않는 QoS 정책명 `TRANSIENT_DURABILITY`→`TRANSIENT_LOCAL` 수정 |

- 모든 문서는 패키지 루트(`openarmx_pick/`) 기준 상대 `path:line` 형식으로 소스를 인용하며, 코드로 검증하지 못한 진술은 본문에서 '추정'으로 명시한다.
- **본 개요의 사실 주장은 12편 전담 문서가 이미 검증한 내용을 요약·연결한 것**이다. 1차 근거(소스 코드 라인)는 각 전담 문서를 참조하라.
