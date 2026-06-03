# openarmx_pick 아키텍처 & 데이터 플로우 분석

**분석일:** 2026-06-03
**패키지:** openarmx_pick
**패키지 루트:** `openarmx_ws/src/openarmx_pick/`

---

## 분석 범위

본 문서는 `openarmx_pick` 패키지가 전체 pick 파이프라인에서 차지하는 위치, ROS2(Robot Operating System 2) 노드 그래프와 토픽 배선, 엔드투엔드(end-to-end) 데이터 흐름, 그리고 frame 전략을 다룬다. Stage A(solver port 검증 단계)와 Stage B(grasp synthesis 단계)의 역할 분리도 명확히 설명한다. 개별 알고리즘(PCA(Principal Component Analysis, 주성분 분석) 수식, QP(Quadratic Program, 이차 계획법)+CBF(Control Barrier Function, 제어 배리어 함수) 수학)이나 검증 결과의 세부 수치는 다른 문서에서 다룬다.

---

## 1. 시스템 전체 맥락

`openarmx_pick`은 OpenArmX 이족(bimanual) 로봇의 단일 팔 박스 픽(pick) 파이프라인을 MoveIt 없이 구현하는 패키지다. 인식 측(perception side)의 `3d_detect_ws`와 제어 측(control side)의 `cyclo_ws` 사이를 이어주는 **글루 레이어(glue layer)** 역할을 담당한다.

패키지가 전체 monorepo에서 차지하는 위치를 워크스페이스 단위로 표현하면 다음과 같다.

```
3d_detect_ws/
  └── (camera + YOLO-World + box_plane RANSAC) ─────────▶  /box_plane/cloud
                                                                     │
openarmx_ws/src/openarmx_pick/  ◀────────────────────────────────────┘
  └── grasp_pose_node  ──────────────────────────────────▶  /openarmx/left/movel
                                                                     │
cyclo_ws/
  └── omx_movel_controller_node (QP+CBF)  ────────────▶  joint_trajectory  ──▶  arm
```

세 워크스페이스는 각각 독립적으로 빌드되며 런타임에 ROS2 토픽으로만 결합된다. `openarmx_pick`은 `openarmx_ws`에 속하며, 제어 노드(`omx_movel_controller_node`)의 실행 파일(`cyclo_motion_controller_ros`)은 `cyclo_ws` 오버레이에서 제공된다.

---

## 2. 엔드투엔드 데이터 파이프라인

### 2.1 전체 파이프라인 ASCII 다이어그램

```
┌─────────────────────────────────────────────────────────────────────────────┐
│  인식 스택 (3d_detect_ws, 외부 실행)                                        │
│                                                                             │
│  D435 stereo depth                                                          │
│       │  sensor_msgs/Image + CameraInfo                                     │
│       ▼                                                                     │
│  YOLO-World (yolov8_node)                                                   │
│       │  Detection2DArray (박스 클래스 바운딩박스)                           │
│       ▼                                                                     │
│  box_plane RANSAC (fit_box_plane 노드)                                      │
│       │  /box_plane/cloud  sensor_msgs/PointCloud2                          │
│       │    frame: camera_color_optical_frame                                │
│       │  /box_plane/info   std_msgs/String (JSON: box_height_m 등)          │
└───────┼─────────────────────────────────────────────────────────────────────┘
        │
        │  (토픽 브리지, 추가 TF 불필요 — 아래 §4 frame 전략 참고)
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  openarmx_pick 패키지 (openarmx_ws)                                         │
│                                                                             │
│  grasp_pose_node                                                            │
│    ① /box_plane/cloud 구독                                                  │
│    ② tf2_ros.Buffer.lookup_transform(                                       │
│         target=openarmx_body_link0,                                         │
│         source=camera_color_optical_frame)                                  │
│    ③ 포인트 클라우드 기저 변환 (R, t 적용) → N×3 base frame 좌표           │
│    ④ XY 평면 PCA → 박스 장축(long_axis) 추출                               │
│    ⑤ approach = [0,0,-1]  (기저 -z, 수직 하강)                             │
│    ⑥ opening  = cross([0,0,1], long_axis)  (박스 단축)                     │
│    ⑦ _grasp_rotation() → R_base_tool → quaternion                          │
│    ⑧ grasp_xyz  = centroid - grasp_depth * z_hat                           │
│       pre_xyz   = centroid + pregrasp_height * z_hat                       │
│    ⑨ /openarmx/grasp_pose  PoseStamped  (frame: openarmx_body_link0)       │
│       /openarmx/grasp_markers  MarkerArray  (RViz 화살표)                  │
│   ⑩  (auto_send=true 일 때) _should_send() 디바운스 검사                   │
│       /openarmx/left/movel  openarmx_scenario_player_msgs/MoveL             │
│         → pre_xyz + quat + time_from_start                                 │
└───────┼─────────────────────────────────────────────────────────────────────┘
        │  /openarmx/left/movel
        ▼
┌─────────────────────────────────────────────────────────────────────────────┐
│  cyclo_ws — omx_movel_controller_node (QP+CBF solver)                      │
│                                                                             │
│  입력: /openarmx/left/movel  (goal PoseStamped + duration)                 │
│        /joint_states          sensor_msgs/JointState  (피드백)              │
│                                                                             │
│  내부: Pinocchio FK/Jacobian → OSQP QP                                     │
│        CBF 제약: joint-limit CBF + singularity CBF                         │
│        (Stage 1: collision CBF 미사용, SRDF 비어 있음)                       │
│                                                                             │
│  출력: /openarmx/left_arm/joint_trajectory                                  │
│          trajectory_msgs/JointTrajectory  (arm → forward controller)       │
│        /openarmx/left_ee_pose                                               │
│          geometry_msgs/PoseStamped  (현재 EE 위치, 모니터링용)              │
└─────────────────────────────────────────────────────────────────────────────┘
        │  /openarmx/left_arm/joint_trajectory
        ▼
   OpenArmX 왼쪽 팔 (forward_position_controller 추정)
```

### 2.2 데이터 변환 단계별 설명

| 단계 | 입력 | 변환 | 출력 |
|------|------|------|------|
| 1. 깊이 → 포인트 클라우드 | D435 RGB-D | 스테레오 깊이 + YOLO 마스크 → RANSAC 인라이어 | `/box_plane/cloud` (camera frame) |
| 2. 클라우드 → 기저 변환 | camera frame N×3 | tf2 lookup → R, t 행렬 곱 | base frame N×3 |
| 3. PCA 그래스프 합성 | base frame N×3 | 공분산 고유값 분해 → long_axis → quaternion | `PoseStamped` (base frame) |
| 4. QP 역기구학 | goal pose + joint_states | Pinocchio Jacobian + OSQP | `JointTrajectory` |
| 5. 관절 실행 | `JointTrajectory` | forward controller | 모터 명령 |

---

## 3. ROS2 노드 그래프

### 3.1 노드 목록

`openarmx_pick.launch.py`로 기동할 때 활성화되는 노드는 다음과 같다.

| 노드 이름 | 패키지 | 실행 파일 | 역할 |
|-----------|--------|-----------|------|
| `grasp_pose_node` | `openarmx_pick` | `grasp_pose_node` | Stage B — 클라우드 → grasp PoseStamped (+ 선택적 MoveL) |
| `openarmx_left_movel_controller` | `cyclo_motion_controller_ros` | `omx_movel_controller_node` | QP+CBF MoveL solver (왼쪽 팔) |

바이매뉴얼(bimanual, 양팔) 구성(`openarmx_movel_bimanual.launch.py`)에서는 `openarmx_right_movel_controller`도 추가된다. 두 solver는 공유 상태 없이 독립적으로 동작하며 `/joint_states`만 공통으로 구독한다.

### 3.2 토픽 배선 전체도

```
외부 인식 스택
  fit_box_plane ──────/box_plane/cloud (PointCloud2)──────▶ grasp_pose_node
  fit_box_plane ──────/box_plane/info  (String, JSON)─────▶ grasp_pose_node

  로봇 / 시뮬레이터
  joint_state_publisher ─/joint_states (JointState)───────▶ openarmx_left_movel_controller

grasp_pose_node
  ──▶ /openarmx/grasp_pose   (PoseStamped, TRANSIENT_LOCAL)
  ──▶ /openarmx/grasp_markers (MarkerArray, TRANSIENT_LOCAL)
  ──▶ /openarmx/left/movel   (MoveL, depth=10)  [auto_send=true 시]

openarmx_left_movel_controller
  ◀── /openarmx/left/movel   (~/movel remap)
  ──▶ /openarmx/left_arm/joint_trajectory  (JointTrajectory)
  ──▶ /openarmx/left_ee_pose  (~/current_pose remap)
```

`launch/openarmx_movel.launch.py:78–81`에서 solver 내부 토픽을 remap한다.

```python
remappings=[
    ("~/movel", LaunchConfiguration("movel_topic")),
    ("~/current_pose", LaunchConfiguration("ee_pose_topic")),
],
```

바이매뉴얼 launch(`launch/openarmx_movel_bimanual.launch.py:60–63`)에서는 side 변수로 left/right를 파라미터화하여 두 solver를 동일한 코드로 생성한다.

```python
remappings=[
    ("~/movel", f"/openarmx/{side}/movel"),
    ("~/current_pose", f"/openarmx/{side}/ee_pose"),
],
```

---

## 4. Frame 전략 — TF 추가 불필요

### 4.1 핵심 설계 결정

`openarmx_body_link0`는 solver URDF의 root frame이자 인식 파이프라인의 출력 frame이다. URDF에서 `world → body_link0` joint를 identity(위치·회전 모두 0)로 선언했으므로, world frame과 body_link0 frame은 동일하다.

이 설계의 결과는 다음과 같다.

- `grasp_pose_node`가 발행하는 `PoseStamped`의 `header.frame_id = "openarmx_body_link0"` (`openarmx_pick/grasp_pose_node.py:124, 219`)
- solver가 사용하는 base_frame도 `openarmx_body_link0` (`launch/openarmx_movel.launch.py:41–42`)
- 따라서 인식(perception)에서 제어(control)로 넘어가는 경계에서 **추가 TF 변환이 없다**.

유일하게 필요한 TF 조회는 카메라 optical frame → base frame 변환이며, 이는 카메라 외부 파라미터 캘리브레이션 결과로 이미 TF 트리에 정적(static) 브로드캐스트되어 있다(별도 캘리브레이션 문서 참고).

### 4.2 TF 조회 코드

`openarmx_pick/grasp_pose_node.py:171–174`:

```python
tf = self.tf_buffer.lookup_transform(
    self.base_frame, cloud.header.frame_id, rclpy.time.Time())
```

`rclpy.time.Time()`(시간=0)으로 조회하면 TF 버퍼에서 가장 최근 변환을 사용한다. 카메라 → 기저 TF는 정적이므로 타임스탬프 불일치 문제가 없다.

---

## 5. Stage A / Stage B 단계 구분

### 5.1 Stage A — Solver Port (QP+CBF 검증)

Stage A는 `openarmx_pick` 패키지가 `cyclo_control`의 `omx_movel_controller_node`를 OpenArmX 팔 geometry에 맞게 구동할 수 있는지 검증하는 단계다. 이 단계에서 `grasp_pose_node`는 사용하지 않는다.

**관련 파일:**

- `urdf/openarmx_left_solver.urdf` / `urdf/openarmx_right_solver.urdf` — 충돌(collision) 형상이 제거된 단일 팔 7-DOF solver URDF. root = `openarmx_body_link0`. 반대쪽 팔과 모든 손가락 관절은 `fixed`로 동결.
- `scripts/gen_solver_urdf.py` — 전체 xacro 전개 결과에서 위 URDF를 재생성하는 도구(`--no-collision` / `--strip-visual` 옵션 포함).
- `scripts/verify_solver.py` — fake `/joint_states` + MoveL goal을 발행하고, 결과 `joint_command`가 IK 해로 수렴하는지 확인. 검증 결과: Stage A PASS (`README.md:123–124`).
- `launch/openarmx_movel.launch.py` — Stage A 실행 진입점. SRDF(Semantic Robot Description Format) 경로 기본값이 비어 있어 충돌 쌍 없이 joint-limit CBF + singularity CBF만 동작.

**Stage A QP solver 주요 파라미터** (`launch/openarmx_movel.launch.py:67–76`):

| 파라미터 | 값 | 설명 |
|----------|-----|------|
| `kp_position` | 4.0 | 위치 비례 이득 |
| `kp_orientation` | 2.5 | 자세 비례 이득 |
| `weight_task_position` | 10.0 | QP 위치 태스크 가중치 |
| `weight_damping` | 0.05 | 관절 속도 댐핑 |
| `slack_penalty` | 1000.0 | CBF 슬랙 페널티 |
| `cbf_alpha` | 5.0 | CBF class-K 함수 계수 |
| `control_frequency` | 100.0 Hz | 제어 루프 주파수 |

바이매뉴얼 launch에서는 `kp_position=50.0`, `kp_orientation=50.0`, `weight_damping=0.001`로 upstream `cyclo_control/config/omx_config.yaml` 기본값을 그대로 사용한다 (`launch/openarmx_movel_bimanual.launch.py:49–53`). 단일 팔 launch는 더 보수적인 이득을 사용한다.

### 5.2 Stage B — Grasp Synthesis (그래스프 합성)

Stage B는 `grasp_pose_node`가 `/box_plane/cloud`로부터 top-down 6-DoF 그래스프 포즈를 합성하는 단계다.

**관련 파일:**

- `openarmx_pick/grasp_pose_node.py` — Stage B 핵심 노드.
- `scripts/verify_grasp.py` — 카메라 없이 합성 클라우드만으로 Stage B를 검증. 박스 위치 오차 ≈ 2 mm, approach ≈ (0,0,-1), opening ≈ 박스 단축 방향 (`README.md:125–126`).
- `scripts/verify_e2e.py` — Stage A + B 통합 검증. 합성 클라우드 → grasp → MoveL → solver → EE 수렴 확인. 수렴 기준: XY 오차 < 0.03 m (`scripts/verify_e2e.py:97`).

**Stage B 내부 처리 흐름** (`openarmx_pick/grasp_pose_node.py:164–199`):

```
_on_cloud() 콜백
  1. _read_xyz()  — PointCloud2 raw bytes → N×3 float64 (stride=4 서브샘플)
  2. tf2 lookup   — camera_color_optical_frame → openarmx_body_link0
  3. _tf_to_Rt()  — TransformStamped quaternion → 3×3 rotation + 3-vec translation
  4. pts = (R @ pts_cam.T).T + t   — 기저 frame 변환
  5. centroid = pts.mean(axis=0)
  6. XY 2D PCA:
       xy  = pts[:, :2] - centroid[:2]
       cov = xy.T @ xy / (N-1)
       evals, evecs = np.linalg.eigh(cov)
       long_axis = evecs[:, argmax(evals)]   — 장축 (XY 평면 내)
  7. opening = cross([0,0,1], long_axis)     — 단축 방향 (gripper 벌림 방향)
  8. approach = [0, 0, -1]                   — 수직 하강
  9. _grasp_rotation(approach, opening, tool_a, tool_o)
       → R_base_tool (3×3) → _quat_from_matrix() → quaternion [x,y,z,w]
 10. grasp_xyz = centroid;  grasp_xyz[2] -= grasp_depth    (기본 0.005 m)
     pre_xyz  = centroid;  pre_xyz[2]  += pregrasp_height (기본 0.10 m)
 11. _publish_pose()    → /openarmx/grasp_pose
     _publish_marker()  → /openarmx/grasp_markers
 12. (auto_send=true) _should_send() → _send_movel() → /openarmx/left/movel
```

---

## 6. grasp_pose_node 상세 설계

### 6.1 파라미터 목록

`openarmx_pick/grasp_pose_node.py:104–121`:

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `cloud_topic` | `/box_plane/cloud` | 박스 상면 인라이어 클라우드 입력 |
| `info_topic` | `/box_plane/info` | 박스 높이 JSON (`box_height_m`) |
| `base_frame` | `openarmx_body_link0` | 그래스프 포즈 출력 frame |
| `movel_topic` | `/openarmx/left/movel` | MoveL 명령 출력 토픽 |
| `grasp_pose_topic` | `/openarmx/grasp_pose` | 그래스프 포즈 출력 토픽 |
| `pregrasp_height` | 0.10 m | centroid 위 pre-grasp 높이 |
| `grasp_depth` | 0.005 m | 상면 아래 그래스프 깊이 |
| `cloud_stride` | 4 | 클라우드 서브샘플 간격 (PCA 속도) |
| `auto_send` | false | MoveL 자동 발행 여부 |
| `move_time` | 4.0 s | MoveL 실행 시간 |
| `send_min_interval` | 5.0 s | MoveL 재발행 최소 대기 시간 |
| `send_min_delta` | 0.02 m | MoveL 재발행 최소 목표 이동량 |
| `tool_approach_axis` | [0, 0, 1] | TCP(Tool Center Point) 접근 축 (tool frame) |
| `tool_opening_axis` | [1, 0, 0] | TCP 손가락 벌림 축 (tool frame) |

### 6.2 MoveL 디바운스(debounce) 메커니즘

카메라 프레임마다 MoveL을 발행하면 solver가 trajectory를 매 프레임 재시작하여 팔이 아주 조금씩만 움직이는 문제가 생긴다. 이를 방지하기 위해 `_should_send()` (`openarmx_pick/grasp_pose_node.py:226–238`)가 두 조건 중 하나가 만족될 때만 MoveL을 발행한다.

1. 목표 위치가 이전 발행 위치에서 `send_min_delta`(기본 0.02 m) 이상 이동
2. 마지막 발행 후 `send_min_interval`(기본 5.0 s, ≥ `move_time`) 이상 경과

`send_min_interval ≥ move_time`으로 설정하면 한 MoveL 동작이 완료된 후에만 다음 명령을 발행한다.

### 6.3 QoS(Quality of Service) 설정

`openarmx_pick/grasp_pose_node.py:141–148`:

- `/openarmx/grasp_pose`, `/openarmx/grasp_markers`: `RELIABLE` + `TRANSIENT_LOCAL` (depth=1) — 나중에 구독하는 노드도 마지막 메시지를 받을 수 있도록 latched 방식으로 발행.
- `/openarmx/left/movel`: 표준 `depth=10` — solver는 최신 목표만 사용하므로 latching 불필요.
- `/box_plane/cloud` 구독: `depth=5`.
- `/box_plane/info` 구독: `depth=10`.

### 6.4 tool_approach_axis / tool_opening_axis 규약

TCP 관련 링크명은 `openarmx_left_hand_tcp`이며, tool frame 기준으로 그래스프 접근 방향이 +z, 손가락 벌림 방향이 +x로 가정된다. `_grasp_rotation()` 함수 (`openarmx_pick/grasp_pose_node.py:83–97`)는 다음을 계산한다.

```
B = [opening_base | binormal_base | approach_base]   — 원하는 자세 (기저 frame)
T = [opening_tool | binormal_tool | approach_tool]   — 현재 tool frame
R_base_tool = B @ T^T
```

이 rotation matrix를 `_quat_from_matrix()`로 quaternion으로 변환하여 `PoseStamped.pose.orientation`에 넣는다.

---

## 7. Launch 구성 선택지

패키지는 세 가지 launch 진입점을 제공한다.

### 7.1 `openarmx_movel.launch.py` — 단일 팔 solver만

왼쪽 팔 solver(`openarmx_left_movel_controller`)만 기동한다. Stage A 검증 또는 외부에서 직접 MoveL을 발행하는 경우에 사용한다.

```
기동 노드: openarmx_left_movel_controller
구독: /joint_states, /openarmx/left/movel
발행: /openarmx/left_arm/joint_trajectory, /openarmx/left_ee_pose
```

### 7.2 `openarmx_pick.launch.py` — solver + grasp_pose_node

`openarmx_movel.launch.py`를 include한 뒤 `grasp_pose_node`를 추가로 기동한다. `start_solver:=false`로 solver 기동을 생략할 수 있다 (`launch/openarmx_pick.launch.py:35–39`). `auto_send:=true`를 설정하면 grasp → MoveL 자동 전송이 활성화된다. **전체 pick 파이프라인의 표준 진입점.**

```
기동 노드: openarmx_left_movel_controller, grasp_pose_node
추가 인수: auto_send (기본 false), pregrasp_height (기본 0.10), cloud_topic, start_solver
```

### 7.3 `openarmx_movel_bimanual.launch.py` — 양팔 solver

left/right 두 solver를 동시에 기동한다. `grasp_pose_node`는 포함되지 않으며, 양팔 작업은 별도로 구현한다. 각 solver의 `joint_command_topic`은 `/{side}_joint_trajectory_controller/joint_trajectory`로 설정되어 ros2_control JTC(Joint Trajectory Controller, 관절 궤적 컨트롤러) 표준 네임스페이스를 따른다 (`launch/openarmx_movel_bimanual.launch.py:44`).

---

## 8. 패키지 의존성 요약

`package.xml`에 선언된 실행 의존성:

| 패키지 | 역할 |
|--------|------|
| `rclpy` | ROS2 Python 클라이언트 라이브러리 |
| `geometry_msgs` | `PoseStamped` 등 기본 기하 메시지 |
| `std_msgs` | `String` (JSON box info) |
| `visualization_msgs` | `MarkerArray` (RViz 시각화) |
| `tf2_ros` | TF2 Buffer / TransformListener |
| `cyclo_motion_controller_ros` | QP+CBF solver 실행 파일(`omx_movel_controller_node`) |
| `openarmx_scenario_player_msgs` | `MoveL` 명령 메시지 타입 |

빌드 의존성으로는 Pinocchio(FK/Jacobian 라이브러리), OSQP(OSQP Solver, 이차 계획법 라이브러리), NLopt가 필요하며 이들은 apt로 설치된다 (`README.md:70–71`). Python 패키지 자체는 `ament_python` 빌드 타입으로 별도 컴파일이 없다.

---

## 9. 검증 구조

세 스크립트가 파이프라인을 계층적으로 검증한다.

```
scripts/verify_solver.py   → Stage A: fake JointState + MoveL → joint_command 수렴 확인
scripts/verify_grasp.py    → Stage B: 합성 클라우드 → grasp pose 정확도 확인
scripts/verify_e2e.py      → Stage A+B 통합: 합성 클라우드 → grasp → MoveL → EE 수렴
```

`verify_e2e.py`는 solver의 `/openarmx/left_ee_pose`와 `grasp_pose_node`의 `/openarmx/grasp_pose`(z에 `PRE_H=0.10` m 더한 값)를 비교하여 XY 오차 < 0.03 m를 PASS 기준으로 삼는다 (`scripts/verify_e2e.py:97–98`).

---

## 10. 현재 아키텍처의 제약 및 미완성 사항

README에 명시된 미완성 사항(`README.md:131–141`)을 아키텍처 관점에서 정리한다.

| 항목 | 현재 상태 | 아키텍처 영향 |
|------|-----------|---------------|
| 메인 박스 필터 | `box_plane`이 최대 3개 박스 상면 후보를 출력 → grasp pose가 후보 간 점핑 | `grasp_pose_node`에 최대 인라이어 클라우드 선택 로직 추가 필요 |
| Pick FSM(Finite State Machine, 유한 상태 기계) | pre-grasp hover만 명령; 하강→그리퍼 닫기→리프트 없음 | 별도 FSM 노드 또는 `grasp_pose_node` 확장 필요 |
| Stage-2 collision CBF | SRDF 비어 있음, 충돌 URDF 미사용 | solver URDF 재생성 + SRDF 추가 필요 (`scripts/gen_solver_urdf.py` 활용) |
| 실제 컨트롤러 매핑 | `joint_command` → `forward_position_controller` 배선 미검증 | ros2_control 컨트롤러 spawner 설정 필요; 바이매뉴얼 launch는 JTC 표준 토픽으로 이미 전환됨 |
