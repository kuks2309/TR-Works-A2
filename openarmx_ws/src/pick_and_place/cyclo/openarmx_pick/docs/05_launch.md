# openarmx_pick Launch 파일 분석

분석일: 2026-06-03
패키지: openarmx_pick

---

## 분석 범위

`launch/` 디렉터리에 존재하는 3개 launch 파일을 대상으로 한다.

| 파일 | 역할 요약 |
|---|---|
| `launch/openarmx_movel.launch.py` | 단일 left arm QP+CBF (Quadratic Programming + Control Barrier Function, 이차계획법 + 제어 장벽 함수) MoveL solver 기동 |
| `launch/openarmx_movel_bimanual.launch.py` | left·right 양팔 solver 독립 인스턴스 동시 기동 |
| `launch/openarmx_pick.launch.py` | solver + `grasp_pose_node` (시각 기반 파지 자세 합성 노드) 동시 기동 (Stage-B pick 전체 파이프라인) |

각 launch 파일의 LaunchArgument, 기동 노드, 파라미터, 토픽 remapping, URDF (Unified Robot Description Format, 로봇 통합 기술 형식) 주입 경로, 노드 간 토폴로지를 분석한다. 카메라·YOLO·`box_plane` 스택, 하드웨어 드라이버, ros2_control (ROS2 제어 프레임워크) 컨트롤러 스포너는 범위 밖이다.

---

## 1. `openarmx_movel.launch.py` — 단일 left arm MoveL solver

### 1.1 파일 개요

```
launch/openarmx_movel.launch.py:1-84
```

모듈 docstring이 설명하는 바와 같이, `cyclo_motion_controller_ros` 패키지의 로봇 비종속 실행 파일 `omx_movel_controller_node`를 재사용하여 OpenArmX left arm에 바인딩한다. URDF와 제어 대상 링크를 파라미터로 받으므로 실행 파일 자체는 arm에 종속되지 않는다.

### 1.2 LaunchArgument 목록

| 인자 | 기본값 | 설명 |
|---|---|---|
| `urdf_path` | `share/openarmx_pick/urdf/openarmx_left_solver.urdf` | 단일팔 solver URDF (루트 = `openarmx_body_link0`) |
| `srdf_path` | `""` (빈 문자열) | Stage-1에서는 비어 있음 — 충돌 쌍 없음 |
| `base_frame` | `openarmx_body_link0` | solver 루트 프레임 |
| `controlled_link` | `openarmx_left_hand_tcp` | 제어 대상 End-Effector (EE, 말단장치) 링크 |
| `joint_states_topic` | `/joint_states` | JointState 피드백 토픽 |
| `joint_command_topic` | `/openarmx/left_arm/joint_trajectory` | ros2_control forward controller 입력 토픽 |
| `movel_topic` | `/openarmx/left/movel` | MoveL 목표 수신 토픽 |
| `ee_pose_topic` | `/openarmx/left_ee_pose` | 현재 EE 자세 발행 토픽 |
| `control_frequency` | `100.0` (Hz) | 제어 루프 주기 |

`urdf_path`는 `FindPackageShare("openarmx_pick")`를 통해 설치된 share 경로에서 해석된다 (`launch/openarmx_movel.launch.py:30-37`).

### 1.3 기동 노드

단일 노드만 기동한다.

```
launch/openarmx_movel.launch.py:53-82
```

| 속성 | 값 |
|---|---|
| `package` | `cyclo_motion_controller_ros` |
| `executable` | `omx_movel_controller_node` |
| `name` | `openarmx_left_movel_controller` |
| `output` | `screen` |

### 1.4 하드코딩 파라미터 (QP+CBF 튜닝값)

인자화되지 않고 launch 파일에 고정된 수치 파라미터들이다 (`launch/openarmx_movel.launch.py:67-76`):

| 파라미터 | 값 | 의미 |
|---|---|---|
| `time_step` | `0.01` | QP 적분 스텝 (s) |
| `trajectory_time` | `0.05` | 궤적 보간 시간 (s) |
| `kp_position` | `4.0` | 위치 비례 이득 |
| `kp_orientation` | `2.5` | 자세 비례 이득 |
| `weight_task_position` | `10.0` | QP 위치 태스크 가중치 |
| `weight_task_orientation` | `1.0` | QP 자세 태스크 가중치 |
| `weight_damping` | `0.05` | QP 감쇠 가중치 |
| `slack_penalty` | `1000.0` | CBF 슬랙 변수 페널티 |
| `cbf_alpha` | `5.0` | CBF α 계수 (강도) |
| `joint_state_timeout` | `0.5` | JointState 타임아웃 (s) |

주석(`launch/openarmx_movel.launch.py:66`)에 "cyclo defaults, tuned conservative"라고 명시되어 있다. `trajectory_time=0.05`는 bimanual 파일의 `0.0`과 다르다(§2.4 참조).

### 1.5 토픽 Remapping

```
launch/openarmx_movel.launch.py:78-81
```

| 노드 내부 토픽 | 실제 토픽 |
|---|---|
| `~/movel` | `LaunchConfiguration("movel_topic")` → 기본 `/openarmx/left/movel` |
| `~/current_pose` | `LaunchConfiguration("ee_pose_topic")` → 기본 `/openarmx/left_ee_pose` |

`joint_states_topic`과 `joint_command_topic`은 remapping이 아닌 파라미터로 전달된다. 이는 노드 내부에서 파라미터 값을 읽어 직접 구독/발행하는 방식임을 의미한다(추정: `omx_movel_controller_node`의 소스가 이 패키지에 없어 내부 구현은 확인 불가).

### 1.6 URDF 주입 경로

```
launch/openarmx_movel.launch.py:35-37
```

`urdf_path` 기본값은 `PathJoinSubstitution([FindPackageShare("openarmx_pick"), "urdf", "openarmx_left_solver.urdf"])`이다. 실제 파일 `urdf/openarmx_left_solver.urdf`는 `setup.py:17`의 `data_files` 설정에 의해 `share/openarmx_pick/urdf/`로 설치된다. SRDF (Semantic Robot Description Format, 의미론적 로봇 기술 형식)는 Stage-1에서 빈 문자열로, 관절 한계·특이점 CBF만 활성화된다.

### 1.7 주의: `joint_command_topic` 불일치

이 launch 파일의 기본 `joint_command_topic`은 `/openarmx/left_arm/joint_trajectory`이다. 반면 bimanual launch 파일은 같은 팔에 대해 `/{side}_joint_trajectory_controller/joint_trajectory`를 사용한다. 후자가 ros2_control JTC (Joint Trajectory Controller, 관절 궤적 컨트롤러) 표준 토픽이며, 전자는 구독자가 없어 궤적이 묵시적으로 소실된다는 경위가 bimanual 파일 내 주석(`launch/openarmx_movel_bimanual.launch.py:39-43`)에 명시되어 있다. `openarmx_movel.launch.py`의 기본값은 이 수정이 반영되지 않은 구버전 형태이다.

---

## 2. `openarmx_movel_bimanual.launch.py` — 양팔 MoveL solver

### 2.1 파일 개요

```
launch/openarmx_movel_bimanual.launch.py:1-78
```

left·right 각각 독립적인 `omx_movel_controller_node` 인스턴스를 생성한다. 두 solver는 `/joint_states`만 공유하며, 명령 토픽과 MoveL 수신 토픽이 완전히 분리되어 커플링 없이 병렬 동작한다.

### 2.2 LaunchArgument 목록

단일 launch 대비 크게 축소되었다. 팔별 URDF, base_frame, controlled_link, joint_command_topic은 모두 하드코딩된다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `srdf_path` | `""` | Stage-1: 충돌 쌍 없음. SRDF 추가 시 변경 |
| `joint_states_topic` | `/joint_states` | 양팔 공용 JointState 피드백 토픽 |
| `control_frequency` | `100.0` (Hz) | 양팔 공용 제어 루프 주기 |

팔별 URDF, base_frame, controlled_link, joint_command_topic은 `_arm_node()` 헬퍼 함수 내에서 `side` 문자열로 자동 생성된다(`launch/openarmx_movel_bimanual.launch.py:25-64`).

### 2.3 기동 노드

`_arm_node(side, pkg)` 헬퍼 함수가 `side ∈ {"left", "right"}`에 대해 호출된다(`launch/openarmx_movel_bimanual.launch.py:78`).

| 속성 | left 인스턴스 | right 인스턴스 |
|---|---|---|
| `package` | `cyclo_motion_controller_ros` | `cyclo_motion_controller_ros` |
| `executable` | `omx_movel_controller_node` | `omx_movel_controller_node` |
| `name` | `openarmx_left_movel_controller` | `openarmx_right_movel_controller` |
| `urdf_path` | `…/urdf/openarmx_left_solver.urdf` | `…/urdf/openarmx_right_solver.urdf` |
| `controlled_link` | `openarmx_left_hand_tcp` | `openarmx_right_hand_tcp` |
| `joint_command_topic` | `/left_joint_trajectory_controller/joint_trajectory` | `/right_joint_trajectory_controller/joint_trajectory` |

두 URDF 파일(`urdf/openarmx_left_solver.urdf`, `urdf/openarmx_right_solver.urdf`)이 모두 존재함을 파일시스템에서 확인하였다.

### 2.4 하드코딩 파라미터 (QP+CBF 튜닝값)

```
launch/openarmx_movel_bimanual.launch.py:47-58
```

| 파라미터 | 값 | 단일 launch와의 차이 |
|---|---|---|
| `time_step` | `0.01` | 동일 |
| `trajectory_time` | `0.0` | 단일: `0.05` — 보간 비활성화 |
| `kp_position` | `50.0` | 단일: `4.0` — 대폭 강화 |
| `kp_orientation` | `50.0` | 단일: `2.5` — 대폭 강화 |
| `weight_task_position` | `10.0` | 동일 |
| `weight_task_orientation` | `1.0` | 동일 |
| `weight_damping` | `0.001` | 단일: `0.05` — 감쇠 크게 감소 |
| `slack_penalty` | `1000.0` | 동일 |
| `cbf_alpha` | `5.0` | 동일 |
| `collision_buffer` | `0.01` | 단일에 없음 — 충돌 버퍼 추가 |
| `collision_safe_distance` | `0.005` | 단일에 없음 — 충돌 안전 거리 추가 |
| `joint_state_timeout` | `0.5` | 동일 |

내부 주석(`launch/openarmx_movel_bimanual.launch.py:46`)에 "cyclo defaults — restored to upstream cyclo_control/config/omx_config.yaml values"라고 명시된다. 단일 launch의 보수적 튜닝(`kp_position=4.0`)과 bimanual의 upstream 기본값(`kp_position=50.0`) 간 큰 차이는 의도적이다.

### 2.5 토픽 Remapping

```
launch/openarmx_movel_bimanual.launch.py:61-63
```

팔별로 `side` 변수를 사용해 remapping을 고정 생성한다.

| 노드 내부 토픽 | left 실제 토픽 | right 실제 토픽 |
|---|---|---|
| `~/movel` | `/openarmx/left/movel` | `/openarmx/right/movel` |
| `~/current_pose` | `/openarmx/left/ee_pose` | `/openarmx/right/ee_pose` |

단일 launch의 `ee_pose_topic` 기본값(`/openarmx/left_ee_pose`, 언더스코어 구분)과 bimanual의 `/openarmx/left/ee_pose`(슬래시 구분)가 다름에 유의한다.

### 2.6 URDF 주입 경로

```
launch/openarmx_movel_bimanual.launch.py:27
```

`PathJoinSubstitution([pkg, "urdf", f"openarmx_{side}_solver.urdf"])`로 동적 생성된다. `pkg`는 `FindPackageShare("openarmx_pick")`이다. 양팔 URDF 모두 `openarmx_body_link0`를 루트로 가지며, SRDF는 공용 인자 `srdf_path`로 전달된다.

### 2.7 inter-arm 충돌 CBF 상태

docstring(`launch/openarmx_movel_bimanual.launch.py:14-16`)이 명시한다: "Stage 1 uses the --no-collision solver URDFs (SRDF empty) -- only joint-limit and singularity CBF active. Add SRDFs and switch to collision URDFs later to enable inter-arm self-collision CBF." 즉, 현재는 두 solver가 서로의 존재를 모른 채 독립적으로 동작하며, 팔 간 자기 충돌 방지는 미구현 상태이다.

---

## 3. `openarmx_pick.launch.py` — Stage-B 전체 pick 파이프라인

### 3.1 파일 개요

```
launch/openarmx_pick.launch.py:1-58
```

비전 출력 소비(`/box_plane/cloud`, `/box_plane/info`)부터 파지 자세 합성, MoveL 명령 발행까지의 Stage-B 파이프라인을 단일 launch로 구성한다. 카메라·YOLO·`box_plane` 스택은 포함하지 않으며, 별도로 `run_yolov8_ros.sh`를 실행해야 한다는 점이 docstring에 명시되어 있다(`launch/openarmx_pick.launch.py:11-12`).

### 3.2 LaunchArgument 목록

| 인자 | 기본값 | 설명 |
|---|---|---|
| `auto_send` | `"false"` | `"true"` 시 `grasp_pose_node`가 pre-grasp MoveL을 자동 발행 |
| `pregrasp_height` | `"0.10"` | 박스 상단 대비 pre-grasp 높이 오프셋 (m) |
| `cloud_topic` | `"/box_plane/cloud"` | 박스 상단 인라이어 포인트클라우드 입력 토픽 |
| `start_solver` | `"true"` | `"false"` 시 MoveL solver 미기동 (grasp_pose_node만 실행) |

### 3.3 기동 노드 및 포함 launch

두 개의 launch 개체가 기동된다.

#### 3.3.1 IncludeLaunchDescription: `openarmx_movel.launch.py`

```
launch/openarmx_pick.launch.py:35-39
```

```python
solver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        PathJoinSubstitution([pkg, "launch", "openarmx_movel.launch.py"])),
    condition=IfCondition(LaunchConfiguration("start_solver")),
)
```

`start_solver:=false`이면 조건부로 생략된다. 포함 시 solver는 모든 기본값을 상속한다 — 즉, `openarmx_movel.launch.py`의 LaunchArgument 기본값들이 그대로 적용된다. `openarmx_pick.launch.py`는 solver 인자를 재정의(override)하지 않는다.

#### 3.3.2 Node: `grasp_pose_node`

```
launch/openarmx_pick.launch.py:41-56
```

| 속성 | 값 |
|---|---|
| `package` | `openarmx_pick` |
| `executable` | `grasp_pose_node` |
| `name` | `grasp_pose_node` |
| `output` | `screen` |

파라미터:

| 파라미터 | 값 | 출처 |
|---|---|---|
| `cloud_topic` | `LaunchConfiguration("cloud_topic")` | 인자, 기본 `/box_plane/cloud` |
| `info_topic` | `"/box_plane/info"` | 하드코딩 |
| `base_frame` | `"openarmx_body_link0"` | 하드코딩 |
| `movel_topic` | `"/openarmx/left/movel"` | 하드코딩 |
| `auto_send` | `LaunchConfiguration("auto_send")` | 인자, 기본 `false` |
| `pregrasp_height` | `LaunchConfiguration("pregrasp_height")` | 인자, 기본 `0.10` |
| `grasp_depth` | `0.005` | 하드코딩 (m, 박스 상면 아래 진입 깊이) |
| `cloud_stride` | `4` | 하드코딩 (PCA 속도용 포인트클라우드 서브샘플) |

`grasp_pose_node`는 remapping 없이 직접 토픽 이름 파라미터를 사용한다.

### 3.4 `auto_send` 동작 상세

`auto_send:=false`(기본): `grasp_pose_node`는 `/openarmx/grasp_pose` (PoseStamped)와 `/openarmx/grasp_markers` (MarkerArray)만 발행한다. MoveL은 발행되지 않는다.

`auto_send:=true`: 위에 더해 `/openarmx/left/movel` (openarmx_scenario_player_msgs/MoveL)을 발행한다. `grasp_pose_node.py:147-150` 참조. 단, `openarmx_scenario_player_msgs`가 import 불가능한 경우 경고를 출력하고 발행을 건너뛴다. 또한 디바운스 로직(`grasp_pose_node.py:226-238`)이 적용되어, 목표가 `send_min_delta=0.02 m` 이상 이동하거나 `send_min_interval=5.0 s` 이상 경과해야 재발행한다. 이는 카메라 프레임마다 MoveL을 재발행하면 solver가 매 사이클 궤적을 재시작해 팔이 기어가는 현상을 방지하기 위함이다(`grasp_pose_node.py:114-119`).

---

## 4. 노드 간 토폴로지

### 4.1 `openarmx_pick.launch.py` 기준 전체 데이터 흐름

```
[외부: 카메라 + YOLO + box_plane 스택]
        │
        ├─ /box_plane/cloud  (sensor_msgs/PointCloud2) ──────────────────┐
        └─ /box_plane/info   (std_msgs/String, JSON)  ─────────────────┐ │
                                                                        │ │
                                                             grasp_pose_node
                                                                        │
                         ┌──────────────────────────────────────────────┤
                         │                                              │
                         ▼                                              ▼
              /openarmx/grasp_pose              /openarmx/grasp_markers
              (PoseStamped, TRANSIENT_LOCAL)     (MarkerArray, TRANSIENT_LOCAL)
                         │
           [auto_send=true 시]
                         │
                         ▼
              /openarmx/left/movel
              (openarmx_scenario_player_msgs/MoveL)
                         │
                         ▼
              openarmx_left_movel_controller (omx_movel_controller_node)
                         │
           ┌─────────────┴───────────────┐
           │                             │
           ▼                             ▼
/openarmx/left_arm/                /openarmx/left_ee_pose
joint_trajectory                   (PoseStamped)
(trajectory_msgs/JointTrajectory)
           │
           ▼
  [ros2_control JTC / 하드웨어]
```

`/joint_states` (sensor_msgs/JointState)는 ros2_control 측에서 발행되어 `openarmx_left_movel_controller`가 구독한다.

TF (Transform, 좌표 변환) 조회: `grasp_pose_node`는 `tf2_ros.Buffer`를 통해 `openarmx_body_link0 ← cloud.header.frame_id` 변환을 런타임에 조회한다(`grasp_pose_node.py:171-176`). 정적 TF 발행자(`static_transform_publisher` 등)는 이 launch 파일에 포함되지 않으므로, 카메라 외부 교정(extrinsic calibration) TF는 별도로 발행되어 있어야 한다.

### 4.2 `openarmx_movel_bimanual.launch.py` 기준 토폴로지

```
/joint_states ──────────────────┬────────────────────────────────────┐
                                 │                                    │
                                 ▼                                    ▼
                  openarmx_left_movel_controller    openarmx_right_movel_controller
                        │          │                       │          │
                        ▼          ▼                       ▼          ▼
          /openarmx/left/ee_pose  /left_joint_trajectory_controller/joint_trajectory
                                                  /openarmx/right/ee_pose
                                                  /right_joint_trajectory_controller/joint_trajectory
```

MoveL 명령 입력:
- `/openarmx/left/movel` → `openarmx_left_movel_controller`
- `/openarmx/right/movel` → `openarmx_right_movel_controller`

두 노드는 서로 토픽을 공유하거나 참조하지 않는다. 결합도(coupling) 없는 독립 병렬 구조이다.

---

## 5. URDF 주입 메커니즘 요약

모든 launch 파일은 `FindPackageShare("openarmx_pick")`로 설치 경로를 해석한다. `setup.py:17`이 `urdf/*.urdf`를 `share/openarmx_pick/urdf/`로 설치하므로, 설치 전에는 경로가 존재하지 않는다. 소스 내 원본은 `urdf/openarmx_left_solver.urdf`와 `urdf/openarmx_right_solver.urdf`이다.

URDF 루트 프레임이 `openarmx_body_link0`로 고정되어 있어, 비전 파이프라인이 파지 자세를 같은 프레임으로 출력하면 추가 TF 변환 없이 MoveL 목표로 직접 전달 가능하다. 이 설계 의도는 `openarmx_movel.launch.py` docstring(`launch/openarmx_movel.launch.py:9-11`)에 명시되어 있다.

---

## 6. 각 launch 간 비교 요약

| 항목 | `openarmx_movel` | `openarmx_movel_bimanual` | `openarmx_pick` |
|---|---|---|---|
| 기동 노드 수 | 1 | 2 | 2 (solver 1 + grasp_pose_node 1) |
| 팔 수 | left only | left + right | left only |
| solver 실행 파일 | `omx_movel_controller_node` | `omx_movel_controller_node` × 2 | `omx_movel_controller_node` (포함) |
| `grasp_pose_node` | 없음 | 없음 | 있음 |
| `kp_position` | 4.0 (보수적) | 50.0 (upstream 기본) | 4.0 (상속) |
| `trajectory_time` | 0.05 | 0.0 | 0.05 (상속) |
| EE 토픽 네이밍 | `/openarmx/left_ee_pose` | `/openarmx/left/ee_pose` | `/openarmx/left_ee_pose` |
| `joint_command_topic` | `/openarmx/left_arm/joint_trajectory` (구버전) | `/{side}_joint_trajectory_controller/joint_trajectory` (JTC 표준) | 구버전 상속 |
| solver solver 조건부 기동 | 해당 없음 | 해당 없음 | `start_solver` 인자로 제어 가능 |
| SRDF | 인자화 (기본 `""`) | 인자화 (기본 `""`) | solver 상속 (기본 `""`) |

---

## 7. 알려진 주의 사항

1. **`joint_command_topic` 불일치**: `openarmx_movel.launch.py`의 기본값 `/openarmx/left_arm/joint_trajectory`는 ros2_control JTC 표준 경로가 아니다. `openarmx_movel_bimanual.launch.py` 내부 주석이 이 문제를 명시적으로 지적한다. `openarmx_pick.launch.py`는 solver를 포함(include)하므로 이 구버전 토픽을 그대로 상속한다.

2. **EE 토픽 네이밍 불일치**: `openarmx_movel.launch.py`는 `/openarmx/left_ee_pose`(언더스코어)를, `openarmx_movel_bimanual.launch.py`는 `/openarmx/left/ee_pose`(슬래시)를 사용한다. 두 launch를 혼용하거나 downstream subscriber가 고정 토픽명을 가정하는 경우 주의가 필요하다.

3. **TF 발행자 부재**: `openarmx_pick.launch.py`는 카메라-로봇 외부 교정 TF를 발행하지 않는다. `grasp_pose_node`의 TF 조회가 성공하려면 별도 스택에서 해당 TF가 발행되어 있어야 한다.

4. **`auto_send` 기본값 `false`**: 기본 실행 시 `grasp_pose_node`는 자세만 합성·발행하고 MoveL을 보내지 않는다. 실제 팔 이동을 트리거하려면 `auto_send:=true`를 명시하거나 `/openarmx/left/movel`에 외부 발행자가 필요하다.

5. **Stage-1 한계**: 3개 launch 파일 모두 SRDF를 비워 충돌 CBF를 비활성화한 Stage-1 구성이다. 양팔 협업 시나리오에서는 inter-arm 자기충돌 방지가 보장되지 않는다.
