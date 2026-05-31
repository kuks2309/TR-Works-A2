# cyclo_control 저장소 분석

- **원본 저장소**: https://github.com/ROBOTIS-GIT/cyclo_control
- **클론 위치**: `/home/openarmx/TR-Works/kkw/China/cyclo_control`
- **분석일**: 2026-05-14
- **개요**: ROBOTIS Physical AI 로봇 라인업(AI Worker / OpenMANIPULATOR-X / OpenMANIPULATOR-Y)을 위한 ROS 2 Jazzy 기반 모션 제어 스택. **QP(이차계획)** 기반 역기구학을 핵심으로 한다.

---

## 1. 저장소 구조 — 6개 ROS 2 패키지

| 패키지 | 역할 |
|---|---|
| `cyclo_control/` | 메타 패키지 (그룹화용) |
| `cyclo_motion_controller_core/` | ROS 비의존 C++ 코어 (KinematicsSolver, QPBase, 컨트롤러 알고리즘) + Python 리타게팅 |
| `cyclo_motion_controller_ros/` | ROS 2 노드 래퍼 + launch + config |
| `cyclo_motion_controller_ros_py/` | Python 노드 (VR 리타게팅, 손 리타게팅) |
| `cyclo_motion_controller_models/` | URDF/SRDF 모델 + RViz 뷰어 |
| `osqp_eigen_vendor/` | osqp-eigen 솔버 벤더 패키지 |

`cyclo_control_ci.repos`는 [robotis_interfaces](https://github.com/ROBOTIS-GIT/robotis_interfaces) (MoveL 메시지 등) 의존성을 가져온다.

---

## 2. 아키텍처 핵심

```
ROS 노드 (ros)          ──>  컨트롤러 (core/controllers)  ──>  QPBase (core/optimization)
       │                            │                                 │
   joint_states              KinematicsSolver                     osqp-eigen
   /goal_pose            (Pinocchio FK/IK + 충돌)                  (이차계획 풀이)
                                                                       │
                                                              opt_qdot (목표 관절속도)
```

### 3계층 설계

1. **Kinematics 계층** (`cyclo_motion_controller_core/include/.../kinematics/kinematics_solver.hpp`)
   - Pinocchio 기반 URDF/SRDF 로딩
   - FK / Jacobian / 충돌쌍 거리·그래디언트 계산
   - 관절 위치·속도 한계, 링크·조인트 프레임 조회

2. **Optimization 계층** (`cyclo_motion_controller_core/include/.../optimization/qp_base.hpp`)
   - OSQP-Eigen 래퍼 추상 클래스
   - 비용 / 등호 / 부등호 / 경계 제약을 가상함수로 분리
   - **희소 패턴 캐시**: 동일 패턴이면 솔버 재초기화를 건너뛰어 100Hz 실시간 제어 보장
   - QP 형태: `min ½ x' P x + q' x  s.t.  l ≤ A x ≤ u`

3. **Controller 계층** — `QPBase`를 상속한 컨트롤러:
   - `VRController`: 작업공간 트래킹 + CBF(Control Barrier Function) 충돌 회피 + 특이점/관절한계 슬랙 변수
   - `AIWorkerMoveLController` ← `VRController` 상속 (양팔)
   - `AIWorkerMoveJController`
   - `OpenManipulatorMoveJController` / `OpenManipulatorMoveLController` (OMX/OMY 공용)

---

## 3. 제어 모드별 노드

`ai_worker_controller.launch.py`에서 `controller_type` 인자로 분기:

| controller_type | 실행 노드 | 입력 토픽/메시지 |
|---|---|---|
| `movel` (기본) | `ai_worker_movel_controller_node` + (옵션) 양손 인터랙티브 마커 | `/r_goal_move`, `/l_goal_move` (`robotis_interfaces/msg/MoveL`) |
| `movej` | `ai_worker_movej_controller_node` | `/.../raw_joint_trajectory` (`trajectory_msgs/msg/JointTrajectory`) |
| `vr` | `vr_controller_node` + `reference_checker_node` + `arm_retargeting_teleop` (+ 옵션 `hand`) | `/r_goal_pose`, `/l_goal_pose` (`PoseStamped`) |
| `leader` | `leader_controller_node` + `vr_controller_node` | 리더 암 관절상태 → 골 포즈 |

OMX/OMY 도 동일 패턴(`movel`/`movej`)으로 `omx_controller.launch.py`, `omy_controller.launch.py` 제공.

### 실행 예시

```bash
# AI Worker MoveL + 인터랙티브 마커
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
    controller_type:=movel start_interactive_marker:=true

# AI Worker VR 텔레옵 (팔 리타게팅 활성 + 손 추가)
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
    controller_type:=vr hand:=true

# 그리퍼 간 충돌 검사만 끄기 (핸드오버용)
ros2 launch cyclo_motion_controller_ros ai_worker_controller.launch.py \
    disable_gripper_collisions:=true

# OMX/OMY
ros2 launch cyclo_motion_controller_ros omx_controller.launch.py start_interactive_marker:=true
ros2 launch cyclo_motion_controller_ros omy_controller.launch.py start_interactive_marker:=true

# 모델 시각화
ros2 launch cyclo_motion_controller_models view_ffw_sg2_follower.launch.py
ros2 launch cyclo_motion_controller_models view_omx_f.launch.py
ros2 launch cyclo_motion_controller_models view_omy_f3m.launch.py
```

### MoveL 명령 예시

```bash
ros2 topic pub --once /r_goal_move robotis_interfaces/msg/MoveL "{
  pose: {
    header: {frame_id: 'base_link'},
    pose: {
      position: {x: 0.35, y: -0.20, z: 0.85},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  },
  time_from_start: {sec: 2, nanosec: 0}
}"
```

---

## 4. VR/리더 리타게팅 파이프라인

`cyclo_motion_controller_ros_py/scripts/arm_retargeting.py`는 사람 어깨/팔꿈치/손목 `PoseStamped`를 받아:

1. 어깨→팔꿈치, 팔꿈치→손목 **방향 벡터**만 추출
2. 로봇 자체의 상박/전박 길이를 곱해 로봇 좌표계 골 산출
3. **양손 간 거리 우선 보정** (지수 감쇠) + 전박길이 재투영 + 저역통과 평활화
4. 사람 양손 사이 상대 자세를 로봇 양손에 강제 (쿼터니언 곱)

손 리타게팅은 [`dex-retargeting`](https://github.com/dexsuite/dex-retargeting)을 차용 — `cyclo_motion_controller_core/src/retargeting/optimizer.py`, `seq_retarget.py`, `robot_wrapper.py`.

---

## 5. 컨트롤러 파라미터 (`ai_worker_config.yaml`)

| 카테고리 | 파라미터 | 값 |
|---|---|---|
| 제어 주기 | `control_frequency`, `time_step` | 100 Hz, 0.01 s |
| 트래킹 게인 | `kp_position`, `kp_orientation` | 50, 50 |
| 비용 가중치 | `weight_position` vs `weight_orientation` | 10 vs 1 (위치 우선) |
| 댐핑 | `weight_damping` | 0.1 |
| CBF | `collision_buffer`, `collision_safe_distance` | 0.05, 0.02 |
| CBF | `slack_penalty`, `cbf_alpha` | 1000, 50 |
| VR 전용 | `weight_elbow_position` | 8.0 (팔꿈치 보조 트래킹) |

---

## 6. 지원 로봇 모델 (`cyclo_motion_controller_models/models/`)

- **AI Worker**: `ffw_sg2_follower` (양팔 + 그리퍼), `ffw_lg2_leader`, hx5_d20 양팔 (left/right)
- **OMX**: `omx_f` (팔로워), `omx_l` (리더)
- **OMY**: `omy_3m`, `omy_f3m`, `omy_l100`

URDF + 기본/수정 SRDF 쌍 제공 — `disable_gripper_collisions:=true` 시 그리퍼간 충돌 검사만 끄는 `_modified.srdf`로 전환된다.

---

## 7. 빌드 방법

### 전제조건

- ROS 2 **Jazzy** 설치
- `numpy<2`
- Pinocchio, Eigen3, nlopt (rosdep로 자동)
- osqp-eigen은 `osqp_eigen_vendor`로 벤더링

### 빌드 절차

```bash
cd ~/ros2_ws/src
git clone https://github.com/ROBOTIS-GIT/cyclo_control.git
vcs import . < cyclo_control/cyclo_control_ci.repos

cd ~/ros2_ws
sudo apt update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## 8. 의존성 그래프 요약

```
cyclo_motion_controller_ros        ─┬─ cyclo_motion_controller_core
                                    ├─ cyclo_motion_controller_models  (exec)
                                    └─ cyclo_motion_controller_ros_py  (exec)

cyclo_motion_controller_core       ─┬─ Eigen3
                                    ├─ pinocchio
                                    └─ osqp_eigen_vendor

cyclo_motion_controller_ros_py     ─┬─ rclpy
                                    ├─ cyclo_motion_controller_core (Python: retargeting)
                                    └─ cyclo_motion_controller_models (exec)
```

---

## 9. 출처와 라이선스

- **Apache 2.0** (저장소 전체), 일부 리타게팅 코드는 **MIT**.
- 컨트롤러 구현: 서울대 [`dyros_robot_controller`](https://github.com/JunHeonYoon/dyros_robot_controller) (JunHeonYoon, 2025) 파생.
- 리타게팅: [`dex-retargeting`](https://github.com/dexsuite/dex-retargeting) 파생.
- 기구학: [`pinocchio`](https://github.com/stack-of-tasks/pinocchio).
- QP 풀이: [`osqp-eigen`](https://github.com/robotology/osqp-eigen) (벤더링됨).

### 메인테이너 (`package.xml`)

- Pyo `<pyo@robotis.com>` (maintainer)
- Yeonguk Kim `<kyu@robotis.com>` (author)
- Hyunwoo Nam `<nhw@robotis.com>` (author, Python 패키지)

---

## 10. 한 줄 요약

> Pinocchio로 FK/Jacobian/충돌거리를 계산하고 OSQP로 **작업공간 추종 + CBF 충돌회피 + 특이점·관절한계 슬랙**을 단일 QP로 풀어 **100Hz 관절속도 명령**을 내는, ROBOTIS Physical AI 로봇용 통합 모션 제어 스택. **MoveL/MoveJ/VR/Leader** 4가지 텔레옵 모드를 지원하며, VR 모드에서는 사람 팔 자세를 로봇 팔 길이에 맞춰 리타게팅한다.

---

## 11. 주요 파일 인덱스

### 핵심 헤더

- `cyclo_motion_controller_core/include/cyclo_motion_controller_core/kinematics/kinematics_solver.hpp`
- `cyclo_motion_controller_core/include/cyclo_motion_controller_core/optimization/qp_base.hpp`
- `cyclo_motion_controller_core/include/cyclo_motion_controller_core/common/type_define.hpp`
- `cyclo_motion_controller_core/include/cyclo_motion_controller_core/controllers/ai_worker/vr_controller.hpp`

### 핵심 구현

- `cyclo_motion_controller_core/src/kinematics/kinematics_solver.cpp`
- `cyclo_motion_controller_core/src/controllers/ai_worker/vr_controller.cpp`
- `cyclo_motion_controller_core/src/controllers/ai_worker/ai_worker_movel_controller.cpp`
- `cyclo_motion_controller_core/src/controllers/ai_worker/ai_worker_movej_controller.cpp`
- `cyclo_motion_controller_core/src/controllers/open_manipulator/open_manipulator_movel_controller.cpp`
- `cyclo_motion_controller_core/src/controllers/open_manipulator/open_manipulator_movej_controller.cpp`

### ROS 노드

- `cyclo_motion_controller_ros/src/nodes/ai_worker/*.cpp`
- `cyclo_motion_controller_ros/src/nodes/omx/*.cpp`
- `cyclo_motion_controller_ros/src/nodes/omy/*.cpp`
- `cyclo_motion_controller_ros/src/utils/reference_checker_node.cpp`
- `cyclo_motion_controller_ros/src/utils/eef_interactive_marker_node.cpp`

### Python 리타게팅

- `cyclo_motion_controller_ros_py/scripts/arm_retargeting.py`
- `cyclo_motion_controller_ros_py/scripts/teleop_retargeting.py`
- `cyclo_motion_controller_core/src/retargeting/{optimizer,seq_retarget,robot_wrapper}.py`

### Launch/Config

- `cyclo_motion_controller_ros/launch/ai_worker_controller.launch.py`
- `cyclo_motion_controller_ros/launch/omx_controller.launch.py`
- `cyclo_motion_controller_ros/launch/omy_controller.launch.py`
- `cyclo_motion_controller_ros/config/{ai_worker,omx,omy}_config.yaml`
