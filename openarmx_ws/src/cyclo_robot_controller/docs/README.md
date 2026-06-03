# cyclo_robot_controller 문서

QP(Quadratic Programming, 2차 계획법) + CBF(Control Barrier Function, 제어 장벽 함수) 기반
관절 매니퓰레이터 모션 컨트롤러 패키지 모음에 대한 기술 문서입니다.

> 본 문서는 소스 코드를 직접 분석하여 작성한 **사실 기반 레퍼런스**입니다.
> 코드가 변경되면 본 문서도 함께 갱신해야 하며, 코드와 문서가 충돌할 경우 **코드가 정답**입니다.

---

## 1. 패키지 개요

`cyclo_robot_controller/` 는 단일 ROS2 패키지가 아니라 **5개의 ament 패키지**를 담는 디렉터리입니다.
ROBOTIS 의 `cyclo_control`(레퍼런스: https://github.com/ROBOTIS-GIT/cyclo_control) 에서 가져온 것으로,
원본은 SNU DYROS 의 [`dyros_robot_controller`](https://github.com/JunHeonYoon/dyros_robot_controller)
(Apache-2.0, JunHeonYoon, 2025) 에서 파생되었습니다 (각 소스 헤더에 명시).

| 패키지 | 빌드 타입 | 역할 |
| --- | --- | --- |
| [`cyclo_motion_controller_core`](../cyclo_motion_controller_core) | `ament_cmake` | ROS 비의존 C++ 코어 라이브러리. 운동학(Pinocchio), QP 솔버(OSQP), MoveL/MoveJ/VR 컨트롤러 |
| [`cyclo_motion_controller_ros`](../cyclo_motion_controller_ros) | `ament_cmake` | 코어를 감싸는 ROS2 노드(10개 실행파일), launch, config |
| [`cyclo_motion_controller_ros_py`](../cyclo_motion_controller_ros_py) | `ament_python` | 텔레오퍼레이션 리타게팅(팔/손) 파이썬 노드 |
| [`cyclo_motion_controller_models`](../cyclo_motion_controller_models) | `ament_cmake` | URDF/SRDF/메시/RViz (OMX, OMY, AI Worker, HX5-D20 핸드) |
| [`osqp_eigen_vendor`](../osqp_eigen_vendor) | `ament_cmake` | OSQP-Eigen QP 솔버 vendoring (third-party 동봉) |

- **버전**: core/ros = `0.2.0` (2026-05-04), 라이선스 Apache-2.0 / MIT
- **유지보수**: Pyo (ROBOTIS), 저자: Yeonguk Kim, Hyunwoo Nam

---

## 2. 한 문장 요약

> 데카르트/관절 공간 목표를 받아, **운동학(Pinocchio)** 으로 야코비안·충돌거리를 구하고,
> **QP(OSQP)** 한 번 풀어 `qdot`(관절 속도)를 구한 뒤 적분하여 위치 명령을 100 Hz 로 내보내는 컨트롤러.
> 관절 한계와 자기 충돌은 **CBF 부등식 제약 + 슬랙(slack)** 으로 부드럽게 회피한다.

---

## 3. 문서 목차

| 문서 | 내용 |
| --- | --- |
| [01_architecture.md](01_architecture.md) | 전체 구조, 패키지 계층, 제어 데이터 흐름, 라이브러리 의존성 |
| [02_qp_cbf_formulation.md](02_qp_cbf_formulation.md) | QP 정식화(결정변수/비용/제약), CBF 관절한계·충돌회피 수식, 슬랙 |
| [03_controllers.md](03_controllers.md) | `QPBase` / MoveL / MoveJ / VR 컨트롤러 클래스, 운동학 솔버 |
| [04_ros_interface.md](04_ros_interface.md) | 노드별 토픽/파라미터/서비스, launch 인자, 실행파일 목록 |
| [05_parameters.md](05_parameters.md) | omy/omx/ai_worker config yaml 파라미터 전체 표 + 기본값 |

---

## 4. 빌드

`cyclo_motion_controller_ros` 는 빌드 시점에 `cyclo_motion_controller_core` 가 먼저 빌드되어 있어야
합니다(`find_package(... QUIET)` 후 install 경로 fallback). 외부 의존성:

- `eigen3`, `pinocchio`, `python3-nlopt`(core 런타임)
- `osqp_eigen_vendor`(동봉) → `OsqpEigen::OsqpEigen`
- ROS2: `rclcpp`, `std_msgs`, `std_srvs`, `geometry_msgs`, `visualization_msgs`,
  `sensor_msgs`, `trajectory_msgs`, `interactive_markers`, `tf2_ros`, `ament_index_cpp`
- `openarmx_scenario_player_msgs` (← `MoveL` 메시지 정의처)

```bash
# openarmx_ws 루트에서
colcon build --packages-up-to cyclo_motion_controller_ros
source install/setup.bash
```

> 빌드/실행 관련 이슈는 워크스페이스 루트의
> [docs/issues_and_fixes/README.md](../../../../docs/issues_and_fixes/README.md) 규칙을 따른다.

---

## 5. 빠른 실행 (OMY MoveL 예시)

```bash
ros2 launch cyclo_motion_controller_ros omy_controller.launch.py \
  controller_type:=movel \
  start_interactive_marker:=true
```

자세한 launch 인자·로봇별(OMX/AI Worker) 실행은 [04_ros_interface.md](04_ros_interface.md) 참고.
