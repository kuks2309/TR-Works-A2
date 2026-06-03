# openarmx_pick — 의존성 & 빌드

분석일: 2026-06-03  
패키지: openarmx_pick

---

## 분석 범위

본 문서는 `openarmx_pick` 패키지의 **런타임 의존성**, **테스트 의존성**, **빌드 시스템 구성**(build system configuration), **설치 레이아웃**, 그리고 **빌드 절차**(workspace overlay 포함)를 다룬다. 아키텍처·그랩 알고리즘·검증 절차는 다른 문서에서 다루므로 본 문서에서 중복하지 않는다.

---

## 1. 빌드 타입

`package.xml:29`에 선언된 빌드 타입은 `ament_python`이다.

```xml
<export>
  <build_type>ament_python</build_type>
</export>
```

`ament_python`은 C++ 컴파일 단계 없이 Python 소스를 그대로 설치하는 ROS 2(Robot Operating System 2) 빌드 시스템이다. 따라서 `CMakeLists.txt`는 존재하지 않으며 `setup.py`와 `setup.cfg`가 빌드 및 설치를 담당한다.

---

## 2. 런타임 의존성 (exec_depend)

`package.xml:14-21`에 다음 7개의 `exec_depend`가 선언되어 있다.

| 패키지 | 역할 |
|---|---|
| `rclpy` | ROS 2 Python 클라이언트 라이브러리 — 노드 생성, 토픽 pub/sub, 파라미터 |
| `geometry_msgs` | `PoseStamped`, `Point` 메시지 타입 — 그랩 포즈 출력에 사용 |
| `std_msgs` | `String` 메시지 타입 — `/box_plane/info` JSON 수신에 사용 |
| `visualization_msgs` | `MarkerArray`, `Marker` 타입 — RViz(Robot Visualization) 시각화 마커 출력 |
| `tf2_ros` | TF2(Transform Framework 2) — 카메라 프레임→베이스 프레임 좌표 변환 |
| `cyclo_motion_controller_ros` | QP(Quadratic Programming, 이차 계획법)+CBF(Control Barrier Function, 제어 장벽 함수) MoveL 솔버 노드(`omx_movel_controller_node` 실행 파일) |
| `openarmx_scenario_player_msgs` | `MoveL.msg` 정의 — 솔버로 전송하는 목표 포즈+시간 메시지 |

`grasp_pose_node.py`는 `openarmx_scenario_player_msgs`를 소프트 의존성으로 처리한다(`grasp_pose_node.py:38-42`):

```python
try:
    from openarmx_scenario_player_msgs.msg import MoveL
    _HAVE_MOVEL = True
except Exception:  # MoveL only needed when auto_send=True
    _HAVE_MOVEL = False
```

`MoveL` 임포트 실패 시 노드가 종료되지 않고 `auto_send=False` 모드로 동작한다. 단, `package.xml`에는 `exec_depend`로 정식 선언되어 있으므로 colcon 의존성 그래프에서는 hard 의존성으로 취급된다.

### 2.1 `openarmx_scenario_player_msgs/MoveL` 메시지 구조

`openarmx_scenario_player_msgs/msg/MoveL.msg`:

```
geometry_msgs/PoseStamped     pose
builtin_interfaces/Duration   time_from_start
```

이 메시지는 원래 `robotis_interfaces/MoveL`에서 파생되었으며, `openarmx_scenario_player` 스택이 벤더 특화 메시지 패키지에 종속되지 않도록 로컬에 재정의된 것이다. README.md:51-52에 마이그레이션 내역이 명시되어 있다.

> `openarmx_scenario_player_msgs` 패키지는 `ament_cmake` 빌드 타입(`openarmx_scenario_player_msgs/package.xml:20`)이며, `rosidl_default_generators`를 통해 C++/Python 바인딩을 생성한다. 따라서 `openarmx_pick`보다 먼저 빌드되어야 한다.

### 2.2 `cyclo_motion_controller_ros` 의존성 구조

`cyclo_motion_controller_ros/package.xml`에 선언된 핵심 빌드/런타임 의존성은 다음과 같다:

| 의존성 | 유형 | 비고 |
|---|---|---|
| `pinocchio` | `build_depend` | C++ FK(Forward Kinematics, 순기구학)/Jacobian 라이브러리 |
| `osqp_eigen_vendor` | `build_depend` + `exec_depend` | OSQP(Operator Splitting Quadratic Program) solver 벤더 패키지 |
| `cyclo_motion_controller_core` | `depend` | QP+CBF 핵심 수치 라이브러리 |
| `cyclo_motion_controller_models` | `exec_depend` | 로봇 모델 파일 |
| `openarmx_scenario_player_msgs` | `depend` | MoveL 명령 수신용 — cyclo_control도 동일 메시지 사용 |

`pinocchio`가 C++ 노드 빌드 시 노드 1개당 약 2~3 GB의 RAM(Random Access Memory, 메모리)을 소비하는 이유는 `pinocchio`가 Eigen 기반의 헤더-온리 템플릿 라이브러리이기 때문이다(추정). 병렬 컴파일 시 복수의 링크 단위가 동시에 메모리를 점유하여 OOM(Out-Of-Memory, 메모리 부족)이 발생할 수 있다.

---

## 3. 테스트 의존성 (test_depend)

`package.xml:23-26`에 4개의 `test_depend`가 선언되어 있다:

```xml
<test_depend>ament_copyright</test_depend>
<test_depend>ament_flake8</test_depend>
<test_depend>ament_pep257</test_depend>
<test_depend>python3-pytest</test_depend>
```

| 도구 | 역할 |
|---|---|
| `ament_copyright` | 소스 파일 저작권 헤더 존재 여부 검사 |
| `ament_flake8` | PEP 8 스타일 및 Python 문법 오류 린팅 |
| `ament_pep257` | docstring 형식 검사 |
| `python3-pytest` | 단위 테스트 실행 프레임워크 |

현재 `test/` 디렉토리는 별도로 확인되지 않으나(추정), `setup.py:26`에 `tests_require=["pytest"]`가 명시되어 있다.

---

## 4. 빌드 시스템 구성

### 4.1 `setup.py`

`setup.py`는 표준 `setuptools`를 사용하며(`setup.py:3`), `find_packages(exclude=["test"])`로 Python 패키지를 자동 탐색한다(`setup.py:10`).

#### 4.1.1 `data_files` — 설치 레이아웃

`setup.py:11-19`에 정의된 `data_files`는 colcon 빌드 시 `install/openarmx_pick/share/openarmx_pick/` 하위에 다음 리소스를 설치한다:

| 소스 경로 패턴 | 설치 대상 |
|---|---|
| `resource/openarmx_pick` | `share/ament_index/resource_index/packages/` (패키지 등록 마커) |
| `package.xml` | `share/openarmx_pick/` |
| `launch/*.launch.py` | `share/openarmx_pick/launch/` |
| `urdf/*.urdf` | `share/openarmx_pick/urdf/` |
| `config/*` | `share/openarmx_pick/config/` |
| `scripts/*.py` | `share/openarmx_pick/scripts/` |

`glob`을 사용하여 파일을 수집하므로 향후 파일 추가 시 `setup.py` 수정 없이 자동 포함된다. 단, `--symlink-install` 옵션 사용 시 각 파일에 대한 심볼릭 링크가 생성되어 소스 수정이 즉시 반영된다.

현재 설치되는 launch 파일은 3개이다:
- `launch/openarmx_movel.launch.py` — 단일(좌) 팔 MoveL 솔버
- `launch/openarmx_movel_bimanual.launch.py` — 좌/우 양팔 솔버
- `launch/openarmx_pick.launch.py` — 솔버 + `grasp_pose_node` 통합 실행

URDF(Unified Robot Description Format, 통합 로봇 기술 형식) 파일은 2개이다:
- `urdf/openarmx_left_solver.urdf` — 7-DOF(Degrees of Freedom, 자유도) 좌팔 단독 솔버 모델
- `urdf/openarmx_right_solver.urdf` — 7-DOF 우팔 단독 솔버 모델

#### 4.1.2 `entry_points` — 실행 파일 등록

`setup.py:27-32`에 하나의 console_scripts 진입점이 등록된다:

```python
entry_points={
    "console_scripts": [
        # Stage B: box_plane/info -> top-down grasp PoseStamped / MoveL
        "grasp_pose_node = openarmx_pick.grasp_pose_node:main",
    ],
},
```

이 선언으로 `ros2 run openarmx_pick grasp_pose_node` 명령이 `openarmx_pick/grasp_pose_node.py`의 `main()` 함수를 실행한다. 설치된 실행 파일의 경로는 `setup.cfg`에 의해 결정된다.

### 4.2 `setup.cfg`

`setup.cfg` 전체 내용:

```ini
[develop]
script_dir=$base/lib/openarmx_pick
[install]
install_scripts=$base/lib/openarmx_pick
```

`ament_python` 빌드에서 `$base`는 colcon 빌드의 install prefix(접두 경로)로 치환된다. 실행 파일은 `install/openarmx_pick/lib/openarmx_pick/grasp_pose_node`에 배치되며, ROS 2 환경에서 `ros2 run`이 이 경로를 탐색한다.

---

## 5. Workspace Overlay(작업공간 오버레이) 구조

`openarmx_pick`은 단독으로 빌드되지 않으며, 두 개의 별도 workspace(작업공간)가 순서대로 빌드된 뒤 세 개의 overlay를 sourcing(환경 활성화)해야 동작한다.

```
/opt/ros/humble/          ← base layer (ROS 2 Humble Hawksbill 공식 배포)
    └─ overlays ──▶ ~/TR-Works/kkw/China/cyclo_ws/   ← cyclo_control 솔버 레이어
                    └─ overlays ──▶ ~/TR-Works/kkw/China/openarmx_ws/  ← 이 패키지 레이어
```

### 5.1 1단계: base layer sourcing

```bash
source /opt/ros/humble/setup.bash
```

### 5.2 2단계: `cyclo_ws` 빌드 (단일 스레드 필수)

`cyclo_ws/src/`에는 `cyclo_control` 심볼릭 링크(또는 클론)가 존재하며, `cyclo_motion_controller_ros`(Pinocchio 기반 C++ 패키지)를 포함한다.

```bash
cd ~/TR-Works/kkw/China/cyclo_ws
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --symlink-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1
```

**단일 스레드 빌드가 필수인 이유:** Pinocchio(C++ 리지드 바디 역학 라이브러리) 기반의 C++ 노드는 컴파일 단위별로 2~3 GB의 RAM을 소비한다(README.md:55-56). 기본 `-j8` 설정으로 빌드하면 최대 15 GB 이상의 메모리가 동시에 요구되어 OOM 강제 종료(reboot)가 발생할 수 있다. `MAKEFLAGS=-j1`과 `--executor sequential`의 조합으로 make와 colcon 양쪽에서 병렬도를 1로 제한한다. 이 빌드에 약 21분이 소요된다(README.md:56).

`cyclo_ws`에서 빌드되는 주요 패키지:

| 패키지 | 빌드 타입 | 비고 |
|---|---|---|
| `cyclo_motion_controller_core` | `ament_cmake` | QP+CBF 핵심 수치 라이브러리 |
| `cyclo_motion_controller_ros` | `ament_cmake` | `omx_movel_controller_node` 실행 파일 포함 |
| `cyclo_motion_controller_ros_py` | `ament_python` | Python 래퍼 |
| `osqp_eigen_vendor` | `ament_cmake` | OSQP solver 벤더 패키지 |
| `robotis_interfaces` | `ament_cmake` | 레거시 메시지 (현재 MoveL 스택에서는 미사용) |

### 5.3 3단계: `openarmx_ws` 빌드

`cyclo_ws`의 install을 source한 뒤 `openarmx_ws`를 빌드한다:

```bash
cd ~/TR-Works/kkw/China/openarmx_ws
colcon build --packages-select openarmx_pick --symlink-install
```

`openarmx_pick`은 순수 Python 패키지이므로 C++ 컴파일이 없어 빠르게 빌드된다. `openarmx_ws`에는 `cyclo_robot_controller/` 하위에 `cyclo_motion_controller_ros`의 사본이 존재하며(`openarmx_ws/src/cyclo_robot_controller/`), 이는 `cyclo_ws`와 별도로 관리되는 로컬 사본이다(추정).

`openarmx_ws`에서 함께 빌드되어야 하는 핵심 의존 패키지:

| 패키지 | 경로 | 빌드 타입 |
|---|---|---|
| `openarmx_scenario_player_msgs` | `src/openarmx_ros2/openarmx_scenario_player_msgs/` | `ament_cmake` |
| `cyclo_motion_controller_ros` | `src/cyclo_robot_controller/cyclo_motion_controller_ros/` | `ament_cmake` |

`openarmx_scenario_player_msgs`는 `ament_cmake`(`rosidl_default_generators`) 패키지이므로 `openarmx_pick`보다 먼저 빌드되어야 한다. colcon이 `package.xml` 의존성 그래프를 자동으로 해석하여 빌드 순서를 결정한다.

### 5.4 실행 전 3-overlay Sourcing

노드 실행 전 반드시 세 개의 overlay를 순서대로 source해야 한다(README.md:77-81):

```bash
source /opt/ros/humble/setup.bash
source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash
source ~/TR-Works/kkw/China/cyclo_ws/install/setup.bash
```

sourcing 순서가 중요하다: `cyclo_ws`를 마지막에 source해야 `cyclo_motion_controller_ros`의 실행 파일(`omx_movel_controller_node`)이 `PATH`와 `AMENT_PREFIX_PATH`에서 올바르게 해석된다.

---

## 6. apt 의존성

README.md:70-71에 명시된 시스템 패키지 의존성:

```bash
sudo apt install \
  ros-humble-pinocchio \
  ros-humble-osqp-vendor \
  ros-humble-ament-cmake-vendor-package \
  python3-nlopt
```

| 패키지 | 용도 |
|---|---|
| `ros-humble-pinocchio` | Pinocchio(리지드 바디 역학 C++ 라이브러리) — FK/Jacobian 계산 |
| `ros-humble-osqp-vendor` | OSQP(Operator Splitting Quadratic Program) solver apt 배포판 |
| `ros-humble-ament-cmake-vendor-package` | vendor 패키지 빌드 헬퍼 CMake 모듈 |
| `python3-nlopt` | NLopt(Non-Linear Optimization, 비선형 최적화 라이브러리) — 추정: 특정 cyclo 모드에서 사용 |

`python3-nlopt`의 구체적인 사용 위치는 `openarmx_pick` 소스 내에서 직접 확인되지 않으므로 **추정**으로 명시한다(cyclo_control 내부 의존성으로 추정).

---

## 7. `robotis_interfaces` → `openarmx_scenario_player_msgs` 마이그레이션

MoveL 명령 메시지의 원본은 ROBOTIS사의 `robotis_interfaces/MoveL`이었다. 현재는 `openarmx_scenario_player_msgs/MoveL`로 완전히 마이그레이션되었다(README.md:51-52):

> the old `robotis_interfaces` msg is no longer used — the whole MoveL stack migrated to `openarmx_scenario_player_msgs`

`openarmx_scenario_player_msgs/msg/MoveL.msg`의 코멘트에도 기원이 명시되어 있다:

```
# Origin: cyclo_motion_controller_ros (forked from robotis_interfaces/MoveL).
# Defined locally to keep openarmx_scenario stack independent of any
# robot-vendor-specific message package.
```

마이그레이션 결과:
- `openarmx_pick/package.xml`에 `robotis_interfaces` 의존성이 없다.
- `cyclo_motion_controller_ros/package.xml`도 `openarmx_scenario_player_msgs`를 `depend`로 선언하여 동일 메시지를 사용한다.
- `cyclo_ws/src/robotis_interfaces`는 여전히 소스에 존재하지만 MoveL 스택에서는 참조되지 않는다.

---

## 8. `.gitignore` 구성

`.gitignore`는 다음 세 범주를 무시한다:

```
# Python
__pycache__/
*.py[cod]
*.egg-info/

# ROS / colcon (this package is built from the parent workspace)
build/
install/
log/

# OMC / tooling state
.omc/
```

`build/`, `install/`, `log/`는 패키지 루트가 아닌 상위 workspace 루트에 생성되므로(colcon은 workspace 루트에서 실행), 이 항목들은 실질적으로 패키지 내부에서 생성될 가능성이 낮다. 주석 `"this package is built from the parent workspace"`가 이를 확인해 준다.

---

## 9. 요약

| 항목 | 값 |
|---|---|
| 빌드 타입 | `ament_python` |
| Python 버전 | CPython 3.10 (`__pycache__` 파일명 기준 확인) |
| 패키지 버전 | 0.1.0 |
| 라이선스 | Apache-2.0 |
| exec_depend 수 | 7개 |
| test_depend 수 | 4개 |
| entry_points | 1개 (`grasp_pose_node`) |
| 설치되는 data 디렉토리 | launch, urdf, config, scripts, resource |
| 빌드 workspace | `openarmx_ws` (Python, 빠름) |
| 솔버 빌드 workspace | `cyclo_ws` (C++, `-j1` 필수, ~21분) |
| sourcing overlay 수 | 3개 (base → openarmx_ws → cyclo_ws) |
| apt 의존성 | pinocchio, osqp-vendor, ament-cmake-vendor-package, python3-nlopt |
