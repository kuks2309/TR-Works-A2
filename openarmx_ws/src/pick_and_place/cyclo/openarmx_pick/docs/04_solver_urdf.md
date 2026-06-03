# Solver URDF 및 생성 스크립트 분석

분석일: 2026-06-03
패키지: openarmx_pick

## 분석 범위

본 문서는 `openarmx_pick` 패키지가 cyclo_control QP(Quadratic Programming, 이차계획법)+CBF(Control Barrier Function, 제어 장벽 함수) MoveL 솔버에 공급하기 위해 관리하는 **reduced single-arm solver URDF(Unified Robot Description Format, 통합 로봇 기술 형식)** 모델 두 종과, 해당 파일을 생성·재생성하는 `scripts/gen_solver_urdf.py` 스크립트를 분석한다. 다루는 대상은 다음과 같다.

- `scripts/gen_solver_urdf.py` — full xacro expansion 결과에서 solver URDF를 파생하는 변환 스크립트
- `urdf/openarmx_left_solver.urdf` — 왼팔이 movable인 7-DOF(Degree of Freedom, 자유도) 모델
- `urdf/openarmx_right_solver.urdf` — 오른팔이 movable인 7-DOF 모델

launch 파일, QoS(Quality of Service, 서비스 품질) 파라미터, grasp pose 파이프라인, 검증 스크립트의 상세 내용은 각각 별도 문서에서 다룬다.

---

## 1. 배경: 왜 별도의 solver URDF가 필요한가

cyclo_control의 `KinematicsSolver`는 Pinocchio 라이브러리를 사용하여 URDF를 파싱하고, **파일 안에 존재하는 movable joint(revolute/prismatic/continuous) 전체를 제어 자유도로 간주한다.** OpenArmX 로봇의 full URDF에는 왼팔 7개 + 오른팔 7개 관절, 양측 그리퍼 finger 관절이 모두 movable로 선언되어 있다. 이를 그대로 솔버에 공급하면 DOF가 14개 이상이 되어 단일 팔 IK(Inverse Kinematics, 역기구학) 문제의 Jacobian 차원이 불필요하게 커지고, collision CBF 적용 시 쌍체 arm 간 충돌 쌍이 폭발적으로 늘어 QP가 infeasible 상태에 빠진다.

따라서 `gen_solver_urdf.py`는 **반대 팔의 모든 관절과 양측 finger 관절을 `fixed`로 변환(freeze)** 하여 솔버 입장에서 이들이 정적 형상(static geometry)이 되도록 한다. 결과 파일은 DOF가 정확히 7이고 collision 처리 전략까지 단계별로 선택할 수 있는 경량 모델이다.

---

## 2. `scripts/gen_solver_urdf.py` 분석

### 2.1 진입점 및 인수

```
gen_solver_urdf.py IN.urdf OUT.urdf [--arm left|right] [--no-collision] [--strip-visual]
```

스크립트는 `argparse`를 사용하며 필수 위치 인수 두 개(입력/출력 파일)와 세 개의 옵션을 받는다 (`scripts/gen_solver_urdf.py:41-51`).

| 옵션 | 기본값 | 역할 |
|---|---|---|
| `--arm` | `left` | movable로 유지할 팔 선택 |
| `--no-collision` | 미지정 | 모든 `<collision>` 요소 제거 |
| `--strip-visual` | 미지정 | 모든 `<visual>` 요소 제거 |

### 2.2 joint freeze 로직

스크립트의 핵심은 `freeze_joint()` 함수와 joint 순회 루프다.

```python
# scripts/gen_solver_urdf.py:33-38
def freeze_joint(joint: ET.Element) -> None:
    joint.set("type", "fixed")
    for tag in _MOVABLE_ONLY:
        for el in joint.findall(tag):
            joint.remove(el)
```

`_MOVABLE_ONLY = ("axis", "limit", "dynamics", "mimic", "safety_controller")`로 선언된 하위 요소들을 제거한다(`scripts/gen_solver_urdf.py:30`). `<origin>`, `<parent>`, `<child>` 등 fixed joint에서도 유효한 요소는 그대로 보존된다.

joint 분류 기준은 다음 세 조건으로 결정된다(`scripts/gen_solver_urdf.py:60-77`):

```python
is_finger = "finger" in name
is_drop_arm = f"_{drop}_" in name          # 반대 팔 토큰이 이름에 있으면
is_keep_arm_joint = (f"_{keep}_joint" in name) and not is_finger
```

- `is_keep_arm_joint`가 참이면 → movable 유지 (`kept` 리스트)
- `is_drop_arm` 또는 `is_finger`이면 → freeze
- 위 어느 조건도 해당하지 않는 미지 movable joint → 방어적 freeze (DOF 누수 방지)

이름 기반 패턴 매칭이므로 joint 이름이 `openarmx_{arm}_joint{N}` 규칙을 따르는 한 정확히 동작한다.

### 2.3 collision 및 visual 처리

`--no-collision` 플래그 지정 시 모든 `<link>` 요소에서 `<collision>` 자식을 제거한다(`scripts/gen_solver_urdf.py:79-82`). 이것이 **stage-1 bring-up 운영 모드**다. 스크립트 docstring은 그 이유를 명시한다:

> cyclo의 KinematicsSolver는 항상 `buildGeom(COLLISION)` + `addAllCollisionPairs()`를 호출한다. SRDF(Semantic Robot Description Format, 의미적 로봇 기술 형식) 없이는 인접 링크를 포함한 모든 쌍이 활성화되어 collision-CBF QP가 infeasible 상태가 된다.

(`scripts/gen_solver_urdf.py:15-20`)

`--strip-visual` 옵션은 파일 크기 절감 목적으로 `<visual>` 요소를 추가 제거한다. 두 옵션은 독립적으로 조합 가능하다.

### 2.4 출력 및 진단 메시지

처리 후 `tree.write()`로 UTF-8 XML 선언을 포함한 파일을 저장하고, 표준 출력으로 kept/frozen 관절 목록과 collision/visual 처리 결과를 출력한다(`scripts/gen_solver_urdf.py:88-96`). 이 메시지는 생성 의도를 추후 확인하는 데 사용할 수 있다.

### 2.5 workflow 위치

`setup.py`는 `scripts/*.py`를 `share/openarmx_pick/scripts/`로 설치하므로, `ament` 빌드 후에도 접근 가능하다. 단, `console_scripts` entry_point로 등록되지 않아 `ros2 run`으로는 직접 실행할 수 없고 `python3`으로 호출해야 한다.

---

## 3. `urdf/openarmx_left_solver.urdf` 실측 분석

### 3.1 구조 개요

파일은 `--arm left --no-collision` 옵션으로 생성된 결과다 (collision 요소 수 = 0, visual 요소 수 = 23으로 확인). `--strip-visual`은 적용되지 않았다.

- robot name: `openarmx`
- 총 link 수: 30
- 총 joint 수: 29 (movable 7 + fixed 22)
- collision 요소: **0** (stage-1: stripped)
- visual 요소: **23** (보존됨)

### 3.2 movable joint 7개 (7-DOF 검증)

실제 파일을 파싱한 결과, `revolute` 타입 관절은 정확히 7개이며 전부 `openarmx_left_joint{1~7}`이다.

| 관절 | axis | lower [rad] | upper [rad] | effort [Nm] | velocity [rad/s] |
|---|---|---|---|---|---|
| `openarmx_left_joint1` | Z(0 0 1) | −3.3444 | 0.9056 | 120 | 10.47 |
| `openarmx_left_joint2` | −X(−1 0 0) | −3.2708 | 0.1292 | 120 | 10.47 |
| `openarmx_left_joint3` | Z(0 0 1) | −1.5700 | 1.5700 | 60 | 10.47 |
| `openarmx_left_joint4` | Y(0 1 0) | 0.0 | 1.8000 | 60 | 10.47 |
| `openarmx_left_joint5` | Z(0 0 1) | −1.5000 | 1.5000 | 14 | 10.47 |
| `openarmx_left_joint6` | X(1 0 0) | −0.7500 | 0.7500 | 14 | 10.47 |
| `openarmx_left_joint7` | −Y(0 −1 0) | −1.4000 | 1.4000 | 14 | 10.47 |

**7-DOF 주장 검증 결과: 정확히 7개의 revolute joint만 movable 상태다.** (`urdf/openarmx_left_solver.urdf:58-196` 참조, joint7 요소는 line 190 시작)

### 3.3 world → body_link0 identity

`openarmx_body_world_joint`는 `type="fixed"`이며 `origin xyz="0 0 0" rpy="0 0 0"`로 선언되어 있다(`urdf/openarmx_left_solver.urdf:7-11`). 즉 `world` 프레임과 `openarmx_body_link0` 프레임이 완전히 동일하다. launch 파일의 주석이 이 설계 의도를 설명한다:

> 비전 파이프라인이 grasp pose를 `openarmx_body_link0` 프레임으로 publish하므로, 솔버 FK에 추가 TF(Transform, 좌표 변환) 없이 goal pose를 바로 입력할 수 있다.

(`launch/openarmx_movel.launch.py:9-11`)

### 3.4 freeze된 고정 구조

오른팔 7개 관절(`openarmx_right_joint1~7`)은 모두 `type="fixed"`로 변환되었으며 `<axis>`, `<limit>` 등 movable 전용 하위 요소가 제거되었다(`urdf/openarmx_left_solver.urdf:237-361`). 오른팔은 링크 기하와 관성 데이터를 포함하는 정적 형상으로만 남는다.

왼팔 finger 관절(`openarmx_left_finger_joint1`, `openarmx_left_finger_joint2`)은 원본 full URDF에서도 이미 `fixed` 타입이었으므로 변환이 불필요했다. 오른팔 finger 관절도 마찬가지다.

`openarmx_left_joint4_ext` 및 `openarmx_left_link7_pico_joint`도 원래부터 `fixed`이므로 변환 대상이 아니다.

### 3.5 ros2_control 블록 부재

left solver URDF에는 `<ros2_control>` 블록이 포함되어 있지 않다 (파싱 결과 0개 확인). 이 파일은 Pinocchio 기반 KinematicsSolver가 직접 로드하는 전용 모델이며, ros2_control 하드웨어 인터페이스 스택과는 무관하다.

---

## 4. `urdf/openarmx_right_solver.urdf` 실측 분석

### 4.1 구조 개요

파일은 `--arm right --no-collision` 옵션으로 생성된 결과다.

- robot name: `openarmx`
- 총 link 수: 30
- 총 joint 수: 29 (movable 7 + fixed 22)
- collision 요소: **0** (stage-1: stripped)
- visual 요소: **23** (보존됨)

link 수와 joint 수가 left solver와 동일하다. 이는 양측 팔 링크와 손 관련 링크가 모두 포함되되 active 관절만 뒤바뀐 구조임을 뒷받침한다.

### 4.2 movable joint 7개 (7-DOF 검증)

| 관절 | axis | lower [rad] | upper [rad] | effort [Nm] | velocity [rad/s] |
|---|---|---|---|---|---|
| `openarmx_right_joint1` | Z(0 0 1) | −1.2500 | 3.0000 | 120 | 10.47 |
| `openarmx_right_joint2` | −X(−1 0 0) | −0.1292 | 3.2708 | 120 | 10.47 |
| `openarmx_right_joint3` | Z(0 0 1) | −1.5700 | 1.5700 | 60 | 10.47 |
| `openarmx_right_joint4` | Y(0 1 0) | 0.0 | 1.8000 | 60 | 10.47 |
| `openarmx_right_joint5` | Z(0 0 1) | −1.5000 | 1.5000 | 14 | 10.47 |
| `openarmx_right_joint6` | X(1 0 0) | −0.7500 | 0.7500 | 14 | 10.47 |
| `openarmx_right_joint7` | Y(0 **+**1 0) | −1.4000 | 1.4000 | 14 | 10.47 |

**7-DOF 주장 검증 결과: 정확히 7개의 revolute joint만 movable 상태다.** (`urdf/openarmx_right_solver.urdf:223-361` 참조)

### 4.3 ros2_control 블록 존재

right solver URDF에는 `<ros2_control>` 블록이 **2개** 포함되어 있다(`urdf/openarmx_right_solver.urdf:370-610`). 이는 full URDF 원본에 포함되어 있던 `ros2_control` 요소가 gen_solver_urdf.py의 변환 대상이 아니기 때문에 그대로 잔류한 것이다. `gen_solver_urdf.py`는 `joint` 및 `link` 요소만 처리하며 `ros2_control` 요소를 인식하거나 제거하지 않는다. left solver에 `ros2_control` 블록이 없는 것은 입력으로 사용된 full URDF에서 `ros2_control` 블록이 right solver 입력 파일에만 존재했던 것에 기인한다(추정). Pinocchio의 `urdf::buildModel()`은 알려지지 않은 XML 요소를 무시하므로 KinematicsSolver 동작에는 영향이 없다(추정).

---

## 5. left / right 미러 관계 비교

### 5.1 body_link0 기준 arm 장착 위치

두 팔은 `openarmx_body_link0`에서 좌우 대칭으로 장착된다.

| 팔 | parent joint 이름 | xyz | rpy |
|---|---|---|---|
| 왼팔 | `openarmx_left_openarmx_body_link0_joint` | `0.0 0.031 0.735` | `−1.5708 0 0` |
| 오른팔 | `openarmx_right_openarmx_body_link0_joint` | `0.0 −0.031 0.735` | `+1.5708 0 0` |

Y축 오프셋 부호(±0.031 m)와 roll 각도 부호(±π/2)가 정반대이며, Z 높이(0.735 m)는 동일하다.

### 5.2 mesh scale mirror

왼팔 link0~link3, link5~link7의 `<visual>` mesh는 scale `1.0 -1.0 1.0` (Y축 반전)이고, 오른팔은 `1.0 1.0 1.0`이다. link4는 양측 모두 `1.0 1.0 1.0`으로 동일하다.

### 5.3 관절 축 및 관절 한계 차이

대부분의 관절은 axis가 동일하지만, joint1과 joint7에서 한계 범위가 좌우 대칭이 아니다.

| 관절 번호 | left lower / upper [rad] | right lower / upper [rad] | 비고 |
|---|---|---|---|
| joint1 | −3.3444 / +0.9056 | −1.2500 / +3.0000 | 반전 비대칭 |
| joint2 | −3.2708 / +0.1292 | −0.1292 / +3.2708 | 부호 반전 대칭 |
| joint3 | ±1.57 | ±1.57 | 동일 |
| joint4 | 0.0 / 1.8 | 0.0 / 1.8 | 동일 |
| joint5 | ±1.5 | ±1.5 | 동일 |
| joint6 | ±0.75 | ±0.75 | 동일 |
| joint7 | axis `0 −1 0`, ±1.4 | axis `0 +1 0`, ±1.4 | **axis 부호 반전** |

joint7의 axis는 왼팔 `0 −1 0`, 오른팔 `0 +1 0`으로 Y축 부호가 반전되어 있다(`urdf/openarmx_left_solver.urdf:190-196`, `urdf/openarmx_right_solver.urdf:355-361`). joint1의 한계는 단순 부호 반전이 아니라 수치가 다르다(left: −3.3444~+0.9056, right: −1.25~+3.0). 이는 물리적 workspace 차이를 반영한 것으로, 동일한 링크 기구를 장착 방향만 바꾸었기 때문에 나타나는 설계상 비대칭이다.

### 5.4 관성 데이터 좌우 차이

동일 링크에 대한 관성 원점의 Y 성분이 좌우에서 부호가 반전된다. 예시: link6의 `<inertial origin>`.

- 왼팔: `xyz="-0.03564 0.005114 -0.003582"` (`urdf/openarmx_left_solver.urdf:164`)
- 오른팔: `xyz="-0.03564 -0.005114 -0.003582"` (`urdf/openarmx_right_solver.urdf:329`)

이 패턴은 원본 xacro가 좌우를 별도로 CAD 측정하여 관성 텐서를 설정하였음을 나타낸다.

---

## 6. collision 처리 단계 설계

스크립트 docstring은 두 단계 운영 계획을 명시한다(`scripts/gen_solver_urdf.py:15-22`).

| 단계 | collision 처리 | SRDF | CBF 활성 |
|---|---|---|---|
| stage-1 (현재 파일 상태) | `<collision>` 완전 제거 (`--no-collision`) | 없음 | joint-limit + singularity CBF만 |
| stage-2 (계획) | `<collision>` 보존 | SRDF로 인접 링크 쌍 제외 | collision CBF + joint-limit + singularity CBF |

현재 `urdf/openarmx_left_solver.urdf`와 `urdf/openarmx_right_solver.urdf` 모두 collision 요소가 0개이므로 **stage-1 상태**임이 확인된다. Pinocchio의 `buildGeom(COLLISION)` 호출이 빈 geometry model을 반환하며, 이로써 QP가 collision CBF 없이 관절 한계 및 특이점 CBF만으로 구동된다.

---

## 7. solver URDF와 launch 파라미터의 연결

`launch/openarmx_movel.launch.py`는 `urdf_path` 기본값으로 `openarmx_left_solver.urdf`를 지정하고, `controlled_link`는 `openarmx_left_hand_tcp`를 사용한다(`launch/openarmx_movel.launch.py:36-43`). TCP(Tool Center Point, 공구 중심점) 링크는 `openarmx_left_hand` → `openarmx_left_hand_tcp` 체인의 끝 링크로, `openarmx_left_link7`로부터 fixed joint 두 개를 통해 연결되며 end-effector 위치는 link7 기준 Z 방향 0.1001 + 0.08 = **0.1801 m** 전방이다(`urdf/openarmx_left_solver.urdf:384-395`).

`verify_solver.py`의 `MoveL` goal은 `frame_id="openarmx_body_link0"`으로 발행하고 있어(`scripts/verify_solver.py:67`), world==body_link0 identity 설계가 정상 동작해야 한다는 것을 검증 스크립트가 전제하고 있음을 확인할 수 있다.

---

## 8. 재생성 절차 (참고)

full xacro expansion 결과 URDF가 변경되었을 때 solver URDF를 재생성하는 표준 명령은 다음과 같다 (스크립트 docstring 및 옵션 분석으로부터 도출한 것이며, 실행 환경 경로는 추정):

```bash
# full URDF를 xacro로 먼저 전개
ros2 run xacro xacro /path/to/openarmx.urdf.xacro -o /tmp/full.urdf

# left solver 재생성 (stage-1: collision stripped)
python3 scripts/gen_solver_urdf.py /tmp/full.urdf urdf/openarmx_left_solver.urdf \
    --arm left --no-collision

# right solver 재생성
python3 scripts/gen_solver_urdf.py /tmp/full.urdf urdf/openarmx_right_solver.urdf \
    --arm right --no-collision
```

`--strip-visual`을 추가하면 파일 크기가 줄지만 RViz(ROS Visualization) 시각화가 불가능해지므로 현재 파일에는 적용하지 않았다.

---

## 9. 검증 결과 요약

| 항목 | 확인 방법 | 결과 |
|---|---|---|
| left solver movable joint 수 | XML 파싱 (`revolute` 카운트) | **7개** (joint1~7) |
| right solver movable joint 수 | XML 파싱 | **7개** (joint1~7) |
| root link | XML 파싱 (`world` 링크 + world_joint child) | `openarmx_body_link0` |
| world→body_link0 identity | origin xyz/rpy 확인 | xyz=`0 0 0`, rpy=`0 0 0` ✓ |
| collision 요소 수 (left) | XML 파싱 | **0** (stage-1 stripped) |
| collision 요소 수 (right) | XML 파싱 | **0** (stage-1 stripped) |
| visual 요소 수 (양측) | XML 파싱 | **23** (보존됨) |
| right 팔 관절 freeze (left solver) | joint type 확인 | `fixed` 변환 완료 ✓ |
| left 팔 관절 freeze (right solver) | joint type 확인 | `fixed` 변환 완료 ✓ |
| finger 관절 freeze (양 solver) | joint type 확인 | 원본부터 `fixed` (변환 불필요) ✓ |
