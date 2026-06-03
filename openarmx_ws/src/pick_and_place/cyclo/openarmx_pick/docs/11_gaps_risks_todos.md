# openarmx_pick — 공백 · 리스크 · 코드 리뷰 · TODO

> 분석일: 2026-06-03 · 패키지: openarmx_pick

본 문서는 `openarmx_pick` 패키지의 **미완성 공백(gap)**, **운영/안전 리스크(risk)**, **코드 레벨 결함(code review finding)**, 그리고 **잔여 TODO**를 한곳에 종합한다. 모든 항목은 실제 코드 인용으로 근거를 달았으며, 코드로 검증하지 못한 부분은 명시적으로 **(추정)** 으로 표기한다. 본 문서는 분석 전용이며 어떤 소스도 수정하지 않는다.

---

## 분석 범위

- **포함**: 패키지가 README에서 스스로 선언한 "Not yet done / next" 4개 공백의 정리, 그리고 그 외에 코드를 직접 읽어 발견한 edge case 처리, 예외처리 품질, `auto_send` 안전성, debounce 동작 한계, 좌표/축 가정의 미검증 리스크. 각 항목에 심각도(High/Med/Low)와 근거 코드 위치(`path:line`)를 병기한다.
- **분석 대상 파일**: `openarmx_pick/grasp_pose_node.py`(런타임 노드), `README.md`(선언된 공백·검증 상태), `scripts/verify_e2e.py` / `scripts/verify_grasp.py` / `scripts/verify_solver.py`(검증 스크립트), `launch/openarmx_pick.launch.py` / `launch/openarmx_movel.launch.py`(런치), `setup.py`(패키징). 교차 확인용으로 `launch/openarmx_movel_bimanual.launch.py`, `config/` 디렉터리 존재 여부도 확인했다.
- **제외**: 그래스프 합성 수학 이론의 상세 유도(별도 문서 `docs/03_grasp_synthesis_theory.md` 범위), URDF 생성 절차 상세(`docs/04_solver_urdf.md` 범위), 아키텍처/토픽 흐름 일반 서술(`docs/01_architecture.md`, `docs/05_launch.md` 범위). 본 문서는 **결함·리스크·할 일**에만 집중한다.
- 모든 경로는 패키지 루트(`openarmx_pick/`) 기준 상대경로다. ROS2(Robot Operating System 2, 로봇 운영체제 2), QP(Quadratic Program, 2차 계획법), CBF(Control Barrier Function, 제어 장벽 함수), PCA(Principal Component Analysis, 주성분 분석), quaternion(쿼터니언, 4원수 회전 표현), TF(Transform, 좌표 변환), RANSAC(Random Sample Consensus, 무작위 표본 합의), FK(Forward Kinematics, 순기구학), Jacobian(야코비안), QoS(Quality of Service, 통신 품질 정책), TCP(Tool Center Point, 공구 중심점), FSM(Finite State Machine, 유한 상태 기계), SRDF(Semantic Robot Description Format, 의미 기반 로봇 기술 형식), EE(End-Effector, 말단 장치)는 첫 등장 시 풀어쓴다.

---

## 1. 심각도별 요약 (Executive summary)

| # | 항목 | 분류 | 심각도 | 근거 위치 |
|---|---|---|---|---|
| R1 | `auto_send=true` 시 실로봇이 디바운스만 거쳐 자동 이동 (안전 정지·E-stop 게이트 부재) | 안전 | **High** | `openarmx_pick/grasp_pose_node.py:198-199`, `:240-249` |
| R2 | main-box 필터 부재 → 최대 3개 top candidate 사이로 grasp pose 점프 | 공백(①) | **High** | `README.md:133-134`, `openarmx_pick/grasp_pose_node.py:180-187` |
| R3 | Pick FSM 부재 (descend→close→lift 없음, pre-grasp hover만) | 공백(②) | **High** | `README.md:135-136`, `openarmx_pick/grasp_pose_node.py:194,199` |
| R4 | tool axis 가정이 실측 FK로 미검증 (잘못되면 손목이 박스로 돌진) | 미검증 가정 | **High** | `openarmx_pick/grasp_pose_node.py:120-121,134-135`, `README.md:139-140` |
| R5 | `_on_info`의 bare `except` — 모든 예외 무음 삼킴 | 예외처리 | **Med** | `openarmx_pick/grasp_pose_node.py:158-162` |
| R6 | PCA degenerate (정사각/원형 top) 시 yaw 임의 결정 | edge case | **Med** | `openarmx_pick/grasp_pose_node.py:182-187` |
| R7 | Stage-2 collision CBF 미적용 (충돌 없는 URDF + 빈 SRDF) | 공백(③) | **Med** | `README.md:137-138`, `launch/openarmx_movel.launch.py:38-40` |
| R8 | real controller 배선 미완 (joint_command→forward_position_controller) | 공백(④) | **Med** | `README.md:139-140`, `launch/openarmx_movel.launch.py:44-47` |
| R9 | `_read_xyz` 순수 파이썬 루프 — 고해상도 cloud에서 처리 지연·프레임 적체 가능 | 성능 | **Med** | `openarmx_pick/grasp_pose_node.py:46-62` |
| R10 | debounce가 yaw(회전) 변화·정보 노후화를 감지하지 못함 | debounce 한계 | **Med** | `openarmx_pick/grasp_pose_node.py:226-238` |
| R11 | `opening` / `long_axis` norm 0 분기 미보호 (`/= norm`) | edge case | **Low~Med** | `openarmx_pick/grasp_pose_node.py:186-187,184` |
| R12 | `pts<30` 임계 외에 상한·면적 검증 없음, stride 후 카운트 의존 | edge case | **Low** | `openarmx_pick/grasp_pose_node.py:164-169` |
| R13 | cloud stamp 시점의 TF가 아닌 최신 TF(`Time()`) 사용 → 이동 시 외삽 오차 | 시간 동기 | **Low** | `openarmx_pick/grasp_pose_node.py:171-172` |
| R14 | `setup.py`의 `config/*` glob이 빈 디렉터리를 가리킴 (무해하나 사문화) | 패키징 | **Low** | `setup.py:17` |
| R15 | grasp z가 박스 top 평면에서만 산출, 실 grasp 깊이/높이 정보(`box_height_m`) 미사용 | 설계 | **Low** | `openarmx_pick/grasp_pose_node.py:160,193,203` |

---

## 2. README가 선언한 공백 (Not yet done / next)

`README.md:131-140`이 명시한 4개 항목을 코드와 대조해 정확한 현 상태와 영향을 기술한다.

### 2.1 공백① — main-box 필터 (R2, High)

> README: `box_plane`이 최대 3개 top candidate를 내보내 grasp pose가 점프; 최대 inlier cloud만 선택 필요. (`README.md:133-134`)

코드 측 현실: `_on_cloud`는 **수신한 cloud 1건을 무조건 그대로** 처리한다(`openarmx_pick/grasp_pose_node.py:164-180`). 어떤 box가 main인지 선택하는 로직이 전혀 없다. centroid·PCA·grasp pose는 들어온 inlier 집합 그대로 산출된다(`:180-191`). 따라서 `box_plane`이 프레임마다 다른 candidate cloud를 publish하면 grasp pose가 candidate 사이를 **튀게** 된다.

- 영향: `auto_send=true`이면 R10(yaw 비감지) + 점프가 결합되어 실제 MoveL 목표가 박스 간을 오갈 수 있다 — **High**.
- 근거: 선택 로직 부재(`:164-180`), inlier 카운트만 검사(`:166`).

### 2.2 공백② — Pick FSM (R3, High)

> README: descend → gripper close(`openarmx_gripper_panel`) → lift; 현재는 pre-grasp hover만. (`README.md:135-136`)

코드 측 현실: 노드가 산출하는 동작은 **pre-grasp hover 한 점**뿐이다. `pre_xyz = centroid + pregrasp_h`(`openarmx_pick/grasp_pose_node.py:194`)가 유일한 MoveL 타깃이고(`:199` → `_send_movel`), descend(grasp_xyz로 하강), gripper close, lift에 해당하는 상태 전이·그리퍼 토픽·후속 MoveL이 **존재하지 않는다**. `grasp_xyz`는 산출만 되고(`:193`) marker/pose에만 쓰이며 실제 하강 명령으로 이어지지 않는다(`:196-197`).

- 즉, 현재 패키지는 "박스 위 호버"까지만 자동화한다. 실제 집기는 사람이 별도 절차로 수행해야 한다 — **High**(기능 공백).

### 2.3 공백③ — Stage-2 collision CBF + SRDF (R7, Med)

> README: collision 포함 URDF + SRDF 재생성으로 self-collision avoidance 활성화. (`README.md:137-138`)

코드 측 현실: 솔버 런치가 **빈 SRDF**로 기동한다 — `srdf_path` 기본값 `""`(`launch/openarmx_movel.launch.py:38-40`)이고 주석이 "Empty for stage-1 (no collision pairs)"라고 명시한다. URDF는 collision이 stripped된 stage-1 모델(`README.md:34`). 따라서 솔버는 **joint-limit + singularity CBF만** 활성, 자기 충돌/환경 충돌 CBF는 없다(`launch/openarmx_movel.launch.py:18-20` 주석 일치).

- 영향: descend/FSM이 붙기 전에는 손목이 테이블·박스·반대 팔과 충돌해도 솔버가 막지 못한다 — **Med**(현재는 hover만이라 즉각 위험은 낮으나, FSM 도입 전 반드시 선행 필요).

### 2.4 공백④ — real controller mapping (R8, Med)

> README: `joint_command`을 `forward_position_controller`에 배선; tool axes 실측 검증. (`README.md:139-140`)

코드 측 현실: 런치는 `joint_command_topic` 기본값을 `/openarmx/left_arm/joint_trajectory`로 두고 주석이 "Map to the OpenArmX left-arm forward controller input"이라고만 적는다(`launch/openarmx_movel.launch.py:44-47`). 즉 **토픽 이름만 가정**되어 있고, 실제 ros2_control `forward_position_controller`/`joint_trajectory_controller`와의 연결·타입 정합은 이 패키지 안에서 검증되지 않는다(컨트롤러 정의가 이 패키지에 없음 — `config/`가 비어 있음, `setup.py:17`). 검증 스크립트들은 모두 명령을 자기 자신에게 echo-back하는 **시뮬레이션**이다(`scripts/verify_solver.py:49-57`, `scripts/verify_e2e.py:69-76`).

- 영향: 실제 컨트롤러 타입(JointTrajectory vs Float64MultiArray)·관절 이름·순서 불일치 시 실로봇에서 무동작 또는 오동작 — **Med**. tool axes 검증 미완은 R4와 직결.

---

## 3. 안전 리스크

### 3.1 `auto_send` 자동 동작 — 안전 게이트 부재 (R1, High)

`auto_send=true`면 노드는 카메라 cloud를 받을 때마다 디바운스만 통과하면 **실제 MoveL 명령을 publish**한다(`openarmx_pick/grasp_pose_node.py:198-199`, 발행은 `:240-249`). 그 사이에 **확인 단계·E-stop 연동·작업 영역(workspace) 한계 검사·사람 승인**이 전혀 없다. 박스가 인식되는 즉시 팔이 그쪽으로 움직이기 시작한다.

- 완화 장치는 두 가지뿐이다: (a) 기본값 `auto_send=False`(`:112`, 런치도 `false` 기본 `launch/openarmx_pick.launch.py:27`), (b) 디바운스(R10 참조). README도 "sends motion — only with the robot ready"로 경고한다(`README.md:97-100`).
- 그러나 일단 `auto_send=true`로 켜면, R2(box 점프)·R4(축 가정 오류)·R6(yaw 임의)·R13(TF 외삽)의 어떤 결함이든 **즉시 실로봇 모션으로 전파**된다. 별도 안전 인터록이 없다 — **High**.
- 검증: 노드 내 어디에도 cloud 유효성/임계 거리/관절 한계 외의 사전 검사가 없으며(전체 `_on_cloud` `:164-203`), MoveL 발행 전 유일한 분기는 `_should_send` 디바운스다(`:198`).

### 3.2 tool axis 가정 미검증 (R4, High)

approach/opening 회전은 `tool_approach_axis=[0,0,1]`, `tool_opening_axis=[1,0,0]`라는 **가정**(`openarmx_pick/grasp_pose_node.py:120-121,134-135`)에 전적으로 의존한다. 모듈 docstring도 "is **assumed** to grasp along its local +z … `tool_*_axis` make this explicit"라고 명시한다(`:19-21`). README는 이 가정을 "validate the assumed tool axes against real FK"로 명시적 TODO에 올렸다(`README.md:139-140`).

- 만약 실제 `openarmx_left_hand_tcp`의 공구 축이 가정과 다르면, `_grasp_rotation`(`:83-97`)이 산출하는 quaternion이 틀어져 **gripper가 박스에 비스듬히/거꾸로 접근**한다. `auto_send=true`와 결합되면 충돌 위험.
- 현재 코드 어디에서도 실측 FK로 이 축을 검증하지 않는다(검증 스크립트는 합성 cloud에서 approach `(0,0,-1)`만 확인 `scripts/verify_grasp.py:79-90`, 실 TCP 기하는 미검증) — **High** (FSM/실로봇 도입 전 차단 항목).

---

## 4. 예외처리 품질

### 4.1 `_on_info`의 bare except — 무음 실패 (R5, Med)

```python
def _on_info(self, msg: String):
    try:
        self._box_height = json.loads(msg.data).get("box_height_m")
    except Exception:
        pass
```
(`openarmx_pick/grasp_pose_node.py:158-162`)

`except Exception: pass`는 JSON 파싱 실패, 키 부재, 인코딩 오류 등 **모든 예외를 로그 한 줄 없이 삼킨다**. `box_height_m`이 손상돼도 노드는 조용히 이전 값(또는 `None`)을 유지한다. 디버깅 시 `box_height_m`이 왜 갱신 안 되는지 추적 불가.

- 추가로 `MoveL` import 실패도 광범위 `except Exception`으로 처리된다(`:38-42`) — 이 경우는 의도된 optional import라 합리적이나, 동일 패턴이 진짜 오류를 가릴 위험을 공유한다.
- `rclpy.shutdown()` 중복 호출 보호용 `except Exception`(`scripts/verify_grasp.py:103-106`, `scripts/verify_e2e.py:110-113`)도 동일 스타일 — 종료 경로라 영향은 낮음.
- 심각도: `box_height_m`가 현재 grasp 계산에 **쓰이지 않으므로(R15)** 당장 기능 영향은 작아 **Med**. 다만 향후 FSM이 box_height로 lift 거리를 정하면 이 무음 실패가 위험으로 승격된다.

---

## 5. Edge case 처리

### 5.1 작은/빈 cloud 처리 (R12, Low)

`pts_cam.shape[0] < 30`이면 경고 후 return(`openarmx_pick/grasp_pose_node.py:164-169`). 빈 cloud(필드 누락 포함)는 `_read_xyz`가 `(0,3)`을 반환(`:48-50,62`)하므로 같은 분기로 안전하게 걸린다.

- 한계: 임계 `30`은 **stride 적용 후** 카운트다(`_read_xyz(cloud, self.stride)`, `:165`, `stride=4` 기본 `:111`). 즉 원본 120점 미만이면 누락될 수 있고, 임계가 매직넘버로 하드코딩됐다(`:166`).
- 상한·면적·평면성 검증은 없다. 노이즈로 inlier가 과도하게 많거나 비현실적으로 큰 cloud도 그대로 통과한다 — **Low**.

### 5.2 PCA degenerate (정사각/원형 top) (R6, Med)

XY-PCA로 long axis를 잡는다(`:182-184`). 박스 top이 **정사각형이거나 원형**이면 공분산 두 고유값이 거의 같아 `np.argmax(evals)`(`:184`)가 수치 노이즈로 long axis를 **임의 선택**한다. 결과적으로 gripper yaw가 프레임마다 90° 가까이 튈 수 있다.

- 가드 없음: 고유값 비(anisotropy) 검사·이력(hysteresis)·이전 yaw 유지 로직이 전혀 없다. R10(yaw 비감지 디바운스)과 결합되면, yaw가 튀어도 디바운스가 재전송을 막아 오히려 **낡은/틀린 yaw로 고착**될 수도 있다 — **Med**.

### 5.3 opening / long_axis 정규화 norm 0 (R11, Low~Med)

`opening /= np.linalg.norm(opening)`(`:187`)와 `_grasp_rotation` 내부의 여러 `/= norm`(`:87,91,95`)은 분모 0 보호가 부분적이다.

- `_grasp_rotation`은 첫 정규화에 대해 0 처리 가드를 둔다: `o`의 norm이 `1e-6` 미만이면 fallback 축을 쓴다(`:89-91`). 그러나 `approach`(`:87`)와 `tool_*`(`:93,95`)의 정규화에는 가드가 없다 — approach는 상수 `(0,0,-1)`(`:189`)라 실질 안전, tool 축은 파라미터라 사용자가 0 벡터를 주면 `nan` 발생 가능(검증 안 됨, **추정**).
- 노드 본문의 `opening = cross([0,0,1], long_axis)`(`:186`)는 long_axis가 거의 z축과 평행할 때만 0이 되는데, long_axis는 z=0으로 강제되므로(`:185`) 실질적으로 0이 되기 어렵다(**추정**: long_axis가 정확히 0 벡터일 때만 위험, 이는 cov가 0인 단일점 cloud인데 그 경우 R12의 `<30` 분기에서 이미 걸림). 종합 심각도 **Low~Med**.

---

## 6. Debounce 동작 한계 (R10, Med)

`_should_send`는 **위치 이동량**과 **시간 쿨다운**만 본다:

```python
moved = float(np.linalg.norm(np.asarray(pre_xyz) - self._last_sent_xyz))
elapsed = now - self._last_sent_t
if moved > self.send_min_delta or elapsed > self.send_min_interval:
```
(`openarmx_pick/grasp_pose_node.py:233-235`)

- **yaw(회전) 변화를 감지하지 않는다.** `pre_xyz`(위치)만 비교 대상이고 quaternion은 디바운스 입력이 아니다(`:226,231-236`). 따라서 박스 위치는 그대로인데 PCA yaw가 뒤집혀도(R6), `moved`가 작으면 재전송이 안 되어 **틀어진 자세로 갈 수도, 갱신이 안 될 수도** 있다.
- 쿨다운(`send_min_interval=5.0`, `:118`)이 `move_time=4.0`(`:113`)보다 크게 설정돼 모션 완료 후 재전송하도록 의도됐으나(`:114-119` 주석), 두 값은 독립 파라미터라 사용자가 `send_min_interval < move_time`로 설정하면 **모션 중 재전송**으로 솔버 trajectory가 매번 리셋되어 팔이 기어가는 원래 문제가 재발한다(`:115-117` 주석이 경고하는 바로 그 현상).
- 첫 호출은 무조건 전송(`:230-232`)이므로 노드 기동 직후 첫 박스에서 즉시 모션이 나간다 — R1과 결합 시 "켜자마자 움직임" — **Med**.

---

## 7. 좌표/축·시간 가정의 미검증 리스크

### 7.1 approach = 항상 수직 하강 가정 (Low~Med)

`approach = [0,0,-1]`로 **고정**돼 박스 top normal이 항상 `+z_base`(테이블 위 수평 박스)라고 가정한다(`:189`, docstring `:9-12`). 기울어진 박스·경사면·비수평 표면에서는 top-down 가정이 깨져 grasp가 빗나간다. 코드는 cloud로부터 실제 평면 normal을 추정하지 않는다(공분산은 XY 2D로만 계산 `:181-183`).

- 영향: 현 사용 시나리오(테이블 위 박스)에서는 타당하나, 이 가정은 **명시적으로 미검증된 환경 제약**이다 — **Low~Med**.

### 7.2 base_frame == solver URDF root == grasp frame 동일성 (Low)

README/런치 주석은 `openarmx_body_link0`가 grasp frame이자 솔버 URDF root이며 `world→body_link0`가 identity라 추가 TF가 불필요하다고 한다(`README.md:24-26`, `launch/openarmx_movel.launch.py:9-11`). 이 동일성이 깨지면(예: URDF root 변경) grasp pose가 잘못된 frame으로 해석된다. 노드는 base_frame을 파라미터로만 받아 검증하지 않는다(`:124`) — 현재는 일관되나 **암묵적 결합** — **Low**.

### 7.3 TF 시간 동기 — 최신 TF 외삽 (R13, Low)

TF lookup이 cloud의 stamp가 아니라 **최신 시각** `rclpy.time.Time()`(zero time)으로 수행된다(`openarmx_pick/grasp_pose_node.py:171-172`). 카메라가 고정(static TF)이면 무해하나, 카메라가 움직이는 구성(추정: eye-in-hand)에서는 cloud 캡처 시점과 변환 시점의 불일치로 위치 오차가 생긴다. 또한 발행되는 pose의 stamp는 cloud stamp를 그대로 쓴다(`:196,221`)므로 stamp와 실제 적용 변환 시점이 어긋난다 — **Low**(현재 static D435 가정 하에서).

---

## 8. 성능 · 패키징

### 8.1 `_read_xyz` 순수 파이썬 루프 (R9, Med)

`_read_xyz`는 포인트마다 `struct.unpack_from` 3회를 파이썬 for 루프로 돈다(`:55-61`). `sensor_msgs_py.point_cloud2`나 numpy `frombuffer` 벡터화 대신 점별 호출이라, 고해상도 cloud나 작은 stride에서 콜백이 길어진다. cloud 구독 QoS depth=5(`:153`)라 콜백이 늘어지면 프레임이 적체·드롭될 수 있다.

- stride=4 기본(`:111`)로 완화하지만 stride는 PCA 품질과 trade-off다. 박스 top 수백~수천 점 규모에선 실용상 동작(추정)하나, 구조적으로 비효율이며 입력이 커지면 병목 — **Med**.

### 8.2 `setup.py`의 `config/*` glob 사문화 (R14, Low)

`setup.py:17`이 `(f"share/{package_name}/config", glob("config/*"))`로 config를 설치 목록에 넣지만, `config/` 디렉터리는 **비어 있다**(확인됨: `ls config/` 결과 파일 없음). 빈 glob은 빌드 실패를 일으키지 않으나, 존재하지 않는 자산을 가리키는 사문화된 선언이다 — **Low**(정리 후보).

---

## 9. 잔여 TODO 목록 (실행 우선순위 제안)

아래는 본 분석에서 도출한 할 일이며, **문서로만 기록**하고 코드 수정은 하지 않았다. 우선순위는 안전·차단성 기준이다.

| 우선 | TODO | 대응 항목 | 비고 |
|---|---|---|---|
| P0 | tool axis를 실측 FK로 검증 후 파라미터 확정 | R4, 공백④ | `auto_send`/FSM 전 **차단 항목** |
| P0 | `auto_send` 경로에 안전 인터록(E-stop·작업영역·승인) 추가 | R1 | 실로봇 모션 전 필수 |
| P1 | main-box 필터(최대 inlier cloud 선택) 구현 | R2, 공백① | grasp 점프 제거 |
| P1 | Pick FSM(descend→close→lift) 구현 | R3, 공백② | gripper 토픽 연동 |
| P1 | Stage-2 collision URDF + SRDF 생성·런치 배선 | R7, 공백③ | FSM 하강 전 선행 |
| P1 | real controller(forward_position_controller) 배선·타입 정합 검증 | R8, 공백④ | 실로봇 동작 전 |
| P2 | PCA degenerate yaw 안정화(이력/이방성 검사) | R6 | 정사각/원형 박스 |
| P2 | debounce에 yaw 변화·정보 노후화 반영, `send_min_interval≥move_time` 강제 | R10 | trajectory 리셋 방지 |
| P2 | `_on_info` bare except 제거·로깅 추가 | R5 | box_height 신뢰성 |
| P3 | `_read_xyz` 벡터화(numpy `frombuffer`/`sensor_msgs_py`) | R9 | 처리량 |
| P3 | cloud 상한·면적·평면성 검증, `<30` 매직넘버 파라미터화 | R12 | 입력 견고성 |
| P3 | TF를 cloud stamp로 lookup(또는 정책 명시) | R13 | 시간 동기 |
| P3 | tool 축 norm 0 가드 추가 | R11 | 파라미터 견고성 |
| P3 | `setup.py`의 빈 `config/*` glob 정리 또는 config 추가 | R14 | 패키징 위생 |
| P3 | `box_height_m`를 grasp/lift에 실제 사용하거나 수집 제거 | R15 | 죽은 데이터 흐름 |

---

## 10. 검증 노트 (무엇을 코드로 확인했는가)

- **확인됨**: 위 모든 코드 인용 라인은 `grasp_pose_node.py`, 런치, `setup.py`를 직접 읽어 대조했다. `config/` 비어 있음, 디바운스가 yaw 미포함, `_on_info` bare except, `auto_send` 전 안전 검사 부재, 빈 SRDF 기본값은 모두 코드/디렉터리에서 직접 확인.
- **추정으로 표기한 부분**: R9의 실측 처리시간(벤치마크 없음), R11의 tool 축 0 벡터 시 `nan` 발생(런타임 미실행), R13의 eye-in-hand 시나리오 존재 여부, 7.1의 환경 제약 위반 빈도. 이들은 코드 정적 분석으로 가능성을 짚었을 뿐 실행 검증은 하지 않았으므로 본문에 **(추정)** 으로 명시했다.
- 검증 스크립트 3종은 모두 **시뮬레이션**(명령 echo-back, 합성 cloud)이며 실로봇·실 TCP 기하·실 컨트롤러를 검증하지 않는다(`scripts/verify_solver.py:49-57`, `scripts/verify_e2e.py:69-83`, `scripts/verify_grasp.py:61-67`). 따라서 README의 "Live camera — PASS"(`README.md:128-129`)는 **pose 산출**까지의 검증이며 "Robot not moved"로 명시된 대로 실모션은 미검증이다.
