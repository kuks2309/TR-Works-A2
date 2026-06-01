# ROS2 연속 추론 노드 → On-Demand Action Server 변환 가이드라인

고비용 노드(YOLO 검출 / 세그멘테이션 / 포인트클라우드 처리 등)가 센서 콜백마다 무조건 실행되어 idle 시에도 CPU/GPU 를 낭비할 때, **요청(goal)이 올 때만 1회 실행**하는 ROS2 Action Server 로 바꾸는 재사용 플레이북. 2026-06-01 `yolov8_detection` 전환 사례에서 일반화함 (worked example는 §7).

---

## 1. 언제 적용하나

- 추론/연산 노드가 카메라·센서 콜백마다 무조건 실행 → idle 에도 자원 낭비.
- 결과가 실제로는 **이벤트당 1회**만 필요 (pick 사이클, 트리거 시점 등).
- 적용 부적합: 결과가 진짜로 연속 스트림이어야 하는 경우(예: 실시간 추종 제어, 라이브 영상 오버레이가 본질인 경우). 이때는 throttle/조건 게이트가 더 맞음.

## 2. 사전 분석 (변환 전 필수)

1. **연속 실행 메커니즘 특정**: 어느 콜백이 프레임마다 무엇을 호출하는가 (`file:line`). enable 플래그/서비스/라이프사이클/throttle 이 이미 있는지 확인 (보통 없음).
2. **다운스트림 소비자 맵**: 누가 결과 토픽을 구독하는가, 각자 *연속* 필요인가 *1회* 필요인가.
   - ★ **핵심 통찰**: 토픽 체인에서 **소스(추론)만 게이팅하면 다운스트림은 자동으로 idle** 되는 경우가 많다(소스가 발행을 멈추면 콜백이 안 뜸). → 최소 변경으로 끝낼 수 있는지 먼저 판단.
   - 진짜 연속 소비자(RViz 라이브 오버레이 등)는 별도 고려(§6).
3. **상주 vs 요청별 상태 구분**: 모델 로드/가중치/intrinsics/TF = 상주(1회). 디코드/추론/RANSAC/임시버퍼 = 요청별.
4. **기존 action 인프라 조사**: 같은 repo 의 `.action`/ActionServer/ActionClient 패턴을 찾아 컨벤션 재사용 (새로 발명 금지).

## 3. 범위 결정 (사용자와 합의할 의사결정 포인트)

| 결정 | 옵션 A (최소) | 옵션 B (통합) |
|---|---|---|
| **범위** | 소스 노드에만 action server. 결과는 **기존 토픽으로 1회 발행**, 다운스트림 무수정 | YOLO+후처리를 1개 action 노드로 합치고 **결과를 action result 로 반환**, 소비자를 client 로 전환 |
| 장점 | 변경·리스크 최소, 다운스트림 자동 idle | 단일 요청/응답(idiomatic), 명확한 통합점 |
| 단점 | result 가 요약만, 실데이터는 토픽 비동기 | 변경 범위·리스크 큼 |

추가 결정: **인터페이스 패키지 위치**(워크스페이스 경계 — 별도 ws 에 두면 빌드 결합 발생), **전환 안전성**(legacy 경로를 launch 플래그로 병립 유지 vs 즉시 교체).

## 4. 구현 패턴

### 4.1 인터페이스 (`.action`)

```
# Goal — 모두 선택적 오버라이드 (빈 문자열 / <=0 이면 노드 기본값)
string  param_a
float32 threshold
bool    want_extra
---
# Result — success/message + 소비자가 실제로 쓰는 데이터(또는 요약 JSON)
bool    success
string  message
int32   num_items
string  result_json        # 기존 토픽 payload 와 동일 → 구독 없이 사용
---
# Feedback
string  phase
float32 progress            # 0.0 .. 1.0
```

- **원시 타입 위주** → msg 의존성 최소화, result 작게 유지.
- 큰 데이터(PointCloud2/Image)를 result 에 담을 땐 **subsample 상한**을 두고 크기를 문서화 (DDS result 한계).
- `ament_cmake` + `rosidl_generate_interfaces()` 패키지로 빌드. 같은 repo 의 기존 msgs 패키지 CMakeLists/package.xml 미러.

### 4.2 노드 — idle 비용 ≈ 0 의 핵심

- 모델/가중치는 `__init__` 에서 **1회 로드 후 상주**. 절대 요청마다 재로드 금지.
- 구독 콜백은 **최신 메시지 레퍼런스만 캐시**(디코드·추론 금지):
  ```python
  def _on_color(self, msg): self._last_color = msg   # O(1) 포인터 저장만
  def _on_depth(self, msg): self._last_depth = msg
  ```
  → 30Hz 로 들어와도 idle CPU ≈ 0.
- 추론은 **execute_callback 안에서만**: 캐시된 최신 프레임 grab → decode → predict → 결과 조립.
- **가드**: intrinsics/TF/프레임 미가용 시 블록하지 말고 `abort()` + 명확한 message. cold-start 레이스는 짧은 `frame_wait_timeout` 으로 대비.
- `message_filters` 동기는 제거 → goal 시점에 color/depth 최신 ref 스냅샷 + stamp-skew(`> sync_slop`) 경고 후 진행.

### 4.3 ActionServer 골격

```python
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading

# __init__
self._goal_lock = threading.Lock(); self._active = False
self._srv = ActionServer(self, MyAction, "~/detect",
    execute_callback=self._execute,
    goal_callback=self._on_goal, cancel_callback=self._on_cancel,
    callback_group=ReentrantCallbackGroup())

def _on_goal(self, _):                      # 중복 goal 거절(busy-flag)
    with self._goal_lock:
        if self._active: return GoalResponse.REJECT
        self._active = True; return GoalResponse.ACCEPT

def _on_cancel(self, _): return CancelResponse.ACCEPT

def _execute(self, gh):
    res = MyAction.Result()
    try:
        frame = self._last_color
        if frame is None:
            gh.abort(); res.success = False; res.message = "no frame yet"; return res
        if gh.is_cancel_requested:
            gh.canceled(); res.success = False; return res
        # ... decode + predict + publish(기존 토픽) + result 채우기 ...
        gh.succeed(); res.success = True; return res
    finally:
        with self._goal_lock: self._active = False

# main(): rclpy.spin 대신 MultiThreadedExecutor → execute 중에도 구독이 최신 프레임 갱신
executor = MultiThreadedExecutor(); executor.add_node(node); executor.spin()
```

- **cancel 입도**: `predict()` 같은 단일 블로킹 호출은 비협조적 → `is_cancel_requested` 는 **단계 경계**에서만 체크(입도 = 1 스테이지). 문서화.
- `finally` 에서 busy 플래그 반드시 해제.

### 4.4 소비자 (옵션 B 일 때만)

ActionClient 로 전환하되 **기존 처리 로직은 그대로**, action result 를 그 함수에 먹이기만:
```python
send_goal_async(goal, feedback_callback=...).add_done_callback(_on_goal_resp)
# _on_goal_resp: gh.accepted 확인 → gh.get_result_async().add_done_callback(_on_result)
# _on_result: 기존 _on_cloud(result.cloud) / _on_info(result.json) 그대로 호출
```

## 5. 검증 체크리스트

- [ ] 빌드: 인터페이스 패키지 → 노드 패키지 순. `ros2 interface show <pkg>/action/<Name>`.
- [ ] 런타임 env(venv 등)에서 생성 인터페이스 import OK, `py_compile` OK.
- [ ] **idle 비용 0**: `ros2 topic hz <결과토픽>` → 무발행, `top -p <pid>` → CPU ≈ 0 (모델은 RAM 상주).
- [ ] **goal 1회**: feedback 단계 → `success=true`, 실데이터 반환, 다운스트림 1회 반응.
- [ ] goal 후 **다시 idle**.
- [ ] **실물 E2E**: 실제 대상(박스 등)으로 결과 정확성까지. one-shot 발행은 **goal 보내기 전에 echo 를 미리 걸어** 캡처.

## 6. 흔한 함정

- **venv shebang**: `ros2 run <pkg> <node>` 는 설치 엔트리포인트의 **시스템 python shebang** 으로 실행 → venv 패키지(ultralytics 등) 미적용 `ModuleNotFoundError`. venv 노드는 실행 스크립트의 shebang sed-patch 또는 venv python 직접 실행(`python -m pkg.node`)으로 기동.
- **one-shot 발행 놓침**: 1회성 발행은 latch(TRANSIENT_LOCAL) 안 하면 나중 `echo --once` 로 못 잡음 → goal 전에 echo 백그라운드로 걸거나 result 에 담기. RViz 라이브 프리뷰가 필요하면 result 의 이미지/마커를 goal 당 1회 발행(같은 토픽명 유지 → RViz 설정 무수정) 또는 선택적 `preview_rate` 파라미터.
- **echo string 절단**: `ros2 topic echo` 는 긴 string 을 `...` 로 자름 → 노드 로그나 action result 로 값 확인.
- **워크스페이스 경계**: 인터페이스를 다른 colcon ws 에 두면 빌드 결합 발생. 소스 노드와 같은 ws 에 두고, 순수 rclpy 소비자(client)는 **런타임 의존**(install 소싱)만으로 충분.
- **임계값 불일치**: 소스 confidence 와 다운스트림 필터 임계값(예: `min_conf`)이 다르면, 소스는 발행하나 다운스트림이 버림. 게이팅 후 한 번에 드러남 → 함께 점검.

## 7. Worked example — yolov8_detection (이 repo)

- 변경: `yolov8_node` 매 프레임 `predict` → on-demand `~/detect` (`yolov8_detection_msgs/action/DetectBox`). 옵션 A(소스만 게이팅) 채택 → `box_plane_node`/`grasp_pose_node`/`openarmx_pick` 무수정(YOLO 멈추면 자동 idle).
- 커밋 `7e0d9a2`, 이슈 기록 [docs/issues_and_fixes/2026-06-01_yolov8_on_demand_action_server.md](../issues_and_fixes/2026-06-01_yolov8_on_demand_action_server.md).
- 실물 검증: goal 1회 → box conf 0.36 검출 → box_plane 1회 반응 → 박스 윗면 핏(body_z=0.162m, inliers=3853) + 바닥(0.109m) + box_height=5.3cm. idle 시 추론 0.
- 핵심 코드: [3d_detect_ws/src/yolov8_detection/yolov8_detection/yolov8_node.py](../../3d_detect_ws/src/yolov8_detection/yolov8_detection/yolov8_node.py) (구독=ref캐시, ActionServer busy-flag, MultiThreadedExecutor), 인터페이스 [3d_detect_ws/src/yolov8_detection_msgs/](../../3d_detect_ws/src/yolov8_detection_msgs/).
