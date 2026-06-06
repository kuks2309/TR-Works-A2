# 2026-06-07 — pick_and_place UI 검출/pick 지연: ros2 action send_goal CLI subprocess 콜드스타트

## 증상
scenario_ui 의 pick_and_place 탭에서 "검출요청" 또는 pick(AlignToBoxes) 을 누르면 결과가 나오기까지 체감상 오래 걸림(약 0.8~0.9초). 사용자 의문: "UI 타이머(QTimer) 때문인가?"

반면 액션 서버(원격 Hailo 브리지) 자체는 빠름 — 이미지 획득→Pi YOLO 분류 E2E 가 ~46ms 수준.

## 측정 (라이브 실측, 2026-06-07, 격리 도메인 ROS_DOMAIN_ID=42 / ROS_LOCALHOST_ONLY=1, 박스 7~8개)

### 1) 이미지 획득 → Pi YOLO 분류 (raw 경로, `experiments/measure_live_pipeline.py`)

| 단계 | mean | median |
| --- | --- | --- |
| [1] D435 캡처 + align | 11.0 ms | 10.9 |
| [2a] JPEG 인코딩 (로봇 PC) | 2.4 ms | 2.6 |
| [2b+3+4] POST 왕복 (RTT) | 33.2 ms | 32.9 |
| &nbsp;&nbsp;┗ [3] Pi Hailo 추론 (NPU) | **26.9 ms** | 26.6 |
| &nbsp;&nbsp;┗ 네트워크 + decode + Flask | 6.3 ms | 6.3 |
| [5] 3D 역투영 | 0.05 ms | — |
| **E2E (RViz 제외)** | **46.6 ms** | **47.4** |

### 2) 액션 호출 방식별 1회 소요 비교

| 호출 방식 | 1회 소요 | 비고 |
| --- | --- | --- |
| in-process 액션 클라이언트 (`experiments/measure_detect_action.py`) | **43.8 ms** (median 42.5) | ROS 오버헤드 +11.6ms, 콜드스타트 없음(첫 goal 42.6ms = 정상상태), discovery 0ms, goal-accept ~1ms |
| **UI 방식 `ros2 action send_goal` CLI** | **~850 ms** (0.81/0.92/0.82s) | 실제 작업분은 44ms 뿐 |

→ UI 방식이 약 **19배 느림**. 차이 ~800ms 는 전부 CLI subprocess 콜드스타트.

## 원인

**UI 타이머가 아니다.** pick_and_place_tab.py 의 QTimer 는 검출/pick 경로와 분리됨:
- `_status_timer` (2000ms) → `_refresh_status`: 실행 중 프로세스 라벨만 갱신(ps/proc 조회, ros2 CLI 안 띄움).
- `_readout_timer` (500ms) → `_refresh_readout`: 자세 readout(TF 2Hz).

근본 원인은 UI 가 검출요청·pick 을 매번 `ros2 action send_goal` **CLI 를 QProcess 로 새로 spawn** 하는 방식. 매 호출마다 콜드스타트 발생:
- `bash -c "source 3d_detect_ws/install/setup.bash"` (워크스페이스 source ~0.2s)
- ros2 CLI 프레임워크 로드(ament index / 엔트리포인트)
- 새 rclpy 노드 생성 + **DDS discovery 로 기존 액션 서버 재탐색**
- 타입 로드 → goal 전송 → 결과 수신 → 노드 소멸

근거 (file:line):
- 검출요청: `_detect_request_cmd` / `_request_detect` — [pick_and_place_tab.py:561-584](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py#L561-L584)
  (`bash -c "source … && ros2 action send_goal /yolov8_node/detect yolov8_detection_msgs/action/DetectBox …"`)
- 메인 pick: `_run` — [pick_and_place_tab.py:484-495](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/pick_and_place_tab.py#L484-L495)
  (`self._proc.start("ros2", ["action","send_goal","--feedback", …])`)

참고: 격리 도메인의 액션 ROS 오버헤드(11.6ms)는 과거 측정 ~48ms(2026-06-05, ROS_DOMAIN_ID=0 오염 그래프)보다 훨씬 작음 — DDS 오염 영향. 단 UI 느림의 지배 요인은 도메인과 무관하게 CLI subprocess(~800ms). 측정 당시 production 도메인(0)에는 타 PC 노드가 없어 오염 상태는 재현 불가.

## 수정 (미적용 — 보류)

이번 세션은 진단·측정만 수행, 코드 변경 없음. **수정은 전용 pick and place UI 생성 시 함께 적용 예정**(사용자 결정, 2026-06-07).

적용 방향:
- UI 에 in-process rclpy 액션 클라이언트를 두고 검출/pick goal 을 직접 send_goal. UI 에는 이미 rclpy 노드+스레드 인프라 존재([scenario_action_client.py:146](../../openarmx_ws/src/openarmx_ros2/openarmx_scenario_ui/openarmx_scenario_ui/scenario_action_client.py#L146)). → 0.85s → ~44ms.
- 검출을 반복 루프로 쓸 경우 raw HTTP POST(28~30Hz)도 가능.

## 재발 방지
- 반복 호출하는 ROS2 액션/서비스는 UI 에서 `ros2 ... send_goal` CLI 를 QProcess 로 spawn 하지 말 것(매회 ~0.8s 콜드스타트). in-process 액션 클라이언트로 호출한다. CLI spawn 은 일회성 launch(노드 기동)에만 사용.
