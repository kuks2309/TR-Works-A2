# 2026-06-01 19:18 (KST) — YOLOv8 연속 추론 자원 낭비 → on-demand DetectBox action server 전환

## 증상

`3d_detect_ws` 의 `yolov8_node` 가 카메라 컬러(+깊이) 토픽을 구독해 **들어오는 모든 프레임마다**(~30FPS) `YOLO.predict` 를 무조건 실행. YOLO-World large 모델 추론이 CPU 에서 쉬지 않고 돌아 박스를 보지 않을 때도 자원 낭비가 큼.

## 원인

- 구독 콜백 `_on_color_only` / `_on_color_depth` → `_process` 가 프레임마다 무조건 `predict` ([yolov8_node.py:154](../../3d_detect_ws/src/yolov8_detection/yolov8_detection/yolov8_node.py)). enable 플래그·서비스·라이프사이클·throttle 전무.
- 다운스트림(`box_plane_node` RANSAC, `grasp_post_node` grasp 계산)은 YOLO 발행에만 반응 → YOLO 만 게이팅해도 전체 체인이 자동 idle.

## 수정

`yolov8_node` 를 **on-demand ROS2 Action Server** 로 in-place 교체 (스트리밍 → action-gated). 사용자 결정: ① YOLO 만 게이팅(최소), ② 인터페이스는 신규 `yolov8_detection_msgs`(3d_detect_ws), ③ 기존 스트리밍 노드 즉시 교체.

- **신규** `3d_detect_ws/src/yolov8_detection_msgs/` — `DetectBox.action` (goal: `prompts`/`confidence`/`publish_annotated`, result: `success`/`message`/`num_detections`/`detections_json`, feedback: `phase`/`progress`) + ament_cmake rosidl `CMakeLists.txt` / `package.xml`.
- **수정** `yolov8_detection/yolov8_detection/yolov8_node.py`:
  - 구독 콜백은 최신 메시지 ref 만 캐시(`_on_color`/`_on_depth`, decode·추론 0) → idle 비용 ≈ 0, 모델은 RAM 상주.
  - `~/detect` ActionServer 추가: busy-flag(REJECT)·cancel·feedback, `ReentrantCallbackGroup` + `MultiThreadedExecutor`.
  - `_process` 는 payload 를 return 하도록 리팩터(여전히 `~/detections`·`~/image_annotated` 발행 → box_plane_node 반응 유지). per-goal `prompts`/`confidence`/`publish_annotated` 오버라이드.
  - `message_filters` 동기 제거 → goal 시 color/depth 최신 ref 스냅샷 + stamp-skew 경고.
- **수정** `yolov8_detection/package.xml` — `<depend>yolov8_detection_msgs</depend>` 추가, 미사용 `<depend>message_filters</depend>` 제거.
- **수정** `yolov8_detection/README.md` — §3.1 on-demand 트리거 사용법.
- **무수정**: `box_plane_node` / `grasp_pose_node` / `openarmx_pick` / launch / `run_yolov8_ros.sh` 구조.

## 검증

- `colcon build --packages-select yolov8_detection_msgs` / `yolov8_detection` 모두 성공.
- `ros2 interface show yolov8_detection_msgs/action/DetectBox` 필드 정상.
- venv python(런타임 env) 에서 `DetectBox`·`rclpy.action` import OK, `py_compile` OK.
- **실제 D435 연결 런타임 스모크**: 노드 idle 기동(`DetectBox action ready`) → `ros2 action list` 에 `/yolov8_node/detect` → goal 1회 송신 시 feedback(inferring→publishing) 후 `success=true`, 실제 카메라 프레임 헤더(`camera_color_optical_frame`) 포함 `detections_json` 반환, `SUCCEEDED`. goal 전후 추론 0.
- (기본 모델 `yolov8n.pt`/conf 0.35 라 박스 0건 — 정상. 박스 pick 체인 확인은 `run_yolov8_ros.sh` 의 `yolov8l-worldv2.pt`+prompts 로 현장에서.)

## 재발 방지

- 고비용 추론/연산 노드는 "프레임마다 무조건 실행" 대신 **on-demand(action/service) + 최신 ref 캐시** 패턴을 기본으로 한다.
- `ros2 run` 은 설치 엔트리포인트의 **시스템 python shebang** 으로 실행되어 venv 미적용 → `ModuleNotFoundError: ultralytics`. yolov8 계열 노드는 반드시 `run_yolov8_ros.sh`(매 실행 shebang sed-patch) 또는 venv python 직접 실행으로 기동.
