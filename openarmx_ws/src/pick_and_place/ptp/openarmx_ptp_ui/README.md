# openarmx_ptp_ui

ptp pick-and-place **전용 Qt(PyQt5) UI**. 구동에 필요한 launch 를 버튼으로
띄우고, `AlignToBoxes` 액션을 실행/취소하며 상태를 한 창에서 본다.
모션 백엔드(C++ `openarmx_ptp_box_align`)와 액션 메시지
(`openarmx_ptp_box_align_msgs`)에 이어 ptp 묶음의 **세 번째 패키지(UI)**.

## 표시와 구동의 분리 (설계 원칙)

| 계층 | 파일 | 역할 |
|---|---|---|
| **표시 (Display)** | `ui/ptp_pnp_ui.ui` | Qt Designer 레이아웃. ROS/subprocess 직접 호출 없음 |
| 표시 결선 | `openarmx_ptp_ui/main_window.py` | `.ui` 로드 + 위젯↔구동 시그널 연결만 |
| **구동 (Drive) · 프로세스** | `openarmx_ptp_ui/managed_process.py` | launch 프로세스그룹 start/stop (SIGINT→SIGKILL, 로그캡처) |
| **구동 (Drive) · ROS** | `openarmx_ptp_ui/ptp_ros_bridge.py` | 백그라운드 스레드 rclpy `ActionClient`, feedback/result→Qt 시그널 |

- 표시 계층은 구동 계층의 메서드만 호출하고, 구동 계층은 Qt 시그널로만 표시를 갱신한다 → **Qt 스레드는 절대 블록되지 않는다.**
- "**가능한 코드는 C++, UI 는 Python**": 모션 구동은 이미 C++(`ptp_box_align_node`).
  본 패키지는 전부 Python(UI + 오케스트레이션)이며 C++ 노드는 건드리지 않는다.
- `managed_process.py` 는 `openarmx_scenario_ui` 의 동일 파일을 **vendoring**(복사)한 것이다.
  시나리오 패키지에 의존하지 않고 전용 UI 를 자기완결로 유지하기 위함(백엔드 분리 정책과 동일).
  kill 로직이 원본에서 갱신되면 함께 동기화한다.

## 구동 파이프라인 (Launch 버튼)

ptp 는 MoveIt-free(Pinocchio IK → 단일 JTC 끝점)라 `move_group` 이 불필요하다.
검출 결과는 `box_perception_node` 가 `/detected_boxes`(PoseArray, base)로 발행하고
ptp 백엔드가 이를 구독한다.

| 순서 | 버튼 | launch / run |
|---|---|---|
| L0 (택1) | 하드웨어 SIL | `openarmx_scenario_player openarmx_hardware.launch.py use_fake_hardware:=true` |
| L0 (택1) | 하드웨어 실로봇 | `… use_fake_hardware:=false control_mode:=mit right_can_interface:=can2 left_can_interface:=can3 can_fd:=false` (확인 경고) |
| L1 | 컨트롤러 스폰 | `controller_manager spawner` (JSB + 좌/우 JTC + 그리퍼, `--unload-on-kill`) |
| 센서 | D435 카메라 | `openarmx_scenario_player d435_camera.launch.py` |
| 검출 | 원격검출+인지 | `yolov8_detection yolo_remote.launch.py node_name:=yolov8_node` (3d_detect_ws source) → `yolov8_node` + `box_perception_node` → `/detected_boxes` |
| 모션 | ptp 정렬 백엔드 | `openarmx_ptp_box_align ptp_box_align.launch.py` → 액션 `/openarmx/ptp_align_to_boxes` |
| 시각화 | RViz | `rviz2 -d <pkg>/config/openarmx_ptp.rviz` (UI 시작 시 자동 spawn) |

- RViz config 는 scenario 의 `openarmx_scenario.rviz` 를 **이 패키지로 복사한 자체 사본** [config/openarmx_ptp.rviz](config/openarmx_ptp.rviz) 를 src 에서 로드한다(자립 — scenario_player 부재에도 동작). scenario 뷰가 바뀌면 이 사본을 다시 복사해 동기화한다.
- 각 행 상태는 `● Stopped / Running (this UI) / Running (external)` 으로 2초 주기 갱신.
  외부 실행 감지는 rclpy 노드그래프 + `/proc` cmdline 스캔(서브프로세스 미사용).
- 중복 실행 감지·확인, 실로봇 안전 확인, 창 종료 시 this-UI 프로세스 일괄 정리(killpg) 포함.

## AlignToBoxes 골

`z`(TCP 높이, 기본 0.80 m), `roll/pitch/yaw`(기본 180/0/0 = 수직하강), `arms`(both/left/right).
`confidence`/`prompts` 는 ptp 가 무시(검출은 `box_perception_node` 책임)하므로 노출하지 않는다.
실행/취소는 in-process `ActionClient` (`send_goal_async`/`cancel_goal_async`) — feedback
(`phase`/`progress`)는 진행바·라벨에, result(`success`/`assignments_json`/`detections_json`)는 로그에 표시.

## 설정 (config · load-only)

운용값은 [config/ptp_pnp_ui.yaml](config/ptp_pnp_ui.yaml) 에 모아 두고 **UI 시작 시 한 번 읽는다**
(쓰기 없음 — 값 변경은 이 파일을 편집하고 UI 재기동). 로더 [openarmx_ptp_ui/app_config.py](openarmx_ptp_ui/app_config.py)
가 `DEFAULTS` 위에 per-key 병합하므로 일부만 적어도 동작한다. 우선순위는 src 파일(편집 즉시 반영) → 설치 share.

| 키 | 의미 | 기본 |
|---|---|---|
| `action_name` | AlignToBoxes 액션 이름 | `/openarmx/ptp_align_to_boxes` |
| `status_timer_ms` | launch 상태 표시 갱신 주기(ms) | `2000` |
| `detect_ws_setup` | 원격검출+인지 launch 전 source 할 3d_detect_ws setup | `/…/3d_detect_ws/install/setup.bash` |
| `hw_can.{right,left,mode,can_fd}` | 실로봇 CAN 인터페이스/모드 | `can2`/`can3`/`mit`/`false` |
| `goal_defaults.{z,roll_deg,pitch_deg,yaw_deg,arms}` | 골 입력칸 시작값 | `0.80`/`180`/`0`/`0`/`both` |

> launch 의 노드/프로세스 패턴·sweep 같은 **내부 식별자는 운용값이 아니므로** config 가 아닌
> `main_window.build_presets` 에 둔다(거의 안 바뀜).

## 탭 구성

`Pick and Place`(예비·빈) · `Detection` · `Motion Jog`(예비·빈) · `Launch`(구동 파이프라인) · `Pipe Health`.
- **Detection**: 좌 = 카메라 + YOLO(You Only Look Once) 오버레이(라이브 컬러 + 검출 시 `image_annotated` 2초 우선),
  우 = **3D 포인트클라우드**(`/camera/camera/depth/color/points`, pyqtgraph OpenGL 임베드, 마우스 회전/줌, XYZ+RGB
  다운샘플·throttle) + AlignToBoxes 골/Run/Cancel/로그. 영상·클라우드는 이 탭이 보일 때만 lazy 구독.
- **Launch**: 구동 8행(하드웨어 SIL/실로봇 · 컨트롤러 · D435 · 원격검출+인지 · ptp 백엔드 · RViz · TOF) Start/Stop/상태.
- **Pipe Health**: ptp 파이프라인 14토픽 발행률(Hz)/latched/age 표(`diag_spec.py`), 탭이 보일 때만 rate 카운트.

## 빌드 & 실행

```bash
# 3D 포인트클라우드 뷰 의존(최초 1회, pip --user — numpy 1.x 유지됨):
pip install --user pyqtgraph PyOpenGL

cd ~/TR-Works/kkw/China/openarmx_ws
colcon build --packages-select openarmx_ptp_box_align_msgs openarmx_ptp_ui --symlink-install
source install/setup.bash
ros2 launch openarmx_ptp_ui ptp_pnp_ui.launch.py        # 또는: ros2 run openarmx_ptp_ui ptp_pnp_ui.py
# RViz 자동 spawn 생략: ros2 run openarmx_ptp_ui ptp_pnp_ui.py --no-rviz
```

> 신규 `.py` 모듈을 추가하면 `--symlink-install` 이라도 **colcon 재빌드**해야 install 에 반영된다(미실행 시 ImportError).
> `pyqtgraph`/`PyOpenGL` 미설치 시 3D 뷰 자리에 "3D 뷰 사용 불가" 라벨이 뜨고 나머지 UI 는 정상 동작.

의존: `python3-pyqt5`, `rclpy`, `openarmx_ptp_box_align_msgs`(액션 타입),
`openarmx_ptp_box_align`(백엔드 launch), `openarmx_scenario_player`(하드웨어/카메라/RViz launch·config).
검출 파이프라인(`yolov8_detection`)은 별도 워크스페이스(`3d_detect_ws`)라 버튼이 내부에서 source 한다.

## 범위 밖 (이번 미포함)

라이브 자세 readout, 카메라 영상 뷰, joint/cartesian 제어, 색(class) 필터,
검출요청(1회) 버튼 — 본 요청(launch 버튼 + 실행/취소/상태)에 불필요. 필요 시 후속.
