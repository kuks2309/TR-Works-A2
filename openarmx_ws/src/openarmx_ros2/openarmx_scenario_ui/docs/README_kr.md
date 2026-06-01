# openarmx_scenario_ui — 시나리오 작성/재생 GUI

## 프로젝트 목적

OpenArmX 양팔 로봇에서 **자세 시나리오를 GUI로 작성하고 재생**하는 통합 환경.

| 단계 | 사용자가 하는 일 | 산출물 |
|---|---|---|
| **1. 로봇 움직이기** | 다양한 입력 방식으로 원하는 자세 만듦 | 현재 EE/joint 자세 |
| **2. 자세 저장** | "Save Pose" / "Capture" 클릭 | `~/openarmx_ws/scenarios/poses/<name>.json` |
| **3. 시나리오 작성** | 저장된 pose들을 순서·속도와 함께 조합 | `scenarios/<name>/scenario.json` |
| **4. 시나리오 재생** | Scenario Player 노드가 step 시퀀스 자동 실행 | 양팔 협조 동작 |

`scenario_ui.py`는 1·2 단계를 직접 제공하고, 3·4 단계는 `openarmx_scenario_player` 백엔드와 함께 동작.

---

## 로봇을 움직이는 방법 (현재 지원)

| # | 방법 | 위치 | 동작 단위 | 상태 |
|---|---|---|---|---|
| 1 | **Joint 슬라이더** | Joint Control 탭 | 관절각 (deg). 좌/우 7관절 + 그리퍼 14축. Home/Init/Mirror 지원 | ✅ 동작 |
| 2 | **RViz EE Leader Marker 드래그** | Cartesian Control → Marker sub-tab | EE pose 전체 (RViz 6-DoF 인터랙티브 링). idle 시 link7 따라옴 (auto_follow_link@10Hz) | ✅ 동작 |
| 3 | **Cartesian Jog** | Cartesian Control → Jog sub-tab | ±X/Y/Z (mm), ±RX/RY/RZ (°) discrete step. step size 1/5/10/50/100mm · 1/5/10/30° | ✅ 동작 (MoveIt 필요) |
| 4 | **Manual pose 수치 입력** | Cartesian Control → Manual sub-tab | 6 spinbox (Δx/y/z + ΔR/P/Y). Delta 또는 Absolute 모드 | ✅ 동작 (MoveIt 필요) |
| 5 | **OpenArmX Leader arm 직접 핸들링** | (예정) | 사람이 leader arm 손으로 조작 → follower arm 추종 | ⏳ 곧 통합 예정 |

### 1. Joint 슬라이더 — 가장 직관적
- 슬라이더 또는 spinbox에 각도 입력
- `Auto publish` 토글 ON이면 매 변경 시 자동 송신
- HIL 모드(컨트롤러 active): 디바운스된 trajectory 송신
- SIL 모드(컨트롤러 없음): `/joint_states` 직접 publish
- `Mirror L→R` 토글로 양팔 대칭 동작

### 2. RViz Marker 드래그 — 시각적
- RViz에서 좌/우 link7에 붙은 빨간/파란 6-DoF 링을 마우스로 드래그
- 마우스 release 시점에 `/openarmx/{arm}/ee_leader/goal_pose` (PoseStamped) publish
- 두 가지 follower 경로 (UI CLI `--follower`):
  - `--follower=moveit` (default): UI가 토픽 받아 MoveGroup plan&execute (PILZ_LIN/PTP/CIRC/OMPL)
  - `--follower=cyclo`: `vr_controller_node` (QP+CBF)가 자체 추종 — `cyclo_motion_controller_ros` 빌드 필요
- **auto_follow_link** (ee_leader_marker 파라미터): idle 상태에서는 마커가 link7 현재 위치 자동 추적 (10Hz). robot이 어떤 수단으로 움직여도 마커는 EE에 stick.

### 3. Cartesian Jog — 한 축씩 미세 조정
- 6축(±X/Y/Z, ±RX/RY/RZ) 버튼 클릭 1회 = step만큼 discrete 이동
- Frame: `openarmx_{arm}_link0` (base) | `openarmx_{arm}_link7` (tool) | `world` 선택
- 매 클릭마다 plan&execute (PILZ_LIN/PTP/CIRC/OMPL)

### 4. Manual pose 수치 입력 — 정확한 좌표
- 6 spinbox에 직접 숫자 입력
- Mode: **Delta from current** (현재 EE에서 offset) | **Absolute** (frame 기준 절대 좌표)
- `Copy current → Absolute` 버튼으로 현재 pose를 입력란에 복사
- `Plan (preview)` → `Plan & Execute` 두 단계 또는 한 번에

### 5. Leader Arm 직접 핸들링 — (곧 통합)
- OpenArmX에서 별도 제공 예정. 통합 후 본 표에 추가.

---

## UI 자동 spawn 구조

`scenario_ui.py` 실행 시 다음이 자동으로 같이 뜸 (closeEvent에 정리):

```
scenario_ui.py
  ├─ rviz2 -d openarmx_scenario.rviz       (전용 RViz)
  ├─ ee_leader_right_marker  (controlled_link = openarmx_right_link7)
  └─ ee_leader_left_marker   (controlled_link = openarmx_left_link7)
```

`--follower=cyclo` 추가 시:
```
  ├─ cyclo_sim (mock HW + JTC + RSP)
  └─ vr_controller_node (QP+CBF bimanual follower)
```

`--no-rviz` 옵션으로 자동 spawn 비활성 가능 (외부 RViz 사용 시).

전용 RViz config: `openarmx_scenario_player/config/openarmx_scenario.rviz` (Grid + RobotModel + EE Leader Right/Left InteractiveMarkers, Fixed Frame = `openarmx_body_link0`). MotionPlanning 등 무관한 패널 없음.

---

## UI 탭 구성

| 탭 | 역할 |
|---|---|
| **Scenario Player** | 시나리오 로드/재생 (3·4 단계) |
| **Joint Control** | 1번 방식 (관절각 슬라이더) |
| **Cartesian Control** | 2/3/4번 방식 sub-tab — `Marker` / `Jog` / `Manual` |
| **Launch Manager** | SIL/HW Bringup, MoveIt Demo, Scenario Player node 별도 기동 |
| **Diagnostics** | 토픽 rate, 노드 alive 모니터링 |

공통 header (Cartesian Control): Arm 선택, Planner 선택 (PILZ_LIN/PTP/CIRC/OMPL), Frame dropdown, Current EE Pose 표시 + Refresh.

---

## 실행

### 기본 (MoveIt path)
```bash
source /opt/ros/humble/setup.bash
source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash

# 1. UI (RViz + 마커 자동 spawn)
ros2 run openarmx_scenario_ui scenario_ui.py --no-auto

# 2. UI Launch Manager에서:
#    - SIL Bringup Start  (가상 HW + 컨트롤러)
#    - MoveIt Demo Start  (Cartesian Jog/Manual 사용 시)
```

### cyclo path (QP+CBF 자체 추종)
```bash
ros2 run openarmx_scenario_ui scenario_ui.py --no-auto --follower=cyclo
```
> `cyclo_motion_controller_ros` 패키지가 워크스페이스에 빌드돼 있어야 cyclo_sim + vr_controller 노드가 정상 spawn.

### 외부 RViz 사용 (자동 spawn off)
```bash
ros2 run openarmx_scenario_ui scenario_ui.py --no-auto --no-rviz
```

---

## 시나리오 파일 포맷

```
~/openarmx_ws/scenarios/
├── poses/
│   └── <pose_name>.json          # 단일 자세 (Joint Control "Save Pose" 또는 Cartesian "Capture")
└── <scenario_name>/
    ├── scenario.json             # 최상위 — "sequence": [<sub_name>, ...]
    └── <sub_name>/
        └── scenario.json         # sub — "steps": [<step dict>, ...]
```

step 타입 (예시):
- `{"type": "movej", "arm": "right", "joint_positions": [...], "duration_sec": 2.0}`
- `{"type": "movel", "arm": "right", "target_pose": {...}, "planner": "pilz_lin", "duration_sec": 3.0}`
- `{"type": "gripper", "arm": "left", "position": 0.0, "max_effort": 14.0}`
- `{"type": "wait", "duration_sec": 0.5}`

상세 step schema는 [`tr_works_scenario_ui_port_plan.md`](../../../../../../.omc/plans/tr_works_scenario_ui_port_plan.md) §6 참조.

---

## 참고 문서
- Pilz LIN 사용법: [`openarmx_bimanual_moveit_config/docs/RUN.md`](../../openarmx_bimanual_moveit_config/docs/RUN.md)
- ee_leader_marker 소스: [`src/ee_leader_marker/`](../../../ee_leader_marker/)
- scenario_player 백엔드: [`openarmx_scenario_player/`](../../openarmx_scenario_player/)
