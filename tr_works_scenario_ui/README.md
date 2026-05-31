# tr_works_scenario_ui

TR-Works 시나리오 플레이어용 PyQt5 GUI. 두 종류의 창을 제공한다.

| Executable | 창 | 용도 |
|---|---|---|
| `scenario_ui.py` | Scenario Player (콤보박스 + Play/Cancel) | 프로세스 매니지먼트 + registered 시나리오 전체 재생 |
| `scenario_box_ui.py` | **Scenario Box UI (본 문서)** | `scenario.json` 로드 후 sub-motion 단위 원클릭 재생 |

두 UI 모두 내부적으로 같은 `ScenarioRosBridge`를 사용해 `scenario_player/play` Action (`PlayScenario.action`) 및 `scenario_player/list` Service를 호출한다.

---

## Scenario Box UI

### 목적

`scenario.json`(또는 `steps` 배열이 있는 sub-scenario 파일)을 직접 열고, 내부에 정의된 모션(sub-scenario) 하나하나를 **큰 박스 버튼**으로 렌더링하여 클릭 한 번으로 해당 모션만 재생한다. 긴 시퀀스를 순차 재생하기보다는 **개별 모션 단위 검증/시연**에 적합.

### 사전 조건

1. Hardware bringup 가동:
   ```bash
   ros2 launch tr_works_bringup hardware.launch.py
   ```
2. scenario_player 가동 (`scenario_player/play` Action 서버가 살아있어야 클릭이 동작한다):
   ```bash
   ros2 run tr_works_scenario_player scenario_player_node.py \
     --ros-args -p scenario_search_path:=$PWD/Scenarios
   ```
   (기존 `scenario_ui.py`가 띄워져 있다면 이미 둘 다 기동됨)

### 실행

```bash
cd ~/Project/TR-Works-Dev/kkw/TR-Works_ros2_ws
source install/setup.bash

ros2 run tr_works_scenario_ui scenario_box_ui.py
# 또는
ros2 launch tr_works_scenario_ui scenario_box_ui.launch.py
```

### 사용 순서

1. **`Load Scenario...` 클릭** → 파일 선택 다이얼로그.
   - 기본 시작 경로: 워크스페이스 내부 `Scenarios/` 디렉토리 (환경변수 `TR_WORKS_SCENARIOS_DIR`로 재정의 가능).
   - 고를 파일: **`scenario.json`** (top-level `sequence` 배열이 있는 파일). sub-scenario 파일(`steps`만 있는 것)을 고르면 오류 다이얼로그로 옆 디렉토리의 scenario.json을 안내한다.
2. **모션 박스 렌더링**
   - `sequence` 배열 순서대로 sub-scenario 단위 박스 생성.
   - 박스 라벨: `1\n<sub_name>`, `2\n<sub_name>`, ... (4열 grid).
3. **재생**
   - 재생할 박스 클릭 → `bridge.play(scenario_path, start_sub=i, end_sub=i, speed, dry_run)` 호출.
   - 재생 중에는 활성 박스가 노란 테두리로 강조되고, 다른 박스는 비활성화.
   - Progress bar + 라벨에 `[phase]`, `elapsed`, 현재 step이 표시됨.
   - Status / Feedback / Result 로그는 하단 로그 창에 타임스탬프와 함께 출력.
4. **중단**: `Cancel` 클릭 → Action goal cancel.
5. 완료 시 박스들이 다시 활성화됨.

### 옵션

| 컨트롤 | 동작 |
|---|---|
| `Speed` 스핀박스 | `speed_scale` 값 (기본 0.5, 범위 0.10~2.00) |
| `Dry-run` 체크박스 | `true`면 scenario_player가 실제 컨트롤러에 publish하지 않고 로그만 남김 |
| `Clear Log` | 로그창 초기화 |

### 시나리오 파일 포맷

본 UI는 **top-level `scenario.json`만** 받는다.

```json
{
  "name": "pick_and_place",
  "description": "...",
  "sequence": ["01_init", "02_pick", "03_place"]
}
```

sub-scenario 파일(`steps` 배열만 있는 `01_init.json` 등)을 고르면 `'sequence' must be a non-empty list` 오류로 scenario_player가 거부한다 — 이는 player의 의도된 동작이므로 UI 단에서 Load 시점에 차단하고, 같은 디렉토리의 `scenario.json` 경로를 오류 메시지에 함께 표시한다.

---

## 기존 UI (`scenario_ui.py`)

- Hardware bringup + scenario_player 프로세스 자동 기동/정지
- `scenario_player/list` service로 **등록된 시나리오** 콤보박스 채움
- Sub 선택 / Speed / Dry-run / Play / Cancel
- 자세한 auto-start 시퀀스: [tr_works_scenario_ui/main_window.py](tr_works_scenario_ui/main_window.py) 상단 주석 참조

---

## 파일 구성

```
src/tr_works_scenario_ui/
├── CMakeLists.txt
├── package.xml
├── launch/
│   ├── scenario_ui.launch.py
│   └── scenario_box_ui.launch.py       ← 신설
├── scripts/
│   ├── scenario_ui.py                   (chmod +x 필수)
│   └── scenario_box_ui.py               (chmod +x 필수)  ← 신설
├── ui/
│   ├── scenario_ui.ui
│   └── scenario_box_ui.ui               ← 신설
└── tr_works_scenario_ui/
    ├── __init__.py
    ├── main_window.py                   (기존 UI)
    ├── scenario_box_window.py           ← 신설
    ├── managed_process.py
    └── scenario_action_client.py        (공용 ROS 브리지)
```

## 주의

- **`ros2 run` "No executable found"**: 신설 스크립트는 반드시 `chmod +x`를 확인할 것. `--symlink-install` 환경에서 install path는 source 파일 mode를 그대로 따른다.
- **Action 서버 미존재**: scenario_player가 죽은 상태에서 박스를 클릭하면 3초 대기 후 로그에 `scenario_player/play action server unavailable` 출력. UI는 멈추지 않음.
