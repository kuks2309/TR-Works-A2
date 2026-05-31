# VLA GUI 원클릭 실행 가이드

## 목표

명령 하나를 실행하여 `README_kr(1).md`의 수동 절차를 따라하는 것처럼, 여러 터미널 창을 순차적으로 띄웁니다.

## 파일

- ⚠️ `config/vla_collect.env`: 로봇, 카메라, 데이터 수집 파라미터를 통합 관리하는 설정 파일입니다. **<span style="color:red">설정하실 파라미터는 이 파일에서 수정해 주세요.</span>**
- `scripts/vla_collect_gui.sh`: GUI 멀티 터미널 원클릭 실행 스크립트

## 세 가지 모드

```bash
cd /home/openarmx/openarmx_ws/src/openarmx_vla

# 1. 로봇 실기 + Pico Bridge + VR Teleop만 실행
bash scripts/vla_collect_gui.sh base

# 2. 로봇 하위 계층 + 카메라 퍼블리시 실행
bash scripts/vla_collect_gui.sh base_camera

# 3. 로봇 하위 계층 + 카메라 퍼블리시 + LeRobot 데이터 수집 실행
bash scripts/vla_collect_gui.sh collect

# 본 스크립트가 띄운 모든 터미널을 원클릭으로 종료
bash scripts/vla_collect_gui.sh stop
```

## 실행 전 자가 점검

```bash
bash scripts/vla_collect_gui.sh check
```

`stop`은 사용자가 일부 터미널을 수동으로 먼저 종료한 후 실행해도 오류 없이 동작합니다.

사전에 다음 항목을 점검합니다.

- `setup.bash`
- `DISPLAY` / `WAYLAND_DISPLAY`
- `gnome-terminal`
- `ros2`
- `lerobot-env` (필요 시 인터랙티브 셸 방식으로 점검)

`collect` 모드에서는 데이터 수집 터미널 내에서 먼저 `lerobot-env`를 실행한 다음 `lerobot-record`를 실행합니다.

## 파라미터 수정

다음 파일을 직접 수정합니다. **<span style="color:red">설정하실 파라미터는 우선적으로 이 파일에서 수정해 주세요.</span>**

- `config/vla_collect.env`

해당 파일에는 다음 항목이 포함되어 있습니다.

- 로봇 하위 계층 파라미터
- Pico / VR 텔레오퍼레이션 파라미터
- 카메라 해상도, 프레임 레이트, 시리얼 번호, 모델명
- `CAM_RIGHT_COLOR_EXPOSURE`
- `CAM_RIGHT_COLOR_GAIN`
- 기타 `cam_left/right/head_color_*` 파라미터
- LeRobot 데이터 수집 파라미터

## 주의 사항

- 반드시 그래픽 데스크톱 터미널에서 실행해야 합니다. 스크립트가 `gnome-terminal`을 호출합니다
- `W/H/FPS`는 카메라 퍼블리시와 `lerobot-record`에 동시에 사용됩니다
- 띄워진 각 터미널은 명령 종료 후에도 유지되어 현장 디버깅에 편리합니다
- 본 버전에서는 로그 디렉터리 생성 기능이 제거되었습니다
- 터미널 상태 파일은 기본적으로 `STATE_FILE`에 기록되며, `stop` 명령의 정확한 종료에 사용됩니다
