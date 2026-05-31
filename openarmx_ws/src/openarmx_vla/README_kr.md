# OpenArmX VLA 사용 가이드 (LeRobot)

본 문서는 OpenArmX 로봇을 사용하여 LeRobot 프레임워크에서 VLA 데이터 수집, ACT 학습 및 추론을 수행하는 방법을 안내합니다.

<span style="background:#E8F5E9;padding:2px 8px;border-radius:6px;"><b>✅ 성공 안내</b></span>
<span style="background:#FFF8E1;padding:2px 8px;border-radius:6px;"><b>⚠️ 주의 사항</b></span>
<span style="background:#FFEBEE;padding:2px 8px;border-radius:6px;"><b>🚨 고위험 항목</b></span>
<span style="background:#E3F2FD;padding:2px 8px;border-radius:6px;"><b>💡 실용 팁</b></span>

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 본 문서에서 가장 중요한 세 가지:</b></span>  
> <b>1)</b> 실행 순서는 반드시 단계대로 엄격히 따라야 합니다.  
> <b>2)</b> `W/H/FPS`는 카메라 퍼블리시, 수집, 추론 세 곳에서 완전히 일치해야 합니다.  
> <b>3)</b> 수집 전에 `config/vla_collect.env`의 데이터셋 파라미터를 먼저 수정해야 합니다.

## 목차

1. 디바이스 분담
2. 일반 사전 조건
3. VLA 데이터 수집 플로우 (산업용 PC)
   - 3.1 수동 실행 순서 (반드시 순서대로)
   - 3.2 GUI 원클릭 실행 (권장)
   - 3.3 토픽 빠른 확인 (선택)
   - 3.4 녹화 인터페이스 키 설명
   - 3.5 데이터 수집 명령 파라미터 설명
4. ACT 학습 플로우 (사용자의 고성능 PC/서버)
   - 4.1 학습 전 안내
   - 4.2 ACT 의존 모델 다운로드
   - 4.3 ACT 학습 명령
5. VLA 추론 플로우 (산업용 PC + 추론 머신)
   - 5.1 추론 사전 조건
   - 5.2 실행 순서
   - 5.3 추론 명령 파라미터 설명
6. 듀얼 머신 협업 시 ROS_DOMAIN_ID 설정
   - 6.1 현재 설정 확인
   - 6.2 동일한 값으로 통합 설정 (예: 77, 불일치 시)
   - 6.3 재검증
7. 카메라 파라미터 설정 참조
   - 7.1 명령에서 변경해야 할 파라미터
   - 7.2 사용 가능한 해상도/프레임 레이트 조합 (D405 / D435)
   - 7.3 3 카메라 대역폭 상한과 권장 설정

---

## 🧩 1. 디바이스 분담

- **산업용 PC (당사 제공)**: 로봇 CAN 제어, Pico VR 텔레오퍼레이션, 3대 카메라 퍼블리시, LeRobot 데이터 수집.
- **사용자 머신 (사용자가 직접 설정)**: 모델 학습 및 추론 (산업용 PC와 연동 가능).

## ✅ 2. 일반 사전 조건

- 산업용 PC에서 워크스페이스가 빌드되어 있고, `source ~/openarmx_ws/install/setup.bash`가 정상 실행 가능해야 합니다.
- 로봇이 정상적으로 실행되고 VR 텔레오퍼레이션이 가능해야 합니다.
- 5절 추론 시나리오에서는 듀얼 머신 통신이 필요합니다. 6절을 참조하여 DOMAIN ID를 설정해 주시기 바랍니다.

> <span style="background:#FFF8E1;color:#8A6D3B;padding:2px 6px;border-radius:4px;"><b>⚠️ 2절 조건을 충족하지 않으면 이후 단계가 대부분 실패할 가능성이 큽니다.</b></span>

---

## 📦 3. VLA 데이터 수집 플로우 (산업용 PC)

### 🚦 3.1 수동 실행 순서 (반드시 순서대로)

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 단계를 건너뛰거나 병렬로 순서 없이 실행하는 것은 엄격히 금지합니다.</b></span>

#### Step 1: 로봇 실기 실행

```bash
cd ~/openarmx_ws
source install/setup.bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

#### Step 2: Pico Bridge 실행

```bash
cd ~/openarmx_ws
source install/setup.bash
ros2 run openarmx_teleop_bridge_vr_pico openarmx_teleop_bridge_vr_pico_node
```

#### Step 3: VR 텔레오퍼레이션 실행

```bash
cd ~/openarmx_ws
source install/setup.bash
ros2 launch openarmx_teleop_vr_pico teleop_vr_pico.launch.py
```

#### Step 4: 3 카메라 퍼블리시 실행

명령의 카메라 모델과 시리얼 번호를 본인의 디바이스 파라미터로 먼저 교체해 주시기 바랍니다.

- 지원 카메라 모델: `D435`, `D405`
- `cam_left_*` / `cam_right_*` / `cam_head_*`는 각각 좌측 핸드, 우측 핸드, 헤드 카메라에 대응합니다
- 시리얼 번호 조회: `rs-enumerate-devices | grep "Serial Number"`

표준 사양 산업용 PC + 표준 사양 확장 도크 기준, 3 카메라의 안정 상한은 `640x480 @ 30fps`입니다.
카메라 파라미터 선택과 사용 가능한 조합은 7절 「카메라 파라미터 설정 참조」를 참조해 주시기 바랍니다.

> <span style="background:#E3F2FD;color:#0D47A1;padding:2px 6px;border-radius:4px;"><b>💡 기본적으로 `424x240 @ 30fps`로 먼저 플로우를 통과시킨 후 점진적으로 해상도를 높일 것을 권장합니다.</b></span>

```bash
cd ~/openarmx_ws
source install/setup.bash
W=424; H=240; FPS=30
ros2 launch openarmx_lerobot camera_publisher.launch.py \
  width:=$W height:=$H fps:=$FPS \
  cam_left_serial:=218622270388 cam_left_type:=D405 \
  cam_right_serial:=218622274446 cam_right_type:=D405 \
  cam_head_serial:=335522070220 cam_head_type:=D435
```

실제 디바이스에 맞게 다음 파라미터를 수정해야 합니다.

- `W` / `H` / `FPS`: 3 카메라의 통합 해상도와 프레임 레이트 (예시는 `424x240@30`).
- `cam_left_serial` / `cam_right_serial` / `cam_head_serial`: 본인의 3대 카메라 시리얼 번호로 교체.
- `cam_left_type` / `cam_right_type` / `cam_head_type`: 실제 카메라 모델에 따라 `D405` 또는 `D435`로 기재.

실행 시 동시에 3대 카메라의 노출 파라미터를 조정하려는 경우, 다음 예시를 사용할 수 있습니다.

```bash
cd ~/openarmx_ws
source install/setup.bash
W=424; H=240; FPS=30
ros2 launch openarmx_lerobot camera_publisher.launch.py \
  width:=$W height:=$H fps:=$FPS \
  cam_left_serial:=218622270388 cam_left_type:=D405 \
  cam_right_serial:=218622274446 cam_right_type:=D405 \
  cam_head_serial:=335522070220 cam_head_type:=D435 \
  cam_left_color_auto_exposure:=true \
  cam_left_color_exposure:=10000 \
  cam_left_color_gain:=32 \
  cam_right_color_auto_exposure:=true \
  cam_right_color_exposure:=10000 \
  cam_right_color_gain:=32 \
  cam_head_color_auto_exposure:=true \
  cam_head_color_exposure:=10000 \
  cam_head_color_gain:=16
```

자주 조정되는 색상 파라미터는 다음과 같습니다.

- `cam_*_color_auto_exposure`: 컬러 자동 노출, 값 `true/false/unset`
- `cam_*_color_exposure`: 컬러 수동 노출, 범위 `1..10000`
- `cam_*_color_gain`: 컬러 수동 게인, 범위 `0..128`
- `cam_*_color_auto_white_balance`: 컬러 자동 화이트밸런스, 값 `true/false/unset`
- `cam_*_color_white_balance`: 컬러 수동 화이트밸런스, 범위 `2800..6500`
- `cam_*_color_brightness`: 밝기, 범위 `-64..64`
- `cam_*_color_contrast`: 대비, 범위 `0..100`
- `cam_*_color_saturation`: 채도, 범위 `0..100`
- `cam_*_color_sharpness`: 선명도, 범위 `0..100`

설명:

- `cam_left_*` / `cam_right_*` / `cam_head_*`는 각각 좌측 핸드, 우측 핸드, 헤드 카메라에 적용됩니다
- `unset`은 해당 파라미터를 능동적으로 설정하지 않고 드라이버 기본 동작을 유지함을 의미합니다
- `cam_*_color_exposure` 또는 `cam_*_color_gain`만 작성하면, launch가 자동으로 `cam_*_color_auto_exposure:=false`를 보완합니다
- `cam_*_color_white_balance`만 작성하면, launch가 자동으로 `cam_*_color_auto_white_balance:=false`를 보완합니다

#### Step 5: LeRobot 데이터 수집 실행

먼저 LeRobot 환경에 진입한 후 녹화 명령을 실행합니다.

- `W/H/FPS`는 수집 시 카메라 해상도와 프레임 레이트 설정에 사용됩니다 (예: `W=640; H=480; FPS=30`).
- 여기의 `W/H/FPS`는 카메라 퍼블리시 노드 `camera_publisher.launch.py`의 `width/height/fps`와 완전히 일치해야 합니다.
- 카메라 퍼블리시 노드의 W/H/FPS를 변경한 경우, 데이터 수집 명령의 W/H/FPS도 동일하게 변경해 주시기 바랍니다. 그렇지 않으면 카메라 포맷 불일치로 오류가 발생합니다.

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 핵심 제약: `수집 W/H/FPS` = `카메라 퍼블리시 width/height/fps`.</b></span>

```bash
lerobot-env
W=424; H=240; FPS=30
HF_HUB_OFFLINE=1 lerobot-record \
  --robot.type=openarmx_follower_ros2 \
  --robot.cameras="{cam_left: {type: ros2, image_topic: /cam_left/color/image, depth_topic: /cam_left/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_right: {type: ros2, image_topic: /cam_right/color/image, depth_topic: /cam_right/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_head: {type: ros2, image_topic: /cam_head/color/image, depth_topic: /cam_head/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}}" \
  --teleop.type=openarmx_leader_ros2 \
  --dataset.repo_id=local/데이터셋_이름 \
  --dataset.single_task="수행할 태스크 이름" \
  --dataset.num_episodes=수집할_총_에피소드_수 \
  --dataset.episode_time_s=에피소드당_시간(초) \
  --dataset.reset_time_s=에피소드_수집_후_간격_시간 \
  --dataset.push_to_hub=false \
  --display_data=true
```

예시:

```bash
lerobot-env
W=424; H=240; FPS=30
HF_HUB_OFFLINE=1 lerobot-record \
  --robot.type=openarmx_follower_ros2 \
  --robot.cameras="{cam_left: {type: ros2, image_topic: /cam_left/color/image, depth_topic: /cam_left/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_right: {type: ros2, image_topic: /cam_right/color/image, depth_topic: /cam_right/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_head: {type: ros2, image_topic: /cam_head/color/image, depth_topic: /cam_head/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}}" \
  --teleop.type=openarmx_leader_ros2 \
  --dataset.repo_id=local/take_box \
  --dataset.single_task="take box" \
  --dataset.num_episodes=70 \
  --dataset.episode_time_s=180 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=false \
  --display_data=true
```

### 🚀 3.2 GUI 원클릭 실행 (권장)

3.1절처럼 정해진 순서대로 여러 터미널 창을 자동으로 띄우고자 하는 경우, 저장소 내의 원클릭 실행 스크립트를 사용할 수 있습니다.

- `scripts/vla_collect_gui.sh`: GUI 멀티 터미널 원클릭 실행 스크립트
- ⚠️ `config/vla_collect.env`: 원클릭 실행 설정 파일, 로봇, 카메라, 데이터 수집 파라미터를 통합 저장. <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>이 파일에서 우선적으로 파라미터를 수정해 주세요</b></span>
- `scripts/README_GUI_kr.md`: 원클릭 실행의 독립 설명 문서

먼저 저장소 디렉터리로 진입할 것을 권장합니다.

```bash
cd /home/openarmx/openarmx_ws/src/openarmx_vla
```

실행 전에 자가 점검을 한 번 수행할 수 있습니다.

```bash
bash scripts/vla_collect_gui.sh check
```

자가 점검은 다음 항목을 확인합니다.

- `WORKSPACE_DIR/install/setup.bash` 존재 여부
- 현재 그래픽 데스크톱 세션 상태 여부 (`DISPLAY` / `WAYLAND_DISPLAY`)
- `GUI_TERMINAL`이 지정한 터미널 명령의 사용 가능 여부 (기본값 `gnome-terminal`)
- `ros2` 사용 가능 여부
- `collect` 시나리오에서 인터랙티브 셸 내에서 `LEROBOT_ENV_CMD`(기본값 `lerobot-env`)를 찾을 수 있는지

자주 사용하는 실행 방식:

```bash
# 1. 로봇 실기 + Pico Bridge + VR Teleop만 실행
bash scripts/vla_collect_gui.sh base

# 2. 로봇 하위 계층 + 카메라 퍼블리시 실행
bash scripts/vla_collect_gui.sh base_camera

# 3. 로봇 하위 계층 + 카메라 퍼블리시 + LeRobot 데이터 수집 실행
# ⚠️ 주의: 원클릭으로 데이터 수집을 실행하기 전에, 먼저 config/vla_collect.env의 DATASET_REPO_ID를 수정해 주세요
bash scripts/vla_collect_gui.sh collect

# 본 스크립트가 띄운 모든 터미널 종료
bash scripts/vla_collect_gui.sh stop
```

> <span style="background:#FFF8E1;color:#8A6D3B;padding:2px 6px;border-radius:4px;"><b>⚠️ `collect` 전에는 반드시 `DATASET_REPO_ID`를 확인해, 잘못된 데이터셋 디렉터리에 기록되는 것을 방지하시기 바랍니다.</b></span>

각 모드 대응 관계:

- `base`: 수동으로 "로봇 실기 + Pico Bridge + VR 텔레오퍼레이션" 세 단계를 실행한 것과 동일
- `base_camera`: `base` 기반에 3 카메라 퍼블리시를 이어서 실행
- `collect`: `base_camera` 기반에 LeRobot 데이터 수집을 이어서 실행
- `stop`: 상태 파일을 기반으로 본 스크립트가 띄운 창을 정확히 종료. 일부 창을 수동으로 닫은 경우에도 오류가 발생하지 않음

스크립트 실행 특성:

- 여러 터미널 창을 순서대로 띄우고, 설정된 지연 시간에 따라 각 모듈을 차례로 실행
- 각 창은 실행 종료 후에도 터미널에 유지되어 현장 오류 디버깅에 편리
- 이전 실행의 창이 여전히 실행 중인 것이 감지되면, 먼저 `bash scripts/vla_collect_gui.sh stop`을 실행하도록 안내
- `collect` 모드는 녹화 터미널에서 먼저 `LEROBOT_ENV_CMD`를 실행한 후 `lerobot-record`를 실행
- 기본 설정 파일 경로는 `config/vla_collect.env`이며, `VLA_CONFIG_FILE=/사용자_경로.env bash scripts/vla_collect_gui.sh collect`로 임시 설정 전환도 가능

일반적으로 `config/vla_collect.env`에서 다음 내용만 수정하면 됩니다.

- 기본 경로 및 GUI 파라미터: 워크스페이스 경로, 터미널 명령, 창 상태 파일 경로
- 로봇 하위 계층 파라미터: `CONTROL_MODE`, `ROBOT_CONTROLLER`, `USE_FAKE_HARDWARE`
- VR 텔레오퍼레이션 파라미터: Pico / Teleop의 제어 속도, 그립 임계값, 토픽명 등
- 카메라 파라미터: `W/H/FPS`, 3대 카메라 시리얼 번호, 카메라 모델, 노출/게인/화이트밸런스 등
- 데이터 수집 파라미터: 데이터셋 이름, 태스크 설명, episode 수, 단일 회차 시간, 리셋 시간, 데이터 표시 여부 등

그중 다음 항목에 특히 유의해야 합니다.

- `W/H/FPS`는 카메라 퍼블리시와 `lerobot-record`에 동시에 사용되므로, 카메라의 실제 출력과 일치하도록 유지해 주시기 바랍니다
- `CAM_LEFT_TYPE` / `CAM_RIGHT_TYPE` / `CAM_HEAD_TYPE`은 실제 디바이스에 따라 `D405` 또는 `D435`로 기재해야 합니다
- `CAM_LEFT_SERIAL` / `CAM_RIGHT_SERIAL` / `CAM_HEAD_SERIAL`은 본인의 카메라 시리얼 번호로 교체해야 합니다
- `collect` 모드를 사용하는 경우, `DATASET_REPO_ID`, `DATASET_SINGLE_TASK` 등 데이터셋 파라미터가 본인의 태스크로 변경되어 있는지 확인해 주시기 바랍니다

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 위 4개 항목은 자주 오류가 발생하는 지점이므로, 수집 전마다 하나씩 점검하실 것을 권장합니다.</b></span>

보다 자세한 원클릭 실행 설명은 다음 문서를 참조해 주시기 바랍니다: `scripts/README_GUI_kr.md`.

### 🔎 3.3 토픽 빠른 확인 (선택)

```bash
ros2 topic list | grep cam
ros2 topic list | grep joint_states
ros2 topic list | grep forward_position_controller/commands
```

최소한 다음이 표시될 것으로 예상됩니다.

- 카메라 토픽: `/cam_left/color/image`, `/cam_right/color/image`, `/cam_head/color/image`
- 관절 상태: `/joint_states`
- 텔레오퍼레이션 출력: `/left_forward_position_controller/commands`, `/right_forward_position_controller/commands`

위 조건이 충족되면 데이터 수집을 진행할 수 있습니다.

### ⌨️ 3.4 녹화 인터페이스 키 설명

- `→`(오른쪽 화살표): 현재 episode를 종료하고 저장한 후 리셋 단계로 진입.
- `←`(왼쪽 화살표): 현재 episode를 폐기하고 다시 녹화.
- `Esc`: 녹화 중지 및 종료, 데이터셋 저장.

### 🧾 3.5 데이터 수집 명령 파라미터 설명

자주 사용하는 파라미터:

- `HF_HUB_OFFLINE=1`: Hugging Face Hub 오프라인 모드 활성화.
- `--robot.type=openarmx_follower_ros2`: 제어 대상 로봇 타입 지정.
- `--teleop.type=openarmx_leader_ros2`: 텔레오퍼레이션 디바이스 타입 지정.
- `--dataset.repo_id=local/xxx`: 데이터셋 저장 식별자 (경로: `~/.cache/huggingface/lerobot/local/`).
- `--dataset.single_task`: 태스크 설명.
- `--dataset.num_episodes`: 총 episode 수.
- `--dataset.episode_time_s`: 에피소드당 최대 시간 (초).
- `--dataset.reset_time_s`: 에피소드 간 리셋 대기 시간 (초).
- `--dataset.push_to_hub`: Hugging Face Hub로 업로드 여부.
- `--display_data`: 실시간 데이터 표시 여부.

기타 파라미터:

- `--dataset.root`: 커스텀 데이터셋 저장 경로.
- `--dataset.fps`: 수집 프레임 레이트 제한.
- `--dataset.video`: 이미지를 비디오로 인코딩할지 여부.
- `--dataset.vcodec`: 비디오 인코더 (기본값 `libsvtav1`).
- `--dataset.video_encoding_batch_size`: 배치 비디오 인코딩의 episode 수.
- `--dataset.private`: Hub 업로드 시 비공개로 설정.
- `--dataset.tags`: Hub 데이터셋 태그.
- `--dataset.num_image_writer_processes`: 이미지 쓰기 프로세스 수.
- `--dataset.num_image_writer_threads_per_camera`: 카메라당 쓰기 스레드 수.
- `--dataset.rename_map`: 관측 키 이름 리네이밍.

---

## 🧠 4. ACT 학습 플로우 (사용자의 고성능 PC/서버)

### 📌 4.1 학습 전 안내

ACT 학습은 사용자가 준비한 고성능 PC 또는 서버에서 진행할 것을 권장합니다 (독립 GPU 사용 권장).
학습을 시작하기 전에, 본 머신에서 LeRobot 환경 설치를 먼저 완료하고 해당 모델을 다운로드해 주시기 바랍니다. 본 문서는 ACT를 예로 들며, 환경 설정과 다양한 모델 학습 튜토리얼에 대한 추가 정보는 공식 웹사이트 문서를 참조해 주시기 바랍니다: <http://docs.openarmx.com/>.

### 📥 4.2 ACT 의존 모델 다운로드

LeRobot 환경 터미널에서 실행합니다.

```bash
mkdir -p ~/.cache/torch/hub/checkpoints
# LeRobot 환경에 진입하여 의존성 설치
lerobot-env
wget https://mirrors.tuna.tsinghua.edu.cn/pytorch/models/resnet18-f37072fd.pth \
  -O ~/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth
```

### 🏋️ 4.3 ACT 학습 명령

#### 4.3.1 단일 GPU 학습 (선택)

```bash
lerobot-env
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
lerobot-train \
  --dataset.repo_id=local/데이터셋_이름 \
  --dataset.root=데이터의_절대_경로 \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir=outputs/학습된_모델_이름 \
  --batch_size=각_학습_스텝의_배치_크기 \
  --steps=총_학습_스텝_수 \
  --log_freq=로그_출력_간격 \
  --save_freq=저장_간격
```

#### 4.3.2 다중 GPU 학습 (선택)

```bash
lerobot-env
export HF_HUB_OFFLINE=1
export TRANSFORMERS_OFFLINE=1
torchrun --nproc_per_node=GPU_개수 \
  "$(which lerobot-train)" \
  --dataset.repo_id=local/데이터셋_이름 \
  --dataset.root=데이터의_절대_경로 \
  --policy.type=act \
  --policy.push_to_hub=false \
  --output_dir=outputs/학습된_모델_이름 \
  --batch_size=각_학습_스텝의_배치_크기 \
  --steps=총_학습_스텝_수 \
  --log_freq=로그_출력_간격 \
  --save_freq=저장_간격
```

학습 완료 후, `output_dir`에 내보내진 `pretrained_model` 경로를 기록하여 5절 추론에 사용하시기 바랍니다.

---

## 🤖 5. VLA 추론 플로우 (산업용 PC + 추론 머신)

본 절에서는 ACT를 예로 들어, 학습된 모델을 로드하여 온라인 추론을 수행하는 방법을 안내합니다.
현재 플로우에서는 **산업용 PC와 사용자 머신 간 듀얼 머신 통신이 필요하므로**, 추론 전에 먼저 6절 `ROS_DOMAIN_ID` 설정을 완료해 주시기 바랍니다.

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 추론 전에 듀얼 머신 통신 설정을 완료하지 않으면 정상적인 연동이 거의 불가능합니다.</b></span>

### ✅ 5.1 추론 사전 조건

- 학습이 완료되어 있고, 모델 경로(보통 `pretrained_model` 디렉터리)를 확보한 상태.
- 산업용 PC에서 로봇과 카메라가 정상적으로 실행 가능한 상태.
- 추론은 일반적으로 다른 사용자 PC에서 실행되므로, 듀얼 머신 통신이 필요합니다. 먼저 6절 `ROS_DOMAIN_ID` 설정을 완료해 주시기 바랍니다.

### 🚦 5.2 실행 순서

#### Step 1: 로봇 실기 실행 (산업용 PC)

```bash
cd ~/openarmx_ws
source install/setup.bash
ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

#### Step 2: 3 카메라 퍼블리시 실행 (산업용 PC)

7절 「카메라 파라미터 설정 참조」에 따라 `W/H/FPS`와 3대 카메라의 `serial/type`을 수정해 주시기 바랍니다.

```bash
cd ~/openarmx_ws
source install/setup.bash
W=424; H=240; FPS=30
ros2 launch openarmx_lerobot camera_publisher.launch.py \
  width:=$W height:=$H fps:=$FPS \
  cam_left_serial:=218622270388 cam_left_type:=D405 \
  cam_right_serial:=218622274446 cam_right_type:=D405 \
  cam_head_serial:=335522070220 cam_head_type:=D435
```

추론 전에 3대 카메라 노출을 고정하려면, 3절과 동일한 3 카메라 노출 예시를 사용해 위 명령 뒤에 다음을 추가할 수도 있습니다.

- 좌측 핸드: `cam_left_color_auto_exposure:=false cam_left_color_exposure:=400 cam_left_color_gain:=32`
- 우측 핸드: `cam_right_color_auto_exposure:=false cam_right_color_exposure:=400 cam_right_color_gain:=32`
- 헤드: `cam_head_color_auto_exposure:=false cam_head_color_exposure:=300 cam_head_color_gain:=16`

#### Step 3: 추론 실행 (추론 머신)

- 추론 명령의 `W/H/FPS`는 현재 카메라 퍼블리시 노드의 `width/height/fps`와 완전히 일치해야 합니다.
- 추론 시 카메라 포맷(해상도/프레임 레이트)은 데이터 수집 시와 일치해야 합니다 (해당 모델 학습에 사용된 데이터 포맷과 일치할 것을 권장).

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 핵심 제약: `추론 W/H/FPS` = `수집 W/H/FPS` = `카메라 퍼블리시 width/height/fps`.</b></span>

```bash
lerobot-env
W=424; H=240; FPS=30
HF_HUB_OFFLINE=1 lerobot-record \
  --robot.type=openarmx_follower_ros2 \
  --robot.cameras="{cam_left: {type: ros2, image_topic: /cam_left/color/image, depth_topic: /cam_left/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_right: {type: ros2, image_topic: /cam_right/color/image, depth_topic: /cam_right/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_head: {type: ros2, image_topic: /cam_head/color/image, depth_topic: /cam_head/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}}" \
  --robot.skip_send_action=false \
  --dataset.repo_id="local/추론_완료_모델_이름" \
  --dataset.single_task="태스크_이름" \
  --dataset.num_episodes=추론_횟수 \
  --dataset.push_to_hub=false \
  --display_data=true \
  --policy.path="학습된_모델_경로"
```

예시:

```bash
lerobot-env
W=424; H=240; FPS=30
HF_HUB_OFFLINE=1 lerobot-record \
  --robot.type=openarmx_follower_ros2 \
  --robot.cameras="{cam_left: {type: ros2, image_topic: /cam_left/color/image, depth_topic: /cam_left/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_right: {type: ros2, image_topic: /cam_right/color/image, depth_topic: /cam_right/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}, cam_head: {type: ros2, image_topic: /cam_head/color/image, depth_topic: /cam_head/depth/image, use_depth: true, width: $W, height: $H, fps: $FPS}}" \
  --robot.skip_send_action=false \
  --dataset.repo_id=local/eval_take_box \
  --dataset.single_task="take the box" \
  --dataset.num_episodes=10 \
  --dataset.push_to_hub=false \
  --display_data=true \
  --policy.path="/home/i4090/openarmx_vla/src/VLA/OUTPUTS/045000/pretrained_model"
```

### 🧾 5.3 추론 명령 파라미터 설명

- `HF_HUB_OFFLINE=1`: 오프라인 모드, 인터넷에서 리소스를 가져오지 않습니다.
- `--robot.type=openarmx_follower_ros2`: 추론 액션을 전송할 대상 로봇 타입.
- `--robot.skip_send_action=false`: `false`는 실제로 액션을 전송함을 의미하며, `true`는 플로우만 검증하고 로봇을 움직이지 않음을 의미.
- `--dataset.repo_id="local/추론_완료_모델_이름"`: 추론 결과 저장 식별자.
- `--dataset.single_task="태스크_이름"`: 추론 태스크명 (메타 정보).
- `--dataset.num_episodes=추론_횟수`: 추론 회차 수.
- `--dataset.push_to_hub=false`: Hugging Face Hub로 업로드하지 않음.
- `--display_data=true`: 추론 과정 데이터 표시.
- `--policy.path="학습된_모델_경로"`: 로컬 모델 경로.

---

## 🌐 6. 듀얼 머신 협업 시 ROS_DOMAIN_ID 설정

산업용 PC와 사용자 머신 간 크로스 머신 통신이 필요한 경우, 두 머신의 `ROS_DOMAIN_ID`가 일치해야 합니다 (동일한 값, 예: `77`로 통합할 것을 권장).

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 두 머신의 `ROS_DOMAIN_ID`가 일치하지 않으면 크로스 머신 토픽 디스커버리가 실패합니다.</b></span>

### 6.1 현재 설정 확인

두 머신에서 각각 실행합니다.

```bash
echo $ROS_DOMAIN_ID
```

### 6.2 동일한 값으로 통합 설정 (예: 77, 불일치 시)

두 머신에서 모두 실행합니다 (먼저 `DOMAIN_ID`를 양측이 통합할 값으로 설정. 예시는 `77`).

```bash
DOMAIN_ID=77
grep -q '^export ROS_DOMAIN_ID=' ~/.bashrc \
  && sed -i "s/^export ROS_DOMAIN_ID=.*/export ROS_DOMAIN_ID=${DOMAIN_ID}/" ~/.bashrc \
  || echo "export ROS_DOMAIN_ID=${DOMAIN_ID}" >> ~/.bashrc
source ~/.bashrc
```

### 6.3 재검증

```bash
echo $ROS_DOMAIN_ID
```

두 머신의 출력이 일치하면 완료입니다.

---

## 📷 7. 카메라 파라미터 설정 참조

### 🛠️ 7.1 명령에서 변경해야 할 파라미터

`camera_publisher.launch.py`를 사용할 때, 일반적으로 다음 파라미터만 수정하면 됩니다.

- `W` / `H` / `FPS`: 해상도와 프레임 레이트.
- `cam_left_serial` / `cam_right_serial` / `cam_head_serial`: 3대 카메라 시리얼 번호.
- `cam_left_type` / `cam_right_type` / `cam_head_type`: 3대 카메라 모델 (`D405` 또는 `D435`).
- 아래 파라미터명의 `*`는 리터럴이 아닌 플레이스홀더이며, 구체적인 카메라 접두사로 교체해야 합니다.
  - 좌측 핸드 카메라: `cam_left`
  - 우측 핸드 카메라: `cam_right`
  - 헤드 카메라: `cam_head`
- 예: `cam_*_color_exposure`는 실제로는 `cam_left_color_exposure`, `cam_right_color_exposure`, `cam_head_color_exposure` 중 하나로 작성해야 합니다.
- `cam_*_color_auto_exposure`: 컬러 자동 노출, 값 `true/false/unset`.
- `cam_*_color_exposure`: 컬러 수동 노출, 범위 `1..10000`.
- `cam_*_color_gain`: 컬러 수동 게인, 범위 `0..128`.
- `cam_*_color_auto_white_balance`: 컬러 자동 화이트밸런스, 값 `true/false/unset`.
- `cam_*_color_white_balance`: 컬러 수동 화이트밸런스, 범위 `2800..6500`.
- `cam_*_color_brightness`: 밝기, 범위 `-64..64`.
- `cam_*_color_contrast`: 대비, 범위 `0..100`.
- `cam_*_color_saturation`: 채도, 범위 `0..100`.
- `cam_*_color_sharpness`: 선명도, 범위 `0..100`.

`lerobot-record` (수집 및 추론) 명령에서도 동일한 `W/H/FPS`를 설정하여 일관성을 유지해야 합니다.

- `lerobot-record`의 `W/H/FPS` = `camera_publisher.launch.py`의 `width/height/fps`.
- 추론 `W/H/FPS` = 데이터 수집 `W/H/FPS` (해당 모델 학습에 사용된 데이터와 일치할 것을 권장).

> <span style="background:#FFEBEE;color:#B71C1C;padding:2px 6px;border-radius:4px;"><b>🚨 여기서의 일관성은 전체 플로우 안정성의 핵심 제약입니다.</b></span>

시리얼 번호 조회 명령:

```bash
rs-enumerate-devices | grep "Serial Number"
```

### 📐 7.2 사용 가능한 해상도/프레임 레이트 조합 (D405 / D435)

`camera_publisher.launch.py`에는 검증이 내장되어 있으며, 다음 유효 조합을 사용해야 합니다.

### Intel RealSense D405

| 해상도 | 지원 프레임 레이트 |
|--------|-----------|
| 1280 x 720 | 5, 15, 30 |
| 848 x 480 | 5, 15, 30, 60, 90 |
| 640 x 480 | 5, 15, 30, 60, 90 |
| 640 x 360 | 5, 15, 30, 60, 90 |
| 480 x 270 | 5, 15, 30, 60, 90 |
| 424 x 240 | 5, 15, 30, 60, 90 |

### Intel RealSense D435 / D435i

| 해상도 | 지원 프레임 레이트 |
|--------|-----------|
| 1920 x 1080 | 6, 15, 30 |
| 1280 x 720 | 6, 15, 30 |
| 848 x 480 | 6, 15, 30, 60, 90 |
| 640 x 480 | 6, 15, 30, 60, 90 |
| 640 x 360 | 6, 15, 30, 60, 90 |
| 480 x 270 | 6, 15, 30, 60, 90 |
| 424 x 240 | 6, 15, 30, 60, 90 |

### 💡 7.3 3 카메라 대역폭 상한과 권장 설정

- 표준 사양 산업용 PC + 표준 사양 확장 도크 기준, 3 카메라 안정 상한: `640x480 @ 30fps`.
- 기본 권장 설정: `424x240 @ 30fps` (대역폭 점유율이 더 낮아 안정적).
- 더 높은 화질을 추구하는 경우, 프레임 레이트를 낮추거나 동시 카메라 수를 줄이시기 바랍니다.
