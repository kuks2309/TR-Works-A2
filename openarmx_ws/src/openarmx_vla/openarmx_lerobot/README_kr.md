# openarmx_lerobot 한국어 설명

## 1. 패키지 구조

```text
openarmx_lerobot/
├── README.md
├── README_CN.md
├── camera_viewer.py
├── package.xml
├── setup.py
├── setup.cfg
├── launch/
│   ├── openarmx_lerobot.launch.py
│   └── camera_publisher.launch.py
└── openarmx_lerobot/
    └── __init__.py
```

## 2. 패키지 포지셔닝

`openarmx_lerobot`은 "조합 실행 및 사용 가이드" 패키지로, 주로 두 가지 역할을 수행합니다.

> 카메라 퍼블리시 및 디버깅 도구 제공: RealSense 3 카메라 통합 토픽 퍼블리시 + OpenCV 로컬 카메라 뷰어.
> 하위 계층 제어 알고리즘은 담당하지 않으며, 시스템 컴포넌트를 빠르게 기동시켜 LeRobot 녹화/텔레오퍼레이션 플로우에 손쉽게 연결할 수 있도록 하는 역할을 합니다.

## 3. 일반적인 사용 워크플로우

1. 워크스페이스 빌드 및 로드:

```bash
cd <워크스페이스 경로>
colcon build --packages-select openarmx_lerobot
source install/setup.bash
```

2. 터미널 1: 듀얼암 로봇 실행

```bash
# 실기 모드: 먼저 CAN 채널 활성화
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

sudo ip link set can1 down
sudo ip link set can1 type can bitrate 1000000
sudo ip link set can1 up

cd <워크스페이스 경로>
source install/setup.bash

ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
  control_mode:=mit \
  robot_controller:=forward_position_controller \
  use_fake_hardware:=false
```

3. 터미널 2: Pico 브릿지 실행

```bash
cd <워크스페이스 경로>
source install/setup.bash
ros2 run openarmx_teleop_bridge_vr_pico openarmx_teleop_bridge_vr_pico_node
```

4. 터미널 3: VR 텔레오퍼레이션 노드 실행

```bash
cd ~/openarmx_ws/
source install/setup.bash
ros2 launch openarmx_teleop_vr_pico teleop_vr_pico.launch.py
```

5. 터미널 4: 카메라 퍼블리시 노드 실행

시리얼 번호와 모델명을 실제 값으로 교체합니다. 시리얼 번호는 보통 카메라 본체 바닥의 라벨에 표시되어 있습니다.

```bash
cd ~/openarmx_ws/
source install/setup.bash
ros2 launch openarmx_lerobot camera_publisher.launch.py \
  width:=424 height:=240 fps:=15 \
  cam_left_serial:=시리얼번호 cam_left_type:=모델명 \
  cam_right_serial:=시리얼번호 cam_right_type:=모델명 \
  cam_head_serial:=시리얼번호 cam_head_type:=모델명
```

예시:

```bash
ros2 launch openarmx_lerobot camera_publisher.launch.py \
  width:=424 height:=240 fps:=15 \
  cam_left_serial:=218622270388 cam_left_type:=D405 \
  cam_right_serial:=218622274446 cam_right_type:=D405 \
  cam_head_serial:=335522070220 cam_head_type:=D435
```

6. 터미널 5: LeRobot 수집-수신 측 실행 (다른 PC에서 데이터를 수신하는 경우 이 단계를 실행하지 않아도 됩니다)

```bash
lerobot-env  # conda의 lerobot 환경 활성화

HF_HUB_OFFLINE=1 lerobot-record \
  --robot.type=openarmx_follower_ros2 \
  --teleop.type=openarmx_leader_ros2 \
  --dataset.repo_id=local/데이터셋_이름 \
  --dataset.single_task="수행할 태스크 이름" \
  --dataset.num_episodes=수집할_총_에피소드_수 \
  --dataset.episode_time_s=에피소드당_시간_초 \
  --dataset.reset_time_s=에피소드_간_리셋_시간_초 \
  --dataset.push_to_hub=false \
  --display_data=true
```

예시:

```bash
lerobot-env  # conda의 lerobot 환경 활성화

HF_HUB_OFFLINE=1 lerobot-record \
  --robot.type=openarmx_follower_ros2 \
  --teleop.type=openarmx_leader_ros2 \
  --dataset.repo_id=local/openarmx_dataset \
  --dataset.single_task="Teleop OpenArmX robot" \
  --dataset.num_episodes=100 \
  --dataset.episode_time_s=60 \
  --dataset.reset_time_s=5 \
  --dataset.push_to_hub=false \
  --display_data=true
```
데이터 기본 저장 경로: `~/.cache/huggingface/lerobot/local`

**수집 단축키 설명**

- `→`: 현재 episode를 종료하고 저장한 뒤 reset으로 진입
- `←`: 현재 episode를 폐기하고 다시 녹화
- `Esc`: 녹화 중지 및 종료

**수집 파라미터 빠른 참조**

자주 사용하는 파라미터:

- `HF_HUB_OFFLINE=1`: HF Hub 오프라인 모드
- `--robot.type`: 로봇 플러그인 타입
- `--teleop.type`: 텔레오퍼레이션 플러그인 타입
- `--dataset.repo_id`: 데이터셋 식별자 (로컬 예시: `local/openarmx_dataset`)
- `--dataset.single_task`: 태스크 설명
- `--dataset.num_episodes`: 수집 총 에피소드 수
- `--dataset.episode_time_s`: 에피소드당 최대 시간
- `--dataset.reset_time_s`: 에피소드 간 리셋 대기 시간
- `--dataset.push_to_hub`: Hub로 업로드 여부
- `--display_data`: 실시간 시각화 여부

기타 옵션 파라미터:

- `--dataset.root`: 커스텀 저장 경로
- `--dataset.fps`: 샘플링 프레임 레이트 제한
- `--dataset.video`: 이미지를 비디오로 인코딩할지 여부
- `--dataset.vcodec`: 비디오 인코더 (기본값 `libsvtav1`)
- `--dataset.video_encoding_batch_size`: 배치당 인코딩할 에피소드 수
- `--dataset.private`: 업로드 시 비공개 설정
- `--dataset.tags`: Hub 태그
- `--dataset.num_image_writer_processes`: 이미지 쓰기 프로세스 수
- `--dataset.num_image_writer_threads_per_camera`: 카메라당 쓰기 스레드 수
- `--dataset.rename_map`: 관측 키 리네이밍

---
