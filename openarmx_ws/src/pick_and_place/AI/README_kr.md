# pick_and_place / AI 백엔드 — VLA(Vision-Language-Action) / ACT

`cyclo` · `pilz` · `ptp` (검출+계획 기반 classical pick)에 이은 **4번째 백엔드**로,
**학습 기반(모방학습)** pick 을 담당한다. OpenArmX 벤더의 `openarmx_vla` 패키지를
**이 프로젝트(China 워크스페이스) 실제 구성에 맞춰 적응 이식**한 것이다.

> 백엔드 분리 정책: cyclo/pilz/ptp 와 동일하게 AI 백엔드도 의도적으로 별도 유지한다.

## 1. 무엇이 적응되었나 (원본 openarmx_vla 대비)

| 항목 | 원본 openarmx_vla | 이 AI 백엔드 |
| --- | --- | --- |
| 텔레오퍼레이션 | VR(Virtual Reality) Pico 컨트롤러 | **물리 leader arm** (`teleop_bimanual`, CAN(Controller Area Network) 직접) |
| 카메라 | 손목 D405 2대 + 헤드 D435 (`/cam_*`) | **중앙(헤드) D435 1대** (`d435_camera.launch.py`, `/camera/camera/...`) |
| 기동 | VR launch/GUI (이 워크스페이스에 패키지 없음) | bringup + `teleop_bimanual` + `d435_camera` |
| 경로 | `~/openarmx_ws` | `…/China/openarmx_ws` |

**바뀌지 않은 것(검증 완료)**: 관절명(`openarmx_{left,right}_joint1..7 + finger_joint1`, 8DOF),
컨트롤러명(`{left,right}_forward_position_controller`), command 토픽, 그리퍼 단위(미터).
→ leader-follower·관절·컨트롤러 계층은 이 프로젝트와 그대로 호환된다.

## 2. 구성

```
AI/
├── README_kr.md
├── config/vla_collect.env                         # 일괄 실행 설정 (먼저 수정)
├── scripts/
│   ├── vla_collect_gui.sh                          # 멀티 터미널 일괄 기동
│   └── install_lerobot_plugins.sh                  # 플러그인 pip 설치
├── lerobot_robot_openarmx_follower_ros2/           # LeRobot follower 플러그인 (단일캠 적응)
└── lerobot_teleoperator_openarmx_leader_ros2/      # LeRobot leader 플러그인
```

데이터 흐름: leader arm(손으로 시연) → command 토픽 → ① follower 로봇 구동
+ ② leader 플러그인이 command 를 **action** 으로 기록 / follower 플러그인이
`/joint_states`+영상을 **observation** 으로 기록 (`skip_send_action=True`, 이중제어 방지).

## 3. 설치 (최초 1회)

플러그인은 colcon 이 아니라 **LeRobot 환경에 pip 설치**한다(별도 GPU 머신/추론 머신에서도 동일).

```bash
lerobot-env                                  # LeRobot 환경 진입
bash scripts/install_lerobot_plugins.sh      # follower/leader entry-point 등록
```

## 4. 데이터 수집 (실로봇 IPC)

```bash
cd /home/openarmx/TR-Works/kkw/China/openarmx_ws/src/pick_and_place/AI
# 1) 설정 점검 — config/vla_collect.env 에서 DATASET_REPO_ID / CAN 번호 / W·H·FPS 확인
bash scripts/vla_collect_gui.sh check
# 2) 일괄 기동: bringup + leader teleop + D435 + lerobot-record
bash scripts/vla_collect_gui.sh collect
# 종료
bash scripts/vla_collect_gui.sh stop
```

녹화 창 키: `→` 저장 후 다음, `←` 폐기 후 재녹화, `Esc` 종료/저장.

> ⚠️ 수집 전 매번 `DATASET_REPO_ID` 를 새 이름으로. `W/H/FPS` 는 실제 D435 스트림과 일치.

## 5. 학습 (별도 고성능 GPU 머신, ACT(Action Chunking Transformer) 예시)

```bash
lerobot-env
export HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1
# ACT 백본(resnet18) 최초 1회 다운로드 필요
lerobot-train \
  --dataset.repo_id=local/ai_take_box \
  --dataset.root=<데이터셋_절대경로> \
  --policy.type=act --policy.push_to_hub=false \
  --output_dir=outputs/ai_take_box \
  --batch_size=8 --steps=100000 --log_freq=200 --save_freq=20000
```

산출물 `outputs/ai_take_box/.../pretrained_model` 경로를 추론에 사용한다.

## 6. 추론 (학습 모델 적재)

```bash
# IPC: bringup(forward_position_controller/mit) + d435_camera 먼저 기동
lerobot-env
W=640; H=480; FPS=30
HF_HUB_OFFLINE=1 lerobot-record \
  --robot.type=openarmx_follower_ros2 \
  --robot.cameras="{cam_head: {type: ros2, image_topic: /camera/camera/color/image_raw, use_depth: false, width: $W, height: $H, fps: $FPS}}" \
  --robot.skip_send_action=false \
  --dataset.repo_id=local/eval_ai_take_box \
  --dataset.single_task="take box" --dataset.num_episodes=10 \
  --dataset.push_to_hub=false --display_data=true \
  --policy.path="<pretrained_model 경로>"
```

- `--robot.skip_send_action=false` ⇒ 정책이 실제로 로봇을 구동(수집 때 `True`와 반대).
- 2머신(IPC+추론머신)이면 양쪽 `ROS_DOMAIN_ID` 일치 필요.

## 7. 주의점

- 🚨 **추론 점프 무방비**: follower `max_relative_target=None`(클리핑 없음). 실로봇 첫 추론 전
  `config_openarmx_ros2.py` 에서 `max_relative_target`(라디안) 설정 권장.
- **단일 시점 한계**: 중앙 캠 1대만으로는 정밀 파지의 천장이 낮다(손목 캠 없음).
  테이블 위 coarse reach/pick 위주로 시작. 추후 손목 D405 추가 시 `--robot.cameras`에
  항목을 더하고 **그 데이터로 재학습**.
- **depth 미기록**: follower `get_observation`은 컬러만 기록 → `use_depth=false`.
- **camera encoding**: realsense 컬러를 numpy 직접 디코드(cv_bridge 우회)로 처리.
- ⚖️ **라이선스**: 플러그인 원본은 CC BY-NC-SA 4.0(**비영리 전용**, Chengdu Changshu Robot).
  상업적 사용은 별도 검토 필요. 원본 라이선스 헤더는 보존되어 있다.

## 8. 원본 대비 이식 검증 요약

관절명/컨트롤러명/8DOF/그리퍼 단위 = **코드로 일치 확인**. leader-follower(`teleop_bimanual`,
8DOF Float64MultiArray, 동일 command 토픽) = 무수정 호환. 카메라 = 단일캠 재설정 완료.
실제 E2E(End-to-End) 수집/추론 런타임 검증은 미수행(하드웨어 필요).
