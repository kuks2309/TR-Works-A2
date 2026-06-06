# 2026-06-07 — openarmx_vla → pick_and_place/AI 백엔드 이식 (단일캠 + leader-follower)

벤더 패키지 `openarmx_vla`(VR(Virtual Reality) Pico + 손목 D405 3캠 가정)를 이 프로젝트
실제 구성(**물리 leader arm + 중앙 D435 단일캠**)에 맞춰 `pick_and_place/AI/` 4번째 백엔드로
적응 이식. cyclo/pilz/ptp(검출+계획 기반)에 이은 **학습 기반(ACT(Action Chunking Transformer)) pick** 백엔드.

## 2026-06-07 07:02 (KST) — pick_and_place/AI 가 .gitignore 'AI/' 에 걸려 추적 불가

### 증상
`pick_and_place/AI/` 에 소스를 두고 커밋하려 했으나 `git status` 에 전혀 나타나지 않음.
`git check-ignore` 결과 `.gitignore:33` 의 `AI/` 패턴(이름이 AI 인 모든 디렉터리)에 매치.

### 원인
`.gitignore` 의 `AI/` 는 "AI training workspace (large weights/dataset/HEF/HAR)"를 git 밖으로
빼기 위한 의도였으나, 이름만으로 매치하는 광범위 패턴이라 **소스코드 디렉터리** `pick_and_place/AI/`
까지 통째로 무시함.

### 수정
`.gitignore` 에 이 경로만 추적 예외(negation) 추가. 단, 하위 대용량 산출물은 계속 제외.
```
AI/
!openarmx_ws/src/pick_and_place/AI/
!openarmx_ws/src/pick_and_place/AI/**
openarmx_ws/src/pick_and_place/AI/**/outputs/
openarmx_ws/src/pick_and_place/AI/**/*.{pth,pt,safetensors,ckpt,bin,onnx,hef,har}
```
`git check-ignore` 로 소스=추적 / weights·outputs=무시 양쪽 검증 완료.

### 재발 방지
이름 기반 광범위 ignore(`AI/`)는 소스 디렉터리와 충돌 가능. 새 백엔드/폴더 추가 시
`git status` 에 보이는지 먼저 확인하고, 필요하면 경로 한정 예외를 둔다.

## 이식 내용 요약

- **복사+적응**: `lerobot_robot_openarmx_follower_ros2`(카메라만 3캠→중앙 D435 1대,
  `/camera/camera/color/image_raw`, `use_depth=False`), `lerobot_teleoperator_openarmx_leader_ros2`(무수정).
- **신규/적응**: `scripts/vla_collect_gui.sh`(VR pico→`teleop_bimanual`, 3캠 D405→`d435_camera`,
  record 단일캠), `config/vla_collect.env`(경로 수정 + leader CAN + 단일캠 W/H/FPS),
  `scripts/install_lerobot_plugins.sh`, `README_kr.md`.

### 이식 호환성 (코드 검증)
- 관절명(`openarmx_{l,r}_joint1..7 + finger_joint1`, 8DOF) / 컨트롤러명
  (`{left,right}_forward_position_controller`) / command 토픽 / 그리퍼 단위(미터) = **일치**.
- leader-follower: `teleop_bimanual_node` 가 8DOF Float64MultiArray 를 동일 command 토픽으로
  발행 → VLA leader 플러그인과 무수정 호환.

### 미검증 / 주의
- 런타임 E2E(End-to-End) 수집/추론 미수행(실로봇 하드웨어 필요).
- `W/H/FPS=640x480x30`은 가정값 — 실제 d435 스트림과 일치 필요(`camera_info` 확인).
- leader CAN(can0/can1)은 launch 기본값 — 실제 배선과 일치 필요.
- 추론 점프 안전(`max_relative_target`)은 기본 None 유지(README 에 설정 권장 명시).
- 플러그인 원본 라이선스 CC BY-NC-SA 4.0(**비영리**) — 상업적 용도 별도 검토.
