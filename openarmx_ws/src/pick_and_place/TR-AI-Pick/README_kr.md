# TR-AI-Pick — 타당성 결론 · 아키텍처 · 진행 상황

> D435 + 라즈베리파이 YOLOv8 박스 3D 인식 → leader-follower 텔레오퍼레이션으로 pick 자세(그리퍼 OPEN) 시연·기록 → AI(VLA/ACT)가 pick 자세를 자동 추론해 로봇 구동.
> 16-에이전트 타당성 조사(2026-06-07) + 실측 검증 결과 정리. **세션 재개용 단일 진입점.**

## 1. 결론 — 조건부 GO

세 기능 모두 구현 가능하며 **대부분 이미 구현·검증돼 있다.** 진짜 신규 작업은 적고, 병목은 코드가 아니라 ① 시연 데이터 수집(현재 0건) ② 학습 인프라 ③ 라이선스(상업용일 경우)다. 팀은 20명 불필요 — 임계경로가 "물리 로봇 1대 시연 수집 + 직렬 하드웨어 검증"이라 **~5명이 적정**.

## 2. 기능별 판정 (객관 근거)

| 기능 | 판정 | 근거 |
| --- | --- | --- |
| 1. D435+Pi YOLOv8 → 박스 3D(base frame) | ✅ 이미 구현+실증 | `3d_detect_ws`: Pi seg HTTP → `yolo_remote_node` 핀홀 역투영 → `box_perception_node`가 깊이+camera→base TF로 `/detected_boxes`(PoseArray, `openarmx_body_link0`) 발행. cyclo 양팔 0mm, 재시험 err 0.098mm. 신규 비전 코드 0. |
| 2. leader-follower로 pick 자세 이동(그리퍼 OPEN) | ✅ 이미 구현 | `openarmx_teleop_bimanual` CAN 직결 8DOF 복사. 그리퍼=명령벡터 8번째 원소(별도 제어/IK/VR 불필요). |
| 3-수집. 자세 기록 | ✅ (1줄 블로커 해결됨) | LeRobot follower=16관절+RGB, leader=action 기록. 그리퍼(finger_joint1)도 기록. **leader entry-point 미등록 → 수정 완료(§4).** |
| 3-추론. AI 자동 pick 추론 | ⚠️ 조건부 | collect→train→infer 골격 존재. 데이터 0·학습 GPU·추론 executor 노드·안전장치 필요. |
| 통합/리스크 | ⚠️ | CC BY-NC-SA 비영리 라이선스(상업 배포 시 하드 블로커), 속도 워치독/E-stop 부재. |

## 3. 배치 토폴로지 (확정)

```
라즈베리파이+Hailo-8 ── 검출(YOLOv8-seg HEF, 이미 학습/컴파일)
제어 PC(IPC, venv) ──── 수집(lerobot-record) + 로봇 제어        ┐ ROS_DOMAIN_ID 일치
Jetson Orin ─────────── ACT 정책 추론(PyTorch FP16)             ┘ (추론 3노드)
GPU PC ──────────────── 학습(lerobot-train, 오프라인)
```

- **Hailo-8로 ACT 추론은 부적합**(조사 결론): Hailo-8은 CNN 비전 전용, 트랜스포머는 Hailo-10H 영역, INT8 양자화가 연속 파지 정밀도 직격 + 검출과 칩 경합. ACT 추론은 **Jetson Orin(PyTorch FP16) 권장**. Hailo는 검출 전담.
- 단일 중앙캠+깊이 미기록 → coarse 테이블 pick이 천장. 정밀 파지는 손목 카메라 추가+재학습 필요.

## 4. 진행 상황 (2026-06-07 완료분)

- ✅ **① leader teleoperator entry-point 등록** — `lerobot_teleoperator_openarmx_leader_ros2/pyproject.toml`에 `[project.entry-points."lerobot.teleoperators"]` 추가. 런타임 검증 PASS(`lerobot.teleoperators: ['openarmx_leader_ros2']`).
- ✅ **② LeRobot venv 구축** — `~/lerobot-venv`(`--system-site-packages`): LeRobot 0.4.4 / numpy 2.2.6 / scipy 1.15.3. leader·follower editable 설치, entry-points 등록, `lerobot-record --help` OK, `rclpy+lerobot+플러그인` 동시 import OK.
- ✅ **install_lerobot_plugins.sh 수정** — 평범한 `pip install -e`는 rclpy(PyPI 부재) 백트래킹으로 행 + 빌드격리 시 leader가 UNKNOWN. → `pip install --no-deps --no-build-isolation -e` + 빌드의존성 선설치로 교체(검증됨).
- 기록: `docs/issues_and_fixes/issues_and_fixes.md`(2026-06-07 항목), 메모리 `lerobot_venv_numpy2_strategy`.

### venv 재개 방법
```bash
source /opt/ros/humble/setup.bash
source ~/lerobot-venv/bin/activate
# 검증: lerobot-record --help  /  python -c "import rclpy,lerobot"
# 플러그인 재설치 필요시: bash openarmx_ws/src/pick_and_place/AI/scripts/install_lerobot_plugins.sh
```
- ⚠️ venv는 LeRobot 전용. **colcon 빌드는 base 환경**(venv setuptools 80 ↔ colcon 충돌).
- ⚠️ `--system-site-packages` 누수: 시스템 numpy 1.x 패키지(scipy 등)가 새어 깨지면 venv에 최신판 강제 설치로 shadow(scipy는 해결됨). cv_bridge는 플러그인이 회피.

## 5. 남은 단계 (③~, 실하드웨어 게이트)

1. **③ 첫 시연 수집 E2E shakedown** — 실로봇+leader+D435+CAN 필요. 1~2 에피소드로 CAN(can0/can1)/카메라 W·H·FPS(640/480/30)/clock 정렬 검증. `AI/scripts/vla_collect_gui.sh`.
2. **검출↔수집 배선** — `vla_collect_gui.sh`에 `yolo_remote.launch.py`(node_name:=yolov8_node) 창 추가.
3. **추론 executor 노드(~200 LOC)** — 학습 ACT 정책 적재 → `/{left,right}_forward_position_controller/commands` 발행 + action chunk→per-frame 언롤. (현재 CLI 경로만 존재.)
4. **안전** 🔴 — 실로봇 추론 전 `config_openarmx_ros2.py`의 `max_relative_target`(현재 None) 설정 + 속도 워치독/E-stop.
5. **학습 인프라** — GPU PC(lerobot-train) + Jetson Orin(추론), ROS_DOMAIN_ID 일치.
6. **라이선스** 🔴 — CC BY-NC-SA 4.0 비영리(Chengdu Changshu Robot). 상업 사용 시 별도 협상(법무/사업 트랙).

## 6. 패키지 처리 권고

신규 colcon 패키지 만들지 말 것(`pick_and_place/AI/`가 이미 4번째 백엔드). TR-AI-Pick은 `3d_detect_ws` 검출 + `AI/` 수집/추론을 조립하는 **thin compose 스크립트/launch**만 두거나 이 문서 같은 계획·기록 용도로 사용.

## 7. 참조

- 기존 백엔드: `pick_and_place/{cyclo,ptp,pilz}` (검출+계획 기반 classical pick), `pick_and_place/AI` (VLA/ACT, 4번째)
- 검출: `3d_detect_ws/src/yolov8_detection`, `pi_yolo_server`
- teleop: `openarmx_teleop_bimanual`(물리 leader), `openarmx_teleop_vr`(참조 — 물리 leader 경로엔 IK 불필요해 미사용)
- 이슈/메모리: `docs/issues_and_fixes/issues_and_fixes.md`, 메모리 `lerobot_venv_numpy2_strategy` · `pick_and_place_ai_vla_backend`
