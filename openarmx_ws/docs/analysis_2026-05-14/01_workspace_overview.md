# 워크스페이스 개요

## 프로젝트 정체

**OpenArmX**는 청두 창수 로봇(Chengdu Changshu Robot Co., Ltd.)이 공개한 **양팔 협동 로봇 플랫폼** 오픈소스 스택입니다.

| 항목 | 값 |
|---|---|
| 로봇 모델 | v10 (양팔, 팔당 7-DOF) |
| 모터 | Robstride RS04 / RS03 / RS00 (CAN 통신) |
| ROS2 배포판 | Humble |
| OS | Ubuntu 22.04 |
| 빌드 시스템 | colcon, C++17 |
| 라이선스 | CC BY-NC-SA 4.0 (비상업용) |
| 다국어 문서 | EN / CN / KR (모터 매니저는 JP / RU 추가) |
| 저장소 브랜치 | `6.0_basic` |

## 전체 스택 흐름

```
로봇 모델(URDF)
   ↓
하드웨어 제어(ros2_control + CAN)
   ↓
텔레오퍼레이션 (양팔 leader-follower / VR / 외골격)
   ↓
임바디드 인텔리전스 (LeRobot / ACT 기반 VLA)
```

## 워크스페이스 구조

```
openarmx_ws/
├── src/
│   ├── openarmx_ros2/              # 코어 (메타, 하드웨어, 브링업, 중력보상, MoveIt)
│   ├── openarmx_description/       # URDF + 메쉬 + 기구학 YAML
│   ├── openarmx_teleop_bimanual/   # 양팔 leader-follower 텔레오프
│   ├── openarmx_teleop_vr/         # VR 텔레오프 (Python IK + C++ UDP 브리지)
│   ├── openarmx_teleop_vr_apk/     # Quest / Pico APK 바이너리
│   ├── openarmx_tools/             # RViz 패널 + teach 도구
│   ├── openarmx_vla/               # LeRobot 연동
│   └── openarmx_motor_manager/     # PySide6 데스크톱 앱 (ROS 아님)
├── build/                          # colcon 빌드 캐시
├── install/                        # 설치된 패키지 16개
├── log/                            # 빌드 로그
└── docs/                           # 본 분석 자료
```

## 배포 프로파일 (.repos)

| 파일 | 용도 |
|---|---|
| `openarmx_minimal.repos` | description만 (모델 시각화) |
| `openarmx.repos` | 표준 (teleop_bimanual + tools + motor_manager) |
| `openarmx_vr.repos` | + VR 텔레오프 + APK |
| `openarmx_vla.repos` | + VLA 전체 스택 |

**추가 의존성:** `openarmx-can_1.0.0_{amd64,arm64}.deb` — 사전 컴파일된 저수준 CAN 드라이버, colcon build 전 설치 필요.

## 빌드 상태 (2026-05-14 기준)

- **빌드 일자:** 2026-04-23 15:30 (~21일 경과 — 재빌드 권장)
- **결과:** 16/16 패키지 정상 설치, 에러 0
- **소스 19개 vs 빌드 16개 차이:** `openarmx_teleop_bimanual`(빌드됨), `openarmx_teleop_vr_apk`(빌드 대상 아님), `openarmx_motor_manager`(ROS 아님) 등이 차이 원인.
