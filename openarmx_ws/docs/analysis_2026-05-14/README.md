# OpenArmX 워크스페이스 분석 (2026-05-14)

본 폴더는 20개 병렬 에이전트로 수행한 워크스페이스 전체 조사와 5개 에이전트로 수행한 solver 심층 분석 결과를 정리한 것입니다.

## 문서 구성

| 파일 | 내용 |
|---|---|
| [01_workspace_overview.md](01_workspace_overview.md) | 프로젝트 정체, ROS2 환경, 라이선스, 패키지 맵 요약 |
| [02_packages.md](02_packages.md) | 16개 빌드 패키지 + 19개 소스 패키지 상세 분석 |
| [03_solvers.md](03_solvers.md) | MoveIt IK / 기구학 YAML / KDL 동역학 / VR IK 솔버 심층 분석 |
| [04_build_state.md](04_build_state.md) | 빌드 상태, 배포 프로파일(.repos), CAN 드라이버 |
| [05_findings.md](05_findings.md) | 주요 관찰사항 및 개선 권장사항 |

## 메타데이터

- **분석 일자:** 2026-05-14
- **워크스페이스 경로:** `/home/openarmx/TR-Works/kkw/China/openarmx_ws/`
- **ROS2 버전:** Humble (Ubuntu 22.04)
- **마지막 빌드:** 2026-04-23 (약 21일 경과)
- **라이선스:** CC BY-NC-SA 4.0
- **제작:** 청두 창수 로봇 (Chengdu Changshu Robot Co., Ltd.) — 6.0_basic
