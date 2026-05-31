# Claude 작업 지침

Claude 가 본 프로젝트(CRCS / ROS2 / 임베디드 등)에서 어떻게 일할지 정의하는 메타 규칙의 단일 근원(SSOT).

본 폴더의 규칙은 워크스페이스 루트 [`CLAUDE.md`](../../CLAUDE.md) 의 "문서 작업 규칙" 섹션에서 진입한다. CLAUDE.md 에는 복제하지 않는다.

## 진입점

| 영역 | 파일 |
|---|---|
| ROS2 / 임베디드 작업 규칙 (해당 프로젝트일 때 가장 먼저) | [ros2.md](ros2.md) |
| 작업 절차 체크리스트 (시작/종료 + 펌웨어 플래시 절차) | [workflow.md](workflow.md) |
| 코드 작업 규칙 (금지사항·수정 규칙·상수 분리·사전 승인 트리거) | [coding.md](coding.md) |
| GitHub 워크플로 (커밋/푸쉬/PR/기록 요청 처리·vendored 가드) | [github.md](github.md) |
| 기술 부채 방지 (정공법 우선) | [tech_debt.md](tech_debt.md) |
| 매뉴얼 / 데이터시트 보관·인용·검증 | [manual.md](manual.md) |
| 스킬 / 자동화 자산 SSOT 등록 절차 | [skill_update.md](skill_update.md) |
| 문서 작성 방법 (메타 규칙) | [documentation.md](documentation.md) |
| 프로젝트별 비공개 override (gitignore) | [local/](local/) |

## 변경 이력

[CHANGELOG.md](CHANGELOG.md)

## 규칙 변경

본 폴더의 규칙은 단일 근원이므로, 변경이 필요하면 해당 파일을 수정하기 전에 사용자 승인을 받는다.
