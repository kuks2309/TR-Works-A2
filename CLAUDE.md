# Claude 작업 지침 (China 워크스페이스 루트)

## 핵심 원칙

1. **사용자가 지시한 사항만 수행한다**
2. **임의로 기능을 추가하거나 변경하지 않는다**
3. **작업 완료 후 지시한 내용만 했는지 확인하고 보고한다**
4. **객관적인 사실을 가지고 판단한다**
5. **관련 이론을 철저히 조사한다**

## 프로젝트 성격

OpenArmX 이족(bimanual) 로봇 매니퓰레이션 monorepo. 주요 워크스페이스: `openarmx_ws/` (ROS2 / MoveIt / 시나리오 플레이어), `cyclo_control` / `cyclo_ws` (QP+CBF 모션 컨트롤러), `3d_detect_ws` (박스 검출), `calibration` (카메라 캘리브레이션).

## 도메인 문서 SSOT

작업 영역에 따라 **시작 전 반드시** 해당 README 를 먼저 읽고 규칙을 따른다. 각 파일이 단일 근원(Single Source of Truth) 이며 본 CLAUDE.md 에 복제하지 않는다.

| 영역 | 진입점 | 트리거 |
| --- | --- | --- |
| 이슈 / 수정 기록 | [docs/issues_and_fixes/README.md](docs/issues_and_fixes/README.md) | 버그 수정·빌드 실패·에러 로그 |
| Claude 실수 기록 (재발 방지) | [docs/claude-mistake/README.md](docs/claude-mistake/README.md) | 사용자 정정·자가 인지 실수 |
| 코드 리뷰 SOP | [docs/claude_guideline/code_review.md](docs/claude_guideline/code_review.md) | "코드 리뷰", "코드 분석", "리뷰해줘" |
| 사용자 지시 기록 | [docs/claude_guideline/user_instruction_recording.md](docs/claude_guideline/user_instruction_recording.md) | (기록 대상: docs/user_instructions/) |
| 외부 참조 문서 처리 | [docs/claude_guideline/external_reference_handling.md](docs/claude_guideline/external_reference_handling.md) | datasheet·매뉴얼·spec·REP/IEEE/RFC |

## 자동화 자산

- **issue-fix 스킬** → [.claude/skills/issue-fix/SKILL.md](.claude/skills/issue-fix/SKILL.md) — 진단→제안→구현→검증→기록 사이클
- **pre-commit hook** → `.git/hooks/pre-commit` — 소스 수정 시 issues_and_fixes.md / claude-mistake 미갱신 경고 (차단 아님)

원본 SSOT: [kuks2309/kuks_claude_setup](https://github.com/kuks2309/kuks_claude_setup). 규칙 변경이 필요하면 해당 README 수정 여부를 먼저 사용자에게 문의한다.
