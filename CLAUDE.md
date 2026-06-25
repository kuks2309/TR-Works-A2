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

<!-- kuks_agent_setup:user_instruction -->
- 사용자 지시 도착 시(상시) **응답 전 의무**: 먼저 docs/claude_guideline/user_instruction/recording.md 규칙에 따라 지시 원문을 docs/user_instructions/user_instructions.md 맨 위에 즉시 prepend 기록한다(작업 후 일괄 금지, 원문만·분석 금지).

<!-- kuks_agent_setup:external_reference -->
- 외부 참조 문서(매뉴얼·datasheet·SDK·표준) 트리거 감지 시 **응답 전 의무 선행 점검**(등록만 알고 건너뛰지 말 것): 먼저 docs/claude_guideline/external_reference/handling.md 를 Read 한 뒤 보관(references/)·인용(출처·페이지·버전)·원문 대조 검증 규칙을 따른다. 기억 의존 추정(환각) 금지. (도메인: docs/claude_guideline/external_reference/domains/)

<!-- kuks_agent_setup:code_review -->
- "코드 리뷰"/"코드 분석" 트리거 감지 시 **응답 전 의무 선행 점검**(등록만 알고 건너뛰지 말 것): 먼저 docs/claude_guideline/code_review/review.md 를 Read 한 뒤 9단계 SOP(인벤토리[목적·함수표·전역표·의존성] + severity 평가 + 산출물 docs/code_review/<주제>/YYYY-MM-DD.md 기록)를 따른다. 일반 탐색+요약으로 대체 금지. (도메인: docs/claude_guideline/code_review/domains/)

<!-- kuks_agent_setup:sw_structure -->
- "SW 구조"/"구조 분석"/"클래스 관계"/"호출 관계" 트리거 감지 시 **응답 전 의무 선행 점검**(등록만 알고 건너뛰지 말 것): 먼저 docs/claude_guideline/sw_structure/structure.md 를 Read 한 뒤 파일 의존 그래프 + 클래스 다이어그램 + 시퀀스 다이어그램 + 연결 관계표 + 구조 관찰(산출물 docs/sw_structure/<주제>/YYYY-MM-DD.md)을 작성한다. 결함 평가는 code_review 소관.

<!-- kuks_agent_setup:coding -->
## 코드 작성 SOP (coding)

코드 작성/구현/수정 트리거 감지 시 **응답 전 의무 선행 점검**(등록만 알고 건너뛰지 말 것) — 바로 구현 직행 말고 먼저 [docs/claude_guideline/coding/coding.md](docs/claude_guideline/coding/coding.md) 를 Read 한 뒤 절차를 따른다 — 입구 작업분류(trivial fast-path) → 사전조사(함수표·전역변수표 read) → 사전승인(ADR) → 구현 → 검증(테스트·보안, never-self-approve) → 후속갱신(이중 기록). 강제는 `⟦CI:<id>⟧` ↔ `checks/<id>.sh`(pre-commit·CI)만 진짜, 그 외는 `⟦권고⟧`. 명명·스타일은 `conventions.md`, 언어/포맷터는 `stack.md`, 도메인(ros2/embedded/numeric/concurrency/memory)은 트리거 시 `docs/claude_guideline/coding/domains/` 적용.

<!-- kuks_agent_setup:debt -->
## 부채 관리 (debt)

기술·이해·의도 부채/TODO/FIXME 트리거 감지 시 **응답 전 의무 선행 점검**(등록만 알고 건너뛰지 말 것) — 먼저 [docs/claude_guideline/debt/debt.md](docs/claude_guideline/debt/debt.md) 를 Read 한 뒤 절차로 **등록·추적·상환**한다 — 식별된 부채는 `docs/debt/registry.md` 에 등록(id·유형·위치·사유·상태·상환계획), 코드의 `TODO`/`FIXME`/`HACK` 은 debt id 를 참조(`# TODO(debt-042): ...`, 맨 마커는 `⟦CI:debt-marker⟧` 차단). 식별은 작업 SOP(coding §2/§4/§5/§6)가, 등록·추적은 debt 가 소유. 미설치 시 식별만 주석/ADR 에 남김(graceful).

<!-- kuks_agent_setup:issue_fix -->
- 버그 수정 / 이슈 해결 / 빌드 실패 / 에러 진단 트리거 감지 시 **응답 전 의무 선행 점검**(등록만 알고 건너뛰지 말 것): 먼저 docs/claude_guideline/issue_fix/issue_fix.md 를 Read 한 뒤 진단→제안(승인)→구현→검증→기록(docs/issues_and_fixes/issues_and_fixes.md) 사이클을 따른다. 즉답 패치 직행 금지.

<!-- kuks_agent_setup:git_workflow -->
- git 작업(commit/push/merge/PR/branch) 트리거 감지 시 **응답 전 의무 선행 점검**(등록만 알고 건너뛰지 말 것): 먼저 docs/claude_guideline/git_workflow/git_workflow.md 를 Read 한 뒤 solo/team 모드 판정·커밋 규약·다중 원격 push·PR 리뷰 게이트를 따른다. 임의 커밋/푸시 직행 금지.
