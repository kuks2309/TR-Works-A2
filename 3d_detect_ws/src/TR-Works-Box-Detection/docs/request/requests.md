# 요청 기록

본 파일의 메타 규칙은 [README.md](README.md) 와 [../claude_guideline/documentation.md](../claude_guideline/documentation.md) 를 따른다.

## 2026-05-07 13:30 (KST) — kuks_claude_setup 브랜치 통합 → TR-Works 적용

### 요청

1. https://github.com/kuks2309/kuks_claude_setup.git 의 모든 브랜치를 분석하고 문서 통합 방법 제안.
2. 통합본을 kuks_claude_setup master 에 push.
3. 본 프로젝트(TR-Works) 에 통합본(v1.3.0) 적용 후 commit / push.
4. `docs/issues_and_fixes/`, `docs/request/`, `docs/claude-mistake/` 도메인 폴더 추가.
5. CLAUDE.md "프로젝트 성격" 을 "Box Pose Detection 기능" 으로 명시.
6. 기록.

### 작업 범위

- kuks_claude_setup: `feat/v1.3.0-fito-contributions` 브랜치(`bd36fc2`) 를 master 에 cherry-pick 통합 후 feat 브랜치 삭제.
- TR-Works (https://github.com/kuks2309/TR-Works-Box-Detection.git):
  - `CLAUDE.md`, `.gitignore`, `docs/claude_guideline/` (v1.3.0), `docs/claude-mistake/`, `docs/issues_and_fixes/`, `docs/request/` 추가
  - git init + 초기 커밋 + remote 설정 + push

### 승인 / 사전 조건

- 통합 방식은 사용자가 옵션 4지선다 중 "푸쉬 보류 — 로컬에서 먼저 확인" 선택 후, 후속 메시지로 직접 push 지시.
- TR-Works remote URL 은 사용자가 후속 메시지에서 명시 (https://github.com/kuks2309/TR-Works-Box-Detection.git).
- CLAUDE.md placeholder 중 `{{PROJECT_DESCRIPTION}}` 는 사용자 명시 ("Box Pose Detection 기능") 로 채움. `{{DOMAIN_REFERENCES}}` 는 placeholder 유지.

### 결과

- kuks_claude_setup master: `7bfe57a feat(claude_guideline): 1.3.0 — ROS2/임베디드/모듈 override 계층 보강` (push 됨, feat 브랜치 삭제됨)
- TR-Works master:
  - `01cd185 docs(claude_guideline): 워크스페이스 메타 규칙 v1.3.0 적용`
  - `ae93614 docs(CLAUDE.md): 프로젝트 성격 — Box Pose Detection 기능`
- 본 기록 commit (요청 6) 으로 마무리.
