# 이슈 / 수정 기록

China 모노레포 (openarmx_ws / cyclo_control / calibration / 3d_detect_ws 등) 에서 발생한 이슈, 그 원인, 수정 방법을 누적 기록한다. 동일 이슈 재발 시 참조 가능한 단일 근원(SSOT).

진단 → 제안 → 구현 → 검증 → 기록 전체 사이클은 `issue-fix` 스킬 (`.claude/skills/issue-fix/SKILL.md`) 을 따른다.

## 파일 구조

- 누적 파일 1개: `issues_and_fixes.md` — 시간순 (최신 위, prepend)
- 사건이 큰 경우: 별도 파일 `YYYY-MM-DD_<짧은제목>.md` 추가 후, 본 README 또는 `issues_and_fixes.md` 에서 링크
- 패키지/모듈 단위로 묶고 싶으면 `<package>.md` 파일을 두고 본 README 에서 링크 (예: 참조 프로젝트 `3d_detect_ws/.../docs/issues_and_fixes/box_detection.md`)

## 항목 형식

```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 증상
어떤 동작이 어떻게 잘못되었는지 (재현 조건 포함).

### 원인
근본 원인 분석. 가설이면 "가설" 명시. 가능하면 `file:line` 근거.

### 수정
어떤 변경으로 해결했는지 (커밋 / PR 링크, 수정 줄 수).

### 재발 방지
같은 이슈를 막기 위한 규칙·테스트·체크리스트 갱신.
```

## 사용 규칙

- 버그 수정 / 빌드 실패 해결이 완료되면 `issues_and_fixes.md` 에 항목을 prepend 한다 (최신 위).
- 우회 / workaround 사용 시 "사유 + 정리 일정" 을 함께 남긴다 (정공법 우선, 우회는 한시적).
- 30 일 이상 미해결 항목은 회수 대상으로 본 폴더에 등록 가능.
- staging 은 `git add <특정 파일>` — `git add .` / `git add -A` 금지.

## 기존 이슈 검토 시점

작업 시작 전, 동일 영역 / 모듈에서 기존 이슈가 있었는지 본 폴더를 빠르게 훑는다.

## 별도 기록 파일

- [2026-06-01 YOLOv8 연속 추론 → on-demand DetectBox action server 전환](2026-06-01_yolov8_on_demand_action_server.md)
