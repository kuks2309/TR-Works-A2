# 이슈 / 수정 기록

본 프로젝트에서 발생한 이슈, 그 원인, 수정 방법을 누적 기록한다. 동일 이슈 재발 시 참조 가능한 단일 근원.

본 문서의 메타 규칙은 [../claude_guideline/documentation.md](../claude_guideline/documentation.md) 를 따른다.

## 파일 구조

- 누적 파일 1개: `issues_and_fixes.md` — 시간순 (최신 위)
- 사건이 큰 경우: 별도 파일 `YYYY-MM-DD_<짧은제목>.md` 추가 후, 본 README 또는 `issues_and_fixes.md` 에서 링크

## 항목 형식

```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 증상
어떤 동작이 어떻게 잘못되었는지 (재현 조건 포함).

### 원인
근본 원인 분석. 가설이면 "가설" 명시.

### 수정
어떤 변경으로 해결했는지 (커밋 / PR 링크).

### 재발 방지
같은 이슈를 막기 위한 규칙·테스트·체크리스트 갱신.
```

## 사용 규칙

- 사용자가 "기록" 지시를 내리면 본 폴더의 `issues_and_fixes.md` 를 갱신한다 ([../claude_guideline/github.md](../claude_guideline/github.md) "기록 명령 처리").
- 우회 / workaround 사용 시 [../claude_guideline/tech_debt.md](../claude_guideline/tech_debt.md) "우회 사용 3 조건" 의 "사유 + 정리 일정" 을 본 폴더에 남길 수 있다.
- 30 일 이상 미해결 ADR Open Question 도 본 폴더에 회수 항목으로 등록 가능.

## 기존 이슈 검토 시점

작업 시작 전, 동일 영역 / 모듈에서 기존 이슈가 있었는지 본 폴더를 빠르게 훑는다.
