# 요청 기록

사용자가 본 프로젝트에 대해 내린 작업 요청을 누적 기록한다. 의도·범위·승인 이력을 추적하기 위한 단일 근원.

본 문서의 메타 규칙은 [../claude_guideline/documentation.md](../claude_guideline/documentation.md) 를 따른다.

## 파일 구조

- 누적 파일 1개: `requests.md` — 시간순 (최신 위)
- 요청이 길거나 별도 ADR 이 필요한 경우: `YYYY-MM-DD_<짧은제목>.md` 추가 후 `requests.md` 에서 링크

## 항목 형식

```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 요청
사용자 요청 원문 또는 요약.

### 작업 범위
어떤 파일·영역을 어떻게 변경하기로 합의했는지.

### 승인 / 사전 조건
[../claude_guideline/coding.md](../claude_guideline/coding.md) "사전 승인 트리거" 해당 시 승인 시점·내용.

### 결과
완료 커밋 / PR / 후속 작업 링크.
```

## 사용 규칙

- 사용자가 "기록" 지시를 내리면 본 폴더의 `requests.md` 를 갱신한다 ([../claude_guideline/github.md](../claude_guideline/github.md) "기록 명령 처리").
- 작업 범위 외 변경이 발견되면 그 시점에 요청을 본 파일에 추가하고 별도 commit 으로 분리한다 ([../claude_guideline/github.md](../claude_guideline/github.md) "작업 단위 = 커밋 단위 = push 단위").
