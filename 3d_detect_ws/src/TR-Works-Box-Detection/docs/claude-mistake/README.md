# Claude 실수 기록

Claude(LLM 어시스턴트) 가 본 프로젝트 작업 중 일으킨 실수와 그 사유, 재발 방지책을 누적 기록한다. 동일한 실수의 반복을 막고, 후속 세션이 학습 자료로 활용한다.

## 파일 구조

- 한 사건 = 한 항목, 시간순 누적 (최신 위)
- 권장: 날짜별 파일 `YYYY-MM-DD.md` (같은 날 여러 건은 한 파일에 모음)
- 사건이 큰 경우: 별도 파일 `YYYY-MM-DD_<짧은제목>.md`

## 항목 형식

```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 무엇을 했는가
Claude 가 시도한 행동 / 작성한 내용을 객관적으로 기술.

### 무엇이 잘못이었나
어떤 결과가 발생했는지, 어떤 규칙·의도와 어긋났는지.

### 사용자 지적
사용자가 어떻게 교정했는지 (인용 또는 요약).

### 사유 / 가설
Claude 가 왜 그 실수를 했는지 (정보 부족, 잘못된 가정, 컨텍스트 누락, 매뉴얼 오독 등).

### 재발 방지
같은 실수를 막기 위해 무엇을 바꿔야 하는지. 가능하면 가이드라인·메모리·체크리스트의 어느 항목에 반영했는지 링크.
```

## 사용 규칙

- 사용자가 직접 지적한 실수는 그 즉시 본 폴더에 기록한다.
- 사용자가 지적하지 않았더라도 Claude 가 스스로 인지한 실수도 기록할 수 있다.
- 기록 후, 가이드라인 / 메모리 / 체크리스트에 재발 방지 규칙을 반영한다 (단순 기록만으로는 학습이 닫히지 않음).
- 민감한 컨텍스트 (개인 정보, 운영 비밀) 가 포함되는 실수는 기록 위치를 `docs/claude_guideline/local/` (gitignore) 로 옮긴다.

## 기존 실수 검토 시점

작업 시작 전, 동일 영역 / 모듈에서 기존 실수가 있었는지 본 폴더를 빠르게 훑는다 (특히 데이터시트 해석, 외부 의존성, 작업 범위 판단 항목).

## 설치 방법

본 컨벤션은 단일 README 만으로 운용된다. 프로젝트 루트에서 다음을 실행하면 `docs/claude-mistake/` 가 생성되고 본 README 가 배치된다.

```bash
mkdir -p docs/claude-mistake
curl -fsSL https://raw.githubusercontent.com/kuks2309/kuks_claude_setup/master/claude-mistake/README.md \
  -o docs/claude-mistake/README.md
```

설치 후, 프로젝트 루트의 `CLAUDE.md` "도메인 문서 SSOT" 표에 다음 한 줄을 추가한다.

```markdown
| Claude 실수 기록 (재발 방지) | [docs/claude-mistake/README.md](docs/claude-mistake/README.md) |
```
