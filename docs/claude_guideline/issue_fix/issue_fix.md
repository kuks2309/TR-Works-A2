# 이슈 수정 워크플로 (Issue Fix Workflow)

> **본 파일은 지시용.** 버그 수정·에러 진단·빌드 실패 해결·수정 기록의 self-contained 단일 근원(SSOT / Single Source of Truth). 진단 → 제안 → (승인) → 구현 → 검증 → **기록** 의 한 사이클을 규정한다.

본 코어는 self-contained 다 — 본문 외 가이드라인·도구·Skill 의존 0. `user_instruction`·`worklog` 번들이 함께 설치된 경우에만 교차참조하며, 없으면 생략한다(교차참조는 선택).

## 설치

본 번들 폴더(`issue_fix/`)의 `install.sh` 로 설치한다:

```bash
cd issue_fix && ./install.sh <타깃-프로젝트-루트>
```

스크립트가 (1) `issue_fix.md` 를 `docs/claude_guideline/issue_fix/` 로 복사하고, (2) 등록 스니펫(`claude.snippet.md`)을 타깃 `CLAUDE.md` 에 append 한다(덮어쓰기 아님, 중복 시 스킵). 설치 산출물은 규칙(`issue_fix.md`)뿐 — install.sh·claude.snippet.md 는 복사하지 않는다. 이슈 로그(`docs/issues_and_fixes/issues_and_fixes.md`)는 설치가 아니라 **첫 기록 시 런타임에 생성**한다(승인 불요). 수동 설치 시 같은 두 단계를 직접 수행한다.

**활성화 게이트**: 본 파일이 `docs/claude_guideline/issue_fix/issue_fix.md` 경로에 없으면 본 룰은 비활성.

## 트리거

사용자 메시지에 다음 신호 등장 시 자동 활성:

- "버그 수정", "이슈 해결", "에러 고쳐", "빌드 실패", "fix issue"
- 에러 로그·traceback·스택 트레이스 붙여넣기
- 구현 완료 직후 수정 사항 기록 요청

## 흐름도 (한눈에)

```
[이슈 / 에러 / 빌드 실패 도착]
   ↓
[Step 1] 과거 이슈 검색            ──→  ✓ issues_and_fixes.md grep, 유사 사례 가설화
   ↓
[Step 2] 진단 (증거 수집)          ──→  ✓ file:line 근본 원인 가설 1개 이상
   ↓
[Step 3] 수정안 제시 (승인 대기)    ──→  ✓ 증상·원인·해결·테스트 4항목 제시
   ↓
[Step 4] 구현 (승인 후)            ──→  ✓ 대상 파일 선독, 최소 변경
   ↓
[Step 5] 검증                     ──→  ✓ 프로젝트 검증 명령 0 errors
   ↓
[Step 6] 기록 (의무)               ──→  ✓ issues_and_fixes.md 최상단 prepend
   ↓
[Step 7] 자체 점검 grep            ──→  ✓ 경로 정규형 + entry 형식 통과
   ↓
[완료]
```

## Step 별 상세

### Step 1. 과거 이슈 검색

이슈 로그를 먼저 읽는다(상위 200줄):

- `docs/issues_and_fixes/issues_and_fixes.md` (메인 로그)
- `docs/issues_and_fixes/` 디렉토리 (개별 이슈 파일, 있을 때)

현재 에러 키워드로 grep. 유사 과거 이슈가 있으면 그 근본 원인을 **시작 가설**로 인용한다(같은 실수 반복 방지).

### Step 2. 진단 (제안 전 증거 수집)

추측 전에 증거부터 모은다:

- 에러를 끝까지 읽는다. 실패 파일·라인 식별.
- 에러 문자열을 소스 전역 grep.
- (git 프로젝트면) `git log --oneline -10`, `git diff HEAD~3` 로 최근 회귀 확인.
- `file:line` 증거와 함께 근본 원인 가설을 세운다.

다중 버그는 심각도로 분류한다:

| 심각도 | 기준 |
| --- | --- |
| CRITICAL | 시스템 중단·데드락·데이터 손실 직접 원인 |
| HIGH | 통신/스레드 안전성·핵심 기능 정확성 위협 |
| MEDIUM | API(Application Programming Interface) 신뢰성·데이터 무결성 |
| LOW | 품질·유지보수성 |

### Step 3. 수정안 제시 (승인 대기)

구현 전에 제시한다:

- **증상**: 관측된 현상
- **원인**: 발생 이유 — `file:line` 증거
- **해결**: 최소 변경 (줄 수 명시)
- **테스트**: 통과할/ xfail→pass 전환할 테스트

**코드를 쓰기 전 사용자 승인을 기다린다.** (단일 파일·최소 수정도 동일.)

### Step 4. 구현 (승인 후)

승인된 수정만 적용한다. 대상 파일을 먼저 읽고, 필요한 최소 범위만 바꾼다.

### Step 5. 검증

프로젝트의 검증 명령을 실행해 **0 errors / 기대 테스트 전부 통과**를 기록 전에 확인한다. 검증 명령은 프로젝트가 정의한 것을 따른다(빌드·테스트 러너·린트).

증거 없는 "고쳤다" 선언 금지 — 검증 출력으로 입증한다.

### Step 6. 기록 (의무)

수정이 검증되면 **즉시** 이슈 로그 최상단에 prepend 한다(작업 완료 후 일괄 기록 금지). 구분선 바로 아래에 최신 항목이 오도록 한다.

**기록 위치(정규형 — 변종 금지)**: `docs/issues_and_fixes/issues_and_fixes.md`. 폴더·파일이 없으면 만든다(승인 불요).

**entry 형식**:

```markdown
## YYYY-MM-DD

### [Fix] <제목>

- **문제**: 증상
- **원인**: 근본 원인 — `file:line`
- **해결**: 변경 내용 (N줄 수정/삭제/추가)
- **파일**: 수정 파일 목록
- **상태**: 완료
```

git 프로젝트면 `git add <명시 경로>` 로 stage — `git add .` / `git add -A` 금지.

**결과·코드 변경 누적(선택)**: 결과·산출물·코드 변경 이력은 본 파일 책임이 아니다. `worklog` 번들 설치 시 `docs/worklog/` 가 담당하며, 미설치면 생략한다(하드 의존 아님).

### Step 7. 자체 점검

아래 grep 통과(→ [자체 점검](#자체-점검)).

## SSOT 정규 식별자 — 변종 금지

본 번들·기록·교차참조는 아래 정규형(canonical)만 사용한다. 같은 대상을 다른 식별자로 표기하면 단일 근원이 깨진다(SSOT 위반). 변종 표기를 발견하면 정규형으로 통일한다.

| 대상 | 정규형(canonical) | 금지 변종 |
| --- | --- | --- |
| 이슈·수정 로그 폴더 | `docs/issues_and_fixes/` | `issues_fixes/`, `issues-fixes/`, `issue_and_fix/` |
| 이슈·수정 로그 파일 | `docs/issues_and_fixes/issues_and_fixes.md` | 상동 폴더 변종 |
| 사용자 지시 기록 파일(교차참조) | `docs/user_instructions/user_instructions.md` | `requirements.md` |

> 근거: 위 두 정규화는 과거 SSOT 정합성 사고(폴더 경로 변종·파일명 변종)에서 도출된 교훈이다. 본 번들은 정규형을 못박고 [자체 점검](#자체-점검) 으로 변종 재유입을 차단한다.

## 룰

1. **검색 선행** — 진단 전 과거 이슈 로그 grep, 유사 사례 가설화
2. **진단 → 제안 → 승인 → 구현** 순서 — 승인 없는 코드 작성 금지(최소 수정 포함)
3. **증거 의무** — 근본 원인은 `file:line` 인용, 추측 금지
4. **검증 후 기록** — 0 errors 확인 전 기록·완료 선언 금지
5. **기록 의무** — 검증 즉시 `issues_and_fixes.md` 최상단 prepend, 5필드(문제·원인·해결·파일·상태) 누락 0
6. **정규 식별자 의무** — 경로·파일명은 정규형만(→ [SSOT 정규 식별자](#ssot-정규-식별자--변종-금지))
7. **명시 staging** — git 프로젝트는 `git add <경로>`, `.`/`-A` 금지

## 자체 점검

자체 점검은 규칙 문서가 아니라 **실제 프로젝트 경로·산출물**을 대상으로 한다(규칙 본문은 변종을 "금지 대상"으로 정당하게 언급하므로 점검 대상에서 제외).

```bash
LOG=docs/issues_and_fixes/issues_and_fixes.md

# 1. 폴더명 정규형 — 변종 디렉토리 부재 (실제 경로 점검)
for v in issues_fixes issues-fixes issue_and_fix; do
  [ -d "docs/$v" ] && echo "✗ 변종 폴더 존재: docs/$v → docs/issues_and_fixes/ 로 통일"
done
[ -d docs/issues_and_fixes ] && echo "✓ 정규 로그 폴더" || echo "(로그 폴더 없음 — 첫 기록 시 생성)"

# 2. 교차참조 파일명 정규형 — 지시 기록 변종(requirements.md) 부재
[ -f docs/user_instructions/requirements.md ] && echo "✗ 변종 파일: docs/user_instructions/requirements.md → user_instructions.md 로 통일" || echo "✓ 파일명 정규형"

# 3. 이슈 로그 존재 + 최신 entry 헤더 형식
[ -f "$LOG" ] && grep -E "^### \[Fix\] " "$LOG" | head -1 || echo "(이슈 로그 없음 — 첫 기록 시 생성)"

# 4. 최신 entry 필수 5필드
[ -f "$LOG" ] && grep -E "^- \*\*(문제|원인|해결|파일|상태)\*\*:" "$LOG" | head -5
```

## 변경 절차

본 룰은 SSOT. 변경 시 사용자 승인 필수. 변경 후 VERSION(semver)·다운스트림 통보, 자체 점검 통과 확인.

---

**VERSION**: 1.0.0 (v1 issue-fix skill 이식 — frontmatter·도메인 하드코딩·외부도구 의존 제거, SSOT 정규 식별자 가드 내장)
