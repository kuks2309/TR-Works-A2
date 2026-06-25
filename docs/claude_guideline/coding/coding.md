# 코드 작성 SOP (Coding SOP)

> **본 파일은 지시용.** 코드 작성 절차의 self-contained 단일 근원(SSOT / Single Source of Truth).
> **강제 로직은 본문에 없다** — 이빨(machine 강제)은 `coding/checks/*.sh`(CI·pre-commit)에 있고, 본문은 **판단기준·태그·선언**만 담는다.

본 코어는 self-contained 다 — 본문 외 가이드라인·도구·Skill·OMC 상태경로 의존 0. 어느 프로젝트든 `git clone` 만으로 동일 동작한다.

## 설치 / 활성화 게이트

본 번들 폴더(`coding/`)의 `install.sh` 로 설치한다:

```bash
cd coding && ./install.sh <타깃-프로젝트-루트> [도메인...|--all]
```

스크립트가 코어(`coding.md`·`conventions.md`·`stack.md`) + 선택 도메인(`domains/`)을 `docs/claude_guideline/coding/` 로 복사하고, 이빨(`checks/*.sh`·`ci/`·`.pre-commit-config.yaml`)을 설치하며, `.omc/` 를 `.gitignore` 에 추가(OMC creep 차단)하고 등록 스니펫을 타깃 `CLAUDE.md` 에 append 한다. **활성화 게이트**: 본 파일이 그 경로에 없으면 본 룰 비활성.

## 0. 강제 모델 (먼저 읽기 — 이 번들의 정직 선언)

진짜 인터록은 **코드에서 재도출(re-derive)되는 것뿐**이다. 모든 규칙에 강제 태그를 단다:

- **`⟦CI:<id>⟧`** = `checks/<id>.sh` 가 커밋된 코드로부터 결정론적으로 재도출·차단(pre-commit·CI). **에이전트가 못 속인다.** (현재 `<id>` ∈ {index-fresh, dup-signature, tests-ran, banned-pattern, adr-fields})
- **`⟦권고⟧`** = 코드 재도출 불가. 에이전트 자기보고에 의존하므로 **정직하게 advisory**. (태그 없는 체크박스는 전부 `⟦권고⟧`.)

핵심 명제 세 줄:

- **green ≠ good, 미탐지 ≠ 무결.** `✅` 는 "검사가 통과"이지 "옳다"가 아니다.
- **자기✅ 금지.** 본 번들에 자기승인 서명란이 **없다**(§5 헌법). 최종 verdict 는 저자가 못 찍는다.
- **무-CI 환경**: `⟦CI⟧` 도 pre-commit advisory 로 강등(`--no-verify` 우회가능). 이 환경의 기계 강제력 = 0, 규칙 텍스트만 생존 — README 가 이를 큰소리로 선언한다.

## 1. 입구 — 작업 분류 (trivial fast-path)

분류는 **diff 메트릭에서 결정**한다(자기서술 아님):

- **trivial** — 코드 0줄 변경(문서·주석·포맷만) **또는** 공개표면(공개 함수 시그니처·API·스키마) 미접촉 → **fast-path**: §4 구현 → §5 검증만. §2·§3 면제.
- 그 외 → **Full**: §2 → §3 → §4 → §5 → §6 전체.

## 2. 사전조사 (read) — §6 write 와 대칭 폐루프

**코딩 계획 전에, 함수표·전역변수표를 먼저 갖춰 읽는다.** §6 이 갱신하는 그 표를 여기서 읽으므로 둘은 같은 산출물의 양방향 폐루프다 — 내가 §6 에서 갱신해야 다음 작업이 §2 에서 최신 표를 읽는다.

**표가 없으면 먼저 만든다** (없으면 읽을 게 없다):

- **신규 파일(처음 작성)** → coding 의 **계획 단계**에서 표를 생성한다(설계할 함수·전역변수를 표로).
- **기존 파일 참조(표 부재)** → `code_review` 번들 인벤토리로 **코딩 전에** 작성한다(위임).
- 표 양식(함수표·전역변수표 컬럼)의 권위는 **`code_review` 단일 SSOT** — coding 은 재정의하지 않고 그 양식을 따른다. (`code_review` 미설치 시에만 간이 표로 대체.)

falsifiable 체크박스(빈 약속 금지 — 무엇을 읽었는지 명시):

- [ ] **계획 전**, 함수표·전역변수표(모듈 로컬 원본 + 루트 집계) + flowchart·ADR(Architecture Decision Record, 설계 결정 기록)를 읽었다(없으면 위 규칙으로 먼저 생성) — *읽은 파일 목록 첨부* `⟦권고⟧`
- [ ] 그 표로 중복 후보 **함수**를 확인했다 — 사후조건: 커밋 시 충돌이 재도출됨 `⟦CI:dup-signature⟧`
- [ ] 그 표로 중복 **변수**·불필요한 전역변수를 확인했다 (평가 권위 → `code_review` 의 `[품질]`) `⟦권고⟧`
- [ ] 외부 매뉴얼·datasheet 인용이 필요하면 `external_reference` 규칙을 따른다(인용 권위는 그 번들 단일 SSOT) `⟦권고⟧` → `docs/claude_guideline/external_reference/`

## 3. 사전승인 트리거 (Full 일 때 — advisory 체크리스트)

kill-test("이 트리거가 없으면 무슨 사고가 나는가" 답 가능) 통과한 **보편 핵심만**. 도메인 트리거는 `domains/` 로 위임하고, **0건 발화는 정상**(domains/ 전체 건너뜀). 충족 시 ADR 기록:

- [ ] 공개 API(Application Programming Interface) 신설·변경 (**언어 경계·결합 포함** → `stack.md` §4) `⟦권고⟧`
- [ ] 되돌림 비가역 변경(영속 상태·스키마·펌웨어) → ADR 에 **Rollback Plan 필드 필수** `⟦CI:adr-fields⟧`
- [ ] 신뢰경계 횡단 입력·비밀정보·외부 명령/직렬화 `⟦권고⟧`
- [ ] 의존성(패키지) 추가 → ADR 에 License·취약점·대안 3필드 `⟦권고⟧`

## 4. 구현

- `conventions.md`(명명·스타일·전역변수 규율) · `stack.md`(언어/프레임워크/UI·포맷터) · 활성 `domains/` 를 따른다.
- **코드 포맷**: 프로젝트 포맷터 설정(`.clang-format`=Microsoft 등)대로 — *선택*은 `stack.md`, *준수*는 기계 검사 `⟦CI:format⟧`
- **금지 패턴**: 하드코딩 secret·`eval`/`exec`·raw SQL(Structured Query Language) 결합·async 내 blocking I/O(Input/Output) `⟦CI:banned-pattern⟧`
- **함수 단위 검증 (작업 크기로 분기)**: 신규 **공개 함수**는 짜자마자 단위 테스트로 검증 후 통합(TDD-lite). 내부 helper·trivial 은 §5 사후 일괄.

## 5. 검증 (verify) — never-self-approve 헌법

- [ ] **전체 회귀** — 모든 테스트 PASS (공개함수 단위 검증은 §4 에서 선행). 변경 공개함수마다 테스트 ≥ 1, 빌드·PASS 카운트 로그 `⟦CI:tests-ran⟧`
- [ ] 보안 자가점검: secret 0 · 입력검증 · 최소권한 · 위험 sink 부재 `⟦CI:banned-pattern⟧` + `⟦권고⟧`
- [ ] 관측성·성능·자원: 실패경로 로그 · 핫패스 O(n²) 미도입 · 자원 누수 없음 `⟦권고⟧`
- **실패 분기(❌)**: 즉시 수정, 못 고치면 **기술 부채로 `debt` 등록 + 사유**(선조치-후정산) → `docs/claude_guideline/debt/`. `⟦권고⟧`
- **판단검증**(의미적 중복·설계 적합·깊은 보안 추론)은 토큰이 비싸므로 **Full-scope·고위험만** 외부 패스로.
- ★ **헌법**: 본 번들에 자기승인 서명란이 **없다.** 최종 `✅` verdict 는 **저자가 못 찍는다** — 사람 PR(Pull Request) 리뷰 또는 `code_review` 자매 번들이 렌더한다(절차가 아니라 *능력 부재*).

## 6. 후속 갱신 (write) — §2 read 와 대칭

- **상태-미러형**(함수표·변수표·flowchart·인덱스): 덮어쓰기. **이중 기록** = 모듈 로컬(권위) + 루트 집계. 인덱스 stale 시 차단 `⟦CI:index-fresh⟧`
- **로그-누적형**(ADR·수정이력): append / supersede(덮어쓰기 금지, 기존은 `Status: Superseded`)
- 미해결 **이해·의도 부채**는 `debt` 번들에 등록(위임 — coding 은 '식별'만; **`debt` 미설치 시 식별만 주석/ADR 에 남김, 무해**) `⟦권고⟧`

## 룰 (요약)

1. trivial 은 fast-path, 사전조사·트리거 면제
2. `⟦CI⟧` 만 진짜 강제, `⟦권고⟧` 는 정직한 advisory (green ≠ good)
3. 비가역 변경 ADR 에 Rollback Plan 필드
4. 금지 패턴 0 (secret·eval·raw SQL·async blocking)
5. 공개함수 단위 검증(§4) + 전체 회귀(§5) — 변경 공개함수마다 테스트 ≥ 1
6. 함수표·flowchart 이중 기록, 인덱스 stale 차단
7. **자기승인 서명란 없음 — 최종 verdict 는 외부가 렌더(never-self-approve)**

> **MUST 예산**: 위 '룰 요약' 7개가 코어의 필수(MUST) 규칙 전체다 — 7개 이내로 유지한다. 전부 필수면 노이즈가 되어 등급이 무의미해진다.

## 자체 점검

```bash
# 활성화 게이트
test -f docs/claude_guideline/coding/coding.md || echo "(coding 룰 비활성)"

# 강제 태그 ↔ 백킹 스크립트 정합 (메타 불변식: 번들이 자기 강제력에 대해 거짓말 못 함)
bash docs/claude_guideline/coding/checks/check-mapping.sh

# ⟦CI⟧ 태그가 실제 스크립트를 가리키는지 빠른 확인
grep -oE '⟦CI:[a-z-]+⟧' docs/claude_guideline/coding/coding.md | sort -u

# MUST 예산 (룰 요약 항목 ≤ 7)
test "$(grep -cE '^[0-9]+\. ' docs/claude_guideline/coding/coding.md)" -le 7 || echo "MUST 예산 초과"
```

## 변경 절차

- SSOT 는 본 번들 폴더. 규칙 변경은 사용자 승인 후 `coding.md`·`conventions.md`·`stack.md`·`domains/` + `checks/*.sh` 를 **단일 번들 VERSION 으로 동반 갱신**(부분 버전 드리프트 금지).
- `⟦CI:<id>⟧` 태그를 추가/변경하면 반드시 `checks/<id>.sh` 와 `check-mapping.sh` 를 함께 갱신한다.
- semver + CHANGELOG. 자매 번들과 동일하게 파일 말미 `VERSION` 으로 표기.

---

**VERSION**: 1.0.0 (CI 재도출 척추 + 작성 규율 advisory + never-self-approve 헌법 + trivial fast-path; 강제 태그 ⟦CI⟧/⟦권고⟧ 2분류; 강제 로직은 checks/*.sh 위임; self-contained·OMC-free)
