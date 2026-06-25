# 코드 리뷰 작업 지침 — 코어 (Code Review SOP — Core)

> **본 파일은 지시용.** 사용자가 "코드 리뷰"/"코드 분석"을 요청했을 때 **분석 방법론**과 **기록 형식**의 self-contained 코어 단일 근원(SSOT / Single Source of Truth). 도메인 특화 리뷰 규칙은 선택 sub-file 로 분리한다 (→ [도메인 Add-on](#도메인-add-on-선택)).

본 코어는 self-contained 다 — 본문 외 가이드라인·도구·Skill 의존 0. `user_instruction` 번들이 함께 설치된 경우에만 리뷰 시각을 지시 기록과 매핑하며, 없으면 생략한다(매핑은 선택).

## 설치

본 번들 폴더(`code_review/`)의 `install.sh` 로 설치한다:

```bash
cd code_review && ./install.sh <타깃-프로젝트-루트> [도메인...]
```

스크립트가 코어(`review.md`)를 `docs/claude_guideline/code_review/` 로, 선택한 도메인(`domains/<도메인>.md`)을 `.../code_review/domains/` 로 복사하고, 등록 스니펫을 타깃 `CLAUDE.md` 에 append 한다 (덮어쓰기 아님).

- **리뷰 산출물**: `docs/code_review/<주제>/YYYY-MM-DD.md` (날짜=버전) — 폴더·파일 없으면 만든다(승인 불요). 주제 폴더에 타임라인 인덱스 `README.md` 동반.
- **활성화 게이트**: 본 파일이 `docs/claude_guideline/code_review/review.md` 경로에 없으면 본 SOP 는 비활성.

## 트리거

사용자 메시지에 다음 키워드 등장 시 자동 활성:

- "코드 리뷰", "리뷰해줘", "리뷰해 주세요"
- "코드 분석", "분석해줘"
- "이 함수 / 이 파일 / 이 모듈 봐줘"
- 특정 파일·디렉토리·PR(Pull Request) 경로를 첨부한 평가 요청

## 흐름도 (한눈에)

```
[코드 리뷰 요청 도착]
   ↓
[Step 1] 대상 범위 식별              ──→  ✓ 단일 파일 / 디렉토리 / 모듈 / PR 확정
   ↓
[Step 2] 단위 / 전체 분기 판정       ──→  ✓ 플로우차트 종류 결정
   ↓
[Step 3] 도메인 자동 감지            ──→  ✓ 설치된 domains/ 트리거 매칭
   ↓
[Step 4] Core 인벤토리 (5 항목)      ──→  ✓ 목적·플로우·함수·전역·의존성 누락 0
   ↓
[Step 5] 도메인 인벤토리 (감지 시)   ──→  ✓ 도메인별 추가 표 작성
   ↓
[Step 6] 평가 (severity 클러스터)    ──→  ✓ Critical/High/Medium/Low/Info + 카테고리 태그
   ↓
[Step 7] docs/code_review/<주제>/YYYY-MM-DD.md 기록 ──→  ✓ 코드 버전 고정 + 인덱스 갱신
   ↓
[Step 8] 자체 점검 grep              ──→  ✓ 헤더·#번호·카테고리 태그 통과
   ↓
[Step 9] 1~2 줄 결과 보고            ──→  ✓ 변경 파일 / 후속 TODO 명시
   ↓
[완료]
```

## 출력 구조 — Core + Domain Add-on

코드 리뷰 결과는 두 층으로 구성된다:

| 층 | 내용 | 적용 조건 |
| --- | --- | --- |
| **Core** (공통) | 목적·플로우차트·함수표·전역변수표·의존성 3-tier | 모든 리뷰 필수 |
| **Domain Add-on** | 도메인별 추가 인벤토리·평가 카테고리 | 트리거 감지 + 해당 sub-file 설치 시 (다중 가능) |

한 파일이 여러 도메인에 해당하면 설치된 add-on 동시 적용 (예: ROS2 임베디드 노드 → ros2-review + embedded-review + concurrency).

---

## Core 인벤토리 (5 항목 — 누락 0)

### 1. 목적

1~3 문단으로 해당 파일·모듈·디렉토리가 무엇을 위한 코드인지 기술. 추측 금지 — 코드·README·주석·데이터시트 인용(추정 금지·실측 검증).

### 2. 코드 플로우차트

| 요청 유형 | 플로우차트 형식 |
| --- | --- |
| 단위 코드 리뷰 (1 함수·1 파일) | **단일 플로우 차트** — 진입/분기/루프/종료, 주요 조건문, 에러 경로 |
| 전체 구조 분석 (디렉토리·다중 모듈) | **전체 코드 흐름도** — 모듈 간 호출 관계, 데이터 흐름, 진입점·종료점, 외부 인터페이스 |

**다중 진입점 분리 룰**: 한 패키지에 2개 이상 진입점(offline/live, CLI/라이브러리)이 있으면 path 별로 흐름도 분리 + 공통 호출 그래프 별도 표시.

### 3. 함수 리스트 표

컬럼 순서 고정: `#`, `함수`, `입력`, `출력`, `기능`, `위치(file:line)`.

**함수 표기 규칙**:

- 일반 함수: `function_name`
- 클래스 메서드: `ClassName.method`
- 이너 함수/클로저: `outer.inner` (한 행 추가, `#` 번호는 부모 행 + 알파벳, 예: `13a`)
- Launch/config 진입점(ROS2 `generate_launch_description` 등): 함수 표에 포함, 위치 컬럼에 `launch/foo.launch.py:N` 명시
- C/C++ `static` 함수, Python `_private` 함수 모두 포함

모든 함수 전수(private/static 포함). 누락 0. **중복/유사 함수**(같은 일을 하는 함수 2개 이상)는 식별해 `[품질]` 로 통합 권고.

### 4. 전역 변수 / 모듈 상수 표

컬럼 순서 고정: `#`, `사용처(함수)`, `기능`, `위치(file:line)`.

**포함 범위**: 진정한 전역 변수(mutable module-level state), 모듈 레벨 상수, 환경 의존 default 경로(절대경로 상수), C/C++ `static` 파일 스코프 변수, `extern` 노출 변수.

상수와 변수가 섞이면 `(상수)`/`(가변)` 표기 추가. 없으면 "전역 변수/모듈 상수 없음" 한 줄 명시(표 생략 가능).

각 전역의 **필요성**도 함께 평가 — 지역 변수·파라미터·의존성 주입으로 대체 가능하면 `[품질]` 로 지적. 불필요한 가변 전역 상태는 결합도·race 위험으로 격상.

### 5. 의존성 3-tier 표

| Tier | 정의 | 표기 |
| --- | --- | --- |
| **빌드** | `package.xml`, `CMakeLists`, `setup.py`, `requirements.txt` 등 선언된 빌드 필수 의존성 | 패키지명, 버전 제약 |
| **런타임 필수** | 실행에 반드시 필요한 외부 노드·서비스·HW 페리페럴·환경 변수 | 대상, **부재 시 동작 명시 의무** |
| **런타임 선택** | 있으면 사용, 없으면 fallback. **fallback 동작 명시 의무** | 대상, fallback 정의 |

표 컬럼: `Tier`, `대상`, `버전/제약`, `부재 시 동작(필수/선택)`, `근거(파일:line)`.

---

## Core 평가 (severity 클러스터 + 카테고리 인라인 태그)

평가는 **severity 클러스터(Critical/High/Medium/Low/Info)** 로 묶고, 각 항목에 **카테고리 인라인 태그**를 붙인다.

### Severity 분포 요약 + Verdict (상단 2줄 의무)

```
severity 분포: Critical 0 / High 1 / Medium 6 / Low 3 / Info 2
Verdict: REQUEST CHANGES
```

**Verdict 기준**: Critical/High ≥ 1 → `REQUEST CHANGES`, Medium 이하만 → `COMMENT`, 이슈 없음 → `APPROVE`. 본 SOP 로 생산한 리뷰는 작성자 본인이 `APPROVE` 할 수 없다 — 별도 lane(다른 세션·리뷰어)에서만 `APPROVE` 가능.

### 평가 카테고리 (Core 인라인 태그)

- `[논리]` — 논리 결함, off-by-one, race, null 미처리
- `[SOLID]` — SRP/OCP/LSP/ISP/DIP 위반
- `[스타일]` — 컨벤션, 네이밍
- `[성능]` — 알고리즘 복잡도, 핫패스, I/O 빈도
- `[테스트]` — 테스트 누락, 회귀 위험, 적절성
- `[품질]` — DRY(Don't Repeat Yourself) 위반(중복 로직·유사 함수)·불필요한 가변 전역 변수·중복/미사용/shadowing 변수·dead code·매직 넘버·리팩토링 권고
- (도메인 add-on 활성 시 추가 태그 — `[QoS]`, `[race]`, `[ISR]` 등)

### 항목 작성 형식

```markdown
**함수 #3 `process_packet` — [논리] null 체크 누락 (High)**
   재현: parser.c:142, packet.data == NULL 진입 시 segfault
   권고: 진입부 가드 추가
```

**Cross-reference 의무**: 모든 평가는 Core 인벤토리의 `#` 번호 인용(단일 또는 다중 — "함수 #1·#3·#7 군집").

**해당 없음 처리**: severity 클러스터는 빈 카테고리 자연 생략. **인벤토리 5 항목**은 빈 경우 "없음" 명시 의무(점검 누락과 구분).

---

## 도메인 Add-on (선택)

도메인 특화 인벤토리·평가 카테고리는 `domains/` 하위의 **도메인별 sub-file** 로 둔다. 도메인은 계속 추가되며, 트리거 감지 + 해당 sub-file 설치 시에만 적용한다.

| sub-file | 도메인 | 트리거 예 |
| --- | --- | --- |
| `domains/ros2-review.md` | ROS2 코드 리뷰 | `package.xml`, `rclpy`/`rclcpp`, `.launch.py`, `rcl_interfaces` |
| `domains/concurrency.md` | 동시성 (Threading/async) | `threading`, `asyncio`, `std::thread`, `std::mutex`, `async def`, callback group |
| `domains/embedded-review.md` | 임베디드 / RTOS | `__attribute__((interrupt))`, `ISR(`, `NVIC_`, FreeRTOS API, `volatile` 빈출 |

> ros2-review·embedded-review 는 **코드 리뷰 관점**이다. ROS2·임베디드 **참조 문서 인용** 규칙은 `external_reference` 번들의 동명(`-reference`) 도메인이 담당(별개·상보).

**새 도메인 5요건** (sub-file 작성 시 모두 충족):
1. 트리거(자동 감지 키워드·파일 패턴)
2. 인벤토리 추가 표(컬럼 순서 고정, 헤더는 grep 가능 식별자)
3. 평가 추가 카테고리(`[xxx]` 인라인 태그, Core 와 충돌 없음)
4. 자체 점검 grep(표 헤더 + 태그 검출 정규식)
5. 다른 도메인과의 의존/충돌 명시(동시 활성 룰)

---

## Step 별 상세

- **Step 1 대상 범위** — 단일 파일/디렉토리/모듈/PR 중 하나로 확정. 모호 시 STOP, 1줄 질문.
- **Step 2 분기 판정** — "전체/구조/아키텍처/모듈" 키워드 또는 디렉토리 지정 → 전체 흐름도, 그 외 → 단위 플로우. 모호 시 1줄 질문.
- **Step 3 도메인 감지** — 설치된 `domains/` 의 트리거 표 확인. 다중 감지 가능. 사용자 명시 지정 우선.
- **Step 4 Core 인벤토리** — 5 항목 작성, 누락 0.
- **Step 5 도메인 인벤토리** — 감지된 도메인의 추가 표 작성. 빈 표는 "해당 항목 없음" 명시.
- **Step 6 평가** — severity 클러스터 + 카테고리 태그. 모든 항목 인벤토리 `#` 인용.
- **Step 7 기록** — `docs/code_review/<주제>/YYYY-MM-DD.md`(날짜=버전). 코드 버전 고정 + 타임라인 인덱스(`README.md`) 갱신.
- **Step 8 자체 점검** — 아래 grep 통과.
- **Step 9 보고** — 1~2 줄. 변경 파일/후속 TODO 명시.

---

## 기록 위치 · 버전 관리 (날짜 기반)

코드는 계속 바뀌므로 리뷰는 **특정 코드 상태의 스냅샷**이다. 요청마다 날짜로 버전을 매겨 누적한다.

**위치**: `docs/code_review/<주제>/YYYY-MM-DD.md` (`<주제>` = 대상 파일명/모듈명/패키지명). 같은 날 재요청은 `YYYY-MM-DD-HHMM.md`. 폴더·파일 없으면 만든다(승인 불요). **날짜가 곧 버전.**

**코드 버전 고정 (의무)** — 각 날짜 파일 헤더에 리뷰한 코드 상태를 박는다 (없으면 어느 코드를 리뷰했는지 추적 불가):

- git: 리뷰 커밋 `<short-hash>` + 브랜치 + (있으면) PR(Pull Request) 번호
- 비-git: 대상 파일 내용 해시 + 일자

**타임라인 인덱스** — `docs/code_review/<주제>/README.md` 에 날짜·코드 버전·Verdict 표(최신 위):

| 날짜 | 코드 버전 | Verdict | 핵심 |
| --- | --- | --- | --- |
| 2026-06-14 | abc1234 (PR #42) | COMMENT | race 1 잔존 |
| 2026-06-10 | def5678 | REQUEST CHANGES | High 2 |

**findings 상태 추적** — 직전 날짜 리뷰 대비 각 항목에 상태: `[해결]`(고침, 커밋 인용)·`[잔존]`·`[신규]`·`[퇴행]`(고쳤다 재발).

**delta 리뷰 (재요청 시)** — 최초는 Core 5항목 전체. 재요청은 `git diff <직전 리뷰 커밋>..<현재>` 변경 파일·함수 중심 + 영향 findings 재평가. 변경 없는 부분은 직전 리뷰 cross-ref(재작성 X).

**staleness** — 현재 `HEAD` 가 최신 리뷰 커밋보다 앞서면 인덱스에 "재리뷰 필요(미리뷰 N 커밋)" 표시.

`user_instruction` 번들 설치 시 같은 시각 entry 와 제목 매핑(선택).

### 날짜 파일 템플릿

```markdown
## YYYY-MM-DD (KST) — <주제> 리뷰

### 코드 버전
- 리뷰 커밋: `abc1234` (branch `feat/x`, PR #42)   <!-- 비-git: 파일 내용 해시 -->
- 직전 리뷰: YYYY-MM-DD @ `def5678` — delta `def5678..abc1234`
- (user_instruction 설치 시) 같은 시각 `user_instructions.md` entry 참조

### 분석 분기 명시
- 분기: 단위 코드 리뷰 / 전체 구조 분석
- 감지된 도메인: ros2-review / concurrency / embedded-review / 없음

### Core 인벤토리
#### 1. 목적
#### 2. 코드 플로우차트
#### 3. 함수 리스트
#### 4. 전역 변수 / 모듈 상수  (없으면 "없음")
#### 5. 의존성 3-tier

### 도메인 (감지 시)
#### <도메인>-... 추가 표

### 평가
severity 분포: Critical 0 / High 1 / Medium 6 / Low 3 / Info 2
Verdict: REQUEST CHANGES

**함수 #3 `name` — [태그][잔존] 요약 (Severity)**
   재현: 위치 / 조건
   권고: 조치

---
```

---

## 룰

1. **Core 5 항목 누락 0** — 누락 = SOP 위반
2. **인벤토리 선행 → 평가**
3. **Cross-reference 의무** — 평가는 인벤토리 `#` 번호 인용
4. **카테고리 인라인 태그 의무** — 평가 항목마다 `[카테고리]` 태그
5. **다중 진입점 흐름도 분리** — path 별 + 공통 호출 그래프
6. **함수 표기 규칙** — `ClassName.method`, `outer.inner`(서브 번호)
7. **의존성 3-tier 의무** — 런타임 필수·선택 모두 부재 시 동작 명시
8. **추측 금지** — grep, LSP, 실측 인용
9. **자동 감지 무력화 금지** — 트리거 충족 + 설치 시 도메인 적용 의무(사용자 명시 거부 외)
10. **인벤토리 5 항목 "없음" 명시 의무** — 비어도 "없음" 한 줄(점검 누락과 구분)
11. **작성자 self-APPROVE 금지** — 별도 lane 에서만 `APPROVE`
12. **날짜=버전 + 코드 버전 고정** — 요청마다 `<주제>/YYYY-MM-DD.md`, 리뷰 커밋/해시 명시, findings 상태(`[해결]`/`[잔존]`/`[신규]`/`[퇴행]`) 추적

---

## 자체 점검

```bash
TARGET=docs/code_review/<주제>/YYYY-MM-DD.md

# 0. 코드 버전 고정 (의무) — 리뷰 커밋 또는 파일 해시
grep -E "리뷰 커밋: |코드 버전|파일 내용 해시" $TARGET

# 1. Core 함수 표 헤더 (6 컬럼 보존)
grep -E "^\| # +\| 함수 +\| 입력 +\| 출력 +\| 기능 +\| 위치 +\|" $TARGET

# 2. Core 의존성 3-tier 표 헤더
grep -E "^\| Tier .*대상.*부재 시 동작" $TARGET

# 3. 평가의 #번호 cross-reference (평가 항목 수 이상)
grep -cE "함수 #[0-9]+" $TARGET

# 4. Core 카테고리 인라인 태그 등장
grep -oE "\[(논리|SOLID|스타일|성능|테스트|품질)\]" $TARGET | sort -u

# 5. severity 분포 요약 + Verdict 2줄 의무
grep -E "severity 분포: Critical [0-9]+ / High [0-9]+ / Medium [0-9]+ / Low [0-9]+ / Info [0-9]+" $TARGET
grep -E "^Verdict: (APPROVE|REQUEST CHANGES|COMMENT)" $TARGET

# 6. 감지된 도메인 표기 의무
grep -E "감지된 도메인: " $TARGET

# 7. user_instructions.md 시각 매핑 (user_instruction 번들 설치 시에만 — 없으면 생략, 무해)
grep "^## " $TARGET | head -1
[ -f docs/user_instructions/user_instructions.md ] && grep "^## " docs/user_instructions/user_instructions.md | head -1 || echo "(user_instructions.md 없음 — 매핑 생략)"
```

설치된 도메인 sub-file 의 자체 점검 grep 도 함께 통과해야 한다.

---

## 변경 절차

본 룰은 SSOT. 변경 시 사용자 승인 필수. 변경 후 VERSION(semver)·CHANGELOG 갱신, 설치된 다운스트림 통보, 자체 점검 통과 확인.

---

## 근거 — Core + Add-on 결합

- **Core 단독**: 도메인 특화 결함(QoS 불일치·priority inversion·race)을 일반 카테고리로 묻어 발견 누락.
- **Add-on 만**: 도메인 평가는 깊으나 코드 구조 매핑 부족 → 후속 재작업.
- **Core + Add-on 결합**: 구조 매핑(Core) 위에 도메인 평가(Add-on)가 cross-reference 로 위치까지 명시 → 후속 작업자가 동일 인벤토리 재사용.
- **확장 인터페이스**: 새 도메인 추가가 다른 블록에 영향 없음.

---

**VERSION**: 1.1.0 (날짜 기반 리뷰 버전 관리 추가 — 코드 버전 고정·타임라인 인덱스·findings 상태 추적)
