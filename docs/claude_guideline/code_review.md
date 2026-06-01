# 코드 리뷰 작업 지침 (Code Review SOP)

사용자가 "코드 리뷰" / "코드 분석" 을 요청했을 때 **분석 방법론**과 **기록 형식**의 단일 근원(SSOT / Single Source of Truth).

## 설치 위치

- **본 파일**: 대상 프로젝트의 `docs/claude_guideline/code_review.md` 에 배치
- **리뷰 산출물**: `docs/code_review/<주제>.md` 에 기록 (대상 프로젝트 루트 기준 상대경로)

본 파일이 `docs/claude_guideline/code_review.md` 위치에 없으면 본 SOP 는 활성화되지 않는다 — 트리거 키워드가 감지되어도 룰 적용 불가. 새 프로젝트 적용 시 본 파일을 위 경로로 복사하는 것이 첫 단계.

## 트리거

사용자 메시지에 다음 키워드 등장 시:

- "코드 리뷰", "리뷰해줘", "리뷰해 주세요"
- "코드 분석", "분석해줘"
- "이 함수 / 이 파일 / 이 모듈 봐줘"
- 특정 파일 / 디렉토리 / PR (Pull Request) 경로를 첨부한 평가 요청

## 흐름도 (한눈에)

```
[코드 리뷰 요청 도착]
   ↓
[Step 1] 대상 범위 식별              ────→  ✓ 단일 파일 / 디렉토리 / 모듈 / PR 확정
   ↓
[Step 2] 단위 / 전체 분기 판정       ────→  ✓ 플로우차트 종류 결정
   ↓
[Step 3] 도메인 자동 감지 (Add-on)   ────→  ✓ ROS2 / 동시성 / 임베디드 / 일반 분류
   ↓
[Step 4] Core 인벤토리 (5 항목)      ────→  ✓ 목적·플로우·함수·전역·의존성 누락 0
   ↓
[Step 5] Add-on 인벤토리 (감지시)    ────→  ✓ 도메인별 추가 표 작성
   ↓
[Step 6] 평가 (severity 클러스터)    ────→  ✓ High/Medium/Low + 카테고리 인라인 태그
   ↓
[Step 7] docs/code_review/<주제>.md 기록 ────→  ✓ KST 시각, user_instructions.md 매핑
   ↓
[Step 8] 자체 점검 grep              ────→  ✓ 헤더·#번호·카테고리 태그 통과
   ↓
[Step 9] 1~2 줄 결과 보고            ────→  ✓ 변경 파일 / 후속 TODO 명시
   ↓
[완료]
```

## 출력 구조 — Core + Domain Add-on

코드 리뷰 결과는 두 층으로 구성된다:

| 층 | 내용 | 적용 조건 |
|----|------|----------|
| **Core** (공통) | 목적·플로우차트·함수표·전역변수표·의존성 3-tier | 모든 리뷰 필수 |
| **Domain Add-on** | 도메인별 추가 인벤토리·평가 카테고리 | 트리거 감지 시 자동 적용 (다중 가능) |

한 파일이 여러 도메인에 해당하면 모든 add-on 동시 적용 (예: ROS2 임베디드 노드 → A + B + C 동시).

---

## Core 인벤토리 (5 항목 — 누락 0)

### 1. 목적

1~3 문단으로 해당 파일·모듈·디렉토리가 무엇을 위한 코드인지 기술. 추측 금지 — 코드 / README / 주석 / 데이터시트 인용 (추정 금지 · 실측 검증).

### 2. 코드 플로우차트

| 요청 유형 | 플로우차트 형식 |
|----------|----------------|
| 단위 코드 리뷰 (1 함수·1 파일) | **단일 플로우 차트** — 그 단위의 진입/분기/루프/종료, 주요 조건문, 에러 경로 |
| 전체 구조 분석 (디렉토리·다중 모듈) | **전체 코드 흐름도** — 모듈 간 호출 관계, 데이터 흐름, 진입점·종료점, 외부 인터페이스 |

**다중 진입점 분리 룰**: 한 패키지에 2개 이상 진입점(예: offline / live, CLI / 라이브러리) 이 있으면 path 별로 흐름도 분리 + 공통 호출 그래프 별도 표시.

### 3. 함수 리스트 표

컬럼 순서 고정: `#`, `함수`, `입력`, `출력`, `기능`, `위치(file:line)`.

**함수 표기 규칙**:

- 일반 함수: `function_name`
- 클래스 메서드: `ClassName.method`
- 이너 함수 / 클로저: `outer.inner` (한 행 추가, `#` 번호는 부모 행 + 알파벳, 예: `13a`)
- Launch / config 진입점 (ROS2 `generate_launch_description` 등): 함수 표에 포함, 위치 컬럼에 `launch/foo.launch.py:N` 명시
- C/C++ `static` 함수, Python `_private` 함수 모두 포함

모든 함수 전수 (private / static 포함). 누락 0.

### 4. 전역 변수 / 모듈 상수 표

컬럼 순서 고정: `#`, `사용처(함수)`, `기능`, `위치(file:line)`.

**포함 범위**:

- 진정한 전역 변수 (mutable module-level state)
- 모듈 레벨 상수 (예: `CAMERAS = [...]`)
- 환경 의존 default 경로 (절대경로 상수)
- C/C++ `static` 파일 스코프 변수
- `extern` 선언으로 노출된 다른 모듈 변수

상수와 변수가 섞이면 표 안에 `(상수)` / `(가변)` 표기 추가.

없으면 "전역 변수 / 모듈 상수 없음" 한 줄로 명시 (표 생략 가능).

### 5. 의존성 3-tier 표

| Tier | 정의 | 표기 |
|------|------|------|
| **빌드** | `package.xml <depend>`, `CMakeLists`, `setup.py`, `pyproject.toml`, `requirements.txt` 등에 선언된 빌드 시 필수 의존성 | 패키지명, 버전 제약 |
| **런타임 필수** | 실행에 반드시 필요한 외부 노드·서비스·토픽 발행자·HW 페리페럴·외부 프로세스·환경 변수 | 의존 대상, **부재 시 동작 명시 의무** |
| **런타임 선택** | 있으면 사용, 없으면 fallback. **fallback 동작 명시 의무** | 의존 대상, fallback 정의 |

표 컬럼: `Tier`, `대상`, `버전/제약`, `부재 시 동작 (필수/선택)`, `근거(파일:line 또는 코드 경로)`.

---

## Core 평가 (severity 클러스터 + 카테고리 인라인 태그)

평가는 **severity 클러스터(Critical / High / Medium / Low / Info)** 로 묶고, 각 항목에 **카테고리 인라인 태그** 를 붙인다.

### Severity 분포 요약 + Verdict (상단 2줄 의무)

```
severity 분포: Critical 0 / High 1 / Medium 6 / Low 3 / Info 2
Verdict: REQUEST CHANGES
```

**Verdict 기준**: Critical/High ≥ 1 → `REQUEST CHANGES`, Medium 이하만 → `COMMENT`, 이슈 없음 → `APPROVE`. 본 SOP 로 생산한 리뷰는 작성자 본인이 `APPROVE` 할 수 없다 — 별도 lane(code-reviewer/verifier 에이전트 또는 다른 세션) 에서만 `APPROVE` 가능.

### 평가 카테고리 (Core 인라인 태그)

- `[논리]` — 논리적 결함, off-by-one, race, null 미처리
- `[SOLID]` — SRP / OCP / LSP / ISP / DIP 위반
- `[스타일]` — 컨벤션, 네이밍
- `[성능]` — 알고리즘 복잡도, 핫패스, I/O 빈도
- `[테스트]` — 테스트 누락, 회귀 위험, 테스트 적절성
- `[품질]` — 리팩토링 권고, 전반 품질 전략, dead code
- (도메인 add-on 활성화 시 추가 태그 — `[QoS]`, `[race]`, `[ISR]` 등)

### 항목 작성 형식

```markdown
**함수 #3 `process_packet` — [논리] null 체크 누락 (High)**
   재현: parser.c:142, packet.data == NULL 진입 시 segfault
   권고: 진입부 가드 추가
```

**Cross-reference 의무**: 모든 평가는 Core 인벤토리의 `#` 번호 인용 (단일 또는 다중 — 예: "함수 #1·#3·#7 군집").

**해당 없음 처리**:
- severity 클러스터는 빈 카테고리 자연 생략 (명시 불필요)
- **인벤토리 5 항목**(목적·플로우·함수·전역·의존성) 은 빈 경우 "없음" 명시 의무 (점검 누락과 구분)

---

## 도메인 자동 감지 — Add-on 트리거

| Add-on | 트리거 (1개 이상 충족 시 활성) |
|--------|------------------------------|
| **A. ROS2** | `package.xml`, `rclpy` / `rclcpp` import, `.launch.py`, `rcl_interfaces`, `ament_python` / `ament_cmake` 빌드 타입 |
| **B. 동시성** | `threading`, `asyncio`, `std::thread`, `std::mutex`, `MutuallyExclusiveCallbackGroup`, `ReentrantCallbackGroup`, multi-callback ROS 노드, `@asynccontextmanager`, `await`, `async def` |
| **C. 임베디드** | `__attribute__((interrupt))`, `ISR(`, `NVIC_`, `IRQHandler`, FreeRTOS API (`xTaskCreate`, `xQueueSend`), STM32 HAL 매크로, `.ld` linker script, register-level access, `volatile` 빈출 |

다중 add-on 동시 적용 가능. 우선순위 없음. 사용자가 명시적으로 도메인을 지정하면 그것이 우선 (자동 감지 무력화는 사용자 명시 거부 시에만).

---

## Add-on A — ROS2

### 인벤토리 추가 항목

**A-1. Subscriptions 표** — 컬럼: `토픽`, `메시지 타입`, `QoS (depth·reliability·durability·history)`, `콜백 함수`, `위치(file:line)`

**A-2. Publications 표** — 컬럼: `토픽`, `메시지 타입`, `QoS`, `발행 위치(함수)`, `위치(file:line)`

**A-3. Services / Actions 표** — 컬럼: `이름`, `타입`, `클라이언트/서버`, `콜백/요청 위치`, `위치(file:line)`

**A-4. Parameters 표** + **YAML 예시 블록**

표 컬럼: `이름`, `타입`, `default`, `declare 위치`, `사용 위치`

YAML 예시는 **의미 그룹별 분리 + 단위·물리량 주석 의무**:

```yaml
node_name:
  ros__parameters:
    # 의미 그룹 1 (예: 섹터)
    front_sector_start: -1.5708    # rad, -90°
    front_sector_end:    1.5708    # rad, +90°

    # 의미 그룹 2 (예: 거리)
    base_stop_distance:    0.3      # m, d_base
    base_warning_distance: 0.8      # m

    # 의미 그룹 3 (예: 타이밍)
    publish_rate_hz:  20.0          # Hz
```

**A-5. TF frames 표** (TF 사용 시) — 컬럼: `frame`, `parent`, `발행 노드`, `정적/동적`, `위치`

### 평가 추가 카테고리 (인라인 태그)

- `[QoS]` — pub/sub QoS 호환성 (RELIABLE↔BEST_EFFORT 불일치, TRANSIENT_LOCAL latching 정합, depth 부족)
- `[ns]` — 네임스페이스 / 토픽 충돌
- `[exec]` — 콜백 그룹·executor 선택 적합성 (single vs multi-threaded)
- `[param]` — 파라미터 default 의 물리량·범위·단위 일관성 (YAML 의미 그룹별 검토)
- `[runtime]` — 런타임 필수 노드 부재 시 동작 정의 명확성 (의존성 표 tier 2 와 연계)

---

## Add-on B — 동시성 (Threading / async)

### 인벤토리 추가 항목

**B-1. 동기화 객체 표** — 컬럼: `객체`, `종류(Mutex/Lock/Event/Semaphore/Atomic/Condvar)`, `보호 자원`, `획득 위치`, `해제 위치`

**B-2. 공유 상태 표** — 컬럼: `변수`, `읽기 위치`, `쓰기 위치`, `보호 객체` (보호 없으면 `"비보호"`)

**B-3. 실행 컨텍스트 표** — 컬럼: `이름`, `종류(thread/task/coroutine/callback group/executor)`, `우선순위·executor`, `생성 위치`

### 평가 추가 카테고리 (인라인 태그)

- `[race]` — 공유 변수 비보호 쓰기, race condition 후보
- `[deadlock]` — 다중 lock 획득 순서 일관성, 순환 의존
- `[timing]` — 콜백 차단성, 시간 budget, jitter 원인
- `[reentrant]` — 재진입 가능 콜백에서 비-reentrant 호출

---

## Add-on C — 임베디드 / RTOS

### 인벤토리 추가 항목

**C-1. ISR / 인터럽트 표** — 컬럼: `벡터 이름`, `NVIC 우선순위`, `사용 자원(레지스터·전역)`, `WCET (Worst-Case Execution Time)`, `위치(file:line)`

**C-2. Task / Thread 표** — 컬럼: `이름`, `우선순위`, `stack 크기`, `주기 (또는 이벤트 driven)`, `위치`

**C-3. 공유 자원 표** — 컬럼: `자원`, `사용 ISR/Task`, `보호 메커니즘(disable IRQ / semaphore / atomic / volatile)`

**C-4. 하드웨어 인터페이스 표** — 컬럼: `페리페럴(UART/SPI/I2C/CAN/GPIO 등)`, `핀맵`, `속도/모드`, `드라이버 위치`

### 평가 추가 카테고리 (인라인 태그)

- `[prio]` — Priority inversion, 낮은 우선순위가 높은 우선순위를 막는 경로
- `[ISR]` — ISR 내 블로킹 호출, malloc, printf, 긴 작업
- `[WCET]` — 인터럽트 latency 추정, 응답성 budget
- `[volatile]` — `volatile` 단독 사용 등 보호 부족 사례
- `[HW]` — 핀맵·속도 변경 사전 승인 ("하드웨어 인터페이스 변경" 트리거)

---

## Add-on 등록 절차 (확장 인터페이스)

새 도메인 add-on 추가 시 다음 5 요건 모두 만족해야 SSOT 등록 가능:

1. **트리거** — 자동 감지 키워드 / 파일 패턴 (1개 이상). 다른 add-on 과 중복 시 모두 활성.
2. **인벤토리 표** — 컬럼 순서 고정. 표 헤더는 grep 가능한 식별자.
3. **평가 추가 카테고리** — 인라인 태그 (`[xxx]`) 형식. Core 카테고리와 충돌 없음.
4. **자체 점검 grep** — 표 헤더 + 카테고리 태그 검출 정규식.
5. **다른 add-on 과의 의존 / 충돌 명시** — 동시 활성 시 룰.

향후 확장 후보 (참고): Web/API, DB 스키마, ML 텐서 / 모델, GPU kernel.

---

## Step 별 상세

### Step 1 — 대상 범위 식별

- 단일 파일 / 디렉토리 / 모듈 / PR 중 하나로 확정
- 모호 시 STOP, 1줄 질문

### Step 2 — 단위 / 전체 분기 판정

| 요청 유형 | 판정 기준 | 플로우차트 형식 |
|-----------|----------|----------------|
| 단위 코드 리뷰 | 1 함수 또는 1 파일 명시 | 단일 플로우 차트 |
| 전체 구조 분석 | "전체", "구조", "아키텍처", "모듈" 키워드 또는 디렉토리 단위 지정 | 전체 코드 흐름도 |

모호 시 1줄 질문.

### Step 3 — 도메인 자동 감지

위 add-on 트리거 표 확인. 다중 감지 가능. 사용자 명시 지정 시 그것 우선.

### Step 4 — Core 인벤토리

5 항목 (목적·플로우·함수·전역·의존성) 작성. 누락 0.

### Step 5 — Add-on 인벤토리

감지된 add-on 의 추가 표 작성. 비어 있는 표는 "해당 항목 없음" 명시.

### Step 6 — 평가

severity 클러스터(Critical/High/Medium/Low/Info) + 카테고리 인라인 태그. 모든 항목 인벤토리 `#` 번호 인용.

### Step 7 — 기록

위치: `docs/code_review/<주제>.md`. 동일 주제 기존 파일 있으면 prepend (최신 위, 시간 역순). 신규 생성 시 `docs/code_review/README.md` 도 함께 생성.

### Step 8 — 자체 점검

자체 점검 섹션 grep 7개 모두 통과.

### Step 9 — 보고

1~2 줄. 변경 파일 / 후속 TODO 명시.

---

## 기록 위치 / 템플릿

기록 위치: `docs/code_review/<주제>.md`

- `<주제>` = 대상 파일명 / 모듈명 / 패키지명
- 동일 주제 기존 파일 → prepend (최신 위)
- `docs/user_instructions/user_instructions.md` 의 같은 시각 entry 와 제목 매핑 (존재 시)

기록 템플릿:

```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

### 트리거 요청
`docs/user_instructions/user_instructions.md` `YYYY-MM-DD HH:MM` entry 참조 (존재 시).

### 분석 분기 명시
- 분기: 단위 코드 리뷰 / 전체 구조 분석
- 감지된 add-on: A(ROS2) / B(동시성) / C(임베디드) / 없음

### Core 인벤토리

#### 1. 목적
<1~3 문단>

#### 2. 코드 플로우차트
<단위 또는 전체 흐름도>

#### 3. 함수 리스트
| # | 함수 | 입력 | 출력 | 기능 | 위치 |
|---|------|------|------|------|------|
| 1 | ... | ... | ... | ... | file:line |

#### 4. 전역 변수 / 모듈 상수
| # | 사용처 | 기능 | 위치 |
|---|--------|------|------|
| 1 | ... | ... | file:line |

(없으면: "전역 변수 / 모듈 상수 없음")

#### 5. 의존성 3-tier
| Tier | 대상 | 버전/제약 | 부재 시 동작 | 근거 |
|------|------|----------|-------------|------|
| 빌드 | ... | ... | — | package.xml:line |
| 런타임 필수 | ... | ... | (실패 / 종료) | 코드:line |
| 런타임 선택 | ... | ... | (fallback 정의) | 코드:line |

### Add-on A — ROS2 (감지 시)
#### A-1. Subscriptions
#### A-2. Publications
#### A-3. Services / Actions
#### A-4. Parameters (표 + YAML 블록)
#### A-5. TF frames

### Add-on B — 동시성 (감지 시)
#### B-1. 동기화 객체
#### B-2. 공유 상태
#### B-3. 실행 컨텍스트

### Add-on C — 임베디드 (감지 시)
#### C-1. ISR / 인터럽트
#### C-2. Task / Thread
#### C-3. 공유 자원
#### C-4. 하드웨어 인터페이스

### 평가

severity 분포: Critical 0 / High 1 / Medium 6 / Low 3 / Info 2
Verdict: REQUEST CHANGES

**함수 #3 `name` — [태그] 요약 (Severity)**
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
6. **함수 표기 규칙** — `ClassName.method`, `outer.inner` (서브 번호)
7. **의존성 3-tier 의무** — 런타임 필수·선택 모두 부재 시 동작 명시
8. **YAML 단위 주석 의무** (ROS2 add-on) — 모든 파라미터 단위·물리량 주석
9. **추측 금지** — grep, LSP, 실측 인용
10. **자동 감지 무력화 금지** — 트리거 충족 시 add-on 적용 의무 (사용자 명시 거부 외)
11. **인벤토리 5 항목 "없음" 명시 의무** — 비어 있어도 "없음" 한 줄 (점검 누락과 구분)

---

## 자체 점검

```bash
TARGET=docs/code_review/<주제>.md

# 1. Core 함수 표 헤더 (6 컬럼 보존)
grep -E "^\| # +\| 함수 +\| 입력 +\| 출력 +\| 기능 +\| 위치 +\|" $TARGET

# 2. Core 의존성 3-tier 표 헤더
grep -E "^\| Tier .*대상.*부재 시 동작" $TARGET

# 3. 평가의 #번호 cross-reference (평가 항목 수 이상)
grep -cE "함수 #[0-9]+" $TARGET

# 4. 카테고리 인라인 태그 등장 (Core 6 + add-on 14)
grep -oE "\[(논리|SOLID|스타일|성능|테스트|품질|QoS|ns|exec|param|runtime|race|deadlock|timing|reentrant|prio|ISR|WCET|volatile|HW)\]" $TARGET | sort -u

# 5. severity 분포 요약 + Verdict 2줄 의무
grep -E "severity 분포: Critical [0-9]+ / High [0-9]+ / Medium [0-9]+ / Low [0-9]+ / Info [0-9]+" $TARGET
grep -E "^Verdict: (APPROVE|REQUEST CHANGES|COMMENT)" $TARGET

# 6. add-on 활성 표기 의무
grep -E "감지된 add-on: " $TARGET

# 7. user_instructions.md 시각 매핑
grep "^## " $TARGET | head -1
grep "^## " docs/user_instructions/user_instructions.md | head -1
```

---

## 변경 절차

본 룰은 SSOT. 변경 시 사용자 승인 필수. 변경 후:

1. 사용자 지시 기록(`user_instructions.md` 등) 의 산출물 표 동기화 (해당 시)
2. CHANGELOG / VERSION 갱신 (해당 시)
3. `dogfooding/` 산출물로 재검증

---

## 근거 — Core + Add-on 결합

- **Core 단독 출력**: 도메인 특화 결함(QoS 불일치·priority inversion·race)을 일반 평가 카테고리로 묻혀 발견 누락.
- **Add-on 만 출력**: 도메인 평가는 깊지만 코드 구조 매핑이 부족 → 후속 리뷰 재작업.
- **Core + Add-on 결합**: 구조 매핑(Core) 위에 도메인 평가(Add-on) 가 cross-reference 로 위치까지 명시되어, 후속 작업자가 동일 인벤토리를 재사용.
- **확장 인터페이스 명시**: 새 도메인 추가가 다른 블록에 영향 없음 (DB·Web·ML 등 미래 도메인 흡수 가능).
