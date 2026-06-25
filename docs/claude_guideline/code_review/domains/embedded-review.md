# 코드 리뷰 — 임베디드 Add-on (Code Review — Embedded / RTOS)

> 코어 [review.md](../review.md) 의 도메인 sub-file. **관점: 임베디드 코드 리뷰** — ISR·Task·공유 자원·HW 인터페이스 인벤토리/평가.
> 짝: 임베디드 참조 문서 인용·datasheet 보관은 `external_reference` 번들의 `embedded-reference.md` (별개·상보).

**트리거**: `__attribute__((interrupt))`, `ISR(`, `NVIC_`, `IRQHandler`, FreeRTOS API(`xTaskCreate`, `xQueueSend`), STM32 HAL 매크로, `.ld` linker script, register-level access, `volatile` 빈출.

## 1. 인벤토리 추가 표

**C-1. ISR / 인터럽트** — 컬럼: `벡터 이름`, `NVIC 우선순위`, `사용 자원(레지스터·전역)`, `WCET(Worst-Case Execution Time)`, `위치(file:line)`

**C-2. Task / Thread** — 컬럼: `이름`, `우선순위`, `stack 크기`, `주기(또는 이벤트 driven)`, `위치`

**C-3. 공유 자원** — 컬럼: `자원`, `사용 ISR/Task`, `보호 메커니즘(disable IRQ / semaphore / atomic / volatile)`

**C-4. 하드웨어 인터페이스** — 컬럼: `페리페럴(UART/SPI/I2C/CAN/GPIO 등)`, `핀맵`, `속도/모드`, `드라이버 위치`

## 2. 평가 추가 카테고리 (인라인 태그)

- `[prio]` — Priority inversion, 낮은 우선순위가 높은 우선순위를 막는 경로
- `[ISR]` — ISR 내 블로킹 호출, malloc, printf, 긴 작업
- `[WCET]` — 인터럽트 latency 추정, 응답성 budget
- `[volatile]` — `volatile` 단독 사용 등 보호 부족 사례
- `[HW]` — 핀맵·속도 변경 사전 승인("하드웨어 인터페이스 변경" 트리거)
- `[safety]` — ISO 26262-6 §8 / MISRA(Motor Industry Software Reliability Association) C 정렬 위반. 안전 코드 의무이며 ASIL(Automotive Safety Integrity Level) 높을수록 강제도 상승.

### ISO 26262 / MISRA 안전 원칙 (임베디드 특화)

아래는 **안전/임베디드에 특화**된 원칙으로, 일반 코드엔 부적합할 수 있어(웹·Python 은 재귀·동적 할당 정상) 본 도메인에만 둔다:

- **동적 할당 금지**(초기화 후 malloc/new) — 결정론적 메모리
- **재귀 금지** — 스택 경계 보장
- single entry / single exit (MISRA 권고)
- 변수 사용 전 초기화, 암묵적 형변환 금지, 포인터 제한
- 전역 변수 **정당화 의무** — 코어 `[품질]` 의 불필요 전역 점검을 안전 맥락에서 **강제**로 격상

> **보편 원칙**(변수명 재사용 금지·불필요 전역 회피)은 이미 코어 `[품질]` 에 있어 모든 코드에 적용된다. 본 `[safety]` 는 그 위에 안전 특화 항목만 더한다.
> **등급 ⓦ** — ISO 26262-6:2018 §8 Table 6 / MISRA C 는 1차 source(유료) 대조 필요. 코딩 표준 목록·라이선스·받는 법은 `external_reference/coding_standards.md`, 인용 형식은 standards 도메인.

## 3. 자체 점검 grep

```bash
TARGET=docs/code_review/<주제>.md

# ISR / 공유 자원 표 헤더
grep -E "^\| 벡터 이름 .*NVIC.*WCET|^\| 자원 .*ISR/Task.*보호 메커니즘" $TARGET

# 임베디드 평가 태그 등장
grep -oE "\[(prio|ISR|WCET|volatile|HW|safety)\]" $TARGET | sort -u

# WCET 추정 흔적
grep -E "WCET|latency" $TARGET

# ISO 26262 / MISRA 안전 원칙 흔적 (safety 적용 시)
grep -E "동적 할당|재귀|ASIL|MISRA|ISO 26262|single.*exit" $TARGET
```

## 4. 다른 도메인과의 의존/충돌

- **concurrency 와 상보** — ISR↔Task 공유 자원은 concurrency 의 동기화 객체 표와 cross-reference. `[prio]`(priority inversion)는 deadlock 인접 결함.
- **HW 인터페이스 변경**은 사전 승인 트리거(`[HW]`) — Core 평가에서 사용자 명시.
