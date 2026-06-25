# 임베디드 코딩 (Embedded Coding) — 도메인

> coding.md 가 위임하는 임베디드/RTOS *작성* 규칙(리뷰 관점은 자매 `code_review` 의 `embedded-review.md`). self-contained — 본문 외 의존 0.

## 트리거 (활성 조건)

임베디드/실시간 코드를 쓰면 활성, 아니면 면제:

- ISR(인터럽트 서비스 루틴)·`NVIC_`·`__attribute__((interrupt))` · FreeRTOS/RTOS API · `volatile` 빈출 · 레지스터 직접 접근

## 1. 인터럽트(ISR) 규율

- **ISR 은 짧게** — blocking·heap 할당·`printf`·긴 루프 금지. 플래그만 세우고 본문은 메인 루프/태스크로.
- ISR ↔ 메인이 공유하는 변수는 **`volatile` + 원자적 접근** → `domains/concurrency-coding.md`(인터럽트 비활성 구간 또는 atomic).

## 2. 메모리

- **정적 할당 선호**, 동적 heap 지양(단편화·비결정성). 불가피하면 풀(pool)·고정 크기.
- 메모리 안전 일반은 → `domains/memory-coding.md`.

## 3. 타이밍·실시간

- WCET(최악 실행시간) 고려, busy-wait 지양(타이머/이벤트). 우선순위 역전 주의.
- 결정성: 동적 경로·가변 루프는 상한 명시.

## 4. 하드웨어·안전

- 레지스터 read-modify-write 는 원자성 보장(인터럽트 중 변경 대비).
- **safe-state**: 실패·워치독 리셋 시 안전값으로. 정수 오버플로·고정소수점 → `domains/numeric-coding.md`.

## 5. 강제

대부분 `⟦권고⟧`. 연계:

- 메모리 → `memory` 이빨(clang-tidy/asan) · 동시성/원자성 → concurrency aspect
- MISRA·안전 규칙 점검 → `code_review` 의 `embedded-review`(별도 패스)

## 자체 점검

```bash
grep -rEl 'ISR\(|NVIC_|__attribute__\(\(interrupt|FreeRTOS|\bvolatile\b' --include=*.c --include=*.h --include=*.cpp . >/dev/null 2>&1 \
  && echo "임베디드 — 도메인 적용" || echo "(임베디드 없음 — 면제)"
```

---

**VERSION**: 1.0.0 (ISR 짧게·volatile/원자 + 정적 메모리 + WCET/결정성 + safe-state·레지스터; memory·concurrency·numeric cross-ref; embedded-review 와 write↔review 상보)
