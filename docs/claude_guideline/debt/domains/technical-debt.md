# 기술 부채 (Technical Debt) — 도메인

> debt.md 가 위임하는 **기술 부채** 상세(식별 신호·등록·상환). self-contained — 본문 외 의존 0.

## 트리거 (식별 신호)

- `TODO`/`FIXME`/`HACK` 주석 · 미흡/누락 테스트 · 복붙 중복 로직 · 매직 넘버
- 우회·임시방편(quick fix) · 노후·취약 의존성 · 미룬 리팩토링 · 죽은 코드

## 1. 등록 항목 (registry 공통 + 기술 특화)

- 공통: `id·위치·사유·식별일·상태·상환계획`
- 기술 특화: **이자**(방치 시 커지는 재작업 비용·전파 위험) · 영향 범위(모듈/전역)

## 2. 상환 전략

- 리팩토링(중복 제거·구조 개선) · 테스트 보강 · 의존성 갱신 · 매직 넘버 상수화
- **보이스카웃 규칙**: 그 코드를 만지는 김에 조금씩 상환(한 번에 큰 상환 강요 금지).
- 고이자 우선(자주 바뀌는·핵심 경로 부채 먼저).

## 3. 강제 연계

- 코드 `TODO`/`FIXME`/`HACK` → `debt-marker` 이빨(debt id 강제)
- 중복 → coding `dup-signature` · 미흡 테스트 → coding `tests-ran` · 금지패턴 → coding `banned-pattern`
- (coding 미설치 시 coding 이빨 연계는 생략 — debt 의 `debt-marker`·registry 는 단독 동작.)

## 자체 점검

```bash
grep -rnE '\b(TODO|FIXME|HACK)\b' --include=*.py --include=*.c --include=*.cpp . 2>/dev/null | head
```

---

**VERSION**: 1.0.0 (식별 신호 + 이자/영향 등록 + 리팩토링·테스트·의존성 상환 + 보이스카웃; debt-marker·dup-signature·tests-ran 연계)
