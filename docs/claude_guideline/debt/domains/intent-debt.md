# 의도 부채 (Intent Debt) — 도메인

> debt.md 가 위임하는 **의도 부채** 상세(식별 신호·등록·상환). self-contained — 본문 외 의존 0.

## 트리거 (식별 신호)

- 결정 **근거 없음** · ADR(Architecture Decision Record, 설계 결정 기록) 누락 · "왜 이렇게 했지"
- 사용자 **의도 미기록** · 주석 없는 매직 동작 · 요구사항↔코드 추적 불가

## 1. 등록 항목

- **의도 공백 위치**(결정·기능) · 잃은 맥락 · 복원 단서(커밋·이슈·대화 로그)

## 2. 상환 전략

- **ADR 작성**(맥락·결정·대안·결과) — coding §3 트리거와 연계
- `user_instruction` 기록(사용자 원문 의도 보존)으로 의도 부채 **사전 예방**
- 의도 복원 인터뷰 · 요구사항↔코드 추적 복원

## 3. 강제 연계

- coding **§3 사전승인**에서 ADR 누락 식별 → 등록
- `user_instruction` 번들(지시 원문 기록)이 의도 부채의 1차 방어선 — 함께 설치 권장
- (coding/user_instruction 미설치 시 해당 연계는 생략 — registry 의 '의도' 유형 등록은 단독 동작.)

## 자체 점검

```bash
# ADR 디렉토리 존재·미작성 결정 점검 (의도 부채 1차 신호)
ls docs/**/adr/*.md docs/**/decisions/*.md 2>/dev/null | head || echo "(ADR 없음 — 의도 부채 위험)"
```

---

**VERSION**: 1.0.0 (식별 신호 + 공백위치/단서 등록 + ADR·user_instruction·인터뷰 상환; coding §3 식별·user_instruction 예방 연계)
