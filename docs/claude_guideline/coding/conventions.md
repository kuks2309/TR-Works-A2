# 코딩 관습 (Conventions) — 명명·스타일·변수 규율

> **본 파일은 지시용.** coding.md §4 가 위임하는 **언어 무관** 코딩 관습. self-contained — 본문 외 의존 0. (언어/도구별 구체·포맷터는 `stack.md`, 함수표·전역변수표 *양식*은 `code_review` 단일 SSOT.)

## 1. 명명 (naming)

- **의도가 드러나는 이름.** 약어·한 글자 변수 지양(루프 `i`·`j` 등 관용 예외).
- **일관성**: 같은 개념엔 같은 단어. 케이스 규약(snake/camel)은 언어 관례를 따르고 **포맷터/린터로 강제**(`stack.md`).
- 불리언은 `is`/`has`/`can` 접두. 함수는 동사구, 변수는 명사구.

## 2. 함수 규율

- **단일 책임** — 한 함수는 한 가지 일.
- 길이·중첩 깊이가 과하면 분리(권고). 부수효과는 이름·문서에 드러낸다.
- **중복 금지** → 기존 함수 검색(coding.md §2) + `dup-signature` 이빨이 기계 차단.

## 3. 변수·전역 규율

- **스코프 최소화**: 지역 우선, 전역은 최후 수단.
- **불필요한 가변 전역 금지** — 결합도·race 위험. (적정성 평가 권위 = `code_review` 의 `[품질]`.)
- 매직 넘버 금지(명명 상수로). shadowing·미사용 변수 금지.
- **공유 가변 전역의 writer** 는 §6 전역변수표 "누가 바꾸나" 칸에 기록(동시성 → `domains/concurrency-coding.md`).

## 4. 주석·문서

- 코드가 '무엇'을, 주석이 '왜'를 말한다. 자명한 주석 금지.
- 공개 함수는 docstring/doc 주석(인자·반환·예외·단위).

## 5. 강제

대부분 `⟦권고⟧`(스타일은 포맷터 + 리뷰가 강제). 기계 강제 연계:

- 포맷 → `stack.md` 의 `format` 이빨
- 함수 중복 → `dup-signature` 이빨
- 명명·전역 적정성(의미 판단) → `code_review` `[품질]` (별도 패스)

## 자체 점검

```bash
# 매직 넘버 후보(권고) — 리터럴 산재 탐지(예시)
grep -rnE '[^A-Za-z_."'"'"']-?[0-9]{2,}' --include=*.py --include=*.c --include=*.cpp . 2>/dev/null | grep -vE 'test|#|//' | head -5 || true
# 중복 함수
bash docs/claude_guideline/coding/checks/dup-signature.sh .
```

---

**VERSION**: 1.0.0 (명명 일관성 + 함수 단일책임·중복금지 + 변수/전역 스코프·가변전역 규율 + 주석 why; 양식 권위는 code_review, 포맷은 stack.md, 강제는 dup-signature/format 이빨 연계)
