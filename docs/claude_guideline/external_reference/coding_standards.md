# 코딩 표준 인덱스 (Coding Standards Index)

> external_reference 번들의 코딩 표준 큐레이션 — 어떤 표준을, 어디서 받고, 어떻게 보관·인용하는지 한곳에. code_review 번들의 `[품질]`(보편)·`[safety]`(안전 특화)와 연계. 표준 인용 형식은 [domains/standards.md](domains/standards.md).

본 파일은 **포인터·우리 정리(digest)** 만 담는다 — **유료·재배포 금지 문서 본문은 배포하지 않는다**(→ [handling.md §5 라이선스](handling.md)). 라이선스 등급: 2026-06 웹 확인 기준, ✅=확인, ⓦ=불명확/원문 확인 권장.

## 1. 표준 인덱스 (라이선스·배포)

| 표준 | 범위 | 라이선스 | 재배포 | 배치 |
| --- | --- | --- | --- | --- |
| ISO 26262-6 | 자동차 기능안전 SW | 유료 (ISO) | ❌ | `references/local/`(gitignore) + 공식 URL |
| MISRA C / C++ | 안전 C/C++ | 유료 (MISRA) | ❌ | `references/local/` + URL |
| AUTOSAR C++14 Guidelines | 안전 C++14 | 무료 열람, **복제 금지** ✅ | ❌ | URL 링크만 (autosar.org) |
| SEI CERT C | 보안 C | 조건 불명확 ⓦ | ⓦ | URL 링크 (wiki.sei.cmu.edu) |
| BARR-C:2018 | 임베디드 C | 무료 PDF, **규칙은 출처표기 시 채택** ✅ | △ 규칙 | 무료 PDF URL + 규칙 digest(출처표기) |
| Google C++ Style Guide | C++ 스타일 | **CC-BY(Creative Commons Attribution) 3.0** ✅ | ✅ 출처표기 | URL + 사본 허용 (google.github.io/styleguide) |
| PEP 8 | Python 스타일 | PSF(Python Software Foundation) ⓦ | ✅(추정) | URL (peps.python.org/pep-0008) |
| LLVM Coding Standards | C/C++ | Apache-2.0 ⓦ | ✅(추정) | URL (llvm.org/docs/CodingStandards.html) |
| Linux kernel coding style | C | GPL(General Public License) v2 ⓦ | ✅ 출처표기 | URL (docs.kernel.org) |
| NASA/JPL Power of Ten | 안전 C 10 규칙 | 규칙 이념 공개(원문 IEEE 유료) ⓦ | △ 규칙 | digest(출처표기) |

→ **유료(ISO·MISRA)는 조직 라이선스 사본을 `references/local/` 에만**, 절대 공개 배포 repo 에 넣지 않는다. **복제 금지(AUTOSAR)·불명확(CERT)은 URL 링크만**. **CC-BY/permissive(Google·PEP8·LLVM·Linux)는 링크 우선, 필요 시 출처표기 사본**.

## 2. 보편 코딩 규칙 다이제스트 (모든 언어 — code_review `[품질]`/`[SOLID]`)

특정 문서 복제가 아니라 **우리 정리**다. 코드 리뷰에서 점검:

- **전역 상태** — 가변 전역 회피·정당화 (race·결합도 위험)
- **변수** — 사용 전 초기화, 이름 재사용·shadowing 금지, 최소 스코프, 미사용 변수 제거
- **함수** — 단일 책임, 중복/유사 함수 통합 (DRY(Don't Repeat Yourself))
- **정리** — dead code·매직 넘버 제거
- **명료성** — 암묵적 형변환·hidden control/data flow 회피
- **에러 경로** — 예외·실패 경로 명시 처리

## 3. 안전 특화 규칙 다이제스트 (임베디드 — code_review `[safety]`)

일반 코드엔 부적합할 수 있다 (웹·Python 은 재귀·동적 할당 정상):

- **동적 할당 금지** (초기화 후 malloc/new) — 결정론적 메모리
- **재귀 금지** — 스택 경계 보장
- single entry / single exit, 포인터 제한
- 출처: ISO 26262-6 §8 / MISRA C / Power of Ten / BARR-C (등급 ⓦ — 원문 대조 필요)

## 4. 받는 법 (요약)

1. **무료·재배포 가능** (Google C++ Style Guide 등) — 위 URL 에서 받아 `references/<vendor>/<product>/` 또는 링크 유지.
2. **유료** (ISO 26262·MISRA) — 조직 라이선스로 받아 `references/local/`(gitignore). repo 미배포.
3. **복제 금지·불명확** (AUTOSAR·CERT) — URL 링크만, 사본 미보관.
4. 인용은 [domains/standards.md](domains/standards.md) 형식 (개정판·절·page·accessed 일자).
