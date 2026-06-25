# 외부 참조 — 표준 / 인증 Add-on (External Reference — Standards)

> 코어 [handling.md](../handling.md) 의 도메인 sub-file. 코어 §1~12 규칙(특히 §8 강한 단정어)에 표준/인증 특화 taxonomy·경로·인용 형식·추정 사례·grep 을 더한다.

**트리거**: RFC 인용, IEEE std 인용, ISO/IEC 표준 인용, JEDEC/AEC-Q 인용, 인증 요구사항(KC/CE/FCC 등) 작업.

## 1. 매뉴얼 종류 taxonomy

| 종류 | 발행 단체 | 예 |
| --- | --- | --- |
| **RFC** (Request for Comments) | IETF | RFC 793 (TCP), RFC 7519 (JWT) |
| **IEEE 표준** | IEEE Standards Association | IEEE 802.11 (Wi-Fi), IEEE 754 (float), IEEE 1588 (PTP) |
| **ISO / IEC 표준** | ISO / IEC | ISO 26262 (자동차 기능안전), IEC 61508 (산업 기능안전) |
| **JEDEC** | JEDEC | JESD22 (반도체 신뢰성), JESD79 (DDR) |
| **AEC-Q** | Automotive Electronics Council | AEC-Q100 (IC), AEC-Q200 (passive) |
| **인증 / 규제 spec** | 국가/지역 인증 기관 | KC(한국), CE(EU), FCC(미국), UL(안전) |
| **ROS REP** | ROS community | REP-103, REP-105 (ros2.md 와 중복 — 양쪽 활성) |

## 2. 보관 경로

`references/standards/<body>/<doc-id>/` (예: `references/standards/ieee/802.11/`, `references/standards/iso/26262/`). 유료 표준은 사내 라이선스 사본 + `LICENSE.md` 에 출처·라이선스 메타데이터 기록.

## 3. 인용 형식

```
[IEEE 802.11-2020, §10.3, page 421](references/standards/ieee/802.11/ieee802.11-2020.pdf)
[ISO 26262-3:2018, §7.4.2, "Hazard analysis and risk assessment"](references/standards/iso/26262/iso26262-3-2018.pdf)
[RFC 7519, §4.1.1 "iss" Claim, accessed 2026-05-21](https://datatracker.ietf.org/doc/html/rfc7519)
[AEC-Q100-Rev-H, Table 2 "Temperature Grade"](references/standards/aec-q/aec-q100-rev-h.pdf)
```

## 4. 흔한 추정 단정 사례

- 표준 번호만 인용하고 절/page 미명시 ("IEEE 802.11 위반" → 어떤 절?)
- 개정판 미명시 — 표준은 개정 시 의미가 바뀜 (RFC 의 obsoleted by, ISO 의 :2011 vs :2018)
- 인증 인증서("AEC-Q100 인증")와 spec 준수 혼동 — 인증은 별도 절차
- "표준 위반" 단정을 1차 표준 문서 인용 없이 사용 — 코어 §8 금지 단어 룰 위반
- 표준의 normative(강제) vs informative(참고) 절 혼동
- 표준 인용 시 accessed 일자 누락 — errata / amendment 후속 발행 가능

## 5. 1차 source 확인 절차

- 표준 문서 PDF 또는 공식 발행 페이지 (IEEE Xplore, ISO OBP, IETF datatracker)
- 개정판 / amendment / errata 함께 다운로드
- 인용 시 절·page (또는 RFC/IEEE std 번호) + 개정판 + accessed 일자 함께
- 인증 spec 은 인증 기관 공식 문서만 1차 source — 컨설팅 자료/블로그는 ⓦ
- 사내/외부 컨설팅 작성 "준수 매트릭스" 는 ⓦ — 원문과 별도 검증

## 6. 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# 표준 인용 형식 (번호 + 개정 + 절/page)
grep -oE "\[(RFC [0-9]+|IEEE [0-9]+(-[0-9]+)?|ISO [0-9]+(-[0-9]+)?:[0-9]{4}|IEC [0-9]+|JESD[0-9]+|AEC-Q[0-9]+)[^]]*\]" $TARGET

# "표준 위반" 단정 시 표준 인용 첨부 여부
grep -nE "위반|준수|compliance|non-compliance" $TARGET | grep -vE "RFC [0-9]+|IEEE [0-9]+|ISO [0-9]+|IEC [0-9]+|page [0-9]+|§"

# accessed 일자 (URL 인용 시)
grep -oE "accessed [0-9]{4}-[0-9]{2}-[0-9]{2}" $TARGET
```
