# 외부 참조 문서 처리 — 코어 (External Reference Handling — Core)

> **본 파일은 지시용.** 외부 벤더 매뉴얼·datasheet·SDK(Software Development Kit) 문서·프로토콜·표준(IEEE·ISO·RFC·REP 등)을 **어디에 보관하고, 어떻게 인용하며, 어떻게 검증할지** 정하는 self-contained 코어 단일 근원(SSOT / Single Source of Truth). 도메인 특화 규칙은 선택 sub-file 로 분리한다 (→ [§13](#13-도메인-add-on-선택)).

본 코어는 self-contained 다 — 본문 외 가이드라인·자동화 도구·Skill 의존 0. 본 파일 위반 시 거짓 단정 누적으로 다중 정정 라운드·토큰 낭비·신뢰 손상이 발생한다 (§12 참조).

코딩 표준(ISO 26262·MISRA·Google C++ 등)의 라이선스·배포·규칙 digest 는 [coding_standards.md](coding_standards.md) 참조 (코어와 함께 설치).

## 설치

본 번들 폴더(`external_reference/`)의 `install.sh` 로 설치한다:

```bash
cd external_reference && ./install.sh <타깃-프로젝트-루트> [도메인...]
```

스크립트가 코어(`handling.md`)를 `docs/claude_guideline/external_reference/` 로, 선택한 도메인(`domains/<도메인>.md`)을 `docs/claude_guideline/external_reference/domains/` 로 복사하고, 등록 스니펫을 타깃 `CLAUDE.md` 에 append 한다 (덮어쓰기 아님). 도메인 인자를 생략하면 코어만 설치한다.

- **참조 문서 보관 산출물**: 프로젝트 루트 `references/<vendor>/<product>/`
- **활성화 게이트**: 본 파일이 `docs/claude_guideline/external_reference/handling.md` 경로에 없으면 본 룰은 비활성.

## 트리거

다음 키워드·패턴 등장 시 자동 활성:

- 키워드: `datasheet`, `데이터시트`, `manual`, `매뉴얼`, `spec`, `사양`, `Operation Conditions`, `Electrical Characteristics`, `REP-`, `IEEE`, `ISO`, `IEC`, `RFC`, `User Manual`, `Family Manual`, `Reference Manual`, `Application Note`, `Errata`
- 강한 단정어: `위반`, `초과`, `무보증`, `non-compliance`, `violation`, `규격 위반`, `인증 위반` (§8 사용 조건 점검 강제)
- 외부 1차 source 의존이 명백한 작업: 페리페럴 설정, 통신 프로토콜, 센서 spec 의존 알고리즘, 표준 인증 요구

## 1. 보관 위치

- **저장 폴더가 없으면 만든다 (승인 불요)** — 첫 보관·다운로드 시 `references/<vendor>/<product>/` 경로를 자동 생성한다.
- **저장 폴더가 표준과 다르면 표준 경로로 정규화한다** — 기존 참조 문서가 비표준 위치(예: `docs/references/`, `docs/manual/`, `manuals/`, `datasheets/`, 단수 `reference/`, 루트 산재)에 있으면 `references/<vendor>/<product>/` 로 이동하고, 그 문서를 가리키던 인용·링크를 **모두 갱신**한 뒤 이동 내역을 작업 보고에 남긴다. 모든 프로젝트가 동일 경로 규약을 따르게 하기 위함이다.
- 외부 벤더 매뉴얼은 `references/<vendor>/<product>/` 하위에 보관한다.
- 모듈에 강하게 결합된 매뉴얼은 모듈 내부(예: `<모듈경로>/references/`)에 둘 수도 있으며, 위치는 모듈 CLAUDE.md 가 결정한다.
- 원본 파일명을 가급적 유지하되, 경로·검색이 불편하면 `<vendor>_<product>_<version>.pdf` 형식으로 정규화한다.
- PDF 가 우선이며, 변환본(텍스트 추출 등)은 원본과 함께 보관한다 (예: `pdftotext -layout` 결과를 `.txt` 로 같이 저장).
- 표준 단체 발행 문서는 `references/standards/<body>/<doc-id>/` 에 보관한다.

## 2. 인용 규칙

- 코드 주석·문서에서 인용 시 **문서명·섹션·페이지**(또는 표준의 경우 `RFC-N`·`REP-N`·`IEEE-N`)를 명시한다.
- 인용한 문서의 상대 경로를 함께 적어 추적 가능하게 한다.
- **강제 인용 형식**: `[문서명 v버전, Table N, page P](경로/파일명.pdf)`
  - 벤더 datasheet 예: `[<Vendor> <Product> DataSheet vX.Y, Table N, page P](references/<vendor>/<product>/datasheet.pdf)`
  - 표준 예: `[IEEE 802.11-2020, §10.3, page 421](references/standards/ieee/802.11/ieee802.11-2020.pdf)`
- 외부 매뉴얼에 의존하는 상수·환산식·시퀀스 코드는 인용을 해당 상수/함수 바로 위에 둔다.
- 매뉴얼 버전 차이로 동작이 달라질 수 있는 부분은 **참조한 문서 버전**을 함께 명시한다.
- URL 만 가능한 표준 문서(예: ROS REP, RFC)는 URL + 접근 일자(`accessed YYYY-MM-DD`)를 함께 기록한다.

## 3. source 분리 (가장 중요)

본 §은 본 룰의 핵심이며 모든 도메인 Add-on 에 공통 적용된다.

- **벤더 SDK / 드라이버 default ≠ silicon(또는 device) spec ≠ 표준 명세** ← 본 룰의 핵심
- SDK docstring(권장 사용 범위) ≠ datasheet(silicon spec) ≠ 표준 권장값. 셋은 **별도 검증 항목**으로 다룬다.
- SDK 매크로(예: `<PERIPHERAL>_<PARAM>_MAX`)에서 datasheet spec 추론 **금지**.
- 드라이버 default 파라미터(예: ROS2 드라이버 `frequency` default)에서 1차 spec 추론 **금지**.
- SDK 권장 범위(docstring 의 "Range = [N, M]")도 datasheet 와 별개. stale 가능성 항상 의심.

### 3.1 역방향 비약 경고 (1차 source → 운영점 해석)

- ❌ "TYP = 권장값" 비약: datasheet 의 TYP 컬럼은 **typical 측정 기준점**(대표 silicon, 25°C, 표준 조건). datasheet 가 명시적으로 "recommended operating point" 라 표기하지 않은 한 단정 금지.
- ❌ "Min/Max 안에 들어오면 무조건 OK" 비약: Min~Max 는 spec 보장 범위. 단 측정 조건(온도·전압·부하 등 footnote)이 충족돼야 함. footnote 미인용 시 ⓦ 격하.
- ❌ "SDK / 드라이버 default 수정값 = 1차 spec TYP 일치 = 합리적 설정" 비약: 일치는 **우연**일 수 있음. 의도적 정렬 단정은 commit 메시지·PR(Pull Request) 설명 등 별도 증거 필요.

### 3.2 1차 source 종류는 다중이며 별도 보관·검증

| source 종류 | 다루는 정보 | 검증 책임 |
| --- | --- | --- |
| **DataSheet** (벤더) | pinout, package, 전기 특성 | spec 단정 |
| **User / Family / Reference Manual** (벤더) | register-level 동작, IP 챕터, DMA/IRQ 토폴로지 | 동작 단정 |
| **SDK / 드라이버 문서** (벤더) | 권장 사용 범위, API 시그니처, default | SW 권장 |
| **Application Note / Errata** (벤더) | 응용 사례, 알려진 버그, 회피 절차 | 회피 / 보완 |
| **표준 명세** (IEEE·ISO·IEC·RFC·REP) | 통신·좌표계·알고리즘·보안 표준 | 호환성 / 인증 |
| **알고리즘 원문 논문** (학회·저널) | 알고리즘 의도·가정·권장 파라미터 | 알고리즘 spec |

한 PDF 에 모든 정보가 있지 않다. 도메인별 taxonomy 는 선택 sub-file(§13) 참조.

## 4. 추정 금지 · 실측 검증

- 1차 source 의 모호한 표현(단위 미표기 수치 등)을 **추정으로 단정 적용하지 않는다**.
- 의미가 불분명하면: ① 벤더 추가 자료(Application Note·FAQ·Errata) 확인 → ② 벤더 기술지원·표준 errata 문의 → ③ 실측 검증.
- 실측이 1차 source 와 다를 때: **실측을 신뢰**하고 차이의 원인 가설을 주석에 남긴다.
- 모호한 수치를 단정 적용했다가 silent bug 발견 시 수정 이력(가설·사유)을 코드 또는 모듈 CLAUDE.md 에 보존한다.

## 5. 라이선스 / 외부 공개

- 1차 source PDF 를 공개 저장소에 commit 하기 전 라이선스·저작권을 확인한다.
- 벤더 NDA(Non-Disclosure Agreement)·재배포 금지 문서는 별도 비공개 저장소, `references/local/`(gitignore 대상), 또는 공식 URL 링크 대체 중 하나로 처리한다.
- 공개 가능한 datasheet 는 가능하면 **공식 URL 링크**를 우선하고 로컬 사본은 보조로 둔다(벤더 갱신 시 stale 위험 감소).
- 표준 단체 문서(IEEE·ISO 등)는 유료가 많아 공식 URL + 사내 라이선스 사본 경로를 명시한다.

## 6. 1차 source 누락 / 모호 처리

- 1차 source 가 없거나 핵심 항목이 모호하면 모듈 CLAUDE.md 의 "Open Question" 또는 이슈 트래커에 기록한다.
- 임시 추정값을 써야 한다면 **사유·승인·정리 일정** 세 가지를 기록한다 (정공법 우선, 우회는 한시적).
- 추정값을 코드에 둘 때는 `// TODO(YYYY-MM-DD): 1차 source 확인 또는 실측 검증 필요` 주석을 함께 남긴다.

## 7. 작업 전 체크리스트 (Pre-Work Checklist)

- [ ] 본 작업이 외부 spec·datasheet·표준·인증에 의존하는가?
- [ ] 의존한다면 1차 source(PDF 또는 표준 문서)가 정해진 위치에 있는가?
- [ ] 없으면 사용자에게 source 제공 요청 → **받기 전 spec 관련 결론 보류**
- [ ] 기존 문서·AI(Artificial Intelligence) 보고서에 검증 안 된 spec 주장이 있는가? (있으면 ⓦ/⚠ 격하)
- [ ] 분석 범위 결정 — 1차 source 의존 부분 vs 코드 분석 부분 **명확히 분리**

## 8. 작업 중 체크리스트 (In-Progress Checklist)

- [ ] 검증 등급 표시: **✓**(1차 source 직접) / **ⓦ**(다른 보고만) / **⚠**(UNVERIFIED)
- [ ] ✓ 표시 = file:line 인용 또는 source:page 인용 필수
- [ ] SDK / 드라이버 default 인용 시 → "**silicon(또는 device) spec 아님**" 명시
- [ ] 강한 단정어 사용 룰 (강제):
  - **금지 단어**(primary source 없이): `위반`, `초과`, `무보증`, `non-compliance`, `violation`, `fail`, `규격 위반`, `인증 위반`
  - **사용 조건**: primary source 직접 인용 + page/table 번호(또는 RFC/REP/IEEE 번호 + section) 첨부 시에만
  - primary source 없을 시: `추정`, `의심`, `미확인`, `확인 필요` 등 약한 표현

## 9. 작업 후 체크리스트 (Post-Work Checklist)

- [ ] 모든 **✓** 항목 = 인용(file:line 또는 source:page) 있는가?
- [ ] SDK / 드라이버 default → 1차 spec 비약 있는가? 있으면 **⚠** 로 격하
- [ ] "위반 / fail / non-compliance" 단정어 항목 = primary source 첨부?
- [ ] 미검증 추론을 "✓" 로 표시한 곳 없는가?
- [ ] 정정 이력(vN → vN+1) 명시 — 무엇을 왜 정정?
- [ ] 다음 라운드 필요한 **⚠** 항목을 사용자에게 명시
- [ ] 자체 점검 §14 grep 통과

## 10. 검증 등급 (강제 표기)

| 표기 | 의미 | 허용 단정어 |
| --- | --- | --- |
| **✓** | 1차 source 직접 확인(file:line 또는 source 페이지) | 강한 단정 OK |
| **ⓦ** | 다른 워커·AI 보고만, lead 직접 미확인 | 약한 표현("보고됨", "주장됨") |
| **⚠** | 추론·추측, 1차 source 없음 | "추정", "의심", "확인 필요" 만 |

## 11. 1차 source 다운로드 표준 절차

1. **`references/` 먼저 확인** (이미 있으면 재다운로드 X)
2. WebSearch 로 공식 PDF URL 확인
3. 폴더 생성 후 다운로드: `mkdir -p references/<vendor>/<product>/ && curl -sSL -A "Mozilla/5.0" -o references/<vendor>/<product>/<file>.pdf <URL>` (벤더 차단 시 mouser·farnell·alldatasheet·digikey 미러)
4. **차단 시 사용자에게 수동 다운로드 요청** → 정해진 경로에 배치
5. **검증**: `file <file>.pdf` 로 PDF 매직 바이트 확인(HTML 차단 페이지 아닌지)
6. 텍스트 추출: `pdftotext -layout <file>.pdf <file>.txt`
7. spec 검색: `grep -in "<parameter_name>\|Operation Conditions\|Electrical Characteristics" <file>.txt`
8. spec 표 컨텍스트(전후 50~100줄) 발췌 후 분석
9. 인용 시 §2 형식 강제
10. 유료 표준은 사내 라이선스·단체 회원 계정으로 다운로드, 라이선스 메타데이터를 `references/standards/<body>/LICENSE.md` 에 기록

## 12. 본 룰 위반 시 일반 패턴 (공통 시퀀스)

1. AI·작업자가 SDK 매크로·드라이버 default·docstring 권장값을 보고
2. **1차 source spec 으로 비약** → "현재 동작 값이 spec 위반" 거짓 단정
3. downstream 거짓 결론 누적: "외부 표준 위반", "spec 무보증", "제품·시스템 위험"
4. 후속 검증·협업 워크플로(병렬 워커, 외부 도구)의 **전제 오염** — 모두 거짓 단정을 사실로 수용
5. 다중 정정 라운드, 토큰·시간 낭비, 사용자 신뢰 손상
6. 사용자가 1차 source PDF 직접 다운로드 후 검증 → spec 안쪽 정상, default 는 SW 보수 한계였을 뿐

**핵심**: "벤더 SDK / 드라이버 default ≠ 1차 spec ≠ 표준 명세" 를 항상 의식하고 1차 source 직접 확인을 강제. source 제공 가능성 확인 → 받기 전 단정 보류.

## 13. 도메인 Add-on (선택)

도메인 특화 규칙(매뉴얼 taxonomy·보관 경로·인용 형식·흔한 추정 사례·자체 점검 grep)은 `domains/` 하위의 **도메인별 sub-file** 로 둔다. 도메인은 계속 추가되며, 해당 도메인 작업 시에만 선택 설치·참조한다.

| sub-file | 도메인 | 트리거 예 |
| --- | --- | --- |
| `domains/embedded-reference.md` | 임베디드 | `ISR(`, `NVIC_`, `IRQHandler`, FreeRTOS API, MCU datasheet/Family Manual |
| `domains/ros2-reference.md` | ROS2 | `package.xml`, `rclpy`/`rclcpp`, `.launch.py`, REP 인용, sensor driver |
| `domains/opencv.md` | OpenCV / Computer Vision | `cv2.`/`cv::`, `calibrateCamera`, distortion 모델, 카메라 calibration |
| `domains/standards.md` | 표준 / 인증 | RFC·IEEE·ISO/IEC·JEDEC·AEC-Q 인용, KC/CE/FCC 인증 요구 |

새 도메인 추가 = `domains/<도메인>.md` 한 파일 작성(아래 5요건) + 본 표 한 줄 추가. 코어 골격은 불변.

**새 Add-on 5요건** (sub-file 작성 시 모두 충족):
1. 트리거(자동 감지 키워드·파일 패턴)
2. 매뉴얼 종류 taxonomy(1차 source 종류 + 검증 책임)
3. 보관 경로 + 인용 형식(§2 형식 + 도메인 식별자)
4. 흔한 추정 단정 사례 3개 이상
5. 1차 source 확인 절차 + 자체 점검 grep

## 14. 자체 점검 grep (코어)

```bash
TARGET=<분석 대상 .md>

# 1. 강제 인용 형식 (page/Table/경로 명시) — §2
grep -oE "\[[^]]+(page [0-9]+|Table [0-9]+|§[0-9.]+|RFC [0-9]+|REP-[0-9]+)\]\([^)]+\.(pdf|html|md)\)" $TARGET

# 2. 검증 등급 표기 (✓/ⓦ/⚠) — §10
grep -oE "(^|[^a-zA-Z])(✓|ⓦ|⚠)( |$)" $TARGET | sort -u

# 3. 강한 단정어가 primary source 없이 사용된 의심 라인 — §8
grep -nE "위반|초과|무보증|non-compliance|violation|fail" $TARGET | grep -vE "page [0-9]+|Table [0-9]+|RFC [0-9]+|IEEE [0-9]+|ISO [0-9]+|datasheet|\.pdf"

# 4. SDK / 드라이버 default 인용 시 1차 spec 분리 명시 — §3
grep -nE "_MAX|_MIN|#define|default|기본값" $TARGET | grep -vE "silicon|device|SDK 권장|datasheet 아님|소스 코드"

# 5. 비표준 저장 경로 인용 탐지 (루트 references/ 외) — §1 정규화 대상
grep -nE "\]\((docs/references|docs/manual|manual|manuals|datasheets|reference)/" $TARGET
```

도메인 sub-file 설치 시 각 sub-file 의 grep 도 함께 통과해야 한다.

## 15. 변경 절차

본 파일은 SSOT 이므로 변경 시 **사용자 승인 필수**. 변경 후 VERSION(semver)·CHANGELOG 갱신, 설치된 다운스트림 통보, 자체 점검 §14 통과 확인.

## 룰 (요약)

1. **self-contained** — 본 코어 1개로 자체 완결. 외부 가이드라인·도구·Skill 의존 0.
2. **1차 source 직접 확인 강제** — SDK / 드라이버 default / docstring 에서 1차 spec 추론 금지 (§3).
3. **인용 형식 강제** — `[문서명 v버전, Table N, page P](경로)` (§2).
4. **검증 등급 ✓/ⓦ/⚠ 표기 의무** — 모든 spec 주장에 등급 부착 (§10).
5. **강한 단정어 사용 조건** — `위반/fail/non-compliance` 는 primary source 인용 첨부 시에만 (§8).
6. **추정 금지 · 실측 검증** (§4).
7. **다중 1차 source 분리** — DataSheet ≠ User Manual ≠ SDK ≠ 표준 ≠ 논문 (§3.2).
8. **도메인 Add-on 은 선택 sub-file** — 5요건 충족 시 추가 (§13).
9. **자체 점검 grep 통과 의무** — §14 + 활성 sub-file grep.

---

**VERSION**: 2.0.0 (코어 분리판 — external_reference_handling.md v1.0.0 기반, 도메인 Add-on sub-file 분리)
