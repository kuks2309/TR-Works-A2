# 외부 참조 문서 처리 (External Reference Handling)

외부 벤더의 매뉴얼·데이터시트·SDK 문서·프로토콜 명세·표준 단체 발행 문서(IEEE / ISO / RFC / REP 등)를 **어디에 보관하고, 어떻게 인용하며, 어떻게 검증할지** 정하는 단일 근원(SSOT / Single Source of Truth). 본 룰 위반 시 거짓 단정 누적으로 다중 정정 라운드 / 토큰 낭비 / 신뢰 손상이 발생한다 (사례 §12 참조).

본 파일은 self-contained 다 — 본문 외 가이드라인 파일 / 자동화 도구 / Skill 의존 0. 본 파일 1개만 설치돼 있으면 모든 룰이 동작한다.

## 설치 위치

- **본 파일**: 대상 프로젝트의 `docs/claude_guideline/external_reference_handling.md` 에 배치
- **참조 문서 보관 산출물**: `docs/references/<vendor>/<product>/` (계층) 또는 `references/` (단일 칩 단순 프로젝트)
- **인용 기록 위치**: 코드 주석 또는 모듈별 `docs/` (해당 매뉴얼이 강하게 결합된 모듈)

본 파일이 `docs/claude_guideline/external_reference_handling.md` 위치에 없으면 본 룰은 활성화되지 않는다. 새 프로젝트 적용 시 본 파일을 위 경로로 복사하는 것이 첫 단계.

## 트리거

사용자 메시지 또는 작업 컨텍스트에 다음 키워드 / 패턴 등장 시 자동 활성:

- 키워드: `datasheet`, `데이터시트`, `manual`, `매뉴얼`, `spec`, `사양`, `Operation Conditions`, `Electrical Characteristics`, `REP-`, `IEEE`, `ISO`, `IEC`, `RFC`, `User Manual`, `Family Manual`, `Reference Manual`, `Application Note`, `Errata`
- 강한 단정어: `위반`, `초과`, `무보증`, `non-compliance`, `violation`, `규격 위반`, `인증 위반` (본 룰 §8 의 사용 조건 점검 강제)
- 외부 1차 source 의존이 명백한 작업: 페리페럴 설정, 통신 프로토콜, 센서 spec 의존 알고리즘, 표준 인증 요구

## 흐름도 (한눈에)

```
[외부 참조 문서 의존 작업 도착]
   ↓
[Step 1] 작업 전 체크리스트 §7              ────→  ✓ 1차 source 존재 / 미충족 시 진행 보류
   ↓
[Step 2] 도메인 자동 감지 (§13 Add-on)      ────→  ✓ 임베디드 / ROS2 / OpenCV / 표준 분류
   ↓
[Step 3] 1차 source 확보 §11                ────→  ✓ 다운로드 + magic byte 검증 + 텍스트 추출
   ↓
[Step 4] source 분리 §3 (가장 중요)         ────→  ✓ SDK / 드라이버 default ≠ silicon spec
   ↓
[Step 5] 인용 §2 (강제 형식)                ────→  ✓ [문서명 v버전, Table N, page P](경로)
   ↓
[Step 6] 검증 등급 §10 표기 (✓/ⓦ/⚠)        ────→  ✓ 모든 주장에 등급 부착
   ↓
[Step 7] 작업 중 체크리스트 §8              ────→  ✓ 강한 단정어 사용 조건 충족
   ↓
[Step 8] 작업 후 체크리스트 §9              ────→  ✓ 정정 이력 / 다음 라운드 ⚠ 항목 명시
   ↓
[Step 9] 자체 점검 grep §16                 ────→  ✓ 인용 형식·등급 표기·금지 단어 통과
   ↓
[완료]
```

---

## 1. 보관 위치

- 외부 벤더 매뉴얼은 `docs/references/<vendor>/<product>/` 하위에 보관한다.
- 모듈에 강하게 결합된 매뉴얼은 모듈 내부(예: `<모듈경로>/docs/references/`)에 둘 수도 있으며, 위치는 모듈 CLAUDE.md 가 결정한다.
- **간단 단일 프로젝트** (예: 단일 칩 임베디드) 에서는 루트 `references/` 폴더 단순 사용도 허용 — 모듈 CLAUDE.md 또는 본 절에 명시.
- 원본 파일명을 가급적 유지하되, 경로 / 검색이 불편하면 `<vendor>_<product>_<version>.pdf` 형식으로 정규화한다.
- PDF 가 우선이며, 변환본(텍스트 추출 등)은 원본과 함께 보관한다 (예: `pdftotext -layout` 결과를 `.txt` 로 같이 저장).
- 표준 단체 발행 문서는 `docs/references/standards/<body>/<doc-id>/` (예: `docs/references/standards/ieee/802.11/`)에 보관한다.

## 2. 인용 규칙

- 코드 주석 / 문서에서 인용 시: **문서명·섹션·페이지** (또는 표준의 경우 `RFC-N` / `REP-N` / `IEEE-N`) 를 명시한다.
- 인용한 문서의 상대 경로를 함께 적어 추적 가능하게 한다.
- **강제 인용 형식**: `[문서명 v버전, Table N, page P](경로/파일명.pdf)`
  - 벤더 datasheet 예: `[<Vendor> <Product> DataSheet vX.Y, Table N, page P](docs/references/<vendor>/<product>/datasheet.pdf)`
  - 표준 문서 예: `[IEEE 802.11-2020, §10.3, page 421](docs/references/standards/ieee/802.11/ieee802.11-2020.pdf)` 또는 `[REP-105, Section "Frame Authority"](https://www.ros.org/reps/rep-0105.html)`
- 외부 매뉴얼에 의존하는 상수·환산식·시퀀스 코드는 인용을 해당 상수/함수 바로 위에 둔다.
- 매뉴얼 버전 차이로 동작이 달라질 수 있는 부분은 **참조한 문서 버전**을 함께 명시한다.
- URL 만 가능한 표준 문서 (예: ROS REP, RFC) 는 URL + 접근 일자 (`accessed YYYY-MM-DD`) 함께 기록.

## 3. source 분리 (가장 중요)

본 §은 본 룰의 핵심이다. 모든 도메인 Add-on §14 에서 공통 적용.

- **벤더 SDK / 드라이버 default ≠ silicon (또는 device) spec ≠ 표준 명세** ← 본 룰의 핵심
- SDK docstring (권장 사용 범위) ≠ datasheet (silicon spec) ≠ 표준 권장값
- 셋은 **별도 검증 항목** 으로 다룬다.
- SDK 매크로 (예: 어느 벤더 SDK 의 `<PERIPHERAL>_<PARAM>_MAX` 형태 매크로) 에서 datasheet spec 추론 **금지**.
- 드라이버 default 파라미터 (예: ROS2 드라이버 `frequency` default, OpenCV 함수 default flags) 에서 1차 spec 추론 **금지**.
- SDK 권장 범위 (예: docstring 의 "Range = [N, M]") 도 datasheet 와 별개. stale 가능성 항상 의심.

### 3.1 역방향 비약 경고 (1차 source → 운영점 해석)

1차 source 를 읽고 운영점을 해석할 때 다음 비약을 **금지**:

- ❌ "TYP = 권장값" 비약: datasheet 의 TYP 컬럼은 **typical 측정 기준점** (대표 silicon, 25°C, 표준 조건). datasheet 가 명시적으로 "recommended operating point" 라 표기하지 않은 한 "TYP = 권장 운영값" 으로 단정 금지.
- ❌ "Min/Max 안에 들어오면 무조건 OK" 비약: Min~Max 는 spec 보장 범위. 단 측정 조건 (postcalibration, ENRMS, ripple, 온도, 전압, 부하 등 footnote) 이 충족돼야 함. footnote 미인용 시 ⓦ 격하.
- ❌ SDK / 드라이버 default 수정값 = 1차 spec TYP 와 일치 = "합리적 설정" 비약: 매크로 / default 수정의 의도와 1차 spec TYP 일치는 **우연** 일 수 있음. "수정자가 1차 source 보고 의도적으로 TYP 정렬" 단정은 commit 메시지 / PR (Pull Request) 설명 등 별도 증거 필요.

### 3.2 1차 source 종류는 다중이며 별도 보관·검증

1차 source 종류와 각각의 검증 책임:

| source 종류 | 다루는 정보 | 검증 책임 |
|---|---|---|
| **DataSheet** (벤더 발행) | pinout, package, 전기 특성 (Operation Conditions, Electrical Characteristics), 일반 사양 | spec 단정 |
| **User Manual / Family Manual / Reference Manual** (벤더 발행) | 모듈 / peripheral / API 의 register-level 동작, IP 챕터, SR 라우팅, DMA/IRQ 토폴로지 | 동작 단정 |
| **SDK / 드라이버 문서** (벤더 발행) | 권장 사용 범위, API 시그니처, default 값, sample 코드 | SW 권장 |
| **Application Note / Errata** (벤더 발행) | 응용 사례, 알려진 버그, 회피 절차 | 회피 / 보완 |
| **표준 명세** (IEEE / ISO / IEC / RFC / REP 등) | 통신 / 좌표계 / 알고리즘 / 보안 등 도메인 표준 | 호환성 / 인증 |
| **알고리즘 원문 논문** (학회 / 저널) | 알고리즘 의도, 가정, 유도 과정, 권장 파라미터 | 알고리즘 spec |

한 PDF 에 모든 정보가 있지 않다. 도메인별 source taxonomy 와 흔한 추정 사례는 §14 Add-on 참조.

## 4. 추정 금지 · 실측 검증

- 1차 source 의 모호한 표현(예: "정밀도 1/N", "최대 N pulse", 단위 미표기 수치)을 **추정으로 단정 적용하지 않는다**.
- 의미가 불분명하면 다음 순서로 처리:
  1. 벤더의 추가 자료(예: Application Note, FAQ, Errata) 확인
  2. 벤더 기술지원 / 표준 단체 errata 문의
  3. 실측 / 실험으로 검증
- 실측이 1차 source 표현과 다를 때: **실측을 신뢰**하고, 차이의 원인 가설을 주석에 남긴다.
- 모호한 수치를 단정 적용했다가 silent bug 가 발견되면 수정 이력(v1/v2/v3 가설과 사유)을 코드 또는 모듈 CLAUDE.md 에 보존한다.

## 5. 라이선스 / 외부 공개

- 1차 source PDF 를 GitHub 공개 저장소에 commit 하기 전 라이선스 / 저작권을 확인한다.
- 벤더 NDA (Non-Disclosure Agreement) / 재배포 금지 문서는 다음 중 하나로 처리:
  - 별도 비공개 저장소
  - `docs/references/local/` (gitignore 대상)
  - 공식 URL 링크로 대체 (저장소에는 위치 메타데이터만)
- 공개 가능한 datasheet 는 가능하면 **공식 URL 링크**를 우선하고, 로컬 사본은 보조로 둔다 (벤더가 매뉴얼을 갱신할 때 stale 사본 의존 위험 감소).
- 표준 단체 문서 (IEEE / ISO 등) 는 유료가 많아 공식 URL + 사내 라이선스 사본 경로 명시.

## 6. 1차 source 누락 / 모호 처리

- 1차 source 가 없거나 핵심 항목이 모호한 부분은 모듈 CLAUDE.md 의 "Open Question" 또는 이슈 트래커에 기록한다.
- 모호한 항목에 임시 추정값을 사용해야 한다면 **사유 · 승인 · 정리 일정** 세 가지를 기록한다 (정공법 우선, 우회는 한시적):
  - 사유: 왜 우회가 필요한가
  - 승인: 누가 승인했는가 (사용자 / 팀 리더)
  - 정리 일정: 언제까지 정공법으로 대체할 것인가
- 추정값을 코드에 둘 때는 `// TODO(YYYY-MM-DD): 1차 source 확인 또는 실측 검증 필요` 형식의 주석을 함께 남긴다.

## 7. 작업 전 체크리스트 (Pre-Work Checklist)

분석 작업 시작 전 반드시 확인. 미충족 항목은 **진행 전 해결**:

- [ ] 본 작업이 외부 spec / datasheet / 표준 / 인증에 의존하는가?
- [ ] 의존한다면 1차 source (PDF 또는 표준 문서) 가 정해진 위치 (`docs/references/<vendor>/<product>/` 또는 `references/`) 에 있는가?
- [ ] 없으면 사용자에게 source 제공 요청 → **받기 전 spec 관련 결론 보류**
- [ ] 기존 문서 / AI 보고서에 검증 안 된 spec 주장이 있는가? (있으면 ⓦ/⚠ 격하 표시)
- [ ] 분석 범위 결정 — 1차 source 의존 부분 vs 코드 분석 부분 **명확히 분리**
- [ ] 도메인 자동 감지 §13 — 활성 Add-on 명시

## 8. 작업 중 체크리스트 (In-Progress Checklist)

각 주장 작성 시:

- [ ] 검증 등급 표시: **✓** (1차 source 직접) / **ⓦ** (다른 보고만) / **⚠** (UNVERIFIED)
- [ ] ✓ 표시 = file:line 인용 또는 source:page 인용 필수
- [ ] SDK / 드라이버 default 인용 시 → "**silicon (또는 device) spec 아님**" 명시
- [ ] 강한 단정어 사용 룰 (강제):
  - **금지 단어** (primary source 없이): `위반`, `초과`, `무보증`, `non-compliance`, `violation`, `fail`, `규격 위반`, `인증 위반`
  - **사용 조건**: primary source 직접 인용 + page/table 번호 (또는 RFC/REP/IEEE 번호 + section) 첨부 시에만
  - primary source 없을 시: `추정`, `의심`, `미확인`, `확인 필요` 등 약한 표현

## 9. 작업 후 체크리스트 (Post-Work Checklist)

문서 완성 / 정정 라운드 종료 전:

- [ ] 모든 **✓** 항목 = 인용 (file:line 또는 source:page) 있는가?
- [ ] SDK / 드라이버 default → 1차 spec 비약 있는가? 있으면 **⚠** 로 격하
- [ ] "위반 / fail / non-compliance" 단정어 사용 항목 = primary source 첨부?
- [ ] 미검증 추론을 "✓" 로 표시한 곳 없는가?
- [ ] 정정 이력 (vN → vN+1) 명시 — 무엇을 왜 정정?
- [ ] 다음 라운드 필요한 **⚠** 항목을 사용자에게 명시 (source 추가 다운로드 요청 등)
- [ ] 자체 점검 §16 grep 모두 통과

## 10. 검증 등급 (강제 표기)

| 표기 | 의미 | 허용 단정어 |
|------|------|---------------|
| **✓** | 1차 source 직접 확인 (코드 file:line 또는 source 페이지) | 강한 단정 OK |
| **ⓦ** | 다른 워커 / AI 보고만, lead 직접 미확인 | 약한 표현 ("보고됨", "주장됨") |
| **⚠** | 추론 / 추측, 1차 source 없음 | "추정", "의심", "확인 필요" 만 |

## 11. 1차 source 다운로드 표준 절차

1. **`docs/references/` 또는 `references/` 먼저 확인** (이미 있으면 재다운로드 X)
2. WebSearch 로 공식 PDF URL 확인
3. 다운로드 시도:
   - `curl -sSL -A "Mozilla/5.0" -o docs/references/<vendor>/<product>/<file>.pdf <URL>`
   - 벤더 사이트 직접 차단 시 미러 시도 (mouser, farnell, alldatasheet, digikey 등)
4. **차단 시 사용자에게 수동 다운로드 요청** → 사용자가 정해진 경로에 배치
5. **검증**: `file docs/references/.../<file>.pdf` 로 PDF 매직 바이트 확인 (HTML 차단 페이지 받지 않았는지)
6. 텍스트 추출: `pdftotext -layout docs/references/.../<file>.pdf docs/references/.../<file>.txt`
7. spec parameter 검색: `grep -in "<parameter_name>\|Operation Conditions\|Electrical Characteristics" docs/references/.../<file>.txt`
8. spec 표 컨텍스트 (전후 50~100줄) `sed -n` 으로 발췌 후 분석
9. 인용 시 §2 형식 강제
10. 표준 문서 (IEEE / ISO 등 유료): 사내 라이선스 / 단체 회원 계정으로 다운로드, 라이선스 메타데이터를 `docs/references/standards/<body>/LICENSE.md` 에 기록

---

## 12. 본 룰 위반 시 일반 패턴 (공통 시퀀스)

대표 위반 시퀀스 (도메인 무관 추상):

1. AI / 작업자가 SDK 매크로 / 드라이버 default / docstring 권장값 을 보고
2. **1차 source spec 으로 비약** → "현재 동작 값이 spec 위반" 거짓 단정
3. downstream 거짓 결론 누적: "외부 표준 위반", "spec 무보증", "제품 / 시스템 위험"
4. 후속 검증/협업 워크플로 (병렬 워커, 외부 도구) 의 **부분 전제 오염** — 모두 거짓 단정을 사실로 받아들임
5. 다중 정정 라운드 발생, 토큰 / 시간 낭비, 사용자 신뢰 손상
6. 사용자가 1차 source PDF 직접 다운로드 후 검증 → spec 안쪽 정상, SDK / 드라이버 default 는 SW 보수 한계였을 뿐

**본 룰의 핵심**: "벤더 SDK / 드라이버 default ≠ 1차 spec ≠ 표준 명세" 를 항상 의식하고 1차 source 직접 확인을 강제한다. 사용자가 source 제공 가능성 확인 → 받기 전 단정 보류.

도메인별 구체 사례는 §14 Add-on 참조 (임베디드 / ROS2 / OpenCV / 표준 각각).

---

## 13. 도메인 자동 감지 트리거

| Add-on | 트리거 (1개 이상 충족 시 활성) |
|--------|------------------------------|
| **A. 임베디드** | `__attribute__((interrupt))`, `ISR(`, `NVIC_`, `IRQHandler`, FreeRTOS API, STM32 HAL 매크로, `.ld` linker script, register-level access, `volatile` 빈출, MCU datasheet / Family Manual 참조 |
| **B. ROS2** | `package.xml`, `rclpy` / `rclcpp` import, `.launch.py`, `rcl_interfaces`, `ament_python` / `ament_cmake` 빌드 타입, REP 인용, sensor driver (livox / velodyne / realsense) 참조 |
| **C. OpenCV / Computer Vision** | `cv2.` / `cv::` import, `findChessboardCorners`, `calibrateCamera`, `Mat`, `imread`, BGR/RGB 변환, distortion 모델, 카메라 calibration 작업 |
| **D. 표준 / 인증** | RFC 인용, IEEE std 인용, ISO / IEC 표준 인용, JEDEC / AEC-Q 인용, 인증 요구사항 (KC / CE / FCC 등) 작업 |

다중 Add-on 동시 적용 가능. 우선순위 없음. 사용자가 명시적으로 도메인을 지정하면 그것이 우선.

---

## 14. 도메인 Add-on

각 Add-on 은 §15 Plug-in 5요건 (트리거 · taxonomy · 보관 경로 · 인용 형식 · 흔한 추정 사례 · 1차 source 절차 · 자체 점검 grep) 을 모두 충족한다.

### 14-A. 임베디드

#### A.1 매뉴얼 종류 taxonomy

| 종류 | 다루는 정보 | 예 |
|---|---|---|
| **silicon datasheet** | 전기 특성, 핀맵, 패키지, Operation Conditions | `STM32F4 DataSheet`, `Infineon AURIX TC3xx DataSheet` |
| **User Manual / Family Manual / Reference Manual** | peripheral register-level 동작, SR / IRQ 라우팅, DMA 토폴로지 | `STM32F4 Reference Manual`, `AURIX TC3xx Family Manual` |
| **SDK / HAL 문서** | API 시그니처, sample, default 매크로 | `STM32 HAL UM`, `iLLD AURIX SDK manual` |
| **Application Note** | 응용 사례, 권장 회로, 권장 파라미터 | `AN2867 (STM32 oscillator)`, AURIX AN |
| **Errata** | 알려진 silicon 버그, 회피 절차 | `STM32F4 Errata`, AURIX Errata sheet |

#### A.2 보관 경로

`docs/references/<vendor>/<mcu>/` (예: `docs/references/st/stm32f407/`, `docs/references/infineon/tc375/`).
단일 칩 단순 프로젝트는 루트 `references/` 허용.

#### A.3 인용 형식 (§2 강제 형식 적용)

```
[STM32F4 Reference Manual RM0090 Rev 19, Table 134, page 543](docs/references/st/stm32f407/RM0090.pdf)
[AURIX TC3xx Family Manual v2.0.0, §10.3.2, page 214](docs/references/infineon/tc375/family_manual.pdf)
```

#### A.4 흔한 추정 단정 사례

- 벤더 SDK 의 `<PERIPHERAL>_<PARAM>_MAX` 매크로를 silicon datasheet spec 으로 비약
- `volatile` 단독 사용을 SDK docstring "권장" 만 보고 race 가드 충분으로 단정
- HAL default ADC sampling time 을 datasheet 권장 운영점으로 비약
- Errata 미확인 → 알려진 silicon 버그 회피 누락
- "TYP = 권장값" 비약 (§3.1)

#### A.5 1차 source 확인 절차

- silicon datasheet (벤더 발행) — 전기 특성 단정 시 의무
- User Manual / Family Manual / Reference Manual — register / peripheral / DMA 단정 시 의무
- 두 종류 모두 다운로드 권장: `<vendor>_<mcu>_DataSheet_vX.Y.pdf` + `<vendor>_<mcu>_ReferenceManual_vX.Y.pdf`
- Errata 별도 다운로드 + 작업 전 검토 의무

#### A.6 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# 임베디드 인용 형식 (DataSheet / Reference Manual / Family Manual)
grep -oE "\[(STM32|AURIX|ESP32|nRF|MSP430|PIC|AVR|ATmega)[^]]*\b(DataSheet|Reference Manual|Family Manual|User Manual)[^]]*page [0-9]+\]" $TARGET

# SDK 매크로 인용 시 "silicon spec 아님" 명시 여부
grep -nE "_MAX|_MIN|#define" $TARGET | grep -vE "silicon|device|SDK 권장|datasheet 아님"

# Errata 검토 흔적
grep -E "Errata|errata" $TARGET
```

---

### 14-B. ROS2

#### B.1 매뉴얼 종류 taxonomy

| 종류 | 다루는 정보 | 예 |
|---|---|---|
| **sensor datasheet** (벤더 발행) | LiDAR / IMU / camera 의 물리 spec (scan rate, FoV, range, accuracy) | `Livox MID-360 DataSheet`, `Velodyne VLP-16 DataSheet`, `Bosch BMI088 DataSheet` |
| **벤더 ROS2 driver 문서** | 토픽 이름, frame_id 권장, QoS 권장, 파라미터 default | `livox_ros2_driver` README, `velodyne_driver` docs, `realsense-ros` docs |
| **REP (Robot Enhancement Proposal)** | ROS 표준 명세 — 좌표계, TF tree, IMU frame_id, 단위 | REP-103 (단위·좌표계), REP-105 (TF tree), REP-145 (IMU) |
| **ROS2 design docs** | DDS / QoS 정책 분류, executor 모델, lifecycle | design.ros2.org |
| **메시지 spec** | `sensor_msgs/PointCloud2`, `sensor_msgs/Imu`, `geometry_msgs/Transform` 등 메시지 정의 | docs.ros.org / interfaces |
| **시뮬레이터 bridge 문서** | Gazebo Fortress sensor plugin, `ros_gz_bridge` 토픽 매핑 | gazebosim.org docs, ros_gz README |

#### B.2 보관 경로

- 벤더 sensor datasheet: `docs/references/<vendor>/<sensor>/` (예: `docs/references/livox/mid-360/`)
- 벤더 driver 문서 사본: `docs/references/<vendor>/<sensor>/driver-readme.md`
- REP / design doc: URL 우선 + `docs/references/standards/ros/rep-<N>/` 사본 (변경 추적용)
- 알고리즘 논문: `docs/references/papers/<topic>/`

#### B.3 인용 형식

```
[Livox MID-360 DataSheet v1.2, Table 3, page 8](docs/references/livox/mid-360/datasheet.pdf)
[REP-105, Section "Frame Authority", accessed 2026-05-21](https://www.ros.org/reps/rep-0105.html)
[livox_ros2_driver README v1.0.1, "Parameters" section](docs/references/livox/mid-360/driver-readme.md)
```

#### B.4 흔한 추정 단정 사례 (대표 사례 — §13.2 ROS2)

대표 위반 시퀀스:

1. AI / 작업자가 ROS2 노드 파라미터로 LiDAR 동작점 결정 시
2. 벤더 ROS2 드라이버 (예: `livox_ros2_driver`, `velodyne_driver`) 의 default 파라미터를 datasheet spec 으로 비약
3. "드라이버 default `frequency` = datasheet TYP scan rate" 단정
4. downstream 거짓 결론 누적: "datasheet 위반", "센서 spec 무보증", "벤더 보장 운영점 이탈"
5. 다중 정정 라운드, 토큰 / 시간 낭비
6. 사용자가 LiDAR datasheet PDF 직접 다운로드 → spec 안쪽 정상, 드라이버 default 는 SW 보수 한계였을 뿐

**핵심**: "벤더 ROS2 driver default ≠ sensor datasheet TYP / Min-Max spec ≠ REP / ROS2 design doc 권장".

기타 흔한 추정:
- "REP-105 가 base_link 를 base frame 으로 권장" 단정 (실제 REP 는 `base_link`/`base_footprint` 모두 허용)
- "Gazebo bridge 가 PointCloud2 per-point timestamp 발행" 추정 → 실측 미발행 → SLAM deskew 비활성 → 드리프트
- IMU vendor datasheet 의 mounting axis 를 REP-145 (x=forward, y=left, z=up) 와 일치한다고 단정
- 패키지 README 의 QoS 권장값을 ROS2 design doc spec 으로 비약

#### B.5 1차 source 확인 절차

- 센서 datasheet PDF 다운로드 (§11 절차)
- 드라이버 소스 코드 default 값 직접 인용 (file:line)
- 드라이버 README 는 보조 자료 — datasheet 와 별도 검증 항목
- REP / design doc 은 공식 URL 우선, 변경 가능성 있는 부분은 `accessed YYYY-MM-DD` 함께
- 시뮬레이터 동작은 `ros2 topic info -v <토픽>` / `ros2 topic echo` 실측 의무

#### B.6 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# ROS2 인용 형식 (sensor datasheet / driver README / REP)
grep -oE "\[(Livox|Velodyne|Ouster|Hesai|Bosch|InvenSense)[^]]*DataSheet[^]]*page [0-9]+\]" $TARGET
grep -oE "\[REP-[0-9]+[^]]*\]" $TARGET

# 드라이버 default 인용 시 "datasheet spec 아님" 명시
grep -nE "default|기본값" $TARGET | grep -iE "frequency|rpm|scan_rate|rate" | grep -vE "driver default|datasheet 아님|소스 코드"

# REP / accessed 일자
grep -oE "accessed [0-9]{4}-[0-9]{2}-[0-9]{2}" $TARGET
```

---

### 14-C. OpenCV / Computer Vision

#### C.1 매뉴얼 종류 taxonomy

| 종류 | 다루는 정보 | 예 |
|---|---|---|
| **OpenCV 공식 docs** | API reference, tutorial, 함수 시그니처, default 파라미터, 버전별 차이 | docs.opencv.org/4.x/ |
| **OpenCV 소스 코드** | 구현체, 실제 default, 알고리즘 변형 | github.com/opencv/opencv `modules/<module>/src/<file>.cpp` |
| **알고리즘 원문 논문** | 알고리즘 의도, 가정, 권장 파라미터 범위 | Zhang 1999 (calibration), Lucas & Kanade 1981 (optical flow), Hough 1962 |
| **카메라 calibration spec** | distortion 모델 정의 (Brown-Conrady 5/8-param, fisheye, omnidir) | OpenCV `calib3d` docs, 광학 spec 문헌 |
| **카메라 매뉴얼** (벤더 발행) | sensor type, 렌즈 종류 (pinhole / fisheye / 광각), intrinsic 추정값 | RealSense D435i datasheet, USB camera spec |

#### C.2 보관 경로

- OpenCV 공식 docs (버전 명시 의무): `docs/references/opencv/<version>/` (예: `docs/references/opencv/4.8.0/`)
- 알고리즘 논문: `docs/references/papers/<topic>/<author-year>.pdf` (예: `docs/references/papers/calibration/zhang-1999.pdf`)
- 카메라 매뉴얼: `docs/references/<vendor>/<camera>/` (예: `docs/references/intel/realsense-d435i/`)

#### C.3 인용 형식

```
[OpenCV 4.8.0 docs, cv::calibrateCamera, "Detailed Description"](docs/references/opencv/4.8.0/calib3d_calibrateCamera.html)
[Zhang 1999 "A Flexible New Technique for Camera Calibration", IEEE PAMI 22(11), p.1330](docs/references/papers/calibration/zhang-1999.pdf)
[Intel RealSense D435i DataSheet v1.3, §4.2, page 18](docs/references/intel/realsense-d435i/datasheet.pdf)
```

#### C.4 흔한 추정 단정 사례

- OpenCV 함수 default 파라미터를 algorithm spec 으로 비약 (예: `findChessboardCorners` flags default 를 Zhang 원문 권장으로 단정)
- BGR vs RGB channel order 추정 단정 (`cv::Mat` type 미확인)
- OpenCV 3.x ↔ 4.x API 차이 미인용 (예: `cv2.findContours` 반환값 개수 변경, 일부 함수 모듈 이전)
- `cv::calibrateCamera` 의 5-param Brown-Conrady distortion 모델을 fisheye 렌즈에 그대로 적용 (fisheye 는 `cv::fisheye::calibrate` 별도)
- `imread` default `IMREAD_COLOR` (3채널 BGR) 를 alpha 채널 보존으로 추정
- OpenCV `solvePnP` 의 algorithm flag default (`SOLVEPNP_ITERATIVE`) 가 모든 경우에 최적이라 단정
- 알고리즘 원문 논문 미참조로 OpenCV 함수 동작을 "표준 알고리즘" 단정

#### C.5 1차 source 확인 절차

- **OpenCV 버전 명시 의무** — major.minor.patch (예: 4.8.0). API 가 버전마다 다름
- 공식 docs 의 함수 page (URL + 버전 + accessed 일자)
- 소스 코드 file:line 직접 인용 (`modules/calib3d/src/calibration.cpp:LNNNN`)
- 알고리즘 원문 논문 인용 (저자·학회·연도) — OpenCV 가 채택한 변형과 원문 권장의 차이 명시
- 카메라 sensor 의존 작업은 카메라 datasheet (렌즈 종류, intrinsic 추정값) 별도 인용
- 실측 검증: 체크보드 calibration 의 reprojection error 측정

#### C.6 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# OpenCV 인용 (버전 명시 의무)
grep -oE "\[OpenCV [0-9]+\.[0-9]+\.[0-9]+[^]]*\]" $TARGET

# OpenCV default 인용 시 "algorithm spec 아님" 또는 소스 file:line 명시
grep -nE "OpenCV.*default|cv2\.|cv::" $TARGET | grep -vE "algorithm spec 아님|소스|github.com/opencv|file:line"

# 알고리즘 원문 인용
grep -oE "\[[A-Z][a-z]+ [12][0-9]{3}[^]]*\]" $TARGET

# distortion 모델 / 렌즈 종류 분리 (fisheye vs pinhole vs omnidir)
grep -nE "calibrateCamera|fisheye|distortion" $TARGET | grep -E "pinhole|fisheye|omnidir|Brown-Conrady"
```

---

### 14-D. 표준 / 인증

#### D.1 매뉴얼 종류 taxonomy

| 종류 | 발행 단체 | 예 |
|---|---|---|
| **RFC** (Request for Comments) | IETF | RFC 793 (TCP), RFC 6455 (WebSocket), RFC 7519 (JWT) |
| **IEEE 표준** | IEEE Standards Association | IEEE 802.11 (Wi-Fi), IEEE 754 (floating point), IEEE 1588 (PTP) |
| **ISO / IEC 표준** | ISO / IEC | ISO 26262 (자동차 기능안전), IEC 61508 (산업 기능안전), ISO/IEC 27001 (보안) |
| **JEDEC** | JEDEC Solid State Technology Association | JESD22 (반도체 신뢰성), JESD79 (DDR) |
| **AEC-Q** | Automotive Electronics Council | AEC-Q100 (IC), AEC-Q200 (passive) |
| **인증 / 규제 spec** | 국가 / 지역 인증 기관 | KC (한국), CE (EU), FCC (미국), UL (안전) |
| **ROS REP** | ROS community | REP-103, REP-105, REP-145 (B Add-on 과 중복 — 양쪽 활성) |

#### D.2 보관 경로

`docs/references/standards/<body>/<doc-id>/` (예: `docs/references/standards/ieee/802.11/`, `docs/references/standards/iso/26262/`).
유료 표준은 사내 라이선스 사본 + `LICENSE.md` 에 출처·라이선스 메타데이터 기록.

#### D.3 인용 형식

```
[IEEE 802.11-2020, §10.3, page 421](docs/references/standards/ieee/802.11/ieee802.11-2020.pdf)
[ISO 26262-3:2018, §7.4.2, "Hazard analysis and risk assessment"](docs/references/standards/iso/26262/iso26262-3-2018.pdf)
[RFC 7519, §4.1.1 "iss" Claim, accessed 2026-05-21](https://datatracker.ietf.org/doc/html/rfc7519)
[AEC-Q100-Rev-H, Table 2 "Temperature Grade"](docs/references/standards/aec-q/aec-q100-rev-h.pdf)
```

#### D.4 흔한 추정 단정 사례

- 표준 번호만 인용하고 절 / page 미명시 ("IEEE 802.11 위반" → 어떤 절?)
- 개정판 미명시 — 표준은 개정 시 의미가 바뀜 (RFC 의 obsoleted by 관계, ISO 의 :2011 vs :2018)
- 인증 인증서 (예: "AEC-Q100 인증") 와 spec 준수 혼동 — 인증은 별도 절차
- "표준 위반" 단정을 1차 표준 문서 인용 없이 사용 — §8 금지 단어 룰 위반
- 표준의 normative (강제) vs informative (참고) 절 혼동
- 표준 인용 시 accessed 일자 누락 — 표준 단체 errata / amendment 가 후속 발행될 수 있음

#### D.5 1차 source 확인 절차

- 표준 문서 PDF 또는 공식 발행 페이지 (IEEE Xplore, ISO Online Browsing Platform, IETF datatracker)
- 개정판 / amendment / errata 함께 다운로드
- 인용 시 절·page (또는 RFC 번호 / IEEE std 번호) + 개정판 + accessed 일자 함께
- 인증 spec 은 인증 기관의 공식 문서만 1차 source — 컨설팅 자료 / 블로그는 ⓦ
- 사내 / 외부 컨설팅이 작성한 "준수 매트릭스" 는 ⓦ — 원문과 별도 검증

#### D.6 자체 점검 grep

```bash
TARGET=<분석 대상 .md>

# 표준 인용 형식 (번호 + 개정 + 절 / page)
grep -oE "\[(RFC [0-9]+|IEEE [0-9]+(-[0-9]+)?|ISO [0-9]+(-[0-9]+)?:[0-9]{4}|IEC [0-9]+|JESD[0-9]+|AEC-Q[0-9]+)[^]]*\]" $TARGET

# "표준 위반" 단정 시 표준 인용 첨부 여부
grep -nE "위반|준수|compliance|non-compliance" $TARGET | grep -vE "RFC [0-9]+|IEEE [0-9]+|ISO [0-9]+|IEC [0-9]+|page [0-9]+|§"

# accessed 일자 (URL 인용 시)
grep -oE "accessed [0-9]{4}-[0-9]{2}-[0-9]{2}" $TARGET
```

---

## 15. 도메인 Plug-in 5요건 (확장 인터페이스)

새 도메인 Add-on 추가 시 다음 5요건 모두 만족해야 본 SSOT 등록:

1. **트리거** — 자동 감지 키워드 / 파일 패턴 (1개 이상). 다른 Add-on 과 중복 시 모두 활성. §13 트리거 표 한 줄 추가.
2. **매뉴얼 종류 taxonomy** — 1차 source 종류와 각각의 검증 책임 표. §3 (SDK ≠ 1차 spec ≠ 표준) 분리 룰을 도메인에 맞게 구체화.
3. **보관 경로 + 인용 형식** — `docs/references/<...>/` 하위 정규화 경로. §2 강제 인용 형식 준수 + 도메인 특화 식별자 (버전 / RFC 번호 / 학회 / 연도 등).
4. **흔한 추정 단정 사례** — 본문 §12 의 공통 시퀀스를 도메인에 맞게 구체화. 최소 3개 이상.
5. **1차 source 확인 절차 + 자체 점검 grep** — §11 다운로드 절차 + 도메인 인용 형식 검출 정규식.

기존 Add-on (§14-A 임베디드, §14-B ROS2, §14-C OpenCV, §14-D 표준/인증) 은 모두 위 5요건 충족 모범 사례.

### 15.1 향후 확장 후보 (참고)

| Add-on 후보 | 트리거 예 | 대표 1차 source |
|---|---|---|
| Qt / UI | `QApplication`, `QWidget`, `.ui`, PyQt5/6 import, PySide6 import | Qt 공식 docs (doc.qt.io), PyQt / PySide manual |
| DB / Backend | SQL, ORM, OpenAPI / Swagger, REST endpoint | PostgreSQL manual, MySQL ref, OpenAPI spec |
| ML / GPU | CUDA, cuDNN, PyTorch, TensorFlow, `nvcc` | CUDA Programming Guide, cuDNN docs, framework docs |
| Web / Frontend | HTML5 spec, ECMAScript, React, Vue | WHATWG HTML spec, TC39 ECMAScript spec |
| 통신 프로토콜 | MQTT, gRPC, MAVLink, CAN | OASIS MQTT spec, MAVLink XML, ISO 11898 (CAN) |

본 §은 확장 가능 근거만 명시. 실제 Add-on 파일은 필요 시점에 §15 5요건 충족 후 §14 절로 추가.

### 15.2 서브-도메인 재귀 (Add-on 비대 시)

Add-on 룰이 풍부해지면 (예: 임베디드 → MCU 별 페리페럴·HAL·툴체인이 매우 다름) 단일 §절 대신 폴더로 확장 가능:

```text
kuks_claude_setup_new/claude_guideline/
├── external_reference_handling.md       ← 본 파일 (공통 SSOT)
└── external_reference/                  ← Add-on plug-in 폴더 (확장 시점에 생성)
    ├── embedded/
    │   ├── README.md                    ← 임베디드 공통
    │   ├── stm32.md
    │   ├── aurix.md
    │   └── esp32.md
    ├── ros2.md
    ├── opencv.md
    └── standards.md
```

확장 규칙:

- 본 파일 §1~§13, §15~§18 골격은 폴더 확장 시에도 불변
- §14 절은 폴더 진입점 인덱스로 축소 (각 도메인 폴더 README 가 권위)
- 5요건 §15 은 서브-도메인에도 **동일 적용**
- 서브 트리거는 도메인 README 의 자체 트리거 표에 등록. 본 파일 §13 은 도메인 레벨만 유지

본 §은 확장 가능 근거만 명시. 실제 폴더 분리는 단일 파일이 비대해진 시점에 별도 SSOT 변경 절차 (§17) 거쳐 수행.

---

## 16. 자체 점검 grep

본 룰 적용 결과 문서가 §2 인용 형식·§8 강제 단정어 룰·§10 검증 등급 표기·각 Add-on 도메인 grep 을 모두 통과하는지 자체 검증:

```bash
TARGET=<분석 대상 .md>

# 1. 강제 인용 형식 (page / Table / 경로 명시 의무) — §2
grep -oE "\[[^]]+(v[0-9.]+|Rev [A-Z0-9]+|[0-9]{4})?[^]]*(page [0-9]+|Table [0-9]+|§[0-9.]+|RFC [0-9]+|REP-[0-9]+)\]\([^)]+\.(pdf|html|md)\)" $TARGET

# 2. 검증 등급 표기 존재 (✓ / ⓦ / ⚠) — §10
grep -oE "(^|[^a-zA-Z])(✓|ⓦ|⚠)( |$)" $TARGET | sort -u

# 3. 강한 단정어가 primary source 첨부 없이 사용된 의심 라인 — §8
grep -nE "위반|초과|무보증|non-compliance|violation|fail" $TARGET | grep -vE "page [0-9]+|Table [0-9]+|RFC [0-9]+|IEEE [0-9]+|ISO [0-9]+|datasheet|\.pdf"

# 4. SDK / 드라이버 default 인용 시 1차 spec 분리 명시 — §3
grep -nE "_MAX|_MIN|#define|default|기본값" $TARGET | grep -vE "silicon|device|SDK 권장|datasheet 아님|algorithm spec 아님|소스 코드"

# 5. 도메인 자동 감지 명시 — §13
grep -E "활성 Add-on: |감지된 Add-on: " $TARGET

# 6. 인용 시 버전 / 개정 / accessed 일자 — §2
grep -oE "v[0-9]+\.[0-9]+(\.[0-9]+)?|Rev [A-Z0-9]+|:[0-9]{4}|accessed [0-9]{4}-[0-9]{2}-[0-9]{2}" $TARGET

# 7. Add-on 별 grep (§14-A/B/C/D 각각의 grep 블록)
#    임베디드: §14-A.6
#    ROS2:    §14-B.6
#    OpenCV:  §14-C.6
#    표준:    §14-D.6
```

---

## 17. 변경 절차

본 파일은 SSOT (Single Source of Truth / 단일 근원) 이므로 변경 시 **사용자 승인 필수**. 변경 후:

1. CHANGELOG 갱신 (변경 사유 · 영향 범위 · 이전 버전 호환성 명시)
2. VERSION 갱신 (semver — major: 룰 호환성 깨짐 / minor: Add-on 추가 / patch: 표현 정정)
3. 본 파일이 설치된 다운스트림 프로젝트에 변경 통보 (대상 프로젝트의 `docs/claude_guideline/external_reference_handling.md` 갱신 의무)
4. `dogfooding/` 산출물로 재검증 (있는 경우)
5. 본 파일 자체 점검 grep §16 모두 통과 확인

신규 Add-on 추가 시 §15 5요건 충족 + §14 절 신설 + §13 트리거 표 한 줄 추가 + §16 grep 블록 추가.

---

## 룰 (요약)

1. **self-contained** — 본 파일 1개로 자체 완결. 외부 가이드라인 / 자동화 도구 / Skill 의존 0.
2. **1차 source 직접 확인 강제** — SDK / 드라이버 default / docstring 권장값에서 1차 spec 추론 금지 (§3).
3. **인용 형식 강제** — `[문서명 v버전, Table N, page P](경로)` (§2). 표준은 `RFC-N` / `IEEE-N` / `ISO-N:연도` + 절 + accessed 일자.
4. **검증 등급 ✓/ⓦ/⚠ 표기 의무** — 모든 spec 관련 주장에 등급 부착 (§10).
5. **강한 단정어 사용 조건** — `위반 / fail / non-compliance` 단정어는 primary source 인용 첨부 시에만 (§8).
6. **추정 금지 · 실측 검증** — 모호한 표현은 추정 단정 금지. 실측 / 추가 자료 / 벤더 문의로 해결 (§4).
7. **다중 1차 source 분리** — DataSheet ≠ User Manual ≠ SDK ≠ 표준 ≠ 알고리즘 원문 (§3.2). 각각 별도 검증 항목.
8. **도메인 Add-on 자동 활성** — §13 트리거 충족 시 §14-A/B/C/D 자동 적용. 사용자 명시 거부 시에만 무력화.
9. **Plug-in 5요건** — 새 Add-on 은 §15 5요건 모두 충족 후 등록.
10. **자체 점검 grep 통과 의무** — 본문 §16 + 활성 Add-on 의 grep 모두 통과 (§9 작업 후 체크리스트).

---

**VERSION**: 1.0.0
**Last reviewed**: 2026-05-21
