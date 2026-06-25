# 부채 관리 SOP (Debt Management SOP)

> **본 파일은 지시용.** 기술·이해·의도 3-부채의 **등록·추적·상환 registry** self-contained 단일 근원(SSOT / Single Source of Truth).

본 코어는 self-contained 다 — 본문 외 가이드라인·도구·Skill·OMC 상태경로 의존 0. 강제 로직은 본문에 없고 `debt/checks/*.sh` 에 있다.

## 설치 / 활성화 게이트

```bash
cd debt && ./install.sh <타깃-프로젝트-루트>
```

규칙·이빨을 `docs/claude_guideline/debt/` 로 복사 + `CLAUDE.md` 등록. **활성화 게이트**: 본 파일이 그 경로에 없으면 룰 비활성.

## 0. 강제 모델 (먼저 읽기)

- **`⟦CI:<id>⟧`** = `checks/<id>.sh` 가 코드에서 재도출·차단(pre-commit·CI). **못 속인다.**
- **`⟦권고⟧`** = 재도출 불가, 자기보고 → 정직한 advisory.
- debt 의 목적은 **"식별된 부채를 잃지 않게 등록·추적"**. *식별*은 작업 SOP(coding 등)가, *등록·추적·상환*은 여기가 소유한다.

## 1. 세 부채 (정의)

| 부채 | 무엇 | 식별 지점(coding) |
| --- | --- | --- |
| **기술(technical)** | shortcut·임시방편·미흡 테스트·미룬 리팩토링·`TODO`·노후 의존성 | §4 구현 · §5 검증 |
| **이해(understanding)** | 시스템·코드 이해 공백, "왜 되는지 모름", 미문서 동작 | §2 사전조사 |
| **의도(intent)** | 결정 근거·의도 상실, 미기록 "왜" | §3 ADR(Architecture Decision Record) · `user_instruction` |

각 부채의 **식별 신호·등록 항목·상환 전략 상세**는 `domains/{technical,understanding,intent}-debt.md`(설치 시 함께 복사).

## 2. 등록 (registry) — debt 가 양식 권위 SSOT

- 위치: `docs/debt/registry.md`(단일 표) 또는 `docs/debt/<id>.md`(항목별).
- 각 항목 필드: **`id` · `유형`(기술/이해/의도) · `위치`(파일:줄/모듈) · `사유` · `식별일` · `상태`(미해결/해결) · `상환계획`**.
- **코드 마커**: `TODO`/`FIXME`/`HACK` 는 **debt id 를 참조**한다 — 예 `# TODO(debt-042): ...`. **맨 마커(미등록) 금지** `⟦CI:debt-marker⟧`.

## 3. 추적·상환

- **상태 추적**: 미해결 → 해결. 해결 시 항목에 해결일·커밋 append(덮어쓰기 금지, supersede).
- **우선순위**: 영향 × 이자(부채가 키우는 비용). 고이자 부채 우선 상환.
- **정기 검토**: 미해결 부채 목록을 주기적으로 리뷰(방치 금지).

## 4. 작업 SOP 와의 연동

- **coding §5 검증 실패** → 즉시 못 고치면 **debt 등록 + 사유**(선조치-후정산).
- **coding §6 후속갱신** → 미해결 이해·의도 부채 등록.
- **emergency/hotfix 생략분** → 24h 내 debt 등록(coding hotfix 레인).
- **debt 미설치 시**(graceful): 작업 SOP 는 등록을 생략하되 식별은 주석/ADR 에 남긴다 — dangling 무해.

## 룰 (요약)

1. 식별된 부채는 모두 registry 에 등록 (잃지 않기)
2. 코드 `TODO`/`FIXME`/`HACK` 는 debt id 참조 (맨 마커 금지)
3. 3-유형 분류 (기술 / 이해 / 의도)
4. 상태 append — 해결도 기록 (덮어쓰기 금지)
5. 고이자 우선 상환

> **MUST 예산**: 위 '룰 요약' 5개가 코어 필수 규칙 전체다 — 7개 이내 유지.

## 자체 점검

```bash
test -f docs/claude_guideline/debt/debt.md || echo "(debt 룰 비활성)"
bash docs/claude_guideline/debt/checks/check-mapping.sh    # ⟦CI⟧ 태그↔스크립트 정합
bash docs/claude_guideline/debt/checks/debt-marker.sh .    # 맨 TODO 차단
grep -cE '^[0-9]+\. ' docs/claude_guideline/debt/debt.md   # 룰 요약 ≤7
```

## 변경 절차

- SSOT 는 본 번들 폴더. 규칙 변경은 사용자 승인 후 `debt.md` + `checks/` 동반 갱신.
- `⟦CI:<id>⟧` 태그 추가/변경 시 `checks/<id>.sh` + `check-mapping.sh` 동반. semver + 파일 말미 `VERSION`.

---

**VERSION**: 1.0.0 (3-부채 정의 + registry 양식 권위 + TODO↔debt id 마커 강제 + coding 식별 연동·graceful; 강제 로직은 checks/*.sh 위임; self-contained·OMC-free)
