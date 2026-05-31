# 스킬 등록 규칙

본 워크스페이스에서 신규 스킬 / 자동화 자산을 만들거나 기존 자산을 보강하면, SSOT 스킬 레포 [`kuks_claude_setup`](https://github.com/kuks2309/kuks_claude_setup) 에 등록해 다른 프로젝트가 재사용할 수 있게 한다. 본 규칙은 그 절차의 단일 근원.

## 적용 대상 ("스킬" 의 범위)

다음 자산이 본 규칙의 대상이다.

| 종류 | 워크스페이스 위치 (예) | SSOT 레포 위치 |
| --- | --- | --- |
| Slash command / skill 정의 | `.claude/commands/`, `.claude/skills/` | `skills/<name>.md` |
| Hook 스크립트 | `.claude/hooks/<name>.sh` | `tools/hooks/<name>.sh` (또는 `skills/<name>/`) |
| 가이드라인 보강 | `docs/claude_guideline/<file>.md` | `claude_guideline/<file>.md` |
| 커스텀 sub-agent | `.claude/agents/<name>.md` | `agents/<name>.md` |
| 도메인 템플릿 (이슈 / 실수 / 요청 README) | `docs/<domain>/README.md` | `<domain>/README.md` |

위 카테고리에 명확히 속하지 않는 신규 자산은 SSOT 레포의 README 에서 적합한 폴더를 결정하거나, 새 폴더를 사용자 승인 후 추가한다 ([coding.md](coding.md) "사전 승인 트리거 — 새 패키지 / 모듈 신규 생성").

## 등록 절차

```text
[워크스페이스 검증]
    ↓
[SSOT 레포로 복사 / 갱신]
    ↓
[메타 갱신 (CHANGELOG / VERSION / README 진입점)]
    ↓
[SSOT 레포 commit + push]
    ↓
[워크스페이스 측 sync (필요 시)]
```

1. **워크스페이스에서 검증.** 자산이 의도대로 동작함을 실제로 확인한 뒤에만 등록한다 (검증 안 된 자산을 SSOT 에 올리지 않는다).
2. **SSOT 레포 clone / pull.** 로컬 사본을 최신화한다.
3. **자산 복사.** 위 표의 "SSOT 레포 위치" 로 옮긴다. 워크스페이스 고유 경로(IP, 사설 endpoint 등) 가 포함돼 있으면 placeholder / 환경변수로 추상화한다 ([local/README.md](local/README.md) 참조).
4. **메타 갱신.**
   - `claude_guideline/` 변경이면: `CHANGELOG.md`, `VERSION` (semver), `README.md` 진입점, `templates/CLAUDE.md.template` 갱신.
   - `skills/`, `agents/`, `tools/hooks/` 등 그 외 폴더면: 해당 폴더의 README 또는 INDEX 가 있으면 갱신.
5. **SSOT 레포 commit + push.** 작업 단위 = 커밋 단위 ([github.md](github.md)). 자동 push 금지 — 사용자 명시 확인 후.
6. **워크스페이스 sync.** 필요 시 워크스페이스의 `docs/claude_guideline/update.sh` 또는 수동 복사로 SSOT → 워크스페이스 방향으로 동기화. SSOT 가 단일 근원이므로 워크스페이스 측 직접 수정 후 SSOT 갱신을 잊는 패턴은 금지.

## 워크스페이스 → SSOT 우선순위

자산이 양쪽에 모두 있을 때 충돌이 발생하면 다음 순서로 처리한다.

1. **SSOT 레포가 단일 근원** — 일반 규칙은 SSOT 가 우선.
2. **워크스페이스 override 가 명시적**일 때만 워크스페이스 우선 — 모듈 CLAUDE.md, [local/](local/) 가 그 예 ([documentation.md](documentation.md) "SSOT 원칙").
3. **SSOT 갱신 누락이 의심되면** SSOT 부터 갱신하고 워크스페이스로 sync.

## 비공개 / 프로젝트 고유 자산 처리

- 사설 IP / NDA 매뉴얼 / 운영 endpoint / 하드웨어 핀맵 등 외부 공개 부적합 자산은 SSOT 레포에 올리지 않는다 ([manual.md](manual.md) "라이선스 / 외부 공개").
- 일반화 가능한 부분만 SSOT 에 올리고, 비공개 부분은 워크스페이스 [local/](local/) 또는 비공개 저장소에 둔다.
- 일반화가 어려우면 SSOT 등록을 보류하고 본 규칙의 적용 대상에서 제외한다.

## 등록 후 책임

- 등록자는 다음 분기까지 SSOT 측 자산이 정상 동작하는지 확인한다 (외부 사용자가 install / update 시 깨지지 않는지).
- 자산이 의존하는 환경 (커맨드, 라이브러리, OS 종류) 을 SSOT 의 README 또는 자산 헤더 주석에 명시한다.
- 30 일 이상 미사용 / 사용처 부재 자산은 deprecate 표시 또는 정리 ([tech_debt.md](tech_debt.md) ADR Open Question 30 일 룰과 동일 정신).

## 등록 안 해도 되는 경우

- 워크스페이스 1 회성 패치 (재사용 가치 없음)
- 디버그용 임시 스크립트
- 사용자 1 인 / 1 머신 한정 설정 ([local/](local/) 에만 둠)

판단이 모호하면 사용자에게 한 줄로 확인 ("이 자산을 SSOT 에 등록할까요?").
