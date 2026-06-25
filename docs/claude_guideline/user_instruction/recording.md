# 사용자 지시 기록 (User Instruction Recording)

> **본 파일은 지시용 — 기록 금지.** 사용자 지시는 `docs/user_instructions/user_instructions.md` 에 기록한다. 폴더·파일이 없으면 만든다 (승인 불요).

## 1. 목적

의도 부채(Intent Debt / 사용자 의도 망실) 방지 — 사용자 지시 **원문 보존 전용**.

소프트웨어 부채 중 기술 부채(코드 잔존 → 리팩토링)·인지 부채(외부 소스 → 재학습)는 회복 가능하나, **의도 부채는 한 번 망실되면 추측 외 복원 수단이 없다**. 사용자 지시 원문을 그대로 박제하여 의도 부채를 0 으로 유지하는 것이 본 파일의 존재 이유다.

분석·처리·요약은 본 파일의 책임이 아니다 (→ [§4 경계](#4-경계--기록--분석)).

## 2. 설치 (규칙 파일)

본 번들 폴더(`user_instruction/`)의 `install.sh` 로 설치한다:

```bash
cd user_instruction && ./install.sh <타깃-프로젝트-루트>
```

스크립트가 (1) `recording.md` 를 `docs/claude_guideline/user_instruction/` 로 복사하고, (2) 등록 스니펫(`claude.snippet.md`)을 타깃 `CLAUDE.md` 에 append 한다 (덮어쓰기 아님, 중복 시 스킵). 수동 설치 시 같은 두 단계를 직접 수행한다.

**활성화 게이트**: 본 파일이 `docs/claude_guideline/user_instruction/recording.md` 경로에 없으면 본 룰은 비활성.

## 3. 기록 (지시 도착 즉시)

사용자 지시가 도착하면 즉시 아래 형식으로 **맨 위에 prepend** 한다 (작업 완료 후 일괄 기록 금지).

형식:

```markdown
## YYYY-MM-DD HH:MM (KST) — <짧은 제목>

> "<사용자 원문>"

---
```

룰:

- **KST**(Korea Standard Time) 시각, **시간 역순** (최신 위, prepend)
- **사용자 원문만 인용** — 요약·해석·재구성 금지
- 동일 요구의 단순 재확인은 생략 가능
- 비밀번호 / NDA(Non-Disclosure Agreement) / 자격증명은 마스킹

## 4. 경계 — 기록 ≠ 분석

본 파일은 **원문 캡처까지만** 책임진다. 다음은 모두 **다음 프로세스(분석)** 의 책임이며 본 파일에도 `docs/user_instructions/` 에도 기록하지 않는다:

- 지시의 의도 파악·분류
- 처리 계획·실행·검증
- 결과·결론·산출물 (→ `docs/worklog/` 등 별도 도메인)

기록 단계에서 분석이 새어들면 원문이 오염되어 의도 부채가 발생한다. **기록은 단순하게 유지한다.**
