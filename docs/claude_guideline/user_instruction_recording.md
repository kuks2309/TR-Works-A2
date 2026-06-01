# 사용자 지시 기록 (User Instruction Recording)

**본 파일은 지시용 — 기록 금지. 기록 대상: `docs/user_instructions/user_instructions.md`**

## 설치 위치

- **본 파일**: 대상 프로젝트의 `docs/claude_guideline/user_instruction_recording.md` 에 배치
- **기록 산출물**: `docs/user_instructions/user_instructions.md` (대상 프로젝트 루트 기준 상대경로)

본 파일이 `docs/claude_guideline/user_instruction_recording.md` 위치에 없으면 본 룰은 활성화되지 않는다. 새 프로젝트 적용 시 본 파일을 위 경로로 복사하는 것이 첫 단계.

기록 산출물 (`docs/user_instructions/user_instructions.md`) 부재 시: 폴더 + 빈 파일 자동 생성 후 첫 entry 기록. 본 룰의 목적 (의도 부채 차단) 상 별도 승인 불필요.

## 기록 위치

다운스트림 프로젝트의 `docs/user_instructions/user_instructions.md` — 사용자 원문 전용.

## 형식

````markdown
## YYYY-MM-DD HH:MM KST — <짧은 제목>

> "<사용자 원문 인용>"

---
````

## 룰

- **KST (Korea Standard Time / 한국 표준시) 시각** + **시간 역순** (최신 위, prepend)
- **사용자 원문만 인용** — 요약·해석·재구성 금지
- 동일 요구의 단순 재확인은 생략 가능
- 비밀번호 / NDA (Non-Disclosure Agreement / 비공개 합의) / 자격증명은 마스킹
