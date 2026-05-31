# `.claude/` — 프로젝트 자동화 설정

본 폴더는 Claude Code 가 본 워크스페이스에서 동작할 때 사용하는 프로젝트 단위 설정과 훅 스크립트를 담는다.

## 파일

| 파일 | 역할 |
| --- | --- |
| [settings.json](settings.json) | 프로젝트 공유 훅 / 권한 설정 (commit 됨) |
| [hooks/detect_correction.sh](hooks/detect_correction.sh) | 사용자 교정/지적 발화 감지 → 실수 자동 기록 reminder 주입 |

## 자동 실수 기록 (UserPromptSubmit hook)

### 목적

사용자가 본 세션에서 Claude 의 실수를 지적했을 때, 동일 실수가 향후 재발하지 않도록 [docs/claude-mistake/](../docs/claude-mistake/) 에 항목이 자동 누적되도록 한다.

### 동작

1. `UserPromptSubmit` 시점에 [hooks/detect_correction.sh](hooks/detect_correction.sh) 가 호출된다.
2. 사용자 입력에서 한국어 교정 패턴(`왜 안 했`, `잘못`, `실수`, `되돌`, `답답`, `맞는건지`, `다시 해`, `이미 했`, ...)이 감지되면 system-reminder 를 주입한다.
3. Reminder 내용: 본 응답에서 의도를 처리한 뒤 `docs/claude-mistake/<YYYY-MM-DD>.md` 에 항목 추가, 영구적 재발 방지는 메모리에도 반영, 기록 commit 은 별도 scope.
4. 패턴 미매치 / 단순 질문이면 무음 통과.

### 비활성화

`.claude/settings.local.json` 에서 hook 을 override 하거나, 환경변수로 비활성화:

```bash
export DISABLE_OMC=1   # OMC 와 호환 (전체 훅 차단)
```

### 메인터넌스

- 패턴 추가가 필요하면 [hooks/detect_correction.sh](hooks/detect_correction.sh) 의 `grep -qE` 정규식에 추가.
- 한국어 외 영어 패턴 필요 시 동일 방식으로 확장.
- `python3` 가 없는 환경에서는 prompt 추출 단계가 무음 실패 → 훅 자체는 통과 (사용자 흐름 막지 않음).
