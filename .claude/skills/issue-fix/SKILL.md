---
name: issue-fix
description: Use when fixing a bug, diagnosing an error log, resolving a build failure, or recording a completed fix. Triggers on "fix issue", "버그 수정", "이슈 해결", pasted error/traceback, or after implementation is complete.
---

# Issue Fix Workflow

Full diagnose → propose → implement → verify → record cycle. Works for Python, ROS2, and embedded projects.

## Workflow

### 1. Search Past Issues

Read the issues log (first 200 lines):
- `docs/issues_and_fixes/issues_and_fixes.md`
- `docs/issues_and_fixes/` directory (per-bug files)

Grep for keywords from the current error. If a similar past issue exists, reference its root cause as a starting hypothesis.

### 2. Diagnose

Gather evidence before proposing anything:

- Read the error completely. Identify failing file and line.
- `grep` for the error string across source files.
- `git log --oneline -10` and `git diff HEAD~3` for recent regressions.
- Form a root-cause hypothesis with `file:line` evidence.

For multiple bugs, assess severity:

| 심각도 | 기준 |
|--------|------|
| CRITICAL | 시스템 중단·데드락 직접 원인 |
| HIGH | 통신 안정성·스레드 안전성 위협 |
| MEDIUM | API 신뢰성·데이터 무결성 |
| LOW | 품질·유지보수성 |

### 3. Propose Fix (Wait for Approval)

Present before implementing:

- **증상**: what is observed
- **원인**: why it happens — `file:line` evidence
- **해결**: minimal change (줄 수 명시)
- **테스트**: which tests will pass / convert from xfail

**Wait for user approval before writing any code.**

### 4. Implement

Apply the approved fix. Read the target file first, change only the minimum needed.

### 5. Verify

**Python:**
```bash
python -m pytest tests/ -v
# or specific file
python -m pytest tests/test_<module>.py -v
```

**ROS2:**
```bash
colcon build --packages-select <PKG>
```

Confirm zero errors / all expected tests pass before recording.

### 6. Record (MANDATORY)

Both files must be updated. Newest entry at top.

**A) Issues log** — prepend after the separator:

```markdown
## YYYY-MM-DD

### [Fix] <title>

- **문제**: symptom
- **원인**: root cause — `file:line`
- **해결**: what was changed (N줄 수정/삭제/추가)
- **파일**: list of modified files
- **상태**: 완료
```

**B) Code updates** — `docs/<package>_code_updates.md`:

```markdown
## YYYY-MM-DD / HH:MM — <7-char hash>
- [수정/추가/삭제] 설명 (`relative/path/file.py`)
```

Stage with `git add <specific files>` — never `git add .` or `git add -A`.

## Completion Checklist

- [ ] Root cause identified with `file:line`
- [ ] User approved fix before implementation
- [ ] Tests pass / xfail tests converted to pass
- [ ] Issues log updated (newest entry first)
- [ ] Code updates log updated
- [ ] Modified files staged with `git add`
