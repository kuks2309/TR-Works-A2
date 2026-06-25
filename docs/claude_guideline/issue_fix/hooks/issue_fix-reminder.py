#!/usr/bin/env python3
"""UserPromptSubmit 훅 — 버그/이슈/빌드실패/에러 트리거 감지 시 issue_fix SOP 강제 주입.

배경: CLAUDE.md 등록이 '수동 포인터'라 모델이 issue_fix.md 를 능동적으로 열지 않고
즉답 패치로 직행하는 절차 실패가 발생한다. 본 훅이 트리거 감지 시 진단→제안→구현→
검증→기록 사이클을 응답 전에 강제한다.

계약(Claude Code UserPromptSubmit): stdin JSON → stdout 이 컨텍스트로 주입. 항상 exit 0.
"""
import json
import os
import sys

TRIGGERS = (
    "버그", "디버그", "debug", "이슈", "issue",
    "빌드 실패", "빌드실패", "빌드 에러", "빌드가 안", "빌드 안",
    "에러", "오류", "error", "exception", "예외",
    "고쳐", "고쳐줘", "안 돼", "안돼", "안 된", "동작 안", "작동 안",
    "크래시", "crash", "fix", "bug", "fails", "failing", "broken", "깨졌", "터졌",
)

RULE_MD = "docs/claude_guideline/issue_fix/issue_fix.md"

DIRECTIVE = """[ISSUE-FIX SOP — 강제 게이트]
버그/이슈/빌드 실패/에러 트리거가 감지되었습니다. 즉답 패치로 직행하지 말고, 응답 전 반드시 아래를 선행하세요:

1. {rule} 를 Read 한다 (등록 사실만 알고 건너뛰지 말 것).
2. 진단 → 제안(사용자 승인) → 구현 → 검증 → 기록 사이클을 따른다.
3. 기록은 docs/issues_and_fixes/issues_and_fixes.md (없으면 생성, 승인 불요). 변종 경로(issues_fixes/ 등) 금지.
4. 사용자 승인 없이 구현 단계로 건너뛰지 않는다.""".format(rule=RULE_MD)


def main():
    raw = sys.stdin.read()
    try:
        data = json.loads(raw) if raw.strip() else {}
    except (json.JSONDecodeError, ValueError):
        data = {}

    prompt = str(data.get("prompt", ""))
    if not prompt:
        return
    if not any(t in prompt.lower() for t in TRIGGERS):
        return

    cwd = data.get("cwd") or os.getcwd()
    if cwd and not os.path.isfile(os.path.join(cwd, *RULE_MD.split("/"))):
        return

    print(DIRECTIVE)


if __name__ == "__main__":
    main()
