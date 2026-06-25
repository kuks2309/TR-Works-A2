#!/usr/bin/env python3
"""UserPromptSubmit 훅 — 부채(기술·이해·의도)/TODO/FIXME 트리거 시 debt SOP 강제 주입.

배경: CLAUDE.md 등록이 '수동 포인터'라 모델이 debt.md 를 능동적으로 열지 않고 registry
등록 없이 맨 TODO 만 남기는 절차 실패가 발생한다. 본 훅이 트리거 감지 시 식별→등록→
참조 규칙을 응답 전에 강제한다.

계약(Claude Code UserPromptSubmit): stdin JSON → stdout 이 컨텍스트로 주입. 항상 exit 0.
"""
import json
import os
import sys

TRIGGERS = (
    "부채", "기술 부채", "기술부채", "이해 부채", "의도 부채",
    "debt", "todo", "fixme", "hack", "상환", "registry", "레지스트리",
)

RULE_MD = "docs/claude_guideline/debt/debt.md"

DIRECTIVE = """[DEBT SOP — 강제 게이트]
부채(기술·이해·의도)/TODO/FIXME/HACK 트리거가 감지되었습니다. 응답 전 반드시 아래를 선행하세요:

1. {rule} 를 Read 한다 (등록 사실만 알고 건너뛰지 말 것).
2. 식별된 부채를 docs/debt/registry.md 에 등록한다(id·유형·위치·사유·상태·상환계획).
3. 코드 마커는 debt id 를 참조한다(예: # TODO(debt-042): ...). 맨 TODO/FIXME/HACK 금지(⟦CI:debt-marker⟧).
4. 식별은 coding SOP, 등록·추적은 debt 가 소유. 미설치 시 주석/ADR 에 식별만 남김(graceful).""".format(rule=RULE_MD)


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
