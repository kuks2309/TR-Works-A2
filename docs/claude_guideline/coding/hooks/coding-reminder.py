#!/usr/bin/env python3
"""UserPromptSubmit 훅 — 코드 작성/구현/수정 트리거 시 coding SOP 강제 주입.

배경: CLAUDE.md 등록이 '수동 포인터'라 모델이 coding.md 를 능동적으로 열지 않고 사전조사
(함수표·전역변수표)·ADR·이중기록을 건너뛴 채 바로 구현하는 절차 실패가 발생한다. 본 훅이
트리거 감지 시 작성 SOP 를 응답 전에 강제한다. (코드 작업 전반이 대상이라 트리거가 넓다.)

계약(Claude Code UserPromptSubmit): stdin JSON → stdout 이 컨텍스트로 주입. 항상 exit 0.
"""
import json
import os
import sys

TRIGGERS = (
    "구현", "구현해", "코드 작성", "코드를 작성", "코드 짜", "코드를 짜",
    "만들어줘", "만들어 줘", "함수를", "함수 추가", "클래스를", "클래스 추가",
    "메서드 추가", "메소드 추가", "모듈을 작성", "기능 추가", "기능을 추가",
    "코드 수정", "코드를 수정", "리팩터", "리팩토링", "refactor", "implement",
    "feature 추가", "새 기능",
)

RULE_MD = "docs/claude_guideline/coding/coding.md"

DIRECTIVE = """[CODING SOP — 강제 게이트]
코드 작성/구현/수정 트리거가 감지되었습니다. 바로 구현으로 직행하지 말고, 응답 전 반드시 아래를 선행하세요:

1. {rule} 를 Read 한다 (등록 사실만 알고 건너뛰지 말 것).
2. 입구 작업분류(trivial fast-path 여부) → 사전조사(관련 함수표·전역변수표 Read) → 사전승인(ADR) → 구현 → 검증(테스트·보안, never-self-approve) → 후속갱신(이중 기록) 절차를 따른다.
3. 강제는 ⟦CI:<id>⟧ ↔ checks/<id>.sh(pre-commit·CI)만 진짜, 그 외 ⟦권고⟧. 명명·스타일 conventions.md, 언어/포맷터 stack.md.
4. 도메인(ros2/embedded/numeric/concurrency/memory) 트리거 시 docs/claude_guideline/coding/domains/ 를 함께 적용한다.""".format(rule=RULE_MD)


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
