#!/usr/bin/env python3
"""UserPromptSubmit 훅 — 외부 참조 문서(매뉴얼·datasheet·SDK·표준) 트리거 시 SOP 강제 주입.

배경: CLAUDE.md 등록이 '수동 포인터'라 모델이 handling.md 를 능동적으로 열지 않고
원문 대조 없이 기억으로 답하는(환각) 절차 실패가 발생한다. 본 훅이 트리거 감지 시
보관·인용·검증 규칙을 응답 전에 강제한다.

계약(Claude Code UserPromptSubmit): stdin JSON → stdout 이 컨텍스트로 주입. 항상 exit 0.
"""
import json
import os
import sys

TRIGGERS = (
    "매뉴얼", "datasheet", "데이터시트", "데이터 시트", "data sheet", "데이터북",
    "sdk", "표준", "스펙", "사양", "레지스터 맵", "register map",
    "참조 문서", "참조문서", "reference manual", "user manual",
    "application note", "앱노트", "프로토콜 명세", "규격서",
)

RULE_MD = "docs/claude_guideline/external_reference/handling.md"

DIRECTIVE = """[EXTERNAL-REFERENCE SOP — 강제 게이트]
외부 참조 문서(매뉴얼·datasheet·SDK·표준) 트리거가 감지되었습니다. 응답 전 반드시 아래를 선행하세요:

1. {rule} 를 Read 한다 (등록 사실만 알고 건너뛰지 말 것).
2. 참조 자료(PDF 등)는 프로젝트 루트 references/ 에 보관(docs 와 분리)하고, 인용 시 출처·페이지·버전을 명시한다.
3. 인용 내용을 코드/문서에 반영하기 전 원문과 대조 검증한다 — 기억에 의존한 추정(환각) 금지.""".format(rule=RULE_MD)


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
