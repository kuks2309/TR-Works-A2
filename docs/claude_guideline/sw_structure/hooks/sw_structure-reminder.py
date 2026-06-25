#!/usr/bin/env python3
"""UserPromptSubmit 훅 — SW 구조/구조 분석/클래스·호출 관계 트리거 시 sw_structure SOP 강제 주입.

배경: CLAUDE.md 등록이 '수동 포인터'라 모델이 structure.md 를 능동적으로 열지 않고 그래프·
다이어그램 없이 산문 설명으로 직행하는 절차 실패가 발생한다. 본 훅이 트리거 감지 시 구조
시각화 SOP 를 응답 전에 강제한다.

계약(Claude Code UserPromptSubmit): stdin JSON → stdout 이 컨텍스트로 주입. 항상 exit 0.
"""
import json
import os
import sys

TRIGGERS = (
    "sw 구조", "소프트웨어 구조", "구조 분석", "구조 시각화", "구조도",
    "클래스 관계", "클래스 다이어그램", "호출 관계", "호출 그래프",
    "call graph", "콜 그래프", "의존 그래프", "의존성 그래프",
    "시퀀스 다이어그램", "파일 그래프", "아키텍처 다이어그램", "structure diagram",
)

RULE_MD = "docs/claude_guideline/sw_structure/structure.md"

DIRECTIVE = """[SW-STRUCTURE SOP — 강제 게이트]
SW 구조/구조 분석/클래스·호출 관계 트리거가 감지되었습니다. 응답 전 반드시 아래를 선행하세요:

1. {rule} 를 Read 한다 (등록 사실만 알고 건너뛰지 말 것).
2. 파일 의존 그래프 + 클래스 다이어그램 + 시퀀스 다이어그램 + 연결 관계표 + 구조 관찰(순환·고립)을 작성한다.
3. 산출물은 docs/sw_structure/<주제>/YYYY-MM-DD.md (날짜=버전).
4. 결함·품질 평가는 code_review 소관 — 본 번들은 연결 시각화만 한다.""".format(rule=RULE_MD)


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
