#!/usr/bin/env python3
"""UserPromptSubmit 훅 — 모든 사용자 지시 도착 시 recording 규칙을 상시 주입(키워드 게이트 없음).

배경: CLAUDE.md 등록이 '수동 포인터'라 모델이 recording.md 를 능동적으로 열지 않고 지시
기록을 건너뛰거나 작업 후 일괄 기록하는 절차 실패가 발생한다. user_instruction 은 '모든
지시를 즉시 기록'이 목적이므로 트리거 키워드 없이 매 프롬프트 상시 주입한다(acronym 과 동일).

계약(Claude Code UserPromptSubmit): stdin JSON → stdout 이 컨텍스트로 주입. 항상 exit 0.
"""
import json
import os
import sys

RULE_MD = "docs/claude_guideline/user_instruction/recording.md"

DIRECTIVE = """[USER-INSTRUCTION 기록 — 상시 게이트]
사용자 지시가 도착했습니다. {rule} 규칙에 따라 응답 전 다음을 수행하세요:

1. 이 지시의 원문을 docs/user_instructions/user_instructions.md 맨 위에 즉시 prepend 기록한다(폴더·파일 없으면 생성, 승인 불요).
2. 작업 완료 후 일괄 기록 금지 — 도착 즉시 기록한다.
3. 원문 캡처만 한다. 분석·요약·결론·결과는 섞지 않는다(의도 부채 방지).""".format(rule=RULE_MD)


def main():
    raw = sys.stdin.read()
    try:
        data = json.loads(raw) if raw.strip() else {}
    except (json.JSONDecodeError, ValueError):
        data = {}

    prompt = str(data.get("prompt", ""))
    if not prompt:
        return  # 빈 프롬프트는 기록 대상 아님

    # 활성화 게이트: recording.md 가 없으면 비활성(graceful). 키워드 게이트는 없음(상시).
    cwd = data.get("cwd") or os.getcwd()
    if cwd and not os.path.isfile(os.path.join(cwd, *RULE_MD.split("/"))):
        return

    print(DIRECTIVE)


if __name__ == "__main__":
    main()
