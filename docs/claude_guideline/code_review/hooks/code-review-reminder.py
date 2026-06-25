#!/usr/bin/env python3
"""UserPromptSubmit 훅 — 코드 리뷰/분석 트리거 감지 시 review.md SOP 를 강제 주입.

배경: code_review 번들의 CLAUDE.md 등록은 '수동 포인터'라, 모델이 review.md 가
있다는 사실만 알고 능동적으로 열지 않은 채 일반 탐색+요약으로 직행하는 절차 실패가
발생한다. 본 훅은 트리거 키워드가 감지되면 응답 전 컨텍스트에 'review.md 를 먼저
Read 하고 9단계 SOP 를 따르라'는 구속력 있는 지시를 주입해 등록을 강제 게이트로 만든다.

계약(Claude Code UserPromptSubmit):
- stdin: JSON ({"prompt": ..., "cwd": ..., ...})
- stdout: 응답 작성 전 컨텍스트로 주입됨
- 항상 exit 0 — 사용자 입력을 절대 막지 않는다(트리거 불일치/오류 시 무출력).
"""
import json
import os
import sys

# 트리거 키워드 (review.md §트리거 기준 + 실사용 변종). 소문자 비교.
TRIGGERS = (
    "코드 리뷰", "코드리뷰", "리뷰해줘", "리뷰해 주세요", "리뷰 해줘",
    "리뷰 바람", "리뷰바람", "리뷰 부탁", "리뷰해", "리뷰 요청",
    "코드 분석", "코드분석", "분석해줘", "분석해 주세요", "분석 바람",
    "분석바람", "분석 부탁", "분석 요청",
    "code review", "review this", "review the", "analyze this", "analyse this",
)

REVIEW_MD = "docs/claude_guideline/code_review/review.md"

DIRECTIVE = """[CODE REVIEW SOP — 강제 게이트]
코드 리뷰/분석 트리거가 감지되었습니다. 일반 탐색+요약으로 직행하지 말고, 응답(또는 리뷰 수행) 전 반드시 아래를 선행하세요:

1. {review_md} 를 Read 한다 (생략 금지 — 등록 사실만 알고 건너뛰지 말 것).
2. 설치된 도메인이 있으면 docs/claude_guideline/code_review/domains/ 를 함께 적용한다.
3. review.md 의 9단계 SOP 를 순서대로 따른다:
   범위 식별 → 단위/전체 분기 → 도메인 감지 → 코어 인벤토리 5항목(목적·플로우·함수표·전역표·의존성) → 도메인 인벤토리 → severity 평가(Critical/High/Medium/Low/Info) → docs/code_review/<주제>/YYYY-MM-DD.md 기록 → 자체점검 grep → 1~2줄 결과 보고.
4. 인벤토리·severity 평가·산출물 기록을 생략한 '일반 리뷰'로 대체하지 않는다.""".format(
    review_md=REVIEW_MD
)


def main():
    raw = sys.stdin.read()
    try:
        data = json.loads(raw) if raw.strip() else {}
    except (json.JSONDecodeError, ValueError):
        data = {}

    prompt = str(data.get("prompt", ""))
    if not prompt:
        return  # 프롬프트 없음 — 무출력

    if not any(t in prompt.lower() for t in TRIGGERS):
        return  # 트리거 불일치 — 무출력(평소 프롬프트에 잡음 안 남김)

    # 활성화 게이트: review.md 가 없으면 본 지시 비활성(graceful).
    cwd = data.get("cwd") or os.getcwd()
    review_path = os.path.join(cwd, *REVIEW_MD.split("/"))
    if cwd and not os.path.isfile(review_path):
        return

    print(DIRECTIVE)


if __name__ == "__main__":
    main()
