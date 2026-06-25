#!/usr/bin/env python3
"""UserPromptSubmit 훅 — git 작업(commit/push/merge/PR/branch) 트리거 시 git_workflow SOP 강제 주입.

배경: CLAUDE.md 등록이 '수동 포인터'라 모델이 git_workflow.md 를 능동적으로 열지 않고
임의 커밋/푸시로 직행하는 절차 실패가 발생한다. 본 훅이 트리거 감지 시 모드 판정·커밋
규약·원격 정책을 응답 전에 강제한다.

계약(Claude Code UserPromptSubmit): stdin JSON → stdout 이 컨텍스트로 주입. 항상 exit 0.
"""
import json
import os
import sys

TRIGGERS = (
    "커밋", "commit", "푸시", "push", "머지", "merge",
    "리베이스", "rebase", "pull request", "풀리퀘스트", "풀리퀘",
    "pr 올려", "pr 생성", "pr 만들", "pr 보내",
    "체크아웃", "checkout", "브랜치", "branch", "stash", "스태시",
    "cherry-pick", "체리픽", "force push", "force-push", "포스 푸시",
)

RULE_MD = "docs/claude_guideline/git_workflow/git_workflow.md"

DIRECTIVE = """[GIT-WORKFLOW SOP — 강제 게이트]
git 작업(commit/push/merge/PR/branch) 트리거가 감지되었습니다. 응답 전 반드시 아래를 선행하세요:

1. {rule} 를 Read 한다 (등록 사실만 알고 건너뛰지 말 것).
2. solo/team 모드를 판정하고, 커밋 규약(type(scope): subject + Co-Authored-By), 명시 staging(-A/. 금지), 다중 원격 push 정책을 따른다.
3. team 모드면 main 직접 push 금지 — 브랜치 → PR → 리뷰 ≥1 승인 → merge, 작성자 self-approve 금지.
4. 공유 브랜치 force-push 금지, 충돌은 로컬 rebase.""".format(rule=RULE_MD)


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
