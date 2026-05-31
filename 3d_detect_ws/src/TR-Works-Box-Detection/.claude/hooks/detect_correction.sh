#!/usr/bin/env bash
# Claude 실수 자동 기록 트리거.
#
# UserPromptSubmit 훅에서 호출됨. 사용자 입력에서 교정/지적 패턴이 감지되면
# 시스템 reminder 를 stdout 으로 주입해 Claude 가 docs/claude-mistake/<date>.md
# 에 항목을 추가하도록 유도한다.
#
# 동작 방침:
# - 패턴 매치 실패 / 입력 파싱 오류 시 무음 종료 (exit 0)
# - 오작동으로 사용자 흐름을 막지 않는다

set -u

input="$(cat)"

prompt="$(printf '%s' "$input" | python3 -c 'import sys,json
try:
    d=json.load(sys.stdin)
    print(d.get("prompt",""))
except Exception:
    pass' 2>/dev/null)"

[ -z "$prompt" ] && exit 0

if printf '%s' "$prompt" | grep -qE '(왜.*안.?했|왜.*했어|왜.*했을|잘못|실수|되돌|답답|맞는건지|지적|다시.?해|그렇게 하지|또 그|하지 말|왜 그런|이미 했|또 같은)'; then
  today="$(date +%Y-%m-%d)"
  cat <<EOF
<system-reminder>
[CLAUDE-MISTAKE-DETECTOR] 사용자 교정/지적 패턴 감지.

본 응답에서 사용자의 의도를 정확히 처리한 뒤, 만약 본 세션에 실수가 있었다면 다음을 자동 수행:

1. docs/claude-mistake/${today}.md 파일에 한 항목 추가 (파일 없으면 생성).
2. 형식: docs/claude-mistake/README.md 의 항목 형식 (시간 — 짧은 제목 / 무엇을 했는가 / 무엇이 잘못이었나 / 사용자 지적 / 사유·가설 / 재발 방지).
3. 재발 방지가 영구적이라면 ~/.claude/projects/-home-amap-TR-Works-TR-Works-ros2-ws/memory/feedback_*.md 도 갱신.
4. 기록 commit 은 docs/claude-mistake/ 만을 staged 한 별도 scope 으로 처리 (github.md "작업 단위 = 커밋 단위" 준수).

사용자에게 "기록할까요?" 묻지 말 것 — 자동 수행이 본 훅의 목적.
실수가 없거나 단순 질문이면 본 reminder 는 무시.
</system-reminder>
EOF
fi

exit 0
