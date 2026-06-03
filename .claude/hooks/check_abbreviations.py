#!/usr/bin/env python3
"""Stop hook: block the reply if it contains a bare (un-spelled-out) abbreviation.

The user requires every abbreviation to be spelled out on first use as
"원어(한국어 의미, 약어)" (e.g. "joint trajectory controller(관절 궤적 컨트롤러, JTC)").
This hook reads the assistant's last message and, for each tracked abbreviation
that appears WITHOUT its spelled-out keyword anywhere in the message, blocks the
stop with a reminder so the model revises.

Loop guard: if stop_hook_active is already set, allow (remind at most once).
Edit ABBREVIATIONS below to extend the list.
"""
import json
import re
import sys

# abbreviation (word-boundary regex, case-sensitive) -> spell-out keyword that
# MUST co-occur (case-insensitive) when the abbreviation is used.
ABBREVIATIONS = [
    (r"\bJTC\b", "joint trajectory controller"),
    (r"\bTF\b", "transform"),
    (r"\bQP\b", "quadratic"),
    (r"\bCBF\b", "barrier function"),
    (r"\bEE\b", "end effector"),
    (r"\bIK\b", "inverse kinematic"),
    (r"\bFK\b", "forward kinematic"),
    (r"\bTCP\b", "tool center point"),
    (r"\bDOF\b", "degrees of freedom"),
    (r"\bRPY\b", "roll"),
]


def last_assistant_text(transcript_path):
    try:
        with open(transcript_path, encoding="utf-8") as f:
            lines = f.readlines()
    except Exception:
        return ""
    for line in reversed(lines):
        try:
            obj = json.loads(line)
        except Exception:
            continue
        if obj.get("type") == "assistant" or obj.get("role") == "assistant":
            msg = obj.get("message", obj)
            content = msg.get("content", "")
            if isinstance(content, list):
                t = " ".join(c.get("text", "") for c in content
                             if isinstance(c, dict) and c.get("type") == "text")
            else:
                t = str(content)
            if t.strip():
                return t
    return ""


def main():
    try:
        data = json.load(sys.stdin)
    except Exception:
        return 0
    if data.get("stop_hook_active"):
        return 0
    text = last_assistant_text(data.get("transcript_path", ""))
    if not text:
        return 0
    low = text.lower()
    bare = []
    for pat, keyword in ABBREVIATIONS:
        m = re.search(pat, text)
        if m and keyword.lower() not in low:
            bare.append(m.group(0))
    bare = sorted(set(bare))
    if bare:
        reason = (
            "응답에 풀어쓰지 않은 약어가 있습니다: " + ", ".join(bare) + ". "
            "첫 등장 시 '원어(한국어 의미, 약어)' 형태로 풀어쓴 뒤 다시 답하세요. "
            "예: joint trajectory controller(관절 궤적 컨트롤러, JTC), "
            "transform/좌표 변환(TF)."
        )
        print(json.dumps({"decision": "block", "reason": reason}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
