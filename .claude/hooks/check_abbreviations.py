#!/usr/bin/env python3
"""Stop hook: block the reply if any abbreviation is used WITHOUT an inline
expansion, on EVERY occurrence.

User rule (mandatory): every time an abbreviation appears it must be written as
"ABBR(Full Words)" — e.g. "JTC(Joint Trajectory Controller)". Not just the first
use: EVERY use. So a bare abbreviation token that is not immediately followed by
"(" is a violation.

Detection: for each tracked abbreviation token, if it appears as a standalone
word that is NOT immediately followed by "(" (optionally after one space), the
reply is blocked with a reminder showing the required form.

Loop guard: if stop_hook_active is already set, allow (remind at most once).
Edit ABBREVIATIONS below to extend the list.
"""
import json
import re
import sys

# token -> full words to show in the required "ABBR(Full Words)" form.
ABBREVIATIONS = [
    ("JTC", "Joint Trajectory Controller"),
    ("TF", "Transform"),
    ("QP", "Quadratic Programming"),
    ("CBF", "Control Barrier Function"),
    ("EE", "End Effector"),
    ("IK", "Inverse Kinematics"),
    ("FK", "Forward Kinematics"),
    ("TCP", "Tool Center Point"),
    ("DOF", "Degrees Of Freedom"),
    ("RPY", "Roll Pitch Yaw"),
    ("SIL", "Software In the Loop"),
    ("HIL", "Hardware In the Loop"),
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
    bad = []
    for abbr, full in ABBREVIATIONS:
        # A standalone occurrence NOT immediately followed by "(" (optionally
        # one space) is a bare abbreviation -> violation.
        if re.search(rf"\b{re.escape(abbr)}\b(?!\s?\()", text):
            bad.append(f"{abbr}({full})")
    bad = sorted(set(bad))
    if bad:
        reason = (
            "약자는 매번 'ABBR(Full Words)' 형태로 병기해야 합니다(첫 등장만이 "
            "아니라 모든 등장). 병기 없이 단독으로 쓰인 약자가 있습니다. "
            "다음처럼 고쳐 다시 답하세요: " + ", ".join(bad)
        )
        print(json.dumps({"decision": "block", "reason": reason}))
    return 0


if __name__ == "__main__":
    sys.exit(main())
