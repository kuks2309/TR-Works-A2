#!/usr/bin/env bash
# adr-fields.sh — ADR(설계 결정 기록) 필수 필드 존재를 재도출 (⟦CI:adr-fields⟧).
# 모든 ADR 은 Status·Context·Decision·Consequences·Rollback 을 가져야 한다.
# (Rollback 은 "N/A (가역)" 이라도 명시 — §3 비가역 변경 대비.)
# ADR 위치: */adr/*.md, */decisions/*.md (인자로 루트 지정, 기본 .).
set -uo pipefail

TARGET="${1:-.}"
REQUIRED=(Status Context Decision Consequences Rollback)
fail=0; ran=0

while IFS= read -r adr; do
  [ -n "$adr" ] || continue
  ran=1
  for fld in "${REQUIRED[@]}"; do
    if ! grep -qiE "(^#+[[:space:]]*${fld}|^\*\*${fld}|^${fld}[[:space:]]*:)" "$adr"; then
      echo "✗ [adr] $adr — '${fld}' 필드 없음"; fail=1
    fi
  done
done < <(find "$TARGET" -type f \( -path '*/adr/*.md' -o -path '*/decisions/*.md' \) 2>/dev/null)

if [ "$ran" -eq 0 ]; then
  echo "• ADR 파일 없음(*/adr/*.md · */decisions/*.md) — 검사 생략"
  exit 0
fi
[ "$fail" -eq 0 ] && echo "✓ ADR 필수 필드 충족"
exit $fail
