#!/usr/bin/env bash
# check-mapping.sh — 메타 불변식: debt 번들의 ⟦CI:<id>⟧ 태그 ↔ checks/<id>.sh 1:1 정합.
# 번들이 '자기 강제력'에 대해 거짓말 못 하게 한다.
# 스캔 대상: debt.md + domains/*.md (있으면). placeholder ⟦CI:<id>⟧·일반 ⟦CI⟧ 는 미매칭.
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$DIR/.."

files=()
for f in "$SRC"/debt.md "$SRC"/domains/*.md; do
  [ -f "$f" ] && files+=("$f")
done
[ "${#files[@]}" -gt 0 ] || { echo "오류: 규칙 마크다운 없음: $SRC"; exit 2; }

tags=$(grep -hoE '⟦CI:[a-z][a-z-]*⟧' "${files[@]}" 2>/dev/null | sed -E 's/^⟦CI:(.*)⟧$/\1/' | sort -u)

fail=0
for id in $tags; do
  if [ ! -f "$DIR/$id.sh" ]; then
    echo "✗ 빈 약속: 태그 ⟦CI:$id⟧ 가 가리키는 checks/$id.sh 가 없음"; fail=1
  fi
done
for f in "$DIR"/*.sh; do
  base=$(basename "$f" .sh)
  [ "$base" = "check-mapping" ] && continue
  if ! printf '%s\n' "$tags" | grep -qx "$base"; then
    echo "✗ 고아 스크립트: checks/$base.sh 가 어느 ⟦CI⟧ 태그에도 안 걸림"; fail=1
  fi
done

n=$(printf '%s\n' "$tags" | grep -c .)
if [ "$fail" -eq 0 ]; then echo "✓ 강제 정합: ⟦CI⟧ 태그 ↔ checks/*.sh 1:1 ($n 개)"
else echo "— 약속된 태그 $n 개: $(echo "$tags" | tr '\n' ' ')"; fi
exit $fail
