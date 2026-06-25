#!/usr/bin/env bash
# check-mapping.sh — 메타 불변식: 번들 마크다운의 ⟦CI:<id>⟧ 태그 ↔ checks/<id>.sh 1:1 정합.
# 번들이 '자기 강제력'에 대해 거짓말 못 하게 한다.
#   - 태그가 약속한 스크립트가 없으면(빈 약속)  → 실패
#   - 스크립트가 어느 태그에도 안 걸리면(고아)   → 실패
# 스캔 대상: 코어(coding.md)·stack.md·conventions.md·domains/*.md (모든 규칙 마크다운).
# placeholder ⟦CI:<id>⟧ 는 <id> 가 [a-z] 로 시작 안 하므로 자동 제외.
# 일반 ⟦CI⟧(:id 없음)도 매칭 안 됨 — '후보(미구현)' 표기에 안전.
set -uo pipefail

DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$DIR/.."

# 스캔할 규칙 마크다운 수집 (존재하는 것만)
files=()
for f in "$SRC"/coding.md "$SRC"/stack.md "$SRC"/conventions.md "$SRC"/domains/*.md; do
  [ -f "$f" ] && files+=("$f")
done
[ "${#files[@]}" -gt 0 ] || { echo "오류: 규칙 마크다운 없음: $SRC"; exit 2; }

# 1) 약속한 태그 id (실제 id 만; <id> placeholder 제외)
tags=$(grep -hoE '⟦CI:[a-z][a-z-]*⟧' "${files[@]}" 2>/dev/null | sed -E 's/^⟦CI:(.*)⟧$/\1/' | sort -u)

fail=0

# 2) 태그 → 백킹 스크립트 존재?
for id in $tags; do
  if [ ! -f "$DIR/$id.sh" ]; then
    echo "✗ 빈 약속: 태그 ⟦CI:$id⟧ 가 가리키는 checks/$id.sh 가 없음"
    fail=1
  fi
done

# 3) 스크립트 → 태그에 참조됨? (자기 자신 제외)
for f in "$DIR"/*.sh; do
  base=$(basename "$f" .sh)
  [ "$base" = "check-mapping" ] && continue
  if ! printf '%s\n' "$tags" | grep -qx "$base"; then
    echo "✗ 고아 스크립트: checks/$base.sh 가 어느 ⟦CI⟧ 태그에도 안 걸림"
    fail=1
  fi
done

n=$(printf '%s\n' "$tags" | grep -c .)
if [ "$fail" -eq 0 ]; then
  echo "✓ 강제 정합: ⟦CI⟧ 태그 ↔ checks/*.sh 1:1 ($n 개, 코어+도메인 스캔)"
else
  echo "— 약속된 태그 $n 개: $(echo "$tags" | tr '\n' ' ')"
fi
exit $fail
