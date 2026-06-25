#!/usr/bin/env bash
# debt-marker.sh — 코드의 TODO/FIXME/HACK 가 debt id 를 참조하는지 재도출 (⟦CI:debt-marker⟧).
#   OK:   TODO(debt-042): ...   FIXME[debt-7]   HACK: debt-12 ...
#   차단:  맨 TODO/FIXME/HACK (debt 미등록) → registry 등록 강제.
# 자기보고와 무관하게 실제 코드를 grep 하므로 거짓✅로 우회 불가.
set -uo pipefail

TARGET="${1:-.}"
[ -e "$TARGET" ] || { echo "오류: 대상 없음: $TARGET"; exit 2; }

hits=$(grep -rnE '\b(TODO|FIXME|HACK)\b' \
  --include=*.py --include=*.c --include=*.cc --include=*.cpp --include=*.h --include=*.hpp \
  --include=*.js --include=*.ts --include=*.sh "$TARGET" 2>/dev/null || true)

[ -n "$hits" ] || { echo "✓ TODO/FIXME/HACK 마커 없음"; exit 0; }

# debt id 미참조 마커만 (debt-42 · debt_42 · debt 42 형태 허용)
bad=$(printf '%s\n' "$hits" | grep -ivE 'debt[-_ ]?[0-9]+' || true)

if [ -n "$bad" ]; then
  echo "✗ [debt] 미등록 마커(debt id 참조 없음) — registry 등록 후 'debt-<id>' 부착:"
  printf '%s\n' "$bad" | sed 's/^/   /' | head -20
  exit 1
fi
echo "✓ 모든 TODO/FIXME/HACK 가 debt id 참조"
exit 0
