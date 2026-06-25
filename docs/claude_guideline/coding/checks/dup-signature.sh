#!/usr/bin/env bash
# dup-signature.sh — 중복 함수명/시그니처를 코드에서 재도출 (⟦CI:dup-signature⟧).
# 같은 함수명이 2곳 이상 정의되면 차단(휴리스틱). .dup-allow(한 줄당 함수명) 예외.
# Python def 는 신뢰도 높음, C/C++ 는 best-effort(정당한 오버로드는 .dup-allow).
set -uo pipefail

TARGET="${1:-.}"
ALLOW="$TARGET/.dup-allow"
names=$(mktemp)

# Python: def name(
grep -rhnE '^[[:space:]]*def[[:space:]]+[A-Za-z_][A-Za-z0-9_]*[[:space:]]*\(' --include=*.py "$TARGET" 2>/dev/null \
  | sed -E 's/.*def[[:space:]]+([A-Za-z_][A-Za-z0-9_]*).*/\1/' >> "$names"

# C/C++: <type> name(args) (정의 줄), 제어문 제외
grep -rhnE '^[A-Za-z_][A-Za-z0-9_<>:\*&[:space:]]+[[:space:]][A-Za-z_][A-Za-z0-9_]*[[:space:]]*\([^;]*\)[[:space:]]*\{?[[:space:]]*$' \
  --include=*.c --include=*.cc --include=*.cpp "$TARGET" 2>/dev/null \
  | grep -vE '^[[:space:]]*(return|if|while|for|switch|else|do)\b' \
  | sed -E 's/.*[[:space:]]([A-Za-z_][A-Za-z0-9_]*)[[:space:]]*\(.*/\1/' >> "$names"

dups=$(LC_ALL=C sort "$names" | uniq -d)
rm -f "$names"

# .dup-allow 예외 제거
if [ -f "$ALLOW" ] && [ -n "$dups" ]; then
  dups=$(printf '%s\n' "$dups" | grep -vxF -f "$ALLOW" || true)
fi
dups=$(printf '%s\n' "$dups" | grep -v '^[[:space:]]*$' || true)

if [ -n "$dups" ]; then
  echo "✗ [dup] 중복 함수명(시그니처 충돌 후보):"
  printf '%s\n' "$dups" | sed 's/^/   - /'
  echo "  (정당한 오버로드/재정의면 .dup-allow 에 함수명 추가)"
  exit 1
fi
echo "✓ 중복 함수 시그니처 없음"
exit 0
