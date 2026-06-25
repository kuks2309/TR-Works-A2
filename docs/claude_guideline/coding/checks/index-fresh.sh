#!/usr/bin/env bash
# index-fresh.sh — 함수 인덱스가 코드와 일치하는지 재도출 (⟦CI:index-fresh⟧).
# 코드에서 함수명 인덱스를 재생성(LC_ALL=C 정렬·LF·중복제거)해 커밋된 인덱스와 diff.
# 사용:
#   index-fresh.sh [<target>] [<idx>]            # 검사 (기본 idx = <target>/FUNCTIONS.idx)
#   index-fresh.sh --generate [<target>] [<idx>] # 인덱스 생성/갱신
# 인덱스 파일 없으면 graceful 생략.
set -uo pipefail

if [ "${1:-}" = "--generate" ]; then GEN=1; TARGET="${2:-.}"; IDX="${3:-$TARGET/FUNCTIONS.idx}"
else GEN=0; TARGET="${1:-.}"; IDX="${2:-$TARGET/FUNCTIONS.idx}"; fi

gen() {
  grep -rhoE '^[[:space:]]*def[[:space:]]+[A-Za-z_][A-Za-z0-9_]*' --include=*.py "$TARGET" 2>/dev/null \
    | sed -E 's/.*def[[:space:]]+//' | LC_ALL=C sort -u
}

if [ "$GEN" = 1 ]; then
  gen > "$IDX"; echo "✓ 인덱스 생성: $IDX ($(wc -l < "$IDX" | tr -d ' ') 심볼)"; exit 0
fi

[ -f "$IDX" ] || { echo "• 인덱스 파일 없음($IDX) — 검사 생략. '--generate' 로 생성"; exit 0; }

cur=$(mktemp); gen > "$cur"
if ! diff -q "$IDX" "$cur" >/dev/null 2>&1; then
  echo "✗ [index] stale — 코드와 $IDX 불일치 (< 인덱스 / > 코드):"
  diff "$IDX" "$cur" | sed 's/^/   /' | head -20
  rm -f "$cur"; exit 1
fi
rm -f "$cur"
echo "✓ 인덱스 최신 ($IDX)"
exit 0
