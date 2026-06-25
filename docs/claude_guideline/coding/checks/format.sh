#!/usr/bin/env bash
# format-check.sh — 프로젝트 포맷터를 검사 모드로 돌려 포맷 위반 차단 (⟦CI:format⟧).
# 포맷 '선택'은 프로젝트 설정 파일(.clang-format / pyproject.toml / .prettierrc 등)이 결정한다.
# 이 스크립트는 그 선택대로 지켜졌는지만 기계 검증 — 선택 가이드는 stack.md.
# 포맷터/설정이 없으면 graceful 하게 생략(강제력 0, 정직히 알림).
set -uo pipefail

TARGET="${1:-.}"
[ -e "$TARGET" ] || { echo "오류: 대상 없음: $TARGET"; exit 2; }

fail=0
ran=0

# C/C++ — .clang-format 있고 clang-format 설치 시
if [ -f "$TARGET/.clang-format" ] && command -v clang-format >/dev/null 2>&1; then
  ran=1
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    if ! clang-format --dry-run --Werror "$f" >/dev/null 2>&1; then
      echo "✗ [format] $f (clang-format 위반)"; fail=1
    fi
  done < <(find "$TARGET" -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' \
            -o -name '*.h' -o -name '*.hpp' \) 2>/dev/null)
fi

# Python — black 설치 시
if command -v black >/dev/null 2>&1 && find "$TARGET" -name '*.py' -print -quit 2>/dev/null | grep -q .; then
  ran=1
  if ! black --check -q "$TARGET" >/dev/null 2>&1; then
    echo "✗ [format] black --check 위반"; fail=1
  fi
fi

# JS/TS — prettier + 설정 있을 시
if command -v prettier >/dev/null 2>&1 && ls "$TARGET"/.prettierrc* >/dev/null 2>&1; then
  ran=1
  if ! prettier --check "$TARGET" >/dev/null 2>&1; then
    echo "✗ [format] prettier --check 위반"; fail=1
  fi
fi

if [ "$ran" -eq 0 ]; then
  echo "• 포맷터/설정 없음 — 검사 생략(강제력 0). stack.md 에 포맷터 선택 권장"
  exit 0
fi
[ "$fail" -eq 0 ] && echo "✓ 포맷 준수 ($TARGET)"
exit $fail
