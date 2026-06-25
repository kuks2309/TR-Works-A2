#!/usr/bin/env bash
# memory.sh — 메모리 안전을 '코드에서' 재도출 (⟦CI:memory⟧).
#   (1) 정적: clang-tidy 메모리 검사(누수·use-after-free·new/delete·미초기화).
#   (2) 런타임: MEMORY_TEST_CMD 지정 시 그 테스트 실행(프로젝트가 -fsanitize=address 빌드 권장).
# 자기보고와 무관하게 도구가 실제 코드를 분석/실행하므로 거짓✅로 우회 불가.
# 도구/대상 없으면 graceful skip(강제력 0, 정직히 알림).
set -uo pipefail

TARGET="${1:-.}"
[ -e "$TARGET" ] || { echo "오류: 대상 없음: $TARGET"; exit 2; }

fail=0; ran=0
MEM_CHECKS='-*,clang-analyzer-unix.Malloc,clang-analyzer-cplusplus.NewDelete,clang-analyzer-cplusplus.NewDeleteLeaks,clang-analyzer-core.uninitialized.UndefReturn,bugprone-use-after-move'

# (1) 정적 — clang-tidy 메모리 분석
if command -v clang-tidy >/dev/null 2>&1; then
  while IFS= read -r f; do
    [ -n "$f" ] || continue
    ran=1
    case "$f" in *.c) STD="-std=c11";; *) STD="-std=c++17";; esac
    if [ -f "$TARGET/compile_commands.json" ]; then
      out=$(clang-tidy -p "$TARGET" --checks="$MEM_CHECKS" "$f" 2>/dev/null)
    else
      out=$(clang-tidy --checks="$MEM_CHECKS" "$f" -- -I"$TARGET" "$STD" 2>/dev/null)
    fi
    hits=$(printf '%s\n' "$out" | grep -E 'warning:|error:')
    if [ -n "$hits" ]; then
      printf '%s\n' "$hits" | sed "s/^/✗ [memory] /"
      fail=1
    fi
  done < <(find "$TARGET" -type f \( -name '*.c' -o -name '*.cc' -o -name '*.cpp' \) 2>/dev/null)
fi

# (2) 런타임 — AddressSanitizer (프로젝트가 테스트 명령 지정)
if [ -n "${MEMORY_TEST_CMD:-}" ]; then
  ran=1
  if ! ASAN_OPTIONS="detect_leaks=1" bash -c "$MEMORY_TEST_CMD"; then
    echo "✗ [memory] 런타임 테스트 실패(asan/leak)"; fail=1
  fi
fi

if [ "$ran" -eq 0 ]; then
  echo "• 메모리 도구(clang-tidy) 없거나 C/C++ 대상 없음 — 검사 생략(강제력 0). MEMORY_TEST_CMD 로 asan 테스트 권장"
  exit 0
fi
[ "$fail" -eq 0 ] && echo "✓ 메모리 검사 통과 ($TARGET)"
exit $fail
