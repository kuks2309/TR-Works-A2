#!/usr/bin/env bash
# tests-ran.sh — 테스트 실행·통과 증거를 재도출 (⟦CI:tests-ran⟧).
#   TEST_CMD 지정 시 그 명령 실행(exit 0 요구). 아니면 pytest(tests/ 존재) 자동.
# "변경 공개함수마다 테스트 ≥1" 은 커버리지 매핑이 필요해 ⟦권고⟧ — 본 이빨은
# '테스트가 실제로 돌고 통과했다'를 강제(거짓 ✅ 차단). 도구 없으면 graceful 생략.
set -uo pipefail

TARGET="${1:-.}"

if [ -n "${TEST_CMD:-}" ]; then
  echo "• 테스트: $TEST_CMD"
  if bash -c "$TEST_CMD"; then echo "✓ 테스트 PASS"; exit 0
  else echo "✗ [tests] 테스트 실패"; exit 1; fi
fi

if command -v pytest >/dev/null 2>&1 && find "$TARGET" -type d -name tests -print -quit 2>/dev/null | grep -q .; then
  if pytest -q "$TARGET" >/dev/null 2>&1; then echo "✓ pytest PASS"; exit 0
  else echo "✗ [tests] pytest 실패"; exit 1; fi
fi

echo "• 테스트 명령 없음(TEST_CMD 미지정 · pytest/tests 없음) — 검사 생략(강제력 0)"
exit 0
