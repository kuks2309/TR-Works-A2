#!/usr/bin/env bash
set -euo pipefail

# Claude 작업 지침 업데이트 스크립트
#
# 사용법:
#   bash docs/claude_guideline/update.sh           # 대화식
#   bash docs/claude_guideline/update.sh --auto    # 자동 (patch/minor)

RAW_URL="https://raw.githubusercontent.com/kuks2309/kuks_claude_setup/master/claude_guideline"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

CURRENT_VERSION="0.0.0"
if [ -f "$SCRIPT_DIR/VERSION" ]; then
  CURRENT_VERSION=$(tr -d '\n[:space:]' < "$SCRIPT_DIR/VERSION")
fi

UPSTREAM_VERSION=$(curl -fsSL "$RAW_URL/VERSION" | tr -d '\n[:space:]')

echo "현재 버전:    $CURRENT_VERSION"
echo "Upstream 버전: $UPSTREAM_VERSION"

if [ "$CURRENT_VERSION" = "$UPSTREAM_VERSION" ]; then
  echo "[OK] 이미 최신 버전입니다."
  exit 0
fi

# major 버전 변경 감지 (수동 승인 필수)
CURRENT_MAJOR="${CURRENT_VERSION%%.*}"
UPSTREAM_MAJOR="${UPSTREAM_VERSION%%.*}"

AUTO=false
if [ "${1:-}" = "--auto" ]; then
  AUTO=true
fi

if [ "$CURRENT_MAJOR" != "$UPSTREAM_MAJOR" ]; then
  echo ""
  echo "[!] MAJOR 버전 변경 감지: $CURRENT_VERSION -> $UPSTREAM_VERSION"
  echo "[!] 호환되지 않는 변경일 수 있으니 CHANGELOG 를 먼저 확인하세요:"
  echo "    $RAW_URL/CHANGELOG.md"
  AUTO=false
fi

if [ "$AUTO" = false ]; then
  read -r -p "업데이트하시겠습니까? (y/N) " ans
  if [ "$ans" != "y" ] && [ "$ans" != "Y" ]; then
    echo "취소되었습니다."
    exit 0
  fi
fi

# 백업
BACKUP_DIR="$SCRIPT_DIR/.backup-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$BACKUP_DIR"
for f in README.md github.md coding.md workflow.md documentation.md tech_debt.md VERSION CHANGELOG.md update.sh; do
  if [ -f "$SCRIPT_DIR/$f" ]; then
    cp "$SCRIPT_DIR/$f" "$BACKUP_DIR/$f"
  fi
done
echo "[+] 백업 완료: $BACKUP_DIR"

# 새 파일 다운로드 (local/ 은 건드리지 않음)
FILES=("README.md" "github.md" "coding.md" "workflow.md" "documentation.md" "tech_debt.md" "VERSION" "CHANGELOG.md" "update.sh")
for f in "${FILES[@]}"; do
  echo "[+] Downloading $f"
  curl -fsSL "$RAW_URL/$f" -o "$SCRIPT_DIR/$f"
done

chmod +x "$SCRIPT_DIR/update.sh"

echo ""
echo "[OK] 업데이트 완료: $CURRENT_VERSION -> $UPSTREAM_VERSION"
echo "[i] local/ 폴더는 그대로 유지됩니다."
