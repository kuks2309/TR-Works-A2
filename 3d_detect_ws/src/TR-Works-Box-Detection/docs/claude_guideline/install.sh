#!/usr/bin/env bash
set -euo pipefail

# Claude 작업 지침 설치 스크립트
#
# 사용법 (프로젝트 루트에서):
#   bash <(curl -sSL https://raw.githubusercontent.com/kuks2309/kuks_claude_setup/master/claude_guideline/install.sh)

RAW_URL="https://raw.githubusercontent.com/kuks2309/kuks_claude_setup/master/claude_guideline"
TARGET_DIR="docs/claude_guideline"

if [ ! -d ".git" ]; then
  echo "[!] 현재 디렉토리는 git 저장소가 아닙니다. 프로젝트 루트에서 실행하세요."
  exit 1
fi

if [ -d "$TARGET_DIR" ]; then
  echo "[!] $TARGET_DIR 이미 존재합니다. 업데이트하려면 $TARGET_DIR/update.sh 를 사용하세요."
  exit 1
fi

mkdir -p "$TARGET_DIR/local"

FILES=("README.md" "github.md" "coding.md" "workflow.md" "documentation.md" "VERSION" "CHANGELOG.md" "update.sh")
for f in "${FILES[@]}"; do
  echo "[+] Downloading $f"
  curl -fsSL "$RAW_URL/$f" -o "$TARGET_DIR/$f"
done

chmod +x "$TARGET_DIR/update.sh"

cat > "$TARGET_DIR/local/README.md" <<'EOF'
# 프로젝트별 Override

이 폴더의 파일은 `update.sh` 실행 시 덮어쓰지 않습니다.
프로젝트 고유의 지침은 여기에 별도 파일로 작성하세요.

CLAUDE.md 의 "문서 작업 규칙" 섹션에서 진입 링크를 추가합니다.
EOF

echo ""
echo "[OK] 설치 완료: $TARGET_DIR/"
echo ""
echo "다음 단계:"
echo "  1. CLAUDE.md 의 '문서 작업 규칙' 섹션에 다음 링크를 추가하세요:"
echo "       - 진입점 → docs/claude_guideline/README.md"
echo "  2. 또는 templates/CLAUDE.md.template 을 참고해 새 CLAUDE.md 작성"
echo "  3. 업데이트: bash $TARGET_DIR/update.sh"
