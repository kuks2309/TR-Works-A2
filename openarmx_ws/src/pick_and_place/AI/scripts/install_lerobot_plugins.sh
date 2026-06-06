#!/usr/bin/env bash
# AI 백엔드 LeRobot 플러그인 설치 헬퍼 (China 워크스페이스 적응)
#
# follower/leader 플러그인은 colcon(ament) 패키지가 아니라 pip 패키지이며,
# --robot.type=openarmx_follower_ros2 / --teleop.type=openarmx_leader_ros2 의
# entry-point 가 인식되려면 LeRobot 환경(lerobot-env)에 설치되어 있어야 한다.
#
# 사용:
#   lerobot-env          # 먼저 LeRobot 환경 진입
#   bash scripts/install_lerobot_plugins.sh
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
AI_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)

if ! command -v pip >/dev/null 2>&1; then
  echo "[FAIL] pip 을 찾을 수 없습니다. 먼저 'lerobot-env' 로 LeRobot 환경에 진입하세요." >&2
  exit 1
fi

echo "[INFO] follower 플러그인 설치: $AI_DIR/lerobot_robot_openarmx_follower_ros2"
pip install -e "$AI_DIR/lerobot_robot_openarmx_follower_ros2"

echo "[INFO] leader 플러그인 설치: $AI_DIR/lerobot_teleoperator_openarmx_leader_ros2"
pip install -e "$AI_DIR/lerobot_teleoperator_openarmx_leader_ros2"

echo "[OK] 설치 완료. entry-point 확인:"
python - <<'PY'
try:
    from importlib.metadata import entry_points
    eps = entry_points()
    def names(group):
        try:
            return [e.name for e in eps.select(group=group)]
        except Exception:
            return [e.name for e in eps.get(group, [])]
    print("  lerobot.robots        :", names("lerobot.robots"))
    print("  lerobot.teleoperators :", names("lerobot.teleoperators"))
except Exception as e:
    print("  (entry-point 조회 실패:", e, ")")
PY
