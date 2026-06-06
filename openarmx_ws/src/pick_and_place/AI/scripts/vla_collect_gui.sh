#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
REPO_DIR=$(cd -- "$SCRIPT_DIR/.." && pwd)
CONFIG_FILE=${VLA_CONFIG_FILE:-"$REPO_DIR/config/vla_collect.env"}

if [[ ! -f "$CONFIG_FILE" ]]; then
  echo "配置文件不存在: $CONFIG_FILE" >&2
  exit 1
fi

set -a
source "$CONFIG_FILE"
set +a

: "${WORKSPACE_DIR:?请在配置文件中设置 WORKSPACE_DIR}"
: "${LEROBOT_ENV_CMD:?请在配置文件中设置 LEROBOT_ENV_CMD}"
: "${GUI_TERMINAL:?请在配置文件中设置 GUI_TERMINAL}"
: "${OPEN_WINDOW_INTERVAL_S:?请在配置文件中设置 OPEN_WINDOW_INTERVAL_S}"
: "${STATE_FILE:?请在配置文件中设置 STATE_FILE}"

usage() {
  cat <<USAGE
用法:
  bash scripts/vla_collect_gui.sh base
  bash scripts/vla_collect_gui.sh base_camera
  bash scripts/vla_collect_gui.sh collect
  bash scripts/vla_collect_gui.sh stop
  bash scripts/vla_collect_gui.sh check

说明:
  base         只启动机器人真机 + Pico Bridge + VR Teleop
  base_camera  启动机器人底层 + 相机发布
  collect      启动机器人底层 + 相机发布 + LeRobot 数采
  stop         一键关闭本脚本打开的所有终端
  check        只做启动前自检，不弹出终端窗口
USAGE
}

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "缺少命令: $1" >&2
    exit 1
  fi
}

quote_cmd() {
  local quoted=()
  local arg q
  for arg in "$@"; do
    printf -v q '%q' "$arg"
    quoted+=("$q")
  done
  printf '%s' "${quoted[*]}"
}

optional_arg() {
  local -n ref=$1
  local flag=$2
  local value=$3
  if [[ -n "$value" ]]; then
    ref+=("$flag=$value")
  fi
}

build_bringup_cmd() {
  quote_cmd \
    ros2 launch openarmx_bringup openarmx.bimanual.launch.py \
    "control_mode:=$CONTROL_MODE" \
    "robot_controller:=$ROBOT_CONTROLLER" \
    "use_fake_hardware:=$USE_FAKE_HARDWARE"
}

# China 워크스페이스 적응: VR(Pico) 대신 물리 leader arm 텔레오퍼레이션 사용.
# teleop_bimanual_node 가 leader CAN(can0/can1)에서 8DOF(7관절+그리퍼)를 읽어
# /{left,right}_forward_position_controller/commands 로 발행한다.
# (별도 bridge 노드 불필요 — CAN 직접 읽음)
build_teleop_cmd() {
  quote_cmd \
    ros2 launch openarmx_teleop_bimanual teleop_bimanual.launch.py \
    "leader_right_can:=$LEADER_RIGHT_CAN" \
    "leader_left_can:=$LEADER_LEFT_CAN" \
    "follower_right_prefix:=$FOLLOWER_RIGHT_PREFIX" \
    "follower_left_prefix:=$FOLLOWER_LEFT_PREFIX" \
    "control_rate_hz:=$CONTROL_RATE_HZ"
}

# China 워크스페이스 적응: 3캠 D405 camera_publisher 대신 기존 중앙 D435 launch 사용.
# d435_camera.launch.py 는 realsense 기본 토픽(/camera/camera/...)으로 발행한다(인자 없음).
build_camera_cmd() {
  quote_cmd ros2 launch openarmx_scenario_player d435_camera.launch.py
}

build_record_cmd() {
  local camera_spec
  # 단일 중앙(헤드) D435 — d435_camera.launch.py 가 발행하는 realsense 기본 토픽 구독.
  # depth 는 미기록이므로 use_depth=false. W/H/FPS 는 실제 d435 스트림과 일치시킬 것.
  camera_spec="{cam_head: {type: ros2, image_topic: /camera/camera/color/image_raw, use_depth: false, width: $W, height: $H, fps: $FPS}}"

  local args=(
    lerobot-record
    "--robot.type=openarmx_follower_ros2"
    "--robot.cameras=$camera_spec"
    "--teleop.type=openarmx_leader_ros2"
    "--dataset.repo_id=$DATASET_REPO_ID"
    "--dataset.single_task=$DATASET_SINGLE_TASK"
    "--dataset.num_episodes=$DATASET_NUM_EPISODES"
    "--dataset.episode_time_s=$DATASET_EPISODE_TIME_S"
    "--dataset.reset_time_s=$DATASET_RESET_TIME_S"
    "--dataset.push_to_hub=$DATASET_PUSH_TO_HUB"
    "--display_data=$DISPLAY_DATA"
  )

  optional_arg args --dataset.root "$DATASET_ROOT"
  optional_arg args --dataset.fps "$DATASET_FPS"
  optional_arg args --dataset.video "$DATASET_VIDEO"
  optional_arg args --dataset.vcodec "$DATASET_VCODEC"
  optional_arg args --dataset.video_encoding_batch_size "$DATASET_VIDEO_ENCODING_BATCH_SIZE"
  optional_arg args --dataset.private "$DATASET_PRIVATE"
  optional_arg args --dataset.tags "$DATASET_TAGS"
  optional_arg args --dataset.num_image_writer_processes "$DATASET_NUM_IMAGE_WRITER_PROCESSES"
  optional_arg args --dataset.num_image_writer_threads_per_camera "$DATASET_NUM_IMAGE_WRITER_THREADS_PER_CAMERA"
  optional_arg args --dataset.rename_map "$DATASET_RENAME_MAP"

  quote_cmd "${args[@]}"
}

build_record_window_cmd() {
  local record_cmd
  local interactive_cmd

  record_cmd=$(build_record_cmd)
  interactive_cmd=$(printf '%s\n%s\n%s' "$LEROBOT_ENV_CMD" "export HF_HUB_OFFLINE=$(printf '%q' "$HF_HUB_OFFLINE")" "$record_cmd")

  quote_cmd bash -ic "$interactive_cmd"
}

run_preflight_checks() {
  local need_lerobot=false
  if [[ "$MODE" == "collect" ]]; then
    need_lerobot=true
  fi

  echo "[CHECK] 配置文件: $CONFIG_FILE"

  if [[ ! -f "$WORKSPACE_DIR/install/setup.bash" ]]; then
    echo "[FAIL] 未找到 $WORKSPACE_DIR/install/setup.bash" >&2
    exit 1
  fi
  echo "[OK] setup.bash 存在: $WORKSPACE_DIR/install/setup.bash"

  if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
    echo "[FAIL] 当前没有图形桌面会话，无法弹出终端窗口。" >&2
    exit 1
  fi
  echo "[OK] 图形桌面会话可用: DISPLAY=${DISPLAY:-<empty>} WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-<empty>}"

  require_command "$GUI_TERMINAL"
  echo "[OK] 终端命令可用: $GUI_TERMINAL"

  require_command ros2
  echo "[OK] ros2 命令可用"

  if [[ "$need_lerobot" == true ]]; then
    local lerobot_check_cmd
    printf -v lerobot_check_cmd 'type %q >/dev/null 2>&1' "$LEROBOT_ENV_CMD"
    if ! bash -ic "$lerobot_check_cmd" >/dev/null 2>&1; then
      echo "[FAIL] 在交互式 shell 中找不到 LeRobot 环境命令: $LEROBOT_ENV_CMD" >&2
      exit 1
    fi
    echo "[OK] 交互式 shell 可用 LeRobot 环境命令: $LEROBOT_ENV_CMD"
  else
    echo "[SKIP] 当前模式不需要检查 LeRobot 环境命令"
  fi
}

state_has_live_windows() {
  if [[ ! -f "$STATE_FILE" ]]; then
    return 1
  fi

  local window_id shell_pid shell_pgid
  while IFS='|' read -r window_id shell_pid shell_pgid; do
    [[ -z "${window_id:-}" ]] && continue
    [[ -z "${shell_pid:-}" ]] && continue
    if kill -0 "$shell_pid" 2>/dev/null; then
      return 0
    fi
  done < "$STATE_FILE"

  return 1
}

stop_mode() {
  if [[ ! -f "$STATE_FILE" ]]; then
    echo "没有找到运行中的窗口状态文件: $STATE_FILE"
    return 0
  fi

  local found_any=false
  local closed_any=false
  local line window_id shell_pid shell_pgid
  while IFS='|' read -r window_id shell_pid shell_pgid; do
    [[ -z "${window_id:-}" ]] && continue
    found_any=true

    if [[ -z "${shell_pid:-}" || -z "${shell_pgid:-}" ]]; then
      echo "[SKIP] $window_id 状态不完整，已跳过"
      continue
    fi

    if ! kill -0 "$shell_pid" 2>/dev/null; then
      echo "[SKIP] $window_id 已关闭"
      continue
    fi

    echo "[STOP] 关闭 $window_id (pid=$shell_pid, pgid=$shell_pgid)"
    kill -TERM -- "-$shell_pgid" 2>/dev/null || true
    closed_any=true
  done < "$STATE_FILE"

  if [[ "$closed_any" == true ]]; then
    sleep 1
    while IFS='|' read -r window_id shell_pid shell_pgid; do
      [[ -z "${window_id:-}" ]] && continue
      [[ -z "${shell_pid:-}" || -z "${shell_pgid:-}" ]] && continue
      if kill -0 "$shell_pid" 2>/dev/null; then
        echo "[STOP] 强制关闭 $window_id (pgid=$shell_pgid)"
        kill -KILL -- "-$shell_pgid" 2>/dev/null || true
      fi
    done < "$STATE_FILE"
  fi

  rm -f "$STATE_FILE"

  if [[ "$found_any" == false ]]; then
    echo "状态文件为空，没有需要关闭的窗口。"
  else
    echo "stop 完成。"
  fi
}

make_window_script() {
  local window_id=$1
  local window_title=$2
  local delay_s=$3
  local main_cmd=$4
  shift 4
  local scriptfile
  scriptfile=$(mktemp "/tmp/${window_id}.XXXXXX.sh")
  local extra_lines=("$@")

  {
    echo '#!/usr/bin/env bash'
    echo 'set -euo pipefail'
    printf 'WINDOW_ID=%q\n' "$window_id"
    printf 'WINDOW_TITLE=%q\n' "$window_title"
    printf 'WORKSPACE_DIR=%q\n' "$WORKSPACE_DIR"
    printf 'DELAY_S=%q\n' "$delay_s"
    printf 'STATE_FILE=%q\n' "$STATE_FILE"
    echo 'SHELL_PID=$$'
    echo 'SHELL_PGID=$(ps -o pgid= -p $$ | tr -d " ")'
    echo 'printf "%s|%s|%s\n" "$WINDOW_ID" "$SHELL_PID" "$SHELL_PGID" >> "$STATE_FILE"'
    echo 'echo "[INFO] 窗口: $WINDOW_TITLE"'
    echo 'echo "[INFO] shell pid=$SHELL_PID pgid=$SHELL_PGID"'
    echo 'if [[ "$DELAY_S" != "0" && "$DELAY_S" != "0.0" ]]; then'
    echo '  echo "[INFO] 延时启动 ${DELAY_S}s"'
    echo '  sleep "$DELAY_S"'
    echo 'fi'
    echo 'cd "$WORKSPACE_DIR"'
    echo 'export COLCON_TRACE=${COLCON_TRACE-}'
    echo 'set +u'
    echo 'source "$WORKSPACE_DIR/install/setup.bash"'
    echo 'set -u'
    local line
    for line in "${extra_lines[@]}"; do
      if [[ -n "$line" ]]; then
        printf '%s\n' "$line"
      fi
    done
    printf 'echo %q\n' "[INFO] 执行命令: $main_cmd"
    echo 'set +e'
    printf '%s\n' "$main_cmd"
    echo 'status=$?'
    echo 'set -e'
    echo 'echo'
    echo 'echo "[INFO] 主命令退出，exit code=$status"'
    echo 'echo "[INFO] 你可以手动关闭此窗口。"'
    echo 'exec bash'
  } > "$scriptfile"

  chmod +x "$scriptfile"
  WINDOW_SCRIPTS+=("$window_title|$scriptfile")
}

open_window() {
  local title=$1
  local scriptfile=$2
  local runner
  printf -v runner '%q' "$scriptfile"
  "$GUI_TERMINAL" --window --title="$title" -- bash -lc "$runner" &
  sleep "$OPEN_WINDOW_INTERVAL_S"
}

launch_mode() {
  # China 적응: Pico Bridge 창 제거(leader arm 은 CAN 직접). teleop = leader-follower.
  make_window_script bringup 'VLA-1 Bringup' 0 "$(build_bringup_cmd)"
  make_window_script leader_teleop 'VLA-2 Leader Teleop' "$TELEOP_START_DELAY_S" "$(build_teleop_cmd)"

  case "$MODE" in
    base)
      ;;
    base_camera)
      make_window_script camera 'VLA-3 D435 Camera' "$CAMERA_START_DELAY_S" "$(build_camera_cmd)"
      ;;
    collect)
      make_window_script camera 'VLA-3 D435 Camera' "$CAMERA_START_DELAY_S" "$(build_camera_cmd)"
      make_window_script record 'VLA-4 LeRobot Record' "$RECORD_START_DELAY_S" "$(build_record_window_cmd)"
      ;;
    *)
      echo "未知模式: $MODE" >&2
      exit 1
      ;;
  esac

  local item title scriptfile
  for item in "${WINDOW_SCRIPTS[@]}"; do
    IFS='|' read -r title scriptfile <<< "$item"
    open_window "$title" "$scriptfile"
  done

  echo "已按模式 \"$MODE\" 依次打开终端窗口。"
}

main() {
  local subcommand=${1:-help}
  case "$subcommand" in
    help|-h|--help)
      usage
      exit 0
      ;;
    stop)
      stop_mode
      exit 0
      ;;
    check)
      MODE=collect
      run_preflight_checks
      exit 0
      ;;
    base|base_camera|collect)
      MODE=$subcommand
      ;;
    *)
      echo "未知子命令: $subcommand" >&2
      usage
      exit 1
      ;;
  esac

  WINDOW_SCRIPTS=()
  run_preflight_checks
  mkdir -p "$(dirname -- "$STATE_FILE")"
  if state_has_live_windows; then
    echo "检测到已有一组窗口仍在运行，请先执行: bash scripts/vla_collect_gui.sh stop" >&2
    exit 1
  fi
  : > "$STATE_FILE"
  launch_mode
}

main "$@"
