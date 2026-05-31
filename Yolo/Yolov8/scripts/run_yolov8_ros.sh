#!/usr/bin/env bash
# Source ROS 2 Humble + the yolov8_env venv + the 3d_detect_ws overlay,
# then run any command (default: launch yolov8_d435.launch.py).
set -e

ROS_SETUP="/opt/ros/humble/setup.bash"
VENV_ACTIVATE="/home/openarmx/TR-Works/kkw/China/Yolo/Yolov8/yolov8_env/bin/activate"
OPENARMX_SETUP="/home/openarmx/TR-Works/kkw/China/openarmx_ws/install/setup.bash"
WS_SETUP="/home/openarmx/TR-Works/kkw/China/3d_detect_ws/install/setup.bash"

export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

# shellcheck source=/dev/null
source "$ROS_SETUP"
# Source the openarmx workspace so robot_state_publisher can resolve
# package://openarmx_description meshes when show_robot:=true is used.
if [ -f "$OPENARMX_SETUP" ]; then
  # shellcheck source=/dev/null
  source "$OPENARMX_SETUP"
fi
if [ -f "$WS_SETUP" ]; then
  # shellcheck source=/dev/null
  source "$WS_SETUP"
else
  echo "[run_yolov8_ros] WARNING: $WS_SETUP not found. Build first:" >&2
  echo "  cd /home/openarmx/TR-Works/kkw/China/3d_detect_ws && colcon build --packages-select yolov8_detection --symlink-install" >&2
fi
# Activate venv LAST so its bin/ stays first on PATH (workspace setup re-orders it).
# shellcheck source=/dev/null
source "$VENV_ACTIVATE"

# Force the yolov8_detection entry-point launchers to use the venv's Python so
# ultralytics/torch are importable. colcon regenerates these scripts on every
# build with the host python shebang, so we re-pin them here at runtime.
VENV_PY="/home/openarmx/TR-Works/kkw/China/Yolo/Yolov8/yolov8_env/bin/python"
for ENTRY in \
  "/home/openarmx/TR-Works/kkw/China/3d_detect_ws/install/yolov8_detection/lib/yolov8_detection/yolov8_node" \
  "/home/openarmx/TR-Works/kkw/China/3d_detect_ws/install/yolov8_detection/lib/yolov8_detection/box_plane_node"; do
  if [ -f "$ENTRY" ] && [ -x "$VENV_PY" ]; then
    sed -i "1c #!${VENV_PY}" "$ENTRY"
  fi
done

if [ "$#" -eq 0 ]; then
  # Default: full pipeline — D435 + YOLO World + box-top & ground RANSAC +
  # openarmx URDF + RViz2 overlay.
  exec ros2 launch yolov8_detection yolov8_d435.launch.py \
      rviz:=true \
      show_robot:=true \
      fit_box_plane:=true \
      model:=yolov8l-worldv2.pt \
      prompts:="cardboard box,box,carton,package" \
      confidence:=0.10
else
  exec "$@"
fi
