#!/usr/bin/env bash
# kill_all_ros2.sh — terminate ALL openarmx-related ROS2 processes.
#
# Install: cp openarmx_ws/scripts/kill_all_ros2.sh ~/kill_all_ros2.sh && chmod +x ~/kill_all_ros2.sh
# Invoked by openarmx_scenario_ui main_window "STOP ALL" button (KILL_ALL_SCRIPT=~/kill_all_ros2.sh).

set -u

PATTERNS=(
  'ros2 launch'
  'ros2 run'
  'controller_manager'
  'robot_state_publisher'
  'rviz2'
  'move_group'
  'scenario_player_node'
  'openarmx.bimanual.launch'
  'openarmx_scenario_ui'
  'openarmx_scenario_player'
)

for pat in "${PATTERNS[@]}"; do
  pkill -INT -f "$pat" 2>/dev/null || true
done

sleep 2

for pat in "${PATTERNS[@]}"; do
  pkill -KILL -f "$pat" 2>/dev/null || true
done

exit 0
