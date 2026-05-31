#!/usr/bin/env bash
# Cycle through every bag under bags/20260506/, running visualize_planes
# and ros2 bag play (looped) for ~40 s each so RViz updates smoothly.

cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash

BAGS=(big_box_a medium_box_narrow_a medium_box_wide_a short_box_narrow_a short_box_wide_a)
DURATION=75   # seconds per bag

cleanup() {
  pkill -f visualize_planes 2>/dev/null
  pkill -f 'ros2 bag play' 2>/dev/null
}
trap cleanup EXIT INT TERM

for bag in "${BAGS[@]}"; do
  echo "[cycle] >>> $bag"
  cleanup
  sleep 1
  python3 visualize_planes.py --bag "$bag" >/tmp/plane_viz.log 2>&1 &
  sleep 8   # let RANSAC finish before bag play (so markers appear early)
  ros2 bag play "bags/20260506/$bag" --clock --loop >/tmp/bag_play.log 2>&1 &
  sleep $((DURATION - 8))
done

echo "[cycle] done"
