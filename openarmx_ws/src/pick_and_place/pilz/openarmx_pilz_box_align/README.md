# openarmx_pilz_box_align

Bimanual **box detect → left/right arm assignment → move above box** for OpenArmX,
using **MoveIt Pilz planning** as the motion backend (the cyclo-backed variant is
`openarmx_cyclo_box_align`).

```
DetectBox (YOLOv8) ─▶ accumulate N frames ─▶ 3D cluster ─▶ box-top centroid (base)
        │
        ▼  assign +Y → LEFT arm, −Y → RIGHT arm
  link7 target (box.x, box.y, z) + RPY
        │  Pilz LIN
        ▼  /plan_kinematic_path  (service, 1 round-trip)  ─▶  planned JointTrajectory
        ▼  /<side>_joint_trajectory_controller/joint_trajectory  (topic publish)
```

## Why a service + topic (not the MoveGroup action)

A ROS2 **action is inherently 2-stage** (goal-accept handshake + result). To avoid
that ~100–300 ms handshake this node plans through the **`/plan_kinematic_path`
service** (single request/response) and publishes the resulting trajectory
**directly to the arm JTC topic** (fire-and-forget, like cyclo) — no MoveGroup
action round-trip. Pilz constrains **link7 directly**, so (unlike cyclo) there is
no link7→hand_tcp offset to compensate.

## Prerequisites (run separately)
1. D435 camera (`realsense2_camera`) → `/camera/camera/...`
2. YOLOv8 DetectBox action server `/yolov8_node/detect`
3. calibrated base→camera TF
4. MoveIt `move_group` with the **Pilz pipeline** (`/plan_kinematic_path`) + the
   arm JTCs (`/{left,right}_joint_trajectory_controller/joint_trajectory`)

## Build & run

```bash
cd ~/TR-Works/kkw/China/openarmx_ws
colcon build --packages-select openarmx_pilz_box_align_msgs openarmx_pilz_box_align --symlink-install
# yolov8_detection_msgs lives in 3d_detect_ws:
source /opt/ros/humble/setup.bash
source ~/TR-Works/kkw/China/3d_detect_ws/install/setup.bash
source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash
ros2 launch openarmx_pilz_box_align pilz_box_align.launch.py
```

## Command

```bash
ros2 action send_goal /openarmx/pilz_align_to_boxes \
  openarmx_pilz_box_align_msgs/action/AlignToBoxes \
  "{z: 0.4, roll_deg: 180.0, pitch_deg: 0.0, yaw_deg: 0.0, arms: both, vel_scale: 0.3, planner: LIN}"
```

| Goal field | default | meaning |
|---|---|---|
| `z` | — | link7 target height (absolute, `openarmx_body_link0`), m |
| `roll_deg`/`pitch_deg`/`yaw_deg` | 180/0/0 | hand orientation (default vertical-down) |
| `arms` | `both` | `both` \| `left` \| `right` |
| `confidence` | 0.02 | YOLO confidence (`<=0` keeps default) |
| `prompts` | box vocab | YOLO-World prompts (empty keeps default) |
| `vel_scale` | 0.3 | Pilz max velocity/accel scaling 0..1 (`<=0` keeps default) |
| `planner` | `LIN` | Pilz `LIN` \| `PTP` \| `CIRC` |

Detection robustness (accumulate, 3D cluster, `min_hits`, workspace filter,
stale-frame parking) is shared with the cyclo variant.
