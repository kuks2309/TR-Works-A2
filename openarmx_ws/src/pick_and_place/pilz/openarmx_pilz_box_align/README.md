# openarmx_pilz_box_align

Bimanual **consume detected boxes → left/right arm assignment → move above box**
for OpenArmX, using **MoveIt Pilz planning** as the motion backend (the cyclo-backed
variant is `openarmx_cyclo_box_align`). **Detection/3D is NOT done here** — it is a
separate perception node (`box_perception_node`); this node consumes its output.

```
box_perception_node (perception: detect + 3D)  ─▶  /detected_boxes (PoseArray, base)
        │
        ▼  (consume latest)   assign +Y → LEFT arm, −Y → RIGHT arm
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
1. `box_perception_node` — publishes `/detected_boxes` (geometry_msgs/PoseArray,
   base frame). Detection/3D/box positions are its job (see `3d_detect_ws`); it in
   turn needs the D435 camera + the YOLOv8 bridge.
2. MoveIt `move_group` with the **Pilz pipeline** (`/plan_kinematic_path`) + the
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
| `confidence` | — | **ignored** (detection is in `box_perception_node`) |
| `prompts` | — | **ignored** (detection is in `box_perception_node`) |
| `vel_scale` | 0.3 | Pilz max velocity/accel scaling 0..1 (`<=0` keeps default) |
| `planner` | `LIN` | Pilz `LIN` \| `PTP` \| `CIRC` |

Detection / 3D / workspace filtering now lives in `box_perception_node` (perception),
shared by both the pilz and cyclo backends. This node only consumes the latest
`/detected_boxes` (rejected if older than `max_box_age`, default 60 s).
