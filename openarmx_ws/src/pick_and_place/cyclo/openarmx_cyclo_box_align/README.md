# openarmx_cyclo_box_align

Bimanual **consume detected boxes → left/right arm assignment → move above box**
orchestration for OpenArmX. **Detection/3D is NOT done here** — it is a separate
perception node (`box_perception_node`); one `AlignToBoxes` action call consumes the
latest `/detected_boxes`, decides which arm takes which box, and drives each assigned
arm above its box at a commanded height and hand orientation.

```
box_perception_node (perception: detect + 3D)  ─▶  /detected_boxes (PoseArray, base)
        │
        ▼  (consume latest)   assign by base Y:  +Y → LEFT arm,  −Y → RIGHT arm
  link7 target = (box.x, box.y, z) + commanded RPY  ─▶ /openarmx/<side>/movel (cyclo QP)
```

## Pipeline order (what the user runs)

1. **Perception** — `box_perception_node` publishing `/detected_boxes` (which itself
   needs the D435 camera + the YOLOv8 bridge running upstream).
2. **cyclo MoveL controllers** — `/openarmx/{left,right}/movel`.
3. **This node** — `ros2 launch openarmx_cyclo_box_align box_align.launch.py`.
4. **Command** — send an `AlignToBoxes` goal (below).

## Command

```bash
ros2 action send_goal /openarmx/align_to_boxes \
  openarmx_cyclo_box_align_msgs/action/AlignToBoxes \
  "{z: 0.4, roll_deg: 180.0, pitch_deg: 0.0, yaw_deg: 0.0, arms: both}"
```

| Goal field | default | meaning |
|---|---|---|
| `z` | — | link7 target height (absolute, `openarmx_body_link0`), metres |
| `roll_deg` / `pitch_deg` / `yaw_deg` | 180 / 0 / 0 | hand orientation; default = vertical-down |
| `arms` | `both` | `both` \| `left` \| `right` |
| `confidence` | node default (0.01) | YOLO confidence (`<=0` keeps default) |
| `prompts` | node default box vocab | YOLO-World prompts (empty keeps default) |

Result: `success`, `detections_json` (boxes in base frame), `assignments_json`
(arm → box, link7 target, final position + error).

## How it works

- **Perception is a separate node.** Detection / 3D / box positions are produced by
  `box_perception_node`; this action server only **subscribes to `/detected_boxes`**
  (geometry_msgs/PoseArray, base frame) and uses the latest set. It does **not**
  detect, read depth, or publish box TFs (perception and motion are decoupled).
- **Assignment**: boxes are sorted left→right by base-frame Y; `+Y` → LEFT arm,
  `−Y` → RIGHT arm.
- **Motion**: the cyclo controller commands `hand_tcp`, which sits a fixed offset
  below `link7`; the node looks up `link7→hand_tcp` and compensates so **link7**
  lands at the commanded `z`. A reachable `z` keeps both position and orientation
  exact (unreachable targets make the QP trade orientation for position).

## Parameters

`n_frames` (5), `cluster_radius` (0.06 m), `move_time` (6 s),
`default_confidence` (0.01), `ws_x` ([0.05, 0.70]), `ws_y_abs` (0.45),
`ws_z` ([0.10, 0.32]).

## Build

```bash
cd ~/TR-Works/kkw/China/openarmx_ws
colcon build --packages-select openarmx_cyclo_box_align_msgs openarmx_cyclo_box_align --symlink-install
```

## Run

`yolov8_detection_msgs` lives in `3d_detect_ws`, so source that overlay too:

```bash
source /opt/ros/humble/setup.bash
source ~/TR-Works/kkw/China/3d_detect_ws/install/setup.bash
source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash
ros2 launch openarmx_cyclo_box_align box_align.launch.py
```
