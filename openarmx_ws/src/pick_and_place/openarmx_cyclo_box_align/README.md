# openarmx_cyclo_box_align

Bimanual **box detect → left/right arm assignment → move above box** orchestration
for OpenArmX. One `AlignToBoxes` action call detects the boxes, decides which arm
takes which box, and drives each assigned arm above its box at a commanded height
and hand orientation.

```
DetectBox (YOLOv8 on-demand) ─▶ accumulate N frames ─▶ 3D cluster
        │                                                   │ box-top centroid (base frame)
        ▼                                                   ▼
  assign by base Y:  +Y → LEFT arm,  −Y → RIGHT arm
        │
        ▼
  link7 target = (box.x, box.y, z) + commanded RPY  ─▶ /openarmx/<side>/movel (cyclo QP)
```

## Pipeline order (what the user runs)

1. **Camera** — D435 (`realsense2_camera`) publishing `/camera/camera/...`.
2. **YOLOv8 server** — on-demand `DetectBox` action server `/yolov8_node/detect`
   (`3d_detect_ws`, run via `run_yolov8_ros.sh`, `use_depth:=true`).
3. **base↔camera TF + cyclo MoveL controllers** — e.g. the scenario stack
   (`scenario_player_with_ee_leader.launch.py`) which provides the
   `d435_center→camera_link` bridge and `/openarmx/{left,right}/movel`.
4. **This node** — `ros2 launch openarmx_cyclo_box_align box_align.launch.py`.
5. **Command** — send an `AlignToBoxes` goal (below).

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

- **Detection** reuses the on-demand `DetectBox` action over `n_frames` frames and
  clusters detections in 3D (YOLO-World on plain cubes is stochastic per frame).
  Each box's 3D point is the **box-top surface centroid** (nearest-depth points in
  the bbox), robust to loose low-confidence bboxes. A workspace filter
  (`ws_x`/`ws_y_abs`/`ws_z`) rejects edge/background noise.
- **Assignment**: boxes are sorted left→right by base-frame Y; `+Y` → LEFT arm,
  `−Y` → RIGHT arm. `box_<i>` TFs are published for RViz.
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
