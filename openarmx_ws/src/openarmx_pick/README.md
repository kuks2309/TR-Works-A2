# openarmx_pick

Vision-driven **single-arm box pick** for OpenArmX, **without MoveIt**.

The end-effector pose is solved by the [`cyclo_control`](../../../cyclo_control)
**QP + CBF MoveL controller** (Pinocchio FK/Jacobian → OSQP quadratic program
with joint-limit / singularity / collision Control-Barrier-Function constraints),
fed by a classic geometric grasp synthesised from the existing `box_plane`
detection. No learned grasp network is needed — a box resting on a table is fully
constrained by its top plane + footprint.

```
D435 stereo depth ─▶ YOLO-World ─▶ box_plane RANSAC ─▶ /box_plane/cloud
                                                            │  (box-top inliers)
                                                            ▼
                                       grasp_pose_node  (this package)
                                       tf2 → base frame, centroid + PCA yaw
                                            │  top-down 6-DoF grasp
                                            ▼  /openarmx/grasp_pose  (+ optional MoveL)
                                       cyclo omx_movel_controller_node
                                       QP + CBF  →  joint_trajectory  →  arm
```

The grasp pose is expressed in **`openarmx_body_link0`**, which is also the solver
URDF root (the `world → body_link0` joint is identity), so no extra TF is needed
between perception and control.

---

## Components

| Path | Role |
|---|---|
| `urdf/openarmx_left_solver.urdf` | 7-DOF **left**-arm solver model (right arm + all fingers frozen `fixed`). Root = `openarmx_body_link0`. Collisions stripped (stage-1). |
| `urdf/openarmx_right_solver.urdf` | 7-DOF **right**-arm mirror model. |
| `scripts/gen_solver_urdf.py` | Regenerates a solver URDF from the full xacro expansion (freeze other arm + fingers, optional `--no-collision` / `--strip-visual`). |
| `openarmx_pick/grasp_pose_node.py` | **Stage B** — box-top cloud → tf2 base transform → centroid + XY-PCA yaw → top-down 6-DoF grasp `PoseStamped`; optional debounced pre-grasp `MoveL`. |
| `launch/openarmx_movel.launch.py` | Single (left) arm MoveL solver. |
| `launch/openarmx_movel_bimanual.launch.py` | Left + right solvers on disjoint topics (`/openarmx/{left,right}/...`). |
| `launch/openarmx_pick.launch.py` | Solver + `grasp_pose_node` together. |
| `scripts/verify_solver.py` | Stage-A check: fake joint_states + MoveL → joint_command converges. |
| `scripts/verify_grasp.py` | Stage-B check: synthetic box cloud → grasp pose (no camera). |
| `scripts/verify_e2e.py` | End-to-end: synthetic box → grasp → MoveL → solver → EE converges. |

---

## Build

`cyclo_control` + `robotis_interfaces` build in an overlay workspace; `openarmx_pick`
builds in `openarmx_ws`.

> ⚠️ **Build `cyclo_control` single-threaded.** The Pinocchio-heavy C++ nodes use
> ~2–3 GB RAM each; the default `-j8` overruns 15 GB → OOM → reboot. Use
> `MAKEFLAGS=-j1` (≈21 min for `cyclo_motion_controller_ros`).

```bash
# 1) solver overlay (cyclo_control symlinked + robotis_interfaces cloned)
cd ~/TR-Works/kkw/China/cyclo_ws
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --symlink-install \
  --executor sequential --cmake-args -DCMAKE_BUILD_PARALLEL_LEVEL=1

# 2) this package
cd ~/TR-Works/kkw/China/openarmx_ws
colcon build --packages-select openarmx_pick --symlink-install
```

Dependencies (apt): `ros-humble-pinocchio ros-humble-osqp-vendor
ros-humble-ament-cmake-vendor-package python3-nlopt`.

---

## Run

Source all three overlays first:
```bash
source /opt/ros/humble/setup.bash
source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash
source ~/TR-Works/kkw/China/cyclo_ws/install/setup.bash
```

### Camera integration only (no robot motion)
```bash
# 1) camera + YOLO + box_plane + calibrated body→camera TF (RViz optional)
~/TR-Works/kkw/China/Yolo/Yolov8/scripts/run_yolov8_ros.sh \
  ros2 launch yolov8_detection yolov8_d435.launch.py \
    rviz:=false show_robot:=true fit_box_plane:=true \
    model:=yolov8l-worldv2.pt prompts:="cardboard box,box,carton,package" confidence:=0.10

# 2) grasp synthesis only — auto_send:=false means NO MoveL is ever published
ros2 run openarmx_pick grasp_pose_node --ros-args -p auto_send:=false
ros2 topic echo /openarmx/grasp_pose
```

### Full pick (sends motion — only with the robot ready)
```bash
ros2 launch openarmx_pick openarmx_pick.launch.py auto_send:=true
```

---

## Topics & frames

| Topic | Type | Dir | Note |
|---|---|---|---|
| `/box_plane/cloud` | `sensor_msgs/PointCloud2` | in | box-top inliers (`camera_color_optical_frame`) |
| `/box_plane/info` | `std_msgs/String` | in | JSON; `box_height_m` |
| `/openarmx/grasp_pose` | `geometry_msgs/PoseStamped` | out | top-down grasp, `openarmx_body_link0` |
| `/openarmx/grasp_markers` | `visualization_msgs/MarkerArray` | out | RViz approach arrow |
| `/openarmx/movel` | `robotis_interfaces/MoveL` | out | only when `auto_send:=true` |

Key `grasp_pose_node` params: `base_frame` (`openarmx_body_link0`),
`pregrasp_height` (0.10 m), `grasp_depth` (0.005 m), `auto_send` (false),
`send_min_delta` / `send_min_interval` (MoveL debounce),
`tool_approach_axis` / `tool_opening_axis` (TCP convention).

---

## Verification status (2026-05-31)

- **Stage A** (solver port) — PASS. MoveL goal → QP drives `joint_command` to an
  IK solution; joints clamp exactly at limits → joint-limit CBF confirmed.
- **Stage B** (grasp synthesis) — PASS. Synthetic box → grasp pos err ≈ 2 mm,
  approach `(0,0,-1)`, opening on the box short axis.
- **End-to-end** (sim) — PASS. EE converges within ≈ 6–12 mm of the pre-grasp.
- **Live camera** — PASS. Real D435 + cardboard box (top at body z ≈ 0.204 m,
  height 17.2 cm) → stable top-down grasp pose in the base frame. Robot not moved.

## Not yet done / next

1. **Main-box filter** — `box_plane` can emit up to 3 top candidates; the grasp
   pose currently jumps between them. Select the largest-inlier cloud only.
2. **Pick FSM** — descend → gripper close (`openarmx_gripper_panel`) → lift.
   Today only the pre-grasp hover is commanded.
3. **Stage-2 collision CBF** — regenerate the URDF with collisions + an SRDF to
   enable self-collision avoidance.
4. **Real controller mapping** — wire `joint_command` to the OpenArmX
   `forward_position_controller`; validate the assumed tool axes against real FK.
