# openarmx_ptp_box_align

Bimanual **consume detected boxes → left/right arm assignment → move above box**
for OpenArmX, using a **direct point-to-point (PTP) move** as the motion backend
(the third backend alongside `openarmx_cyclo_box_align` and `openarmx_pilz_box_align`).
**Detection/3D is NOT done here** — it is a separate perception node
(`box_perception_node`); this node only consumes its `/detected_boxes`.

> **[SSOT 2026-06-09] 이 노드는 hover 정렬(디버그/테스트)용으로 격하되었다.** 실제
> pick&place 정본(canonical)은 resident Python 경로(`ptp_pick_resident.py` +
> `box_detect_loop.py` + `container_pick_gate.py` + UI 의 `ptp_pick_bridge.py`)이며
> 하강·파지·놓기 end-to-end 를 수행한다. 이 노드는 박스 위 **hover 정렬만** 한다(파지·놓기 없음).
> 두 경로가 같은 `controller_manager` 를 교차 토글하면 충돌하므로 UI 에서 상호배타하며
> (`btnRun` "Hover 정렬(디버그)" ↔ Pick&Place 탭 수동/자동), **자동 pick&place 루프를 이 노드에
> 얹지 말 것.**

```
box_perception_node (perception: detect + 3D)  ─▶  /detected_boxes (PoseArray, base)
        │
        ▼  (consume latest)   assign +Y → LEFT arm, −Y → RIGHT arm
  hand_tcp (TCP) target (box.x, box.y, z) + RPY   [default; set suffix=link7 for link7]
        │  Pinocchio damped-least-squares IK (one solve, no QP/CBF, no MoveIt)
        ▼  q_goal (7 joints)
        ▼  single JointTrajectory point → /<side>_joint_trajectory_controller
           (FollowJointTrajectory action; topic fallback) → arm interpolates
        ▼  gripper full open
```

## Why a third backend (vs cyclo / pilz)

| Backend | Engine | Safety | Build footprint |
|---|---|---|---|
| `cyclo` | Pinocchio Jacobian + **QP + CBF** velocity integration | joint-limit / singularity / collision CBF | cyclo_control overlay (Pinocchio C++, OOM-prone) |
| `pilz` | MoveIt Pilz LIN/PTP/CIRC planning (`/plan_kinematic_path`) | full MoveIt collision planning | MoveIt stack |
| **`ptp`** (this) | Pinocchio **damped-least-squares IK**, single JTC endpoint | **none** beyond joint-limit clamp + the JTC's own limits | `ros-humble-pinocchio` only, builds in openarmx_ws |

`ptp` is the lightest: the same Pinocchio kinematics cyclo references, but with the
QP/CBF safety layer removed — a plain numerical IK + one endpoint to the JTC. Use it
for top-down vertical grasps inside a known, obstacle-free workspace. **Note:** the
`PTP` here is the *direct single-goal move* style, **not** the MoveIt Pilz `PTP`
planner.

## How the IK works

`solveIK()` is the standard Pinocchio CLIK loop on the controlled frame
(`openarmx_<side>_<controlled_link_suffix>`, default `hand_tcp`):

```
q = neutral
repeat:
  err = log6( FK(q)^-1 · target )         # 6D SE3 error in the frame
  if |err| < ik_eps: done
  J   = Jlog6 · frameJacobian(q, frame)   # LOCAL frame
  dq  = -J^T (J J^T + ik_damp·I)^-1 err    # damped least squares
  q   = integrate(q, dq · ik_dt)
clamp q to joint limits
```

The controlled frame defaults to **`hand_tcp`**, so the goal `z` is the
**tool-center-point (TCP) height** — like cyclo, but because Pinocchio solves IK
for the chosen frame directly, there is **no manual `link7 → hand_tcp` offset** to
apply. Set `controlled_link_suffix:=link7` to constrain link7 instead (pilz-style).
The solver URDFs root at `openarmx_body_link0`, which equals the goal `frame_id`, so
FK already yields the frame pose in the base frame — no TF lookup is needed.

## Prerequisites (run separately)
1. `box_perception_node` — publishes `/detected_boxes` (geometry_msgs/PoseArray,
   base frame). Detection / 3D / box positions are its job (see `3d_detect_ws`).
2. The arm joint_trajectory_controllers
   (`/{left,right}_joint_trajectory_controller/follow_joint_trajectory`) and the
   gripper controllers (`/{left,right}_gripper_controller/gripper_cmd`).

## Build & run

```bash
cd ~/TR-Works/kkw/China/openarmx_ws
colcon build --packages-select openarmx_ptp_box_align_msgs openarmx_ptp_box_align --symlink-install
# yolov8_detection_msgs lives in 3d_detect_ws (for the perception side):
source /opt/ros/humble/setup.bash
source ~/TR-Works/kkw/China/3d_detect_ws/install/setup.bash
source ~/TR-Works/kkw/China/openarmx_ws/install/setup.bash
ros2 launch openarmx_ptp_box_align ptp_box_align.launch.py
```

Dependency (apt): `ros-humble-pinocchio`. No cyclo_control / cyclo_ws overlay is
needed — this package links Pinocchio directly.

## Command

```bash
ros2 action send_goal /openarmx/ptp_align_to_boxes \
  openarmx_ptp_box_align_msgs/action/AlignToBoxes \
  "{z: 0.80, roll_deg: 180.0, pitch_deg: 0.0, yaw_deg: 0.0, arms: both}"
```

> **Operating height (raised-arm calibration, 2026-06-06).** The desk is at
> `0.72 m` (floor), boxes rest on it with their top at base `z ≈ 0.78`
> (ws_z `[0.64, 0.86]`, after the arm_base `0.735 → 1.275`, +0.54 m recalibration).
> With the default `hand_tcp` frame the goal `z` is the **TCP pick height ≈ 0.80**
> (not link7, and not the old `0.4`). The hand-down `R180 P0 Y0` orientation is
> reachable in exactly this region (neutral `hand_tcp` sits at `z = 0.597`, already
> `Rx180`); targets far below (e.g. `z = 0.3`) are outside the arm's reachable
> envelope. If you switch to `controlled_link_suffix:=link7`, use `z ≈ 0.98` instead
> (link7 is ~0.18 m above hand_tcp).

| Goal field | default | meaning |
|---|---|---|
| `z` | — | **TCP** (`hand_tcp`) target height (absolute, `openarmx_body_link0`), m |
| `roll_deg`/`pitch_deg`/`yaw_deg` | 180/0/0 | hand orientation (default vertical-down) |
| `arms` | `both` | `both` \| `left` \| `right` |
| `confidence` | — | **ignored** (detection is in `box_perception_node`) |
| `prompts` | — | **ignored** (detection is in `box_perception_node`) |

Result: `success`, `detections_json`, `assignments_json` (per arm: box, `tcp_target`,
`ik_converged`, `ik_residual`, `err_mm`, `moved`).

## Parameters

`controlled_link_suffix` (`hand_tcp`; the IK target frame is
`openarmx_<side>_<suffix>` — `hand_tcp` = TCP-based, `link7` = pilz-style),
`move_time` (6.0 s, JTC interpolation duration), `max_box_age` (60 s),
`gripper_open_pos` (0.044 m), `gripper_effort` (14.0),
`left_urdf_path` / `right_urdf_path` (default: openarmx_pick solver URDFs),
IK: `ik_eps` (1e-4), `ik_max_iter` (1000), `ik_dt` (0.1), `ik_damp` (1e-6).

## Status

Scaffold (2026-06-06). Builds against `ros-humble-pinocchio`; IK + assignment +
single-endpoint JTC dispatch implemented.

**IK verified (offline, 2026-06-06).** The C++ node (target frame `hand_tcp`) solves
real operating-region poses to sub-millimetre accuracy: a box at base
`(0.05, 0.30, 0.78)` with TCP target `z = 0.80`, `R180` → `ik_converged: true`,
residual `9.1e-05`, `err_mm ≈ 0.09`. Self-consistency confirmed (IK recovers FK of
random valid configs to `1e-4`). The earlier non-convergence seen at `z = 0.3` was a
bad test target (outside the reachable envelope), not a solver bug.

**LIVE-verified on hardware (2026-06-06).** Full pipeline run on the real robot:
remote-Hailo DetectBox → `box_perception_node` `/detected_boxes` → ptp IK → right
`joint_trajectory_controller` → arm moved to TCP `z = 0.80` above a detected box at
base `(0.345, −0.116, 0.782)` → gripper opened. Result `ik_converged: true`,
residual `1.0e-04`, **`err_mm ≈ 0.05`**.

**Limit-aware IK fix (required).** `solveIK()` clamps `q` to the joint limits **every
step**, not just at the end. The first live run converged to an out-of-limit `q`
(joint2 over by 0.11 rad) and the final one-shot clamp landed the TCP **65 mm** off
target while still reporting `ik_converged: true`. Per-step clamping keeps the
descent in-limit and converged to `0.05 mm`; the convergence test now reflects the
in-limit `q`, so `ik_converged=false` correctly flags truly-unreachable targets.

The left solver URDF still carries the known joint1 limit shift (−120°, see
`docs/issues_and_fixes/2026-06-06_raised_arm_staleness...`); it does not block the
operating-region targets above but narrows reach elsewhere.
