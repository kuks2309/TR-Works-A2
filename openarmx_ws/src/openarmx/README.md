# OpenArmX Quick Navigation

[Official Docs](http://docs.openarmx.com/) | [GitHub Organization](https://github.com/openarmx)

![OpenArmX Cover](./img/cover.png)

OpenArmX is an open-source dual-arm collaborative robot platform developed by Chengdu Changshu Robot Co., Ltd. Built on ROS 2, it covers the full stack from robot body description and low-level motor driving, through multi-modal teleoperation, to embodied intelligence (VLA) training and inference. This page summarizes the key information of the platform's core packages to help developers quickly locate the modules they need.

---

## Package Index

| Package | Description |
|---------|-------------|
| [openarmx_description](#1-openarmx_description) | Robot URDF/Xacro description and 3D models |
| [openarmx_ros2](#2-openarmx_ros2) | Core ROS 2 library and bringup configuration (meta-package) |
| [openarmx_motor_manager](#3-openarmx_motor_manager) | GUI-based motor management and CAN interface tool |
| [openarmx_teleop_bimanual](#4-openarmx_teleop_bimanual) | Arm-to-arm dual-arm teleoperation |
| [openarmx_teleop_exo](#5-openarmx_teleop_exo) | Exoskeleton device teleoperation bridge |
| [openarmx_teleop_vr](#6-openarmx_teleop_vr) | VR controller teleoperation pipeline |
| [openarmx_teleop_vr_apk](#7-openarmx_teleop_vr_apk) | VR device-side bridge APK installer |
| [openarmx_tools](#8-openarmx_tools) | Debugging, teaching, and parameter tuning toolkit |
| [openarmx_vla](#9-openarmx_vla) | VLA data collection, model training, and online inference |
| [openclaw_skill_openarmx_motion_player](#10-openclaw_skill_openarmx_motion_player) | OpenClaw natural-language motion playback skill |

---

## 1. openarmx_description

**Overview**
The complete URDF description package for the OpenArmX robot platform. Provides precise kinematics, dynamics, and visualization models, and serves as the foundational dependency for all simulation and control functionality in ROS 2.

**Contents**
- URDF/Xacro files: component descriptions and full assembly for the arm (v10, 7-DOF), body, and end-effector (OpenArmX Hand)
- 3D meshes (STL/DAE): visual meshes and simplified collision geometry
- YAML configs: kinematics (DH parameters), joint limits, link inertia, zero-point offsets
- ros2_control configs: pre-configured hardware interfaces for single-arm and bimanual setups (simulation/real hardware switchable)
- RViz configurations and visualization launch files

**Use Cases**
- Required URDF dependency for all other packages (MoveIt planning, hardware drivers, teleoperation)
- Standalone robot model visualization and kinematics verification in RViz
- Extending configurations when adding new robot variants or end-effectors

**Repository**
https://github.com/openarmx/openarmx_description

---

## 2. openarmx_ros2

**Overview**
The core ROS 2 meta-package for OpenArmX. Aggregates the low-level hardware driver, bringup configurations, and MoveIt planning setup. The primary entry point for controlling real arms or launching simulation environments.

**Contents**
- `openarmx`: meta-package aggregating core components
- `openarmx_hardware`: ros2_control hardware plugin driving arms and grippers over CAN bus
- `openarmx_bringup`: bimanual/single-arm launch files, RViz configs, gripper operation interface
- `openarmx_bimanual_moveit_config`: dual-arm MoveIt 2 planning configuration
- `openarmx_preview_bringup`: robot joint motion preview control package
- `openarmx-can_*.deb`: companion motor CAN driver installer

**Use Cases**
- Powering up and launching a real OpenArmX dual-arm robot (CAN mode)
- Launching simulation mode (`use_fake_hardware:=true`) for software development and testing
- Serving as the underlying control service for teleoperation and tooling packages

**Repository**
https://github.com/openarmx/openarmx_ros2

---

## 3. openarmx_motor_manager

**Overview**
A PySide6-based desktop GUI for managing CAN interfaces and motor states of OpenArmX dual-arm robots, with support for simultaneously managing multiple robots.

**Contents**
- GUI main program (`GUI_MultiRobot.py`): multi-robot tabbed management interface
- CAN interface management: one-click enable/disable, automatic real interface detection
- Motor control: batch enable/stop, go-home, set zero point, single/all motor testing (MIT/CSP modes)
- Real-time status monitoring: position, velocity, torque, temperature, fault states
- CLI scripts: standalone Python scripts for each operation under `scripts/`
- Multi-language support: Chinese, English, Japanese, Russian

**Use Cases**
- Motor initialization and zero-point calibration on first power-up
- Quick motor status checks and fault diagnosis during routine maintenance
- Standalone motor debugging and testing without ROS 2

**Repository**
https://github.com/openarmx/openarmx_motor_manager

---

## 4. openarmx_teleop_bimanual

**Overview**
A ROS 2 teleoperation package that uses one set of OpenArmX arms as the leader and drives another set as the follower in real time. Supports two modes: free drag without gravity compensation, and gravity-compensated drag with a weightless feel.

**Contents**
- `teleop_bimanual.launch.py`: dual-arm teleoperation without gravity compensation, 200 Hz control rate, 8-DOF (7 joints + gripper)
- `teleop_bimanual_with_gravitycomp.launch.py`: URDF-based real-time gravity torque compensation teleoperation
- Gravity compensation parameters: compensation scale, damping coefficient, position-hold stiffness, all configurable
- Mode switching: `bimanual`, `left_only`, `right_only`

**Use Cases**
- Dual-robot leader-follower teleoperation data collection (used with openarmx_vla)
- Natural manual drag-teaching in demonstration and instruction scenarios
- Validating follower controller performance and motion tracking accuracy

**Repository**
https://github.com/openarmx/openarmx_teleop_bimanual

---

## 5. openarmx_teleop_exo

**Overview**
Bridges exoskeleton devices into ROS 2 via WebSocket. Processes data through parsing, joint retargeting, and safety bridging to output dual-arm joint control commands for OpenArmX.

**Contents**
- `websocket_teleoperator`: listens on WebSocket (default port 19091), publishes 16-DOF exoskeleton joint commands and gamepad state, with hardware safety gating (~100 Hz)
- `exo_retargeting_node`: applies index mapping, scaling factors, offset angles, and joint limits per YAML config
- `exoskeleton_bridge_node`: joint-difference safety check on first connection, smooth interpolation transition (default 3 s / 50 Hz), then real-time forwarding
- `exoskeleton_display.launch.py`: RViz exoskeleton model visualization
- Supported robot types: `OpenArm`, `OpenArmX` (switched via YAML config)

**Use Cases**
- Connecting Qnbot and similar exoskeleton devices for human-robot collaborative teleoperation
- Collecting exoskeleton-guided dual-arm motion data for model training
- Debugging and calibrating joint mapping between exoskeleton and robot

**Repository**
https://github.com/openarmx/openarmx_teleop_exo

---

## 6. openarmx_teleop_vr

**Overview**
A complete VR teleoperation pipeline consisting of a C++ UDP bridge package and a Python IK teleoperation package, converting VR/OpenXR controller data into dual-arm joint control commands.

**Contents**
- `openarmx_teleop_bridge_vr` (C++): listens on UDP port 5100, publishes controller pose, trigger, grip, and other ROS 2 topics; optional TF publishing
- `openarmx_teleop_vr` (Python): subscribes to bridge topics, performs IK computation and constraint handling, outputs dual-arm `forward_position_controller` commands
- Supports mainstream VR devices including Pico and Meta Quest (used with openarmx_teleop_vr_apk)

**Use Cases**
- Immersive VR headset teleoperation of OpenArmX dual arms
- High-quality VR teleoperation demonstration data collection for openarmx_vla
- Validating IK algorithms and end-effector pose tracking accuracy

**Repository**
https://github.com/openarmx/openarmx_teleop_vr

---

## 7. openarmx_teleop_vr_apk

**Overview**
APK installer repository for the VR device-side bridge application. Centralizes distribution of the client app that forwards VR controller data to openarmx_teleop_bridge_vr.

**Contents**
- `openarmx-vr-pico.apk`: bridge APK for Pico series devices
- Meta Quest compatible APK
- ADB installation guide (enabling developer mode, USB debugging, adb install procedure)

**Use Cases**
- Installing the device-side bridge software when setting up a VR teleoperation environment for the first time
- Updating the bridge application on Pico or Meta Quest devices

**Repository**
https://github.com/openarmx/openarmx_teleop_vr_apk

---

## 8. openarmx_tools

**Overview**
A collection of engineering debugging and teaching tools. Each sub-package can be compiled and used independently, covering the full workflow from joint control and gripper debugging to parameter tuning and trajectory recording/playback.

**Contents**
- `openarmx_joint_slider_panel`: RViz2 dual-arm joint slider panel with segmented step execution
- `openarmx_gripper_panel`: RViz2 gripper control panel supporting single or synchronized dual-gripper control (GripperCommand action)
- `openarmx_kp_kd_panel`: RViz2 real-time KP/KD parameter tuning panel for stiffness and damping adjustment on real hardware
- `openarmx_teach`: trajectory teaching tool — records YAML trajectories from `/joint_states` and plays them back, with joint filtering and rate scaling

**Use Cases**
- Quickly verifying joint motion and gripper actions during real-hardware debugging
- KP/KD parameter tuning before bringing a controller online
- Recording and replaying teaching trajectories without writing any code

**Repository**
https://github.com/openarmx/openarmx_tools

---

## 9. openarmx_vla

**Overview**
An end-to-end embodied intelligence (VLA) workflow based on the LeRobot framework, covering multi-camera teleoperation data collection, ACT model training, and online inference.

**Contents**
- Data collection pipeline (`lerobot-record`): supports 3 RealSense cameras (D405/D435) with synchronized VR teleoperation recording
- GUI one-click launch script (`scripts/vla_collect_gui.sh`): sequentially opens terminals for robot bringup, camera publishing, and data collection
- Centralized config file (`config/vla_collect.env`): unified management of camera parameters, dataset names, resolution/FPS, etc.
- ACT training commands: supports single-GPU and multi-GPU (torchrun) training
- Inference pipeline: dual-machine coordination (IPC + inference machine) with ROS_DOMAIN_ID synchronization guide

**Use Cases**
- Collecting dual-arm manipulation demonstration datasets via VR teleoperation
- Training ACT and other imitation learning policy models on a dedicated GPU server
- Deploying trained models back to the robot for online autonomous inference validation

**Repository**
https://github.com/openarmx/openarmx_vla

---

## 10. openclaw_skill_openarmx_motion_player

**Overview**
An OpenArmX motion playback skill for the OpenClaw platform. Users specify a motion name in natural language; the skill automatically matches the trajectory file, manages bringup, and drives the robot to play it back.

**Contents**
- Natural language motion name matching: scans YAML trajectory files in the `openarmx_teach/motions` directory
- Bringup auto-check and reuse: avoids redundant restarts; applies default KP/KD only when a new bringup is started in real-hardware mode
- `play_joint_trajectory` wrapper: interfaces with `left/right_joint_trajectory_controller` and gripper action interfaces
- Example trajectories (`motions/`) and one-click install script (`scripts/install_demo_motions.sh`)
- Two installation methods: manual deployment or automated execution by OpenClaw via `DEPLOY_WITH_OPENCLAW.md`

**Use Cases**
- Triggering pre-recorded arm motions via natural language in the OpenClaw chat interface
- Quickly verifying that trajectories recorded with `openarmx_teach` play back correctly
- Providing a low-barrier voice/text-triggered motion interface for demos or production line scenarios

**Repository**
https://github.com/openarmx/openclaw_skill_openarmx_motion_player

---

## License

This work is licensed under the Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License (CC BY-NC-SA 4.0).

Copyright (c) 2026 Chengdu Changshu Robot Co., Ltd.

See the [LICENSE](LICENSE) file or visit: http://creativecommons.org/licenses/by-nc-sa/4.0/

## Acknowledgements

This package is part of the OpenArmX robot platform ecosystem, developed for research and industrial applications in the field of collaborative robotics.

---

## Contact Us

### Chengdu Changshu Robot Co., Ltd.

| Contact           | Information                                                                                                  |
| ----------------- | ------------------------------------------------------------------------------------------------------------ |
| 📧 Email          | [openarmrobot@gmail.com](mailto:openarmrobot@gmail.com)                                                      |
| 📱 Phone / WeChat | +86-17746530375                                                                                              |
| 🌐 Website        | [https://openarmx.com/](https://openarmx.com/)                                                               |
| 📍 Address        | Huacheng Machinery Plant, No.11 Xinye 8th Street, West Area, Tianjin Economic-Technological Development Area |
| 👤 Contact Person | Mr. Wang                                                                                                     |
