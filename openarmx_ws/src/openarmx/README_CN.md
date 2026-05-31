# OpenArmX 快速导航

[官方文档](http://docs.openarmx.com/) | [GitHub 组织](https://github.com/openarmx)

![OpenArmX 封面](./img/cover.png)

OpenArmX 是由成都长数机器人有限公司开发的开源双臂协作机器人平台，基于 ROS 2 构建，覆盖从机器人本体描述、底层电机驱动、多模态遥操作到具身智能（VLA）训练推理的完整技术栈。本页汇总了平台核心软件包的关键信息，帮助开发者快速定位所需模块。

---

## 包索引

| 包名 | 简述 |
|------|------|
| [openarmx_description](#1-openarmx_description) | 机器人 URDF/Xacro 描述与三维模型 |
| [openarmx_ros2](#2-openarmx_ros2) | ROS 2 核心库与启动配置（元包） |
| [openarmx_motor_manager](#3-openarmx_motor_manager) | 图形化电机管理与 CAN 接口工具 |
| [openarmx_teleop_bimanual](#4-openarmx_teleop_bimanual) | 同构遥操作包 |
| [openarmx_teleop_exo](#5-openarmx_teleop_exo) | 外骨骼设备遥操作包 |
| [openarmx_teleop_vr](#6-openarmx_teleop_vr) | VR 手柄遥操作链路 |
| [openarmx_teleop_vr_apk](#7-openarmx_teleop_vr_apk) | VR 设备端桥接 APK 安装包 |
| [openarmx_tools](#8-openarmx_tools) | 调试、示教与参数整定工具集 |
| [openarmx_vla](#9-openarmx_vla) | VLA 数据采集、模型训练与在线推理 |
| [openclaw_skill_openarmx_motion_player](#10-openclaw_skill_openarmx_motion_player) | OpenClaw 自然语言动作回放技能 |

---

## 1. openarmx_description

**概述**
OpenArmX 机器人平台的完整 URDF 描述包，提供精确的运动学、动力学和可视化模型，是 ROS 2 环境下所有仿真与控制功能的基础依赖。

**包含内容**
- URDF/Xacro 文件：机械臂（v10, 7-DOF）、本体、末端执行器（OpenArmX Hand）的组件描述及完整装配文件
- 三维网格（STL/DAE）：可视化网格与简化碰撞几何体
- YAML 配置：运动学参数（DH）、关节限位、连杆惯性、零点偏移
- ros2_control 配置：单臂与双臂硬件接口预配置（支持仿真/真实硬件切换）
- RViz 配置与可视化启动文件

**应用场景**
- 作为所有其他包的 URDF 依赖（MoveIt 规划、硬件驱动、遥操作均需此包）
- 在 RViz 中独立可视化机器人模型、验证运动学参数
- 新增机器人变体或末端执行器时在此包扩展配置

**仓库链接**
https://github.com/openarmx/openarmx_description

---

## 2. openarmx_ros2

**概述**
OpenArmX 的 ROS 2 核心元包，聚合底层硬件驱动、启动配置与 MoveIt 规划配置，是控制真实机械臂或启动仿真环境的主要入口。

**包含内容**
- `openarmx`：元包，聚合核心组件
- `openarmx_hardware`：ros2_control 硬件插件，通过 CAN 总线驱动机械臂与夹爪
- `openarmx_bringup`：双臂/单臂启动文件、RViz 配置、夹爪操作接口
- `openarmx_bimanual_moveit_config`：双臂 MoveIt 2 规划配置
- `openarmx_preview_bringup`：机器人关节运动预览控制包
- `openarmx-can_*.deb`：配套电机 CAN 驱动安装包

**应用场景**
- 上电启动真实 OpenArmX 双臂机器人（CAN 模式）
- 启动仿真模式（`use_fake_hardware:=true`）进行软件开发与测试
- 作为遥操作、工具包等上层模块的底层控制服务

**仓库链接**
https://github.com/openarmx/openarmx_ros2

---

## 3. openarmx_motor_manager

**概述**
基于 PySide6 的图形化桌面工具，用于管理 OpenArmX 双臂机器人的 CAN 接口和电机状态，支持同时管理多台机器人。

**包含内容**
- GUI 主程序（`GUI_MultiRobot.py`）：多机器人标签页管理界面
- CAN 接口管理：一键启动/禁用、自动检测真实接口
- 电机控制：批量使能/停止、回零、设置零点、单电机/全部电机测试（MIT/CSP 模式）
- 实时状态监控：位置、速度、扭矩、温度、故障状态
- 命令行脚本：`scripts/` 下提供各操作的独立 Python 脚本
- 多语言支持：中文、英文、日文、俄文

**应用场景**
- 机器人首次上电后的电机初始化与零点标定
- 日常维护中快速检查电机状态与故障排查
- 不依赖 ROS 2 的独立电机调试与测试

**仓库链接**
https://github.com/openarmx/openarmx_motor_manager

---

## 4. openarmx_teleop_bimanual

**概述**
ROS 2 遥操作包，以一套 OpenArmX 机械臂作为主控端，实时驱动另一套作为从动端，支持无重力补偿（自由拖动）和带重力补偿（无重力感）两种模式。

**包含内容**
- `teleop_bimanual.launch.py`：双臂无重力补偿遥操作，200 Hz 控制频率，8-DOF（7 关节 + 夹爪）
- `teleop_bimanual_with_gravitycomp.launch.py`：基于 URDF 实时计算重力力矩的补偿遥操作
- 重力补偿参数：补偿缩放系数、阻尼系数、位置保持刚度等可配置
- 支持模式切换：`bimanual`、`left_only`、`right_only`

**应用场景**
- 双机器人主从遥操作数据采集（配合 openarmx_vla 使用）
- 演示与示教场景下的自然手动拖动示教
- 验证从动端控制器性能与运动跟随精度

**仓库链接**
https://github.com/openarmx/openarmx_teleop_bimanual

---

## 5. openarmx_teleop_exo

**概述**
将外骨骼设备通过 WebSocket 接入 ROS 2，经过数据解析、关节重定向映射与安全桥接，最终输出双臂关节控制命令驱动 OpenArmX。

**包含内容**
- `websocket_teleoperator`：监听 WebSocket（默认端口 19091），发布 16 维外骨骼关节命令与手柄状态，内置硬件安全门控（约 100 Hz）
- `exo_retargeting_node`：按 YAML 配置完成索引映射、缩放系数、偏置角与关节限位处理
- `exoskeleton_bridge_node`：首次接入关节差值安全检查、平滑插值过渡（默认 3 s / 50 Hz），通过后进入实时转发
- `exoskeleton_display.launch.py`：RViz 外骨骼模型可视化
- 支持机器人类型：`OpenArm`、`OpenArmX`（通过 YAML 配置切换）

**应用场景**
- 接入 Qnbot 等外骨骼设备进行人机协同遥操作
- 采集外骨骼引导的双臂运动数据用于模型训练
- 外骨骼与机器人关节映射关系的调试与标定

**仓库链接**
https://github.com/openarmx/openarmx_teleop_exo

---

## 6. openarmx_teleop_vr

**概述**
VR 遥操作完整链路，包含 C++ UDP 桥接包与 Python IK 遥操作包，将 VR/OpenXR 手柄数据转化为双臂关节控制命令。

**包含内容**
- `openarmx_teleop_bridge_vr`（C++）：监听 UDP 端口 5100，发布手柄位姿、扳机、握把等 ROS 2 话题，可选发布 TF
- `openarmx_teleop_vr`（Python）：订阅桥接话题，执行 IK 计算与约束处理，输出双臂 `forward_position_controller` 命令
- 支持 Pico、Meta Quest 等主流 VR 设备（配合 openarmx_teleop_vr_apk 使用）

**应用场景**
- VR 头显沉浸式遥操作 OpenArmX 双臂
- 配合 openarmx_vla 进行高质量 VR 遥操作示教数据采集
- 验证 IK 算法与末端位姿跟踪精度

**仓库链接**
https://github.com/openarmx/openarmx_teleop_vr

---

## 7. openarmx_teleop_vr_apk

**概述**
VR 设备端桥接应用的 APK 安装包仓库，集中分发用于将 VR 手柄数据转发至 openarmx_teleop_bridge_vr 的客户端应用。

**包含内容**
- `openarmx-vr-pico.apk`：适配 Pico 系列设备的桥接 APK
- Meta Quest 适配 APK
- ADB 安装说明（开发者模式开启、USB 调试、adb install 流程）

**应用场景**
- 首次配置 VR 遥操作环境时安装设备端桥接软件
- Pico 或 Meta Quest 设备更新桥接应用版本

**仓库链接**
https://github.com/openarmx/openarmx_teleop_vr_apk

---

## 8. openarmx_tools

**概述**
面向工程调试与示教的工具集合包，各子包可独立编译使用，覆盖关节控制、夹爪调试、参数整定和轨迹录制回放全流程。

**包含内容**
- `openarmx_joint_slider_panel`：RViz2 双臂关节滑块面板，支持分段步进执行
- `openarmx_gripper_panel`：RViz2 夹爪控制面板，支持单/双夹爪同步控制（GripperCommand action）
- `openarmx_kp_kd_panel`：RViz2 KP/KD 实时参数调节面板，适用于实机刚度/阻尼整定
- `openarmx_teach`：轨迹示教工具，从 `/joint_states` 录制 YAML 轨迹并回放，支持关节筛选与速率缩放

**应用场景**
- 实机调试时快速验证各关节运动与夹爪动作
- 控制器上线前的 KP/KD 参数整定
- 无需编程即可录制示教轨迹并重复回放验证

**仓库链接**
https://github.com/openarmx/openarmx_tools

---

## 9. openarmx_vla

**概述**
基于 LeRobot 框架的具身智能（VLA）端到端工作流，覆盖多相机遥操作数据采集、ACT 模型训练到在线推理的完整流程。

**包含内容**
- 数据采集流程（`lerobot-record`）：支持 3 路 RealSense 相机（D405/D435）、VR 遥操作同步录制
- GUI 一键启动脚本（`scripts/vla_collect_gui.sh`）：按序自动拉起机器人底层、相机发布、数采各终端
- 集中配置文件（`config/vla_collect.env`）：统一管理相机参数、数据集名称、分辨率/帧率等
- ACT 训练命令：支持单卡与多卡（torchrun）训练
- 推理流程：工控机 + 推理机双机协同，ROS_DOMAIN_ID 同步配置说明

**应用场景**
- 采集 VR 遥操作的双臂操作示教数据集
- 在独立 GPU 服务器上训练 ACT 等模仿学习策略模型
- 将训练好的模型部署回机器人进行在线自主推理验证

**仓库链接**
https://github.com/openarmx/openarmx_vla

---

## 10. openclaw_skill_openarmx_motion_player

**概述**
面向 OpenClaw 平台的 OpenArmX 动作回放技能，用户通过自然语言指定动作名称，技能自动匹配轨迹文件、管理 bringup 并驱动机器人回放。

**包含内容**
- 自然语言动作名匹配逻辑：扫描 `openarmx_teach/motions` 目录中的 YAML 轨迹文件
- bringup 自动检查与复用：避免重复启动，真机模式下仅在新开 bringup 时应用默认 KP/KD
- `play_joint_trajectory` 调用封装：对接 `left/right_joint_trajectory_controller` 及夹爪 action 接口
- 示例轨迹（`motions/`）及一键安装脚本（`scripts/install_demo_motions.sh`）
- 双重安装方式：人工手动部署 或 交由 OpenClaw 按 `DEPLOY_WITH_OPENCLAW.md` 自动执行

**应用场景**
- 在 OpenClaw 对话界面中用自然语言触发预录制的机械臂动作
- 快速验证 `openarmx_teach` 录制的轨迹是否可正常回放
- 为演示或产线场景提供低门槛的语音/文字触发动作接口

**仓库链接**
https://github.com/openarmx/openclaw_skill_openarmx_motion_player

---

## 许可证

本作品采用知识共享 署名-非商业性使用-相同方式共享 4.0 国际许可协议 (CC BY-NC-SA 4.0) 进行许可。

版权所有 (c) 2026 成都长数机器人有限公司 (Chengdu Changshu Robot Co., Ltd.)

详情请参阅 [LICENSE\_CN.md](LICENSE) 文件或访问：http://creativecommons.org/licenses/by-nc-sa/4.0/


## 致谢

本包是 OpenArmX 机器人平台生态系统的一部分，专为协作机器人领域的研究和工业应用而开发。

---

## 📞 联系我们

### 成都长数机器人有限公司
**Chengdu Changshu Robotics Co., Ltd.**

| 联系方式 | 信息 |
|---------|------|
| 📧 邮箱 | openarmrobot@gmail.com |
| 📱 电话/微信 | +86-17746530375 |
| 🌐 官网 | https://openarmx.com/|
| 🌐 文档 | http://docs.openarmx.com/|
| 📍 地址 | 天津市西青区・稻潮机器人体验基地（明日之城）・天津市人形机器人中心 |
| 👤 联系人 | 王先生 |
