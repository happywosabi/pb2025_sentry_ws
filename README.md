# PB2025 哨兵机器人开发手册

欢迎使用深圳北理莫斯科大学北极熊战队 RoboMaster 2025 哨兵机器人开发手册！

本手册提供了完整的系统架构、节点详解、话题说明、参数配置和调试指南。

> **当前感知层主方案**：[sp_vision_25](./src/sp_vision_25/) — 单体高性能视觉系统，原生访问相机硬件，
> 内置 YOLO 检测、EKF 跟踪、弹道解算与自主射击决策，直接发布 `/cmd_gimbal` 与 `/cmd_shoot`。
> 旧的 `pb2025_rm_vision`（armor_detector / armor_tracker / projectile_motion）已**废弃保留**，
> 仅在 `use_sp_vision:=False` 时启用，用于回归对比，不再作为主线维护。

## 📚 文档导航

### 基础架构
- **[01. 系统架构总览](./docs/01_系统架构.md)** — 四层架构、数据流、关键技术栈

### 功能层详解
- **[02. 硬件接口层](./docs/02_硬件接口层.md)** — 串口通信 (standard_robot_pp_ros2)、工业相机驱动 (hik_camera_ros2_driver)
- **[03. 感知层](./docs/03_感知层.md)** — sp_vision_25 主感知系统；附录保留旧 pb2025_rm_vision 概览
- **[04. 导航层](./docs/04_导航层.md)** — Point-LIO 定位、small_gicp 重定位、地形分析、Nav2
- **[05. 决策层](./docs/05_决策层.md)** — BehaviorTree.CPP 行为树（仅负责导航与战术决策）

### 开发指南
- **[06. ROS话题详解](./docs/06_ROS话题详解.md)** — 自定义消息、当前感知/导航/决策话题
- **[07. 参数配置指南](./docs/07_参数配置.md)** — sp_vision_25 YAML、`node_params.yaml` 调优
- **[08. 运行与调试指南](./docs/08_运行与调试.md)** — 启动、单模块调试、Rosbag 工作流、排错

### 视觉系统专题
- **[09. sp_vision_25 深度分析](./docs/09_sp_vision_25分析.md)** ⭐ — 项目结构、MPC 轨迹规划、算法详解
- **[13. 四元数转换与参数调优](./docs/13_四元数转换与参数调优.md)** — sp_vision_25 手眼标定与坐标变换调优

### 工程参考
- **[12. 新项目安装教程](./docs/12_新项目安装教程.md)** — Ubuntu/ROS 2 全新环境部署
- **[相机标定与测试指南](./相机标定与测试指南.md)** — sp_vision_25 相机标定流程
- **[00. Mermaid 颜色方案](./docs/00_Mermaid颜色方案.md)** — 文档配图配色标准

## 🚀 快速开始

### 脚本安装与编译（推荐）
仓库提供了两个 Bash 脚本，用于在 Ubuntu 22.04 + ROS 2 Humble 环境中一键准备依赖并编译工作空间：

```bash
# 1. 安装系统依赖、OpenVINO、small_gicp，并生成 scripts/env.sh
./scripts/install_env.sh

# 2. 载入环境变量（后续新终端都建议先执行）
source scripts/env.sh

# 3. 编译工作空间
./scripts/build_ws.sh
```

`scripts/install_env.sh` 会安装基础 apt / ROS 依赖，配置 OpenVINO 与 small_gicp，并根据本机实际路径生成 `scripts/env.sh`。该脚本可重复执行，已安装的依赖会自动跳过。

`scripts/build_ws.sh` 会自动 source `scripts/env.sh`，再使用 `colcon build --symlink-install` 以 Release 模式编译；需要追加 colcon 参数时，可直接跟在脚本后：

```bash
./scripts/build_ws.sh --packages-select sp_vision_25
```

### 构建项目
```bash
# 安装依赖
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 构建工作空间（始终带 --symlink-install）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
source install/setup.bash
```

### 启动系统（推荐：sp_vision_25）
```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=<YOUR_WORLD_NAME> \
  use_sp_vision:=True \
  sp_vision_config:=sentry.yaml \
  use_rviz:=True \
  params_file:=$(pwd)/src/pb2025_sentry_bringup/params/node_params.yaml
```

> 启用 `use_sp_vision:=True` 时，`hik_camera_ros2_driver` 不会启动 — sp_vision_25 直接独占相机硬件。

### 兼容模式（旧 pb2025_rm_vision，已废弃）
```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=<YOUR_WORLD_NAME> \
  use_sp_vision:=False \
  detector:=opencv \
  use_rviz:=True
```

详细说明请参阅 **[运行与调试指南](./docs/08_运行与调试.md)**。

## 📊 系统概览

### 四层架构
```
┌─────────────────────────────────────────────┐
│         决策层 (Decision Layer)             │
│    BehaviorTree.CPP — 仅导航/战术决策        │
└─────────────────┬───────────────────────────┘
                  │ /goal_pose, /cmd_vel
┌─────────────────┴───────────────────────────┐
│         导航层 (Navigation Layer)           │
│  Point-LIO + small_gicp + Nav2 + 地形分析   │
└─────────────────┬───────────────────────────┘
                  │ /Odometry, /cmd_vel
┌─────────────────┴───────────────────────────┐
│         感知层 (Perception Layer)           │
│  sp_vision_25 — 单体程序，原生相机访问       │
│  YOLO 检测 + EKF 跟踪 + 弹道解算 + 自主射击 │
└─────────────────┬───────────────────────────┘
                  │ /cmd_gimbal, /cmd_shoot
┌─────────────────┴───────────────────────────┐
│      硬件接口层 (Hardware Layer)            │
│    standard_robot_pp_ros2（串口/裁判系统）  │
└─────────────────────────────────────────────┘
```

> **注**：集成层 `pb2025_sentry_bringup` 横跨四层，提供统一 launch 与 `node_params.yaml`。

### 关键数据流

**感知流水线（sp_vision_25 内部）**：
```
相机硬件 → YOLO 检测 → EKF 跟踪 → 弹道解算 → /cmd_gimbal + /cmd_shoot → 串口 → STM32
```

**导航流水线**：
```
LiDAR + IMU → Point-LIO → /Odometry → 地形分析 → 代价地图 → Nav2 规划/控制 → /cmd_vel → 串口
```

**决策流水线**：
```
/referee/* + /global_costmap → 全局黑板 → 行为树（5Hz Tick）→ /goal_pose · /cmd_vel
```

> 射击决策完全由 sp_vision_25 自主完成，**不经过行为树**。

## 🛠️ 开发资源

### 外部依赖
- **ROS 2 Humble** — 机器人操作系统
- **OpenVINO 2023.3** — sp_vision_25 与旧 OpenVINO 检测器共用的推理引擎
- **small_gicp** — 点云配准库（重定位）
- **Ignition Fortress** — Gazebo 仿真环境
- **BehaviorTree.CPP v4** — 行为树框架

### 相关仓库
- [SMBU PolarBear Robotics Team GitHub](https://github.com/SMBU-PolarBear-Robotics-Team)
- [dependencies.repos](./dependencies.repos) — 子模块依赖列表

## 📝 文档版本

- **版本**: v2.0（sp_vision_25 主线版）
- **更新日期**: 2026-04-22
- **适用系统**: pb2025_sentry_ws

## 🤝 贡献指南

本文档随代码持续更新。如发现文档问题或需要补充，请提交 Issue 或 Pull Request。

---

**下一步**：阅读 [系统架构总览](./docs/01_系统架构.md) 了解整体设计。
