# PB2025 哨兵机器人开发手册

欢迎使用深圳北理莫斯科大学北极熊战队 RoboMaster 2025 哨兵机器人开发手册！

本手册提供了完整的系统架构、节点详解、话题说明、参数配置和调试指南。

## 📚 文档导航

### 基础架构
- **[01. 系统架构总览](./docs/01_系统架构.md)** - 整体架构、数据流、模块关系
  - 五层架构设计
  - 系统数据流向图
  - 关键技术栈

### 功能层详解
- **[02. 硬件接口层](./docs/02_硬件接口层.md)** - 串口通信、相机驱动
  - 串口通信节点 (standard_robot_pp_ros2)
  - 工业相机驱动节点 (hik_camera_ros2_driver)

- **[03. 感知层](./docs/03_感知层.md)** - 视觉检测与追踪
  - 装甲板检测节点 (OpenCV/OpenVINO)
  - 目标追踪节点 (armor_tracker)
  - 弹道计算节点 (projectile_motion)

- **[04. 导航层](./docs/04_导航层.md)** - 定位、建图与路径规划
  - LiDAR驱动与里程计 (point_lio)
  - 全局重定位 (small_gicp_relocalization)
  - 地形分析 (terrain_analysis)
  - Nav2导航栈配置

- **[05. 决策层](./docs/05_决策层.md)** - 行为树决策框架
  - 行为树服务器
  - 条件节点详解
  - 动作节点详解

### 开发指南
- **[06. ROS话题详解](./docs/06_ROS话题详解.md)** - 所有话题和消息类型
  - 自定义消息接口
  - 话题订阅/发布关系
  - 裁判系统消息详解

- **[07. 参数配置指南](./docs/07_参数配置.md)** - 参数说明与调优
  - 中心配置文件结构
  - 各模块参数详解
  - 调试参数建议

- **[08. 运行与调试指南](./docs/08_运行与调试.md)** - 启动、测试、问题排查
  - 完整系统启动
  - 单模块调试
  - Rosbag工作流
  - 常见问题解决

## 🚀 快速开始

### 构建项目
```bash
# 安装依赖
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 构建工作空间
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
```

### 启动系统
```bash
# 启动完整系统
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=<YOUR_WORLD_NAME> \
  use_rviz:=True \
  params_file:=$(pwd)/src/pb2025_sentry_bringup/params/node_params.yaml
```

详细说明请参阅 **[运行与调试指南](./docs/08_运行与调试.md)**。

## 📊 系统概览

### 五层架构
```
┌─────────────────────────────────────────┐
│         决策层 (Decision Layer)         │
│    BehaviorTree.CPP 行为树框架          │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────┴───────────────────────┐
│         导航层 (Navigation Layer)       │
│  Point-LIO + Nav2 + 地形分析            │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────┴───────────────────────┐
│         感知层 (Perception Layer)       │
│  装甲板检测 + EKF追踪 + 弹道解算        │
└─────────────────┬───────────────────────┘
                  │
┌─────────────────┴───────────────────────┐
│      硬件接口层 (Hardware Layer)        │
│    串口通信 + 工业相机驱动              │
└─────────────────────────────────────────┘
```

### 关键数据流

**感知流水线**：
```
相机图像 → 装甲板检测 → 目标追踪 → 弹道计算 → 云台指令 → 嵌入式系统
```

**导航流水线**：
```
LiDAR+IMU → 点云配准 → 里程计 → 地形分析 → 代价地图 → 路径规划 → 底盘指令
```

**决策流水线**：
```
裁判系统 + 视觉 + 导航 → 全局黑板 → 行为树 → 导航目标/速度指令
```

## 🛠️ 开发资源

### 外部依赖
- **ROS2 Humble** - 机器人操作系统
- **OpenVINO 2023.3** - 神经网络推理引擎
- **small_gicp** - 点云配准库
- **Ignition Fortress** - Gazebo仿真环境
- **BehaviorTree.CPP v4** - 行为树框架

### 相关仓库
- [SMBU PolarBear Robotics Team GitHub](https://github.com/SMBU-PolarBear-Robotics-Team)
- [dependencies.repos](./dependencies.repos) - 子模块依赖列表

## 📝 文档版本

- **版本**: v1.0
- **更新日期**: 2025-11-22
- **适用系统**: pb2025_sentry_ws

## 🤝 贡献指南

本文档随代码持续更新。如发现文档问题或需要补充，请提交 Issue 或 Pull Request。

---

**下一步**：阅读 [系统架构总览](./docs/01_系统架构.md) 了解整体设计。
