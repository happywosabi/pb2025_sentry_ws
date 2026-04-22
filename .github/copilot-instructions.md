# Project Guidelines — pb2025_sentry_ws

RoboMaster 2025 哨兵机器人 ROS2 (Humble) 工作空间。多仓库结构，包含串口通信、视觉、导航、决策四大功能层。

> 完整架构与算法细节见 [CLAUDE.md](../CLAUDE.md) 和 [docs/](../docs/) 目录。

## Build & Test

```bash
# 完整构建（始终使用 --symlink-install）
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10

# 单包构建
colcon build --symlink-install --packages-select <package_name> --cmake-args -DCMAKE_BUILD_TYPE=Release

# 安装依赖
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# 更新子仓库
vcs pull ./src
```

构建后务必执行 `source install/setup.bash`。

## Code Style

- **C++ 标准**: C++14（多数包）/ C++17（pb2025_sentry_behavior）
- **代码风格**: Google style，由 `.clang-format` 和 `.clang-tidy` 强制
- **命名规范**: 类名 `CamelCase`，方法 `camelBack`，变量 `lower_case`，成员变量 `lower_case_`
- **编译选项**: `-Wall -Werror` 全包启用
- **CMake 模式**: 使用 `ament_cmake_auto` + `ament_auto_find_build_dependencies()` + `ament_auto_package()`
- 组件节点注册: `rclcpp_components_register_node()`

## Architecture

四层架构（集成层横跨四层），详见 [docs/01_系统架构.md](../docs/01_系统架构.md)：

| 层 | 关键包 | 说明 |
|---|--------|------|
| 硬件接口 | `standard_robot_pp_ros2`, `hik_camera_ros2_driver` *(use_sp_vision=False 时)* | 串口、相机驱动 |
| 感知 | **`sp_vision_25`** + `sp_vision_ros2_adapter` | YOLO 检测 + EKF 跟踪 + 弹道；自主控制云台/射击 |
| 导航 | `point_lio`, `small_gicp_relocalization`, `terrain_analysis`, `pb_nav2_plugins` | LiDAR 定位 & Nav2 |
| 决策 | `pb2025_sentry_behavior` | BehaviorTree.CPP v4，仅负责导航与战术决策 |
| 集成 | `pb2025_sentry_bringup` | 统一 launch & 参数 |

> 旧 `pb2025_rm_vision`（armor_detector_opencv/openvino + armor_tracker + projectile_motion）已**废弃保留**：仅在 `use_sp_vision:=False` 时启用，新功能不再添加。

**参数中心**: [`src/pb2025_sentry_bringup/params/node_params.yaml`](../src/pb2025_sentry_bringup/params/node_params.yaml)（含旧视觉参数，sp_vision 模式不读取）  
**sp_vision 配置**: [`src/sp_vision_25/configs/sentry.yaml`](../src/sp_vision_25/configs/sentry.yaml)  
**主 Launch**: [`src/pb2025_sentry_bringup/launch/bringup.launch.py`](../src/pb2025_sentry_bringup/launch/bringup.launch.py)

### 视觉系统切换

- **默认（推荐）**: `use_sp_vision:=True` — 启动 sp_vision_25，独占相机硬件，**不**启动 hik_camera_ros2_driver
- **兼容模式（已废弃）**: `use_sp_vision:=False` — 启动 pb2025_rm_vision 多节点链路（detector → tracker → projectile_motion）+ hik_camera_ros2_driver
- 当前 `bringup.launch.py` 默认值仍为 `False`，新部署应显式传 `use_sp_vision:=True`

## Conventions

### 必须遵守

1. **始终 `--symlink-install`** — 否则改 .py/.yaml/.xml 后需要重新构建
2. **地图/PCD 同名约定** — `world:=arena` 需要 `arena.yaml` + `arena.pgm` + `arena.pcd`
3. **BehaviorTree.ROS2 Action 节点 halt bug** — 用 `PubNav2Goal`（话题）替代 `SendNav2Goal`（Action）
4. **坐标系**: sp_vision_25 内部用 `gimbal` 系（详见 docs/13），导航用 `map → odom → base_footprint`
5. **Rosbag 回放（仅旧视觉模式有意义）**: `use_sim_time:=True` + `use_robot_state_pub:=True` + `use_hik_camera:=False`；sp_vision_25 直采相机不便回放

### 多仓库管理

子仓库由 vcstool 管理，见 [`dependencies.repos`](../dependencies.repos)。每个功能模块是独立 Git 仓库。

### 行为树开发

- 插件模式: 条件节点 `plugins/condition/`，动作节点 `plugins/action/`
- Blackboard: `{@key}` 全局引用，`{variable}` 局部变量
- XML 文件: `src/pb2025_sentry_behavior/behavior_trees/`
- 行为树**不控制射击**（由 sp_vision_25 自主完成），仅发布 `/goal_pose` 与 `/cmd_vel`
- 详见 [docs/05_决策层.md](../docs/05_决策层.md)

## Troubleshooting

- **sp_vision 无检测/瞄不准**: 检查 `src/sp_vision_25/configs/sentry.yaml` 的 `enemy_color`、`exposure_ms`、`gain`、`yaw_offset`、`pitch_offset`；启用 `debug.enable_visualization` 看 `/sentry_debug/image`
- **导航卡住**: 检查 costmap 可视化，增大 `inflation_radius`
- **串口连接失败**: `ls -l /dev/ttyACM*`，确认波特率 115200
- **TF 查找失败**: 确保 `robot_state_publisher` 正在运行
- **旧视觉模式无检测（已废弃）**: 检查 `binary_thres`（80±20）、`classifier_threshold`（0.25±0.15）

详细排查: [docs/08_运行与调试.md](../docs/08_运行与调试.md)

## Documentation Index

| 文档 | 内容 |
|------|------|
| [01_系统架构](../docs/01_系统架构.md) | 四层架构、数据流、技术栈 |
| [02_硬件接口层](../docs/02_硬件接口层.md) | 串口通信、相机驱动 |
| [03_感知层](../docs/03_感知层.md) | sp_vision_25 主感知；附录保留旧 pb2025_rm_vision |
| [04_导航层](../docs/04_导航层.md) | Point-LIO、重定位、Nav2 |
| [05_决策层](../docs/05_决策层.md) | 行为树框架（仅导航/战术，不含射击） |
| [06_ROS话题详解](../docs/06_ROS话题详解.md) | 当前感知/导航/决策话题与自定义消息 |
| [07_参数配置](../docs/07_参数配置.md) | sp_vision YAML + node_params 调优 |
| [08_运行与调试](../docs/08_运行与调试.md) | 运行、调试、排错 |
| [09_sp_vision_25分析](../docs/09_sp_vision_25分析.md) | sp_vision 深度分析与 MPC 算法 |
| [12_新项目安装教程](../docs/12_新项目安装教程.md) | 新环境安装教程 |
| [13_四元数转换与参数调优](../docs/13_四元数转换与参数调优.md) | sp_vision_25 坐标变换与手眼标定 |
