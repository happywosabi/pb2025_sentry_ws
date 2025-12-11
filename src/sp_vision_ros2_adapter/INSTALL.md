# INSTALL.md - sp_vision_ros2_adapter 安装配置指南

本文档提供 `sp_vision_ros2_adapter` 包的完整安装和配置步骤，适用于在实车上部署。

## 目录

1. [系统要求](#系统要求)
2. [依赖安装](#依赖安装)
3. [编译安装](#编译安装)
4. [参数配置](#参数配置)
5. [验证安装](#验证安装)
6. [常见问题](#常见问题)

---

## 系统要求

### 硬件要求

- **CPU**: Intel/AMD x86_64 (推荐4核及以上)
- **内存**: 最低4GB (推荐8GB+)
- **存储**: 最低500MB可用空间

### 软件要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS版本**: ROS2 Humble
- **编译器**: GCC 11+ / Clang 14+
- **CMake**: 3.16+

---

## 依赖安装

### 1. ROS2 Humble基础包

```bash
# 如果尚未安装ROS2 Humble，请参考官方文档：
# https://docs.ros.org/en/humble/Installation.html

# 确保ROS2已source
source /opt/ros/humble/setup.bash
```

### 2. 系统依赖库

```bash
# 安装Eigen3、OpenCV和开发工具
sudo apt update
sudo apt install -y \
  libeigen3-dev \
  libopencv-dev \
  build-essential \
  cmake \
  git
```

### 3. ROS2消息包依赖

```bash
# 安装标准ROS2消息包
sudo apt install -y \
  ros-humble-sensor-msgs \
  ros-humble-geometry-msgs \
  ros-humble-std-msgs \
  ros-humble-cv-bridge \
  ros-humble-example-interfaces
```

### 4. pb2025自定义接口包

这些接口包应该已经在 `pb2025_sentry_ws` 工作空间中：

- `pb_rm_interfaces` - 云台控制、裁判系统消息
- `auto_aim_interfaces` - 装甲板检测、跟踪消息

如果缺失，请确保工作空间完整：

```bash
cd /home/happywosabi/pb2025_sentry_ws
ls src/interfaces/pb_rm_interfaces
ls src/interfaces/auto_aim_interfaces
```

### 5. 验证依赖安装

```bash
# 验证Eigen3
pkg-config --modversion eigen3
# 应输出：3.4.0 或更高

# 验证OpenCV
pkg-config --modversion opencv4
# 应输出：4.5.4 或更高

# 验证cv_bridge
ros2 pkg list | grep cv_bridge
# 应输出：cv_bridge
```

---

## 编译安装

### 1. 进入工作空间

```bash
cd /home/happywosabi/pb2025_sentry_ws
source /opt/ros/humble/setup.bash
```

### 2. 编译sp_vision_ros2_adapter包

**推荐方式（使用 `--symlink-install`）**：

```bash
colcon build --symlink-install --packages-select sp_vision_ros2_adapter \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

**为什么使用 `--symlink-install`**：
- 允许修改配置文件（`.yaml`）和启动文件（`.py`）后无需重新编译
- 节省编译时间和磁盘空间
- 适合开发和调试

**编译输出示例**：

```
Starting >>> sp_vision_ros2_adapter
Finished <<< sp_vision_ros2_adapter [10.2s]

Summary: 1 package finished [10.5s]
```

### 3. Source安装环境

```bash
source install/setup.bash
```

**重要**：每次打开新终端都需要执行此命令，或将其添加到 `~/.bashrc`：

```bash
echo "source /home/happywosabi/pb2025_sentry_ws/install/setup.bash" >> ~/.bashrc
```

---

## 参数配置

### 1. 配置文件位置

主配置文件：

```
/home/happywosabi/pb2025_sentry_ws/src/sp_vision_ros2_adapter/config/adapter_params.yaml
```

如果使用了 `--symlink-install`，修改源文件即可生效，无需重新编译。

### 2. 关键参数配置

编辑 `config/adapter_params.yaml`：

```yaml
/**:
  ros__parameters:
    # ============================================================================
    # 子弹速度参数 (m/s)
    # ============================================================================
    # 说明：sp_vision_25需要子弹速度用于弹道计算
    # 由于pb2025的RobotStatus消息不包含bullet_speed字段，需要手动配置
    # 参考值：
    #   - 步兵/哨兵：21.5 m/s（国赛标准）
    #   - 英雄：10.0 m/s（低速大弹丸）
    # 调优方法：如果弹道偏高/偏低，微调此值（±0.5 m/s）
    bullet_speed: 21.5

    # ============================================================================
    # 是否使用sp_vision的目标话题（可选功能）
    # ============================================================================
    # 说明：sp_vision_25可以通过ROS2发布目标位置字符串
    # 如果启用，适配器将订阅sp_vision的目标话题并转换为Target消息
    # 默认：false（Phase 3可选功能，实车上应设为false）
    use_sp_target_topic: false

    # ============================================================================
    # 手眼标定矩阵：云台到IMU的旋转矩阵 R_gimbal_to_imu
    # ============================================================================
    # 说明：用于将关节角度(yaw, pitch)转换为IMU四元数
    # 此矩阵应从sp_vision_25的configs/sentry.yaml中复制
    # 格式：3×3旋转矩阵，行主序（9个元素）
    #
    # 示例：单位矩阵（如果未标定，使用此默认值）
    # R_gimbal_to_imu: [1.0, 0.0, 0.0,
    #                   0.0, 1.0, 0.0,
    #                   0.0, 0.0, 1.0]
    #
    # ⚠️ 实际值应从sp_vision_25/configs/sentry.yaml的R_gimbal2imubody字段复制
    # 当前使用单位矩阵作为占位符，需要更新为真实标定值
    R_gimbal_to_imu: [1.0, 0.0, 0.0,
                      0.0, 1.0, 0.0,
                      0.0, 0.0, 1.0]
```

### 3. 配置手眼标定矩阵（重要！）

**步骤1：定位sp_vision_25标定文件**

```bash
# 假设sp_vision_25位于以下路径（根据实际情况调整）
cd ~/testopenvino/sp_vision_25/configs
cat sentry.yaml
```

**步骤2：查找 `R_gimbal2imubody` 字段**

在 `sentry.yaml` 中找到类似以下内容：

```yaml
R_gimbal2imubody: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [0.999847, 0.0174524, 0.0,
         -0.0174524, 0.999847, 0.0,
         0.0, 0.0, 1.0]
```

**步骤3：复制数据到adapter_params.yaml**

将 `data` 数组中的9个数字复制到 `adapter_params.yaml` 的 `R_gimbal_to_imu` 参数：

```yaml
R_gimbal_to_imu: [0.999847, 0.0174524, 0.0,
                  -0.0174524, 0.999847, 0.0,
                  0.0, 0.0, 1.0]
```

**⚠️ 警告**：
- 如果此矩阵不正确，IMU四元数转换会出错，导致sp_vision跟踪精度下降
- 必须确保矩阵是3×3，共9个元素
- 元素顺序为行主序（第1行3个，第2行3个，第3行3个）

### 4. 配置子弹速度（可选）

根据实际机器人类型调整 `bullet_speed` 参数：

| 机器人类型 | 推荐速度 (m/s) | 说明 |
|-----------|--------------|------|
| 哨兵 | 21.5 | 国赛标准小弹丸速度 |
| 步兵 | 21.5 | 国赛标准小弹丸速度 |
| 英雄 | 10.0 | 低速大弹丸 |

**调优建议**：
- 如果实际弹道偏高，**减小**此值（例如：21.5 → 21.0）
- 如果实际弹道偏低，**增大**此值（例如：21.5 → 22.0）
- 调整步长建议为0.5 m/s

---

## 验证安装

### 1. 检查节点可执行文件

```bash
# 检查节点是否编译成功
ros2 pkg executables sp_vision_ros2_adapter
# 应输出：sp_vision_ros2_adapter adapter_node
```

### 2. 检查启动文件

```bash
# 列出包中的启动文件
ros2 pkg prefix sp_vision_ros2_adapter
# 应输出：/home/happywosabi/pb2025_sentry_ws/install/sp_vision_ros2_adapter
```

### 3. 测试启动节点（无硬件）

```bash
# 启动适配器节点（不连接硬件，仅测试启动）
ros2 launch sp_vision_ros2_adapter adapter_launch.py
```

**预期输出**：

```
[INFO] [sp_vision_ros2_adapter]: Initializing sp_vision_ros2_adapter node
[INFO] [sp_vision_ros2_adapter]: Loaded R_gimbal_to_imu calibration matrix
[INFO] [sp_vision_ros2_adapter]: sp_vision_ros2_adapter node initialized successfully
[INFO] [sp_vision_ros2_adapter]:   - bullet_speed: 21.50 m/s
[INFO] [sp_vision_ros2_adapter]:   - use_sp_target_topic: false
```

**如果看到以上输出**：✅ 安装成功！按 `Ctrl+C` 退出。

**如果看到错误**：参考下方"常见问题"章节。

### 4. 检查话题订阅/发布（节点运行时）

在另一个终端中：

```bash
# 检查节点是否正在运行
ros2 node list | grep sp_vision_ros2_adapter
# 应输出：/sp_vision_ros2_adapter

# 检查订阅的话题
ros2 node info /sp_vision_ros2_adapter | grep "Subscribers:"
# 应输出：
#   /front_industrial_camera/image
#   /serial/gimbal_joint_state
#   /referee/robot_status

# 检查发布的话题
ros2 node info /sp_vision_ros2_adapter | grep "Publishers:"
# 应输出：
#   /cmd_gimbal
#   /cmd_shoot
#   /tracker/target
```

---

## 常见问题

### Q1: 编译失败 - 找不到Eigen3

**错误信息**：

```
CMake Error at CMakeLists.txt:XX (find_package):
  Could not find a package configuration file provided by "Eigen3"
```

**解决方法**：

```bash
# 安装Eigen3开发库
sudo apt install libeigen3-dev

# 验证安装
pkg-config --modversion eigen3
```

### Q2: 编译失败 - 找不到cv_bridge

**错误信息**：

```
Could not find a package configuration file provided by "cv_bridge"
```

**解决方法**：

```bash
# 安装cv_bridge
sudo apt install ros-humble-cv-bridge

# 重新编译
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
```

### Q3: 编译失败 - 找不到pb_rm_interfaces或auto_aim_interfaces

**错误信息**：

```
Could not find a package configuration file provided by "pb_rm_interfaces"
```

**原因**：这些是pb2025工作空间的自定义接口包，需要先编译。

**解决方法**：

```bash
cd /home/happywosabi/pb2025_sentry_ws

# 先编译接口包
colcon build --symlink-install --packages-select pb_rm_interfaces auto_aim_interfaces

# 重新source
source install/setup.bash

# 再编译适配器
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
```

### Q4: 启动失败 - 参数文件未找到

**错误信息**：

```
[ERROR] [sp_vision_ros2_adapter]: Failed to load parameters from file
```

**原因**：未使用 `--symlink-install` 编译，或参数文件路径错误。

**解决方法1**：重新编译使用symlink

```bash
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
source install/setup.bash
```

**解决方法2**：指定绝对路径

```bash
ros2 launch sp_vision_ros2_adapter adapter_launch.py \
  params_file:=/home/happywosabi/pb2025_sentry_ws/src/sp_vision_ros2_adapter/config/adapter_params.yaml
```

### Q5: 修改参数文件后不生效

**原因**：未使用 `--symlink-install` 编译，需要重新编译或重新安装配置。

**解决方法**：

```bash
# 方法1：重新编译（如果未使用symlink）
colcon build --packages-select sp_vision_ros2_adapter
source install/setup.bash

# 方法2：确认使用了symlink安装
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
```

### Q6: R_gimbal_to_imu参数格式错误

**错误信息**：

```
[ERROR] [sp_vision_ros2_adapter]: R_gimbal_to_imu parameter must have 9 elements, got X
[WARN] [sp_vision_ros2_adapter]: Using identity matrix as fallback
```

**原因**：`R_gimbal_to_imu` 数组元素数量不是9。

**解决方法**：

检查 `config/adapter_params.yaml` 中的 `R_gimbal_to_imu` 参数，确保：
1. 共有9个数字
2. 格式正确（用逗号分隔）
3. 没有额外的空格或换行

正确示例：

```yaml
R_gimbal_to_imu: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
```

---

## 下一步

安装完成后，请参考以下文档：

- **[USAGE.md](./USAGE.md)** - 实车使用和调试指南
- **[TROUBLESHOOTING.md](./TROUBLESHOOTING.md)** - 运行时故障排查
- **[README.md](./README.md)** - 项目总览

---

## 技术支持

如果遇到其他问题，请检查：

1. **ROS2环境**：确保 `echo $ROS_DISTRO` 输出 `humble`
2. **编译日志**：查看完整编译输出，定位错误行
3. **依赖完整性**：运行 `rosdep install -r --from-paths src --ignore-src --rosdistro humble -y`

**日志位置**：
- 编译日志：`build/sp_vision_ros2_adapter/`
- 运行日志：通过 `ros2 run` 或 `ros2 launch` 输出到终端
