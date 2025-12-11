# TROUBLESHOOTING.md - sp_vision_ros2_adapter 故障排查指南

本文档汇总了在开发和实车测试过程中遇到的常见问题及其解决方案。

## 目录

1. [编译问题](#编译问题)
2. [启动问题](#启动问题)
3. [数据流问题](#数据流问题)
4. [参数配置问题](#参数配置问题)
5. [性能问题](#性能问题)
6. [已知Bug及解决方案](#已知bug及解决方案)

---

## 编译问题

### 问题1：找不到Eigen3库

**错误信息**：

```
CMake Error at CMakeLists.txt:XX (find_package):
  Could not find a package configuration file provided by "Eigen3"
```

**原因**：未安装Eigen3开发库。

**解决方案**：

```bash
# 安装Eigen3
sudo apt install libeigen3-dev

# 验证安装
pkg-config --modversion eigen3
# 应输出：3.4.0 或更高版本

# 重新编译
cd /home/happywosabi/pb2025_sentry_ws
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
```

---

### 问题2：找不到cv_bridge

**错误信息**：

```
CMake Error: Could not find a package configuration file provided by "cv_bridge"
```

**原因**：未安装ROS2 cv_bridge包。

**解决方案**：

```bash
# 安装cv_bridge
sudo apt install ros-humble-cv-bridge

# 重新编译
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
```

---

### 问题3：找不到pb_rm_interfaces或auto_aim_interfaces

**错误信息**：

```
CMake Error: Could not find a package configuration file provided by "pb_rm_interfaces"
```

**原因**：pb2025工作空间的自定义接口包尚未编译。

**解决方案**：

```bash
cd /home/happywosabi/pb2025_sentry_ws

# 先编译接口包
colcon build --symlink-install --packages-select \
  pb_rm_interfaces \
  auto_aim_interfaces

# 重新source环境
source install/setup.bash

# 再编译适配器
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
```

---

### 问题4：编译警告 - 未使用的变量

**警告信息**：

```
warning: unused variable 'current_bullet_speed' [-Wunused-variable]
```

**原因**：这是预期的警告，因为当前RobotStatus不包含bullet_speed字段，该变量仅用于占位。

**影响**：无实际影响，可以忽略。

**消除警告**（可选）：

在 `src/adapter_node.cpp` 中，已经使用 `(void)current_bullet_speed;` 避免此警告。

---

## 启动问题

### 问题5：节点启动失败 - 参数文件未找到

**错误信息**：

```
[ERROR] [sp_vision_ros2_adapter]: Failed to load parameters from file
```

**原因1**：未使用 `--symlink-install` 编译。

**解决方案1**：

```bash
# 重新编译，使用symlink
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
source install/setup.bash
```

**原因2**：参数文件路径不正确。

**解决方案2**：

```bash
# 方法1：使用默认launch文件（推荐）
ros2 launch sp_vision_ros2_adapter adapter_launch.py

# 方法2：手动指定绝对路径
ros2 launch sp_vision_ros2_adapter adapter_launch.py \
  params_file:=/home/happywosabi/pb2025_sentry_ws/src/sp_vision_ros2_adapter/config/adapter_params.yaml
```

---

### 问题6：节点启动后立即退出

**现象**：节点启动后没有错误信息，但立即退出。

**原因**：可能是依赖的话题不存在，或者ROS2环境未正确source。

**排查步骤**：

1. **检查ROS2环境**：

```bash
echo $ROS_DISTRO
# 应输出：humble

echo $AMENT_PREFIX_PATH
# 应包含：/opt/ros/humble 和 /home/happywosabi/pb2025_sentry_ws/install
```

2. **检查依赖的节点是否运行**：

```bash
# 检查串口节点是否运行
ros2 node list | grep standard_robot_pp_ros2
```

3. **查看详细日志**：

```bash
# 使用DEBUG日志级别启动
ros2 launch sp_vision_ros2_adapter adapter_launch.py \
  --ros-args --log-level sp_vision_ros2_adapter:=DEBUG
```

---

### 问题7：多个节点实例同时运行

**警告信息**：

```
Be aware that are nodes in the graph that share an exact name,
this can have unintended side effects.
```

**原因**：之前启动的节点进程未正确关闭，导致多个实例同时运行。

**解决方案**：

```bash
# 方法1：强制终止所有adapter_node进程
pkill -f "adapter_node"

# 方法2：查找并终止特定PID
ps aux | grep adapter_node
kill <PID>

# 验证清理成功
ros2 node list | grep sp_vision_ros2_adapter
# 应只输出一个实例或无输出
```

---

## 数据流问题

### 问题8：订阅的话题无数据（⭐ 最常见）

**现象**：节点启动正常，但 `ros2 topic echo` 看不到数据。

**排查步骤**：

#### 步骤1：检查话题是否存在

```bash
ros2 topic list | grep gimbal_joint_state
```

如果不存在，说明上游节点（串口节点）未启动或未发布数据。

#### 步骤2：检查话题类型是否匹配

```bash
ros2 topic type /serial/gimbal_joint_state
# 应输出：sensor_msgs/msg/JointState
```

#### 步骤3：检查发布频率

```bash
ros2 topic hz /serial/gimbal_joint_state
# 应输出：~100 Hz
```

如果频率为0，说明上游节点有问题。

#### 步骤4：检查节点连接

```bash
ros2 node info /sp_vision_ros2_adapter | grep "Subscriptions:"
# 应包含：/serial/gimbal_joint_state
```

如果未列出，说明订阅器创建失败。

---

### 问题9：sp_target_callback回调函数不执行（⭐ 关键Bug）

**现象**：
- 发布消息到 `/sp_vision/target`
- 节点日志显示已订阅该话题
- 但是 `/tracker/target` 没有任何输出
- `ros2 topic echo /tracker/target` 超时无数据

**错误定位过程**（历史记录）：
1. 检查CMakeLists.txt - std_msgs依赖已添加 ✅
2. 检查package.xml - std_msgs依赖已添加 ✅
3. 检查话题连接 - ros2 node info 显示正常 ✅
4. 检查节点是否多实例 - 清理后仍然无数据 ❌
5. **发现根本原因**：adapter_node.hpp 缺少 `#include <std_msgs/msg/string.hpp>` ❌

**根本原因**：

在 `include/sp_vision_ros2_adapter/adapter_node.hpp` 文件中，缺少以下关键include：

```cpp
#include <std_msgs/msg/string.hpp>  // ← 这行缺失导致回调函数无法正确注册
```

**⚠️ 这是一个非常隐蔽的Bug**：
- 编译可以通过（因为CMakeLists.txt有依赖）
- 节点可以启动（因为其他部分没问题）
- 订阅器可以创建（ROS2内部机制）
- **但是回调函数无法正确执行**（类型定义不完整）

**解决方案**：

编辑 `include/sp_vision_ros2_adapter/adapter_node.hpp`，在第11行添加缺失的include：

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>  // ← 添加这行
#include <pb_rm_interfaces/msg/gimbal_cmd.hpp>
#include <pb_rm_interfaces/msg/robot_status.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <example_interfaces/msg/u_int8.hpp>
```

**重新编译**：

```bash
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
source install/setup.bash
```

**验证修复**：

```bash
# 终端1：启动适配器（确保use_sp_target_topic: true）
ros2 launch sp_vision_ros2_adapter adapter_launch.py

# 终端2：发布测试数据
ros2 topic pub /sp_vision/target std_msgs/msg/String "{data: '1.5,2.3,0.8,3'}" --rate 10

# 终端3：验证输出
ros2 topic hz /tracker/target
# 应输出：10.000 Hz ✅
```

---

### 问题10：图像话题频率过低

**现象**：`ros2 topic hz /front_industrial_camera/image` 显示频率远低于165 Hz（例如只有50 Hz）。

**原因可能**：
1. 相机配置不正确
2. USB带宽不足
3. CPU负载过高

**排查步骤**：

#### 步骤1：检查相机配置

```bash
# 查看相机参数
ros2 param list /hik_camera_ros2_driver | grep frame_rate

# 获取当前帧率设置
ros2 param get /hik_camera_ros2_driver acquisition_frame_rate
```

#### 步骤2：检查USB连接

```bash
# 确认相机连接到USB 3.0接口
lsusb -t | grep -A 5 HIK
# 应看到"5000M"（USB 3.0）而不是"480M"（USB 2.0）
```

#### 步骤3：检查CPU负载

```bash
top
# 查看CPU使用率，如果接近100%，需要优化或升级硬件
```

---

## 参数配置问题

### 问题11：修改参数文件后不生效

**现象**：修改 `config/adapter_params.yaml` 后重启节点，但参数值没有变化。

**原因**：未使用 `--symlink-install` 编译，或者缓存问题。

**解决方案**：

#### 方法1：确保使用symlink安装

```bash
# 重新编译
colcon build --symlink-install --packages-select sp_vision_ros2_adapter

# 重启节点
ros2 launch sp_vision_ros2_adapter adapter_launch.py
```

#### 方法2：清理构建缓存

```bash
# 删除构建和安装目录
rm -rf build/sp_vision_ros2_adapter install/sp_vision_ros2_adapter

# 重新编译
colcon build --symlink-install --packages-select sp_vision_ros2_adapter
source install/setup.bash
```

#### 方法3：运行时动态修改（临时）

```bash
# 注意：此方法修改在重启后失效
ros2 param set /sp_vision_ros2_adapter bullet_speed 22.0
```

---

### 问题12：R_gimbal_to_imu参数格式错误

**错误信息**：

```
[ERROR] [sp_vision_ros2_adapter]: R_gimbal_to_imu parameter must have 9 elements, got X
[WARN] [sp_vision_ros2_adapter]: Using identity matrix as fallback
```

**原因**：参数文件中的 `R_gimbal_to_imu` 数组元素数量不是9。

**检查方法**：

编辑 `config/adapter_params.yaml`，确保格式正确：

```yaml
# 正确格式（3行×3列=9个元素）
R_gimbal_to_imu: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
```

**常见错误**：

```yaml
# 错误1：缺少元素
R_gimbal_to_imu: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0]  # 只有6个元素 ❌

# 错误2：额外的逗号
R_gimbal_to_imu: [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0,]  # 最后一个逗号导致解析错误 ❌

# 错误3：换行不正确
R_gimbal_to_imu:
  [1.0, 0.0, 0.0,
   0.0, 1.0, 0.0,
   0.0, 0.0, 1.0]  # YAML格式错误 ❌
```

**修复后验证**：

```bash
# 重启节点，检查日志
ros2 launch sp_vision_ros2_adapter adapter_launch.py

# 应看到：
# [INFO] [sp_vision_ros2_adapter]: Loaded R_gimbal_to_imu calibration matrix
```

---

### 问题13：use_sp_target_topic设置无效

**现象**：设置 `use_sp_target_topic: true` 后，节点日志仍显示 `false`。

**原因**：配置文件加载失败，或者配置文件路径错误。

**排查步骤**：

#### 步骤1：检查参数是否加载

```bash
# 查看节点当前参数值
ros2 param get /sp_vision_ros2_adapter use_sp_target_topic
# 应输出：true
```

如果输出false，说明参数未正确加载。

#### 步骤2：检查launch文件参数路径

编辑 `launch/adapter_launch.py`，确认参数文件路径：

```python
parameters=[os.path.join(
    get_package_share_directory('sp_vision_ros2_adapter'),
    'config', 'adapter_params.yaml'
)]
```

#### 步骤3：手动指定参数文件

```bash
ros2 launch sp_vision_ros2_adapter adapter_launch.py \
  params_file:=/home/happywosabi/pb2025_sentry_ws/src/sp_vision_ros2_adapter/config/adapter_params.yaml
```

---

## 性能问题

### 问题14：节点CPU占用过高（>20%）

**现象**：`top` 命令显示 adapter_node 进程CPU占用超过20%。

**原因分析**：

1. **图像转换频率过高**：如果相机以165 Hz发布图像，但当前未使用图像数据，可能造成浪费。
2. **日志输出过多**：频繁的 `RCLCPP_DEBUG` 或 `RCLCPP_INFO` 输出。
3. **回调函数执行时间过长**。

**解决方案**：

#### 方案1：降低图像订阅频率（如果暂不使用）

在 `src/adapter_node.cpp` 中，暂时不订阅图像话题：

```cpp
// 暂时注释掉图像订阅
// image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(...);
```

#### 方案2：减少日志输出

将 `RCLCPP_INFO` 改为 `RCLCPP_DEBUG`，或使用 `RCLCPP_INFO_THROTTLE`：

```cpp
// 每5秒输出一次（5000 ms）
RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                     "Processing gimbal joint state");
```

#### 方案3：优化回调函数

检查回调函数中是否有耗时操作（例如大量计算、磁盘I/O）。

---

### 问题15：话题发布延迟过高

**现象**：从接收输入话题到发布输出话题，延迟超过10ms。

**排查步骤**：

#### 步骤1：测量延迟

```bash
# 使用ros2 topic delay测量（需要安装topic_tools）
ros2 topic delay /serial/gimbal_joint_state /tracker/target
```

#### 步骤2：检查回调函数执行时间

在回调函数中添加时间测量：

```cpp
void AdapterNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  auto start = std::chrono::steady_clock::now();

  // 现有逻辑...
  latest_imu_quaternion_ = joint_angles_to_quaternion(yaw, pitch, R_gimbal_to_imu_);

  auto end = std::chrono::steady_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();

  RCLCPP_DEBUG(this->get_logger(), "Callback execution time: %ld μs", duration);
}
```

#### 步骤3：优化建议

- 确保使用 `-DCMAKE_BUILD_TYPE=Release` 编译（已启用）
- 避免在回调函数中进行复杂计算
- 使用Eigen库的优化版本

---

## 已知Bug及解决方案

### Bug #1：std_msgs/msg/string.hpp缺失导致回调无效 ⭐

**严重程度**：高

**发现版本**：Phase 3 Day 10

**详细描述**：参见上文"[问题9：sp_target_callback回调函数不执行](#问题9sp_target_callback回调函数不执行-关键bug)"。

**状态**：✅ 已修复（已添加include）

---

### Bug #2：配置文件修改后未生效（symlink未使用）

**严重程度**：中

**现象**：修改 `config/adapter_params.yaml` 后重启节点，参数值不变。

**原因**：编译时未使用 `--symlink-install`，导致配置文件被复制到install目录，后续修改源文件无效。

**解决方案**：始终使用 `--symlink-install` 编译。

**状态**：✅ 已在文档中强调

---

## 调试工具和技巧

### 使用rqt_console查看日志

```bash
# 启动rqt_console
rqt_console

# 设置过滤器：只显示sp_vision_ros2_adapter的日志
# 在GUI中设置：Node包含 "sp_vision_ros2_adapter"
```

### 使用rqt_graph可视化节点连接

```bash
# 启动rqt_graph
rqt_graph

# 可视化节点和话题的连接关系
```

### 录制和回放rosbag

```bash
# 录制关键话题
ros2 bag record -o debug_data \
  /serial/gimbal_joint_state \
  /sp_vision/target \
  /tracker/target

# 回放
ros2 bag play debug_data --loop
```

### 使用GDB调试

```bash
# 编译Debug版本
colcon build --packages-select sp_vision_ros2_adapter \
  --cmake-args -DCMAKE_BUILD_TYPE=Debug

# 使用GDB运行
gdb --args ros2 run sp_vision_ros2_adapter adapter_node
```

---

## 获取技术支持

如果以上方法仍无法解决问题，请提供以下信息：

### 1. 系统信息

```bash
# 操作系统版本
cat /etc/os-release

# ROS2版本
echo $ROS_DISTRO

# 包版本
ros2 pkg list | grep sp_vision_ros2_adapter
```

### 2. 完整日志

```bash
# 启动节点并保存日志到文件
ros2 launch sp_vision_ros2_adapter adapter_launch.py \
  --ros-args --log-level DEBUG 2>&1 | tee adapter_debug.log
```

### 3. 话题和节点状态

```bash
# 节点列表
ros2 node list > nodes.txt

# 话题列表
ros2 topic list > topics.txt

# 节点详细信息
ros2 node info /sp_vision_ros2_adapter > node_info.txt
```

### 4. 参数配置

```bash
# 导出当前参数
ros2 param dump /sp_vision_ros2_adapter > current_params.yaml
```

将以上信息打包提供，便于快速定位问题。

---

## 相关文档

- **[README.md](./README.md)** - 项目总览
- **[INSTALL.md](./INSTALL.md)** - 安装配置指南
- **[USAGE.md](./USAGE.md)** - 实际使用指南

---

## 更新记录

| 日期 | 版本 | 更新内容 |
|------|------|---------|
| 2025-12-10 | 1.0 | 初始版本，包含Phase 3开发过程中遇到的所有问题 |
|  |  | 重点记录：std_msgs/msg/string.hpp缺失bug（关键） |

如果遇到新问题，请记录并更新此文档。
