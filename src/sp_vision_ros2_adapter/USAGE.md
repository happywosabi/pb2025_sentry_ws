# USAGE.md - sp_vision_ros2_adapter 实车使用指南

本文档提供在实车上运行 `sp_vision_ros2_adapter` 的完整操作指南，包括启动流程、数据验证、参数调优和性能监控。

## 目录

1. [系统启动流程](#系统启动流程)
2. [数据流验证](#数据流验证)
3. [参数调优指南](#参数调优指南)
4. [性能监控](#性能监控)
5. [与sp_vision_25集成](#与sp_vision_25集成)
6. [实车调试技巧](#实车调试技巧)

---

## 系统启动流程

### 推荐启动顺序

在实车上运行完整系统时，推荐按以下顺序启动各个节点：

#### 终端1：启动串口通信节点（standard_robot_pp_ros2）

```bash
# 确保串口设备已连接
ls -l /dev/ttyACM*
# 应输出：/dev/ttyACM0 或 /dev/ttyACM1

# 启动串口节点
cd /home/happywosabi/pb2025_sentry_ws
source install/setup.bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
  params_file:=/home/happywosabi/pb2025_sentry_ws/src/pb2025_sentry_bringup/params/node_params.yaml
```

**预期输出**：

```
[INFO] [standard_robot_pp_ros2]: Serial port opened successfully
[INFO] [standard_robot_pp_ros2]: Robot status receiving at 10 Hz
[INFO] [standard_robot_pp_ros2]: Gimbal joint state receiving at 100 Hz
```

#### 终端2：启动sp_vision_ros2_adapter适配器节点

```bash
# 新终端
cd /home/happywosabi/pb2025_sentry_ws
source install/setup.bash
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

#### 终端3：验证数据流（可选）

```bash
# 新终端
cd /home/happywosabi/pb2025_sentry_ws
source install/setup.bash

# 检查节点是否都在运行
ros2 node list
# 应输出：
#   /standard_robot_pp_ros2
#   /sp_vision_ros2_adapter

# 检查话题列表
ros2 topic list | grep -E "serial|gimbal|tracker"
```

---

## 数据流验证

### 1. 检查输入话题（pb2025 → adapter）

#### 验证云台关节状态（关键！）

```bash
# 查看频率（应为100 Hz）
ros2 topic hz /serial/gimbal_joint_state

# 查看实时数据
ros2 topic echo /serial/gimbal_joint_state --once
```

**预期输出**：

```yaml
header:
  stamp:
    sec: 1702234567
    nanosec: 123456789
  frame_id: "gimbal_yaw"
name: [yaw, pitch]
position: [1.751, -0.021]  # 单位：rad
velocity: [0.0, 0.0]
effort: []
```

**关键检查点**：
- ✅ 频率稳定在100 Hz
- ✅ `position` 数组有两个元素：[yaw, pitch]
- ✅ 数值在合理范围内：yaw ∈ [-π, π]，pitch ∈ [-0.5, 0.5]

#### 验证裁判系统机器人状态

```bash
# 查看频率（应为10 Hz）
ros2 topic hz /referee/robot_status

# 查看数据
ros2 topic echo /referee/robot_status --once
```

**预期输出**：

```yaml
robot_id: 7  # 红方哨兵
robot_hp: 600
max_hp: 600
shooter_id1_17mm_cooling_rate: 100
shooter_id1_17mm_cooling_limit: 500
# ... 其他字段
```

#### 验证相机图像（如果已连接相机）

```bash
# 查看频率（应为165 Hz）
ros2 topic hz /front_industrial_camera/image

# 查看图像信息
ros2 topic echo /front_industrial_camera/image --once | head -20
```

**预期输出**：

```yaml
header:
  stamp: ...
  frame_id: "camera_optical_frame"
height: 1024
width: 1280
encoding: "bgr8"
is_bigendian: 0
step: 3840
data: [...]  # 图像数据
```

### 2. 检查输出话题（adapter → pb2025）

当前阶段（Phase 3），由于sp_vision_25尚未与ROS2集成，适配器的输出话题可能没有实际数据发布。

但是，可以使用 **可选的测试功能** 验证转换逻辑：

#### 使用测试话题验证转换（可选）

**步骤1：启用测试功能**

编辑 `config/adapter_params.yaml`：

```yaml
use_sp_target_topic: true  # 启用测试功能
```

重启适配器节点：

```bash
# 按 Ctrl+C 停止适配器，然后重新启动
ros2 launch sp_vision_ros2_adapter adapter_launch.py
```

**步骤2：发布测试目标字符串**

在另一个终端：

```bash
# 发布模拟sp_vision输出（格式："x,y,z,id"）
ros2 topic pub /sp_vision/target std_msgs/msg/String "{data: '1.5,2.3,0.8,3'}" --rate 10
```

**步骤3：验证转换后的Target消息**

在另一个终端：

```bash
# 查看转换后的Target消息
ros2 topic echo /tracker/target --once
```

**预期输出**：

```yaml
header:
  stamp: ...
  frame_id: "gimbal_pitch_odom"
tracking: true
id: "2"  # 注意：3 → 2 转换（1-based to 0-based）
armors_num: 4
position:
  x: 1.5
  y: 2.3
  z: 0.8
velocity:
  x: 0.0
  y: 0.0
  z: 0.0
yaw: 0.0
v_yaw: 0.0
radius_1: 0.26
radius_2: 0.26
dz: 0.0
```

**验证通过标准**：
- ✅ 位置正确：(1.5, 2.3, 0.8)
- ✅ ID转换正确：3 → "2" (1-based to 0-based)
- ✅ tracking = true
- ✅ frame_id = "gimbal_pitch_odom"

**测试完成后，记得禁用测试功能**：

```yaml
use_sp_target_topic: false  # 实车运行时应设为false
```

---

## 参数调优指南

### 1. 子弹速度（bullet_speed）调优

**位置**：`config/adapter_params.yaml` → `bullet_speed`

**默认值**：21.5 m/s（哨兵/步兵国赛标准）

**调优方法**：

1. **初步测试**：使用默认值21.5 m/s，观察实际弹道
2. **观察偏差**：
   - 如果弹道**偏高**（打在装甲板上方）→ **减小**此值
   - 如果弹道**偏低**（打在装甲板下方）→ **增大**此值
3. **微调步长**：每次调整±0.5 m/s
4. **迭代测试**：重复测试直到弹道准确

**示例调优过程**：

| 测试轮次 | bullet_speed (m/s) | 观察结果 | 操作 |
|---------|-------------------|---------|------|
| 1 | 21.5 | 弹道偏高3cm | 减小到21.0 |
| 2 | 21.0 | 弹道偏高1cm | 减小到20.5 |
| 3 | 20.5 | 弹道准确 | ✅ 完成 |

**修改后重启节点**：

```bash
# 修改 config/adapter_params.yaml 后
# 按 Ctrl+C 停止适配器节点
# 重新启动
ros2 launch sp_vision_ros2_adapter adapter_launch.py
```

### 2. 手眼标定矩阵（R_gimbal_to_imu）配置

**位置**：`config/adapter_params.yaml` → `R_gimbal_to_imu`

**重要性**：⚠️ **极其重要**，错误的矩阵会导致IMU数据传递错误，影响跟踪精度！

**配置步骤**：

参考 [INSTALL.md 第4节](./INSTALL.md#3-配置手眼标定矩阵重要) 的详细步骤。

**快速检查**：

```bash
# 查看sp_vision_25的标定文件
cat ~/testopenvino/sp_vision_25/configs/sentry.yaml | grep -A 10 "R_gimbal2imubody"
```

**验证矩阵已加载**：

启动适配器节点时，应看到以下日志：

```
[INFO] [sp_vision_ros2_adapter]: Loaded R_gimbal_to_imu calibration matrix
```

如果看到错误或警告：

```
[ERROR] [sp_vision_ros2_adapter]: R_gimbal_to_imu parameter must have 9 elements, got X
[WARN] [sp_vision_ros2_adapter]: Using identity matrix as fallback
```

说明矩阵配置有误，请检查格式。

---

## 性能监控

### 1. 话题频率监控

实时监控各个话题的发布频率，确保数据流稳定：

```bash
# 监控云台关节状态（应为100 Hz）
ros2 topic hz /serial/gimbal_joint_state

# 监控相机图像（应为165 Hz）
ros2 topic hz /front_industrial_camera/image

# 监控裁判系统状态（应为10 Hz）
ros2 topic hz /referee/robot_status
```

**正常值参考**：

| 话题 | 预期频率 | 可接受范围 |
|-----|---------|-----------|
| `/serial/gimbal_joint_state` | 100 Hz | 95-105 Hz |
| `/front_industrial_camera/image` | 165 Hz | 160-170 Hz |
| `/referee/robot_status` | 10 Hz | 9-11 Hz |

**如果频率异常**：
- 低于预期50%以上 → 检查串口/相机连接
- 频率不稳定（波动大） → 检查CPU负载，可能需要优化

### 2. 节点资源占用

监控适配器节点的CPU和内存占用：

```bash
# 查看所有ROS2节点的进程ID
ps aux | grep ros2

# 找到adapter_node的PID，然后监控
top -p <ADAPTER_PID>
```

**正常资源占用**：
- **CPU**: < 5%（单核）
- **内存**: ~50 MB

**如果CPU占用过高（>20%）**：
- 检查图像转换是否有问题（图像尺寸过大？）
- 检查是否有警告/错误日志频繁输出

### 3. 延迟测试

测试从接收图像到发布云台指令的端到端延迟：

```bash
# 使用 ros2 topic delay 命令（需要安装 topic_tools）
ros2 topic delay /front_industrial_camera/image /cmd_gimbal
```

**预期延迟**：
- 图像转换延迟：< 1 ms
- IMU四元数转换：< 0.1 ms
- 总延迟（包括ROS2通信）：< 5 ms

---

## 与sp_vision_25集成

### 当前状态（Phase 3）

⚠️ **重要说明**：当前阶段，sp_vision_25 **尚未**与ROS2深度集成。

- ✅ **已完成**：ROS2适配器节点（转换层）
- ⏳ **待完成**：sp_vision_25修改以接收ROS2数据和发布ROS2结果

### 未来集成方案（Phase 4）

参考文档：`docs/10_视觉系统替换方案.md`

**方案选择**：

1. **方案A（推荐）**：修改sp_vision_25源码，订阅ROS2话题
   - 优点：延迟最低，集成最优雅
   - 缺点：需要修改sp_vision_25源码

2. **方案B（备选）**：使用共享内存/管道传递数据
   - 优点：无需修改sp_vision_25主逻辑
   - 缺点：稍高延迟，需要额外的IPC机制

**当前使用的临时方案**：
- sp_vision_25输出结果到文件：`/tmp/sp_vision_output.txt`
- 适配器可以读取此文件并转换为ROS2消息（需要添加文件监听逻辑）

### 临时文件输出方案（可选）

如果sp_vision_25当前输出到文件，可以添加文件监听功能：

**sp_vision_25输出格式（示例）**：

```
1.5,2.3,0.8,3
```

**适配器读取逻辑（未实现，可扩展）**：

在 `adapter_node.cpp` 中添加定时器：

```cpp
// 每100ms读取一次文件
timer_ = this->create_wall_timer(
  std::chrono::milliseconds(100),
  std::bind(&AdapterNode::read_sp_vision_output, this));
```

---

## 实车调试技巧

### 1. 分步测试策略

**阶段1：验证输入数据流**
- 仅启动串口节点，不启动适配器
- 确认云台关节状态、裁判系统数据正常接收

**阶段2：验证适配器启动**
- 启动适配器节点，检查日志无错误
- 确认参数加载正确（bullet_speed、R_gimbal_to_imu）

**阶段3：验证转换逻辑（使用测试话题）**
- 启用 `use_sp_target_topic: true`
- 手动发布测试数据，验证转换正确

**阶段4：集成sp_vision_25（Phase 4）**
- 启动sp_vision_25
- 验证完整数据流

### 2. 日志级别调整

如果需要查看详细调试信息：

```bash
# 启动时设置日志级别为DEBUG
ros2 launch sp_vision_ros2_adapter adapter_launch.py \
  --ros-args --log-level sp_vision_ros2_adapter:=DEBUG
```

### 3. 使用rqt工具可视化

```bash
# 安装rqt（如果尚未安装）
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins

# 启动rqt
rqt
```

**推荐插件**：
- **Topic Monitor**：实时查看话题频率
- **Node Graph**：可视化节点和话题连接
- **Image View**：查看相机图像（验证图像质量）
- **Plot**：绘制实时数据曲线（云台角度变化）

### 4. 使用rosbag录制数据

录制实车测试数据，用于离线分析：

```bash
# 录制关键话题（5分钟）
ros2 bag record -o test_data \
  /serial/gimbal_joint_state \
  /referee/robot_status \
  /front_industrial_camera/image \
  /tracker/target \
  --duration 300
```

**回放测试**：

```bash
# 回放录制的数据
ros2 bag play test_data

# 在另一个终端启动适配器节点
ros2 launch sp_vision_ros2_adapter adapter_launch.py
```

### 5. 常见调试命令速查

```bash
# 查看节点列表
ros2 node list

# 查看节点详细信息
ros2 node info /sp_vision_ros2_adapter

# 查看话题列表
ros2 topic list

# 查看话题类型
ros2 topic type /serial/gimbal_joint_state

# 查看话题实时数据
ros2 topic echo /serial/gimbal_joint_state

# 查看话题发布频率
ros2 topic hz /serial/gimbal_joint_state

# 查看话题带宽
ros2 topic bw /front_industrial_camera/image

# 查看参数列表
ros2 param list /sp_vision_ros2_adapter

# 查看参数值
ros2 param get /sp_vision_ros2_adapter bullet_speed

# 运行时修改参数（注意：重启后失效）
ros2 param set /sp_vision_ros2_adapter bullet_speed 22.0
```

---

## 实车测试检查清单

在实车上运行完整系统前，请确认：

### 硬件检查
- [ ] 串口设备已连接（`ls /dev/ttyACM*`）
- [ ] 相机已连接（`lsusb | grep HIK`）
- [ ] 电源供电正常
- [ ] 所有线缆连接牢固

### 软件检查
- [ ] ROS2 Humble已安装并source
- [ ] sp_vision_ros2_adapter已编译成功
- [ ] 参数文件已配置（bullet_speed、R_gimbal_to_imu）
- [ ] 依赖包完整（pb_rm_interfaces、auto_aim_interfaces）

### 功能检查
- [ ] 串口节点启动正常，数据接收稳定
- [ ] 适配器节点启动无错误，参数加载正确
- [ ] 输入话题频率正常（100 Hz gimbal_joint_state, 165 Hz image）
- [ ] （可选）使用测试话题验证转换逻辑

### 安全检查
- [ ] 云台限位已设置
- [ ] 机器人处于安全测试环境
- [ ] 急停按钮可用
- [ ] 有专人负责监控

---

## 下一步

运行过程中遇到问题，请参考：

- **[TROUBLESHOOTING.md](./TROUBLESHOOTING.md)** - 常见问题排查指南
- **[INSTALL.md](./INSTALL.md)** - 安装配置问题
- **[README.md](./README.md)** - 项目总览

**Phase 4 集成工作**（待用户完成）：

1. 修改sp_vision_25以接收ROS2图像和IMU数据
2. 修改sp_vision_25以通过ROS2发布目标位置和控制指令
3. 进行完整的硬件闭环测试
4. 性能优化和延迟分析

---

## 技术支持

如果遇到其他问题，请提供以下信息以便排查：

1. **系统信息**：`cat /etc/os-release`
2. **ROS版本**：`echo $ROS_DISTRO`
3. **节点日志**：完整的终端输出
4. **话题信息**：`ros2 topic list` 和 `ros2 node list` 的输出
5. **参数配置**：`config/adapter_params.yaml` 的内容

**调试技巧**：使用 `--ros-args --log-level DEBUG` 查看详细日志。
