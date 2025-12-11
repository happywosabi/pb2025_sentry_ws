# sp_vision_25 依赖检查报告

**日期**: 2025-12-10
**检查人**: Claude Code
**项目路径**: `/home/happywosabi/pb2025_sentry_ws/testopenvino/sp_vision_25`

---

## 1. 编译状态

### 1.1 CMake配置

✅ **成功配置**

- 构建类型: Release
- CMake版本: 系统默认
- 构建目录: `/home/happywosabi/pb2025_sentry_ws/testopenvino/sp_vision_25/build`

```
-- --------------------CMAKE_BUILD_TYPE: Release--------------------
-- Configuring done
-- Generating done
-- Build files have been written to: /home/happywosabi/pb2025_sentry_ws/testopenvino/sp_vision_25/build
```

### 1.2 编译结果

✅ **编译成功** (100%)

**成功编译的可执行文件**：
- ✅ `standard` - 步兵标准版本
- ✅ `standard_mpc` - 步兵MPC版本 ⭐ (推荐测试)
- ✅ `mt_standard` - 多线程版本
- ✅ `uav` - 无人机版本
- ✅ `auto_aim_test`, `auto_buff_test` - 测试程序
- ✅ 标定工具: `calibrate_camera`, `calibrate_handeye`

**未编译的目标**：
- ❌ `sentry` - 哨兵版本 (需要ROS2环境 + sp_msgs包)
- ❌ `sentry_multithread`
- ❌ `sentry_debug`
- ❌ `sentry_bp`

**原因**: 这些目标依赖于ROS2环境，根据CMakeLists.txt第112-131行：
```cpp
if(ament_cmake_FOUND AND rclcpp_FOUND AND std_msgs_FOUND
   AND rosidl_typesupport_cpp_FOUND AND sp_msgs_FOUND)
    add_executable(sentry src/sentry.cpp)
    // ...
else()
    message(STATUS "ROS2 environment not found, skipping ROS2-related code.")
endif()
```

---

## 2. 依赖检查

### 2.1 核心依赖

| 依赖项 | 状态 | 版本 | 说明 |
|-------|------|------|------|
| **Eigen3** | ✅ 已安装 | 3.4.0 | 线性代数库 |
| **Ceres** | ✅ 已安装 | 2.0.0 | 优化库（用于标定） |
| **OpenCV** | ✅ 已安装 | 系统版本 | 计算机视觉库 |
| **yaml-cpp** | ✅ 已安装 | 系统版本 | YAML配置文件解析 |
| **fmt** | ✅ 已安装 | 系统版本 | 格式化输出 |
| **glog** | ✅ 已安装 | 系统版本 | 日志库 |
| **gflags** | ✅ 已安装 | 系统版本 | 命令行参数解析 |

### 2.2 推理依赖

| 依赖项 | 状态 | 版本 | 说明 |
|-------|------|------|------|
| **OpenVINO** | ❌ 未安装 | 要求2024.6 | YOLO模型推理加速 |

**检查方法**：
```bash
$ echo $INTEL_OPENVINO_DIR
(空输出)
```

**影响**:
- 无法使用YOLO检测器（yolov5/yolov8/yolo11）
- 配置文件中的模型路径无效：
  - `assets/yolov5.xml`
  - `assets/yolov8.xml`
  - `assets/yolo11.xml`

**替代方案**:
- ✅ 可以使用传统CV检测器 (`use_traditional: true`)
- 配置文件`configs/sentry.yaml`第13行已启用：`use_traditional: true`

### 2.3 ROS2依赖

| 依赖项 | 状态 | 说明 |
|-------|------|------|
| **ament_cmake** | ✅ 已找到 | ROS2构建系统 |
| **rclcpp** | ✅ 已找到 | ROS2 C++客户端库 |
| **std_msgs** | ✅ 已找到 | ROS2标准消息 |
| **rosidl_typesupport_cpp** | ✅ 已找到 | ROS2类型支持 |
| **sp_msgs** | ❌ 未找到 | sp_vision_25自定义消息 |

**CMake警告**:
```
CMake Warning at io/CMakeLists.txt:49 (message):
  ROS2 not found, skipping ROS2 specific code.
```

**影响**:
- ROS2相关功能（topic发布）不可用
- 对于本次独立运行测试 **不影响**，因为我们使用CAN通信

---

## 3. 硬件接口检查

### 3.1 相机配置

**配置文件**: `configs/sentry.yaml`第36-40行

```yaml
camera_name: "hikrobot"
exposure_ms: 0.8
gain: 16.9
vid_pid: "2bdf:0001"
```

**实际硬件检查** (待验证):
- [ ] 海康工业相机 (HikRobot) 是否连接
- [ ] USB 3.0接口是否正常
- [ ] 驱动是否安装 (MVS SDK)

**检查命令**:
```bash
lsusb | grep -i hik
# 或
lsusb | grep 2bdf
```

### 3.2 CAN通信配置

**配置文件**: `configs/sentry.yaml`第54-58行

```yaml
quaternion_canid: 0x01      # IMU四元数
bullet_speed_canid: 0x110   # 子弹速度
send_canid: 0xff            # 云台控制指令
can_interface: "can0"
```

**实际硬件检查** (待验证):
- [ ] CAN接口 `can0` 是否存在
- [ ] C板是否连接
- [ ] CAN波特率是否匹配

**检查命令**:
```bash
ip link show can0
# 或
candump can0
```

---

## 4. 关键配置参数

### 4.1 检测器配置

**配置文件**: `configs/sentry.yaml`第1-13行

```yaml
enemy_color: "blue"           # 敌方颜色
yolo_name: yolov5             # YOLO模型选择（需OpenVINO）
use_traditional: true         # ✅ 启用传统CV检测
device: GPU                   # OpenVINO设备（GPU加速）
min_confidence: 0.8           # 检测置信度阈值
```

**当前状态**:
- ✅ 已启用传统CV检测器，可以在无OpenVINO情况下运行
- ⚠️ YOLO模型需要OpenVINO才能使用

### 4.2 标定参数

**配置文件**: `configs/sentry.yaml`第85-96行

```yaml
# 重投影误差: 0.1833px
camera_matrix: [2414.93..., ...]    # 相机内参
distort_coeffs: [-0.0209..., ...]   # 畸变系数

# 相机同理想情况的偏角: yaw-1.11 pitch0.01 roll-0.06 degree
R_camera2gimbal: [0.0192..., ...]   # 手眼标定旋转矩阵
t_camera2gimbal: [0.1308..., ...]   # 手眼标定平移向量
```

**验证**:
- ✅ 标定参数已存在
- ✅ 重投影误差0.18px（良好）
- ⚠️ 标定是否适用于当前硬件需要验证

---

## 5. 测试准备状态

### 5.1 可直接运行的版本

| 程序 | 路径 | 功能 | 推荐度 |
|------|------|------|--------|
| **standard_mpc** | `build/standard_mpc` | MPC轨迹规划版本 | ⭐⭐⭐⭐⭐ |
| **standard** | `build/standard` | 传统决策版本 | ⭐⭐⭐⭐ |
| **mt_standard** | `build/mt_standard` | 多线程版本 | ⭐⭐⭐ |

### 5.2 运行命令

```bash
cd /home/happywosabi/pb2025_sentry_ws/testopenvino/sp_vision_25/build

# 推荐：MPC版本（文档分析中的高性能版本）
./standard_mpc ../configs/sentry.yaml

# 或传统版本
./standard ../configs/sentry.yaml
```

### 5.3 预期行为

**正常启动标志**:
- 读取配置文件成功
- 初始化相机（如果连接）
- 初始化CAN接口（如果存在）
- 开始检测和跟踪循环

**可能的错误**:
1. **相机未连接**: 报错"Camera open failed"
   - 解决：检查相机连接，或使用视频文件测试
2. **CAN接口未找到**: 报错"CAN open failed"
   - 解决：检查can0接口，或修改配置使用串口
3. **OpenVINO未安装** (如果use_traditional=false):
   - 解决：设置`use_traditional: true`

---

## 6. 下一步行动

### 6.1 立即可执行（无需额外安装）

✅ **使用传统CV方法测试**:
1. 确认`configs/sentry.yaml`中`use_traditional: true`
2. 检查硬件连接（相机、C板）
3. 运行`./standard_mpc ../configs/sentry.yaml`
4. 观察检测和跟踪效果

**预计时间**: 1-2小时

### 6.2 完整功能测试（需要安装依赖）

⚠️ **安装OpenVINO 2024.6** (如果要用YOLO):
1. 下载OpenVINO 2024.6
2. 安装并配置环境变量
3. 重新编译项目
4. 修改配置：`use_traditional: false`, `yolo_name: yolov8`

**预计时间**: 2-3小时

### 6.3 ROS2集成准备（Phase 2开始）

⚠️ **创建sp_msgs包**:
- sp_vision_25的ROS2消息定义
- 用于sentry程序的topic通信
- 本次独立测试不需要

---

## 7. 风险评估

| 风险项 | 严重程度 | 缓解措施 |
|--------|---------|----------|
| OpenVINO未安装 | 中 | ✅ 使用传统CV方法 |
| 硬件未连接 | 高 | 使用rosbag或视频文件测试 |
| 标定参数不准确 | 中 | 重新运行标定工具 |
| sp_msgs缺失 | 低 | 仅影响ROS2功能，Phase 2处理 |

---

## 8. 结论

### ✅ 可以进行独立运行测试

**满足条件**:
1. ✅ 项目编译成功
2. ✅ 核心依赖全部满足
3. ✅ 传统CV检测器可用
4. ✅ 配置文件完整
5. ⚠️ 硬件连接待验证

**推荐方案**:
- **立即执行**: 使用`standard_mpc`进行传统CV方法测试
- **后续优化**: 如果传统CV效果不佳，再安装OpenVINO使用YOLO

**关键决策点** (Phase 1 Day 1结束时):
- ✅ 如果传统CV命中率>30% → 继续Phase 2
- ⚠️ 如果传统CV效果差 → 安装OpenVINO再测试
- ❌ 如果硬件无法连接 → 使用rosbag录像测试

---

**下一步**: 配置硬件参数并进行独立测试
