# pb2025_rm_vision

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0)
[![Build and Test](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_rm_vision/actions/workflows/ci.yml/badge.svg?branch=main)](https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_rm_vision/actions/workflows/ci.yml)
[![pre-commit](https://img.shields.io/badge/pre--commit-enabled-brightgreen?logo=pre-commit)](https://github.com/pre-commit/pre-commit)

![PolarBear Logo](https://raw.githubusercontent.com/SMBU-PolarBear-Robotics-Team/.github/main/.docs/image/polarbear_logo_text.png)

## 1. 项目介绍

深圳北理莫斯科大学 北极熊战队视觉算法仓库，用于 RoboMaster 机器人装甲板检测/整车状态估计/弹道计算。

- [armor_detector_opencv](./armor_detector_opencv/): 基于 @chenjunnn 开源项目 [rm_vision](https://github.com/chenjunnn/rm_vision) OpenCV 装甲板检测功能包
- [armor_tracker](./armor_tracker/): 基于 @FaterYU 开源项目 [rm_auto_aim](https://github.com/FaterYU/rm_auto_aim/tree/main/armor_tracker) 的整车状态估计功能包
- [projectile_motion](./projectile_motion/): 基于 [rmoss_core](https://github.com/robomaster-oss/rmoss_core) 的弹道计算功能包

## 项目依赖

- [hik_camera_ros2_driver](https://github.com/SMBU-PolarBear-Robotics-Team/hik_camera_ros2_driver.git): HikRobot 工业相机 ROS2 驱动
- [rmoss_core](https://github.com/SMBU-PolarBear-Robotics-Team/rmoss_core.git)：提供弹道解算工具包
- [auto_aim_interfaces](https://github.com/SMBU-PolarBear-Robotics-Team/auto_aim_interfaces.git)：自瞄自定义 ROS 消息接口

## 2. Quick Start

### 2.1 Setup Environment

- Ubuntu 22.04
- ROS: [Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

### 2.2 Create Workspace

```bash
sudo apt install git-lfs
sudo pip install vcstool2
```

```bash
mkdir -p ~/ros_ws
cd ~/ros_ws
```

```bash
git clone --recursive https://github.com/SMBU-PolarBear-Robotics-Team/pb2025_rm_vision.git src/pb2025_rm_vision
```

```bash
vcs import --recursive src < src/pb2025_rm_vision/dependencies.repos
```

### 2.3 Build

```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!NOTE]
> 推荐使用 --symlink-install 选项来构建你的工作空间，因为 pb2025_rm_vision 广泛使用了 launch.py 文件和 yaml 文件。这个构建参数会为那些非编译的源文件使用符号链接，这意味着当你调整参数文件时，不需要反复重建，只需要重新启动即可。

### 2.4 Running

```bash
ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py
```

### 2.5 Launch Arguments

| 参数 | 描述 | 类型 | 默认值 |
|-|-|-|-|
| `namespace` | 顶级命名空间 | string | "" |
| `use_sim_time` | 如果为 True，则使用仿真（Gazebo）时钟 | bool | False |
| `detector` | 选用的识别器，当前可选参数： [`opencv`](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_opencv), [`openvino`](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_openvino), [`tensorrt`](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_tensorrt)。| string | "opencv" |
| `params_file` | 用于所有启动节点的 ROS2 参数文件的完整路径 | string | [vision_params.yaml](./pb2025_vision_bringup/params/reality/vision_params.yaml) |
| `use_composition` | 是否使用 Composable Node 形式启动 | bool | True |
| `use_respawn` | 如果节点崩溃，是否重新启动。本参数仅 `use_composition:=False` 时有效 | bool | False |
| `rviz_config_file` | 要使用的 RViz 配置文件的完整路径 | string | [vision_default_view.rviz](./pb2025_vision_bringup/rviz/vision_default_view.rviz) |
| `use_rviz` | 是否启动 RViz | bool | True |
| `log_level` | 日志级别 | string | "info" |

> [!NOTE]
> `detector` 参数可选值：[`opencv`](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_opencv), [`openvino`](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_openvino), [`tensorrt`](https://github.com/SMBU-PolarBear-Robotics-Team/armor_detector_tensorrt)，需要额外 git clone 链接的功能包，并安装相关依赖。

## 致谢

- [robomaster-oss](https://github.com/robomaster-oss)
- [chenjunnn/rm_vision](https://github.com/chenjunnn/rm_vision.git)
- [Ericsii/rm_vision](https://github.com/Ericsii/rm_vision.git)
- [FaterYU/rm_auto_aim](https://github.com/FaterYU/rm_auto_aim.git)
