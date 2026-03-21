# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is the pb2025_sentry_ws workspace for SMBU PolarBear Robotics Team's RoboMaster 2025 Sentry Robot. The workspace integrates serial communication, computer vision, navigation, decision-making (behavior tree), and simulation modules for an autonomous sentry robot competing in RoboMaster competitions.

### System Architecture

The system follows a modular ROS2 architecture with four main functional layers:

1. **Hardware Interface Layer**
   - `standard_robot_pp_ros2`: Serial communication with embedded systems (STM32), publishes referee system data and gimbal joint states
   - `hik_camera_ros2_driver`: HikRobot industrial camera driver for image acquisition

2. **Perception Layer**
   - **`sp_vision_25`** (NEW): High-performance vision system with native camera access
     - Integrated YOLO detection, EKF tracking, ballistic solver in single node
     - Publishes `/tracker/target` for decision layer
     - Publishes `/cmd_gimbal`, `/cmd_shoot` for gimbal control
     - Direct hardware access for <1ms latency (no ROS2 image topic overhead)
   - `pb2025_rm_vision`: Original armor plate detection and tracking (legacy/backup)
     - `armor_detector_opencv` or `armor_detector_openvino`: Real-time enemy armor detection
     - `armor_tracker`: Extended Kalman Filter based target state estimation
     - `projectile_motion`: Ballistic trajectory calculation with compensation

3. **Navigation Layer**
   - `pb2025_sentry_nav`: Autonomous navigation stack
     - `point_lio`: LiDAR-inertial odometry for localization
     - `small_gicp_relocalization`: Global localization using prior point cloud maps
     - `sensor_scan_generation`: Convert 3D point clouds to 2D laser scans
     - `pb_nav2_plugins`: Custom Nav2 plugins (controller/planner/behavior)
     - `terrain_analysis`: Traversability analysis for rough terrain

4. **Decision Layer**
   - `pb2025_sentry_behavior`: BehaviorTree.CPP based decision-making framework
     - Action nodes: `CalculateAttackPose`, `PubNav2Goal`, `SendNav2Goal`, `PubTwist`
     - Condition nodes: `IsAttacked`, `IsDetectEnemy`, `IsGameStatus`, `IsRfidDetected`, `IsStatusOK`

5. **Integration Layer**
   - `pb2025_sentry_bringup`: Main launch files and unified parameter configuration

### Key Data Flow

- Referee system data (robot HP, ammo, game status) → GlobalBlackboard → Behavior tree conditions
- Camera images → Armor detector → Armor tracker → Projectile motion → Gimbal commands
- LiDAR + IMU → Point-LIO → Odometry → Nav2 → Chassis velocity commands
- Behavior tree decisions → Navigation goals / Gimbal targets / Chassis velocities

## Build Commands

### Initial Setup

Install dependencies:
```bash
rosdep install -r --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y
```

Pull/update all submodules:
```bash
vcs pull ./src
```

### Build Workspace

Standard build (recommended with `--symlink-install` to avoid rebuilding when modifying launch/config files):
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 10
```

Single package build (useful for iterative development):
```bash
colcon build --symlink-install --packages-select <package_name> --cmake-args -DCMAKE_BUILD_TYPE=Release
```

Build with debug symbols:
```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
```

## Running the System

### Full System Launch

Launch all modules (serial, vision, navigation, behavior tree):
```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=<YOUR_WORLD_NAME> \
  use_rviz:=True \
  params_file:=<ABSOLUTE_PATH_TO_PARAMS>
```

**NEW: Use sp_vision_25 high-performance vision**:
```bash
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=<YOUR_WORLD_NAME> \
  use_sp_vision:=True \
  sp_vision_config:=sentry.yaml \
  use_rviz:=True \
  params_file:=<ABSOLUTE_PATH_TO_PARAMS>
```

**Available sp_vision configuration files:**
- `sentry.yaml` (default): Sentry robot configuration
- `standard3.yaml`: Standard robot (hero/infantry) configuration
- `standard4.yaml`: Alternative standard configuration
- `uav.yaml`: UAV configuration
- Custom path: `/absolute/path/to/custom.yaml`

**Note**: When `use_sp_vision:=True`, the hik_camera_ros2_driver will NOT start because sp_vision_25 directly accesses the camera hardware (exclusive access).

Default parameters file: `./src/pb2025_sentry_bringup/params/node_params.yaml`

Important launch arguments:
- `world`: Map/PCD file basename (without extension)
- `use_sp_vision`: Use sp_vision_25 (True) or original pb2025_rm_vision (False, default)
- `sp_vision_config`: Config file for sp_vision_25 (default: `sentry.yaml`)
- `use_sim_time`: Set to `True` when playing rosbags or in simulation
- `use_composition`: Use composable nodes for better performance (default: True)
- `use_robot_state_pub`: Publish TF from joint_state (needed for rosbag playback)
- `detector`: Choose armor detector (`opencv` or `openvino`, only for pb2025_rm_vision)
- `slam`: Enable SLAM mode (default: False)

### Individual Module Testing

Camera node:
```bash
ros2 launch hik_camera_ros2_driver hik_camera_launch.py params_file:=<PARAMS_FILE>
```

Serial communication:
```bash
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py use_rviz:=True params_file:=<PARAMS_FILE>
```

Vision system:
```bash
ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py \
  use_composition:=True \
  use_rviz:=True \
  params_file:=<PARAMS_FILE>
```

Navigation:
```bash
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
  world:=<YOUR_WORLD_NAME> \
  slam:=False
```

Behavior tree:
```bash
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py params_file:=<PARAMS_FILE>
```

## Development Workflow

### Testing with Rosbags

Record rosbag (automatically triggered by referee system):
- Set `standard_robot_pp_ros2.record_rosbag: True` in node_params.yaml
- Recording starts at 5s countdown, stops at match end

Play rosbag for testing:
```bash
ros2 bag play <ROSBAG_NAME>.bag --clock
```

When replaying rosbags, launch nodes with:
- `use_sim_time:=True`
- `use_robot_state_pub:=True` (to generate TF from recorded joint_state)
- `use_hik_camera:=False` (camera data is in bag)

### Tools

Teleop gimbal control:
```bash
ros2 run teleop_gimbal_keyboard teleop_gimbal_keyboard
```

Convert PCD to PGM map:
```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

Save navigation map:
```bash
ros2 run nav2_map_server map_saver_cli -f <YOUR_WORLD_NAME>
```

## Configuration

### Central Parameter File

All node parameters are centralized in: `src/pb2025_sentry_bringup/params/node_params.yaml`

Key parameter groups:
- `standard_robot_pp_ros2`: Serial port, baud rate, detector color, rosbag recording
- `hik_camera_ros2_driver`: Camera frame rate, exposure, gain, pixel format
- `armor_detector_*`: Detection thresholds, model paths (OpenCV/OpenVINO)
- `armor_tracker`: EKF parameters, tracking thresholds
- `projectile_motion`: Ballistic compensation, gimbal offsets, projectile speed

### Map and PCD Files

Maps and point clouds must share the same basename as the `world` parameter:
- Maps: `src/pb2025_sentry_bringup/map/<world>.yaml` and `<world>.pgm`
- Point clouds: `src/pb2025_sentry_bringup/pcd/<world>.pcd`

### sp_vision_25 Configuration

sp_vision_25 uses a standalone YAML configuration format (not ROS2 parameter format):
- **Location**: `src/sp_vision_25/configs/sentry.yaml`
- **Format**: Custom YAML with camera, YOLO, tracker, aimer, shooter configuration sections
- **Customization methods**:
  1. Directly modify config files in sp_vision_25 package
  2. Create custom config file and pass absolute path via `sp_vision_config` parameter
  3. Copy to bringup/configs/ directory for centralized management

**Key parameters:**
- `enemy_color`: Enemy color ("red" or "blue")
- `device`: Inference device ("CPU" or "GPU", OpenVINO)
- `yolo_name`: YOLO model version ("yolov5", "yolov8", or "yolo11")
- `exposure_ms`, `gain`: Camera exposure and gain
- `yaw_offset`, `pitch_offset`: Gimbal calibration offsets (degrees)
- `debug.enable_visualization`: Publish debug images to `/sentry_debug/image`

**Dual configuration system:**
- **ROS2 parameters** (`node_params.yaml`): Used for pb2025_rm_vision, navigation, behavior tree, serial communication
- **sp_vision YAML** (`sentry.yaml`): Only used for sp_vision_25
- **Reason**: sp_vision_25 is an independent high-performance system using its own config format for modularity and backward compatibility

## Special Dependencies

This workspace requires external dependencies beyond standard ROS2 packages:

- **OpenVINO 2023.3**: For armor detector inference (install from official docs)
- **small_gicp**: Point cloud registration library for relocalization
  ```bash
  sudo apt install -y libeigen3-dev libomp-dev
  git clone https://github.com/koide3/small_gicp.git
  cd small_gicp && mkdir build && cd build
  cmake .. -DCMAKE_BUILD_TYPE=Release && make -j
  sudo make install
  ```
- **Ignition Fortress**: For Gazebo simulation

## Known Issues and Workarounds

### BehaviorTree.ROS2 Action Node Halt Bug

There is a known bug where RosActionNode cannot be properly halted, causing the behavior tree to shutdown. As a workaround:
- Use `PubNav2Goal` (topic-based) instead of `SendNav2Goal` (action-based)
- Limitation: Cannot receive real-time action feedback/result
- Related: https://github.com/BehaviorTree/BehaviorTree.ROS2/issues/18

### Symlink Install for Config Files

Always use `--symlink-install` when building to avoid rebuilding after modifying:
- Launch files (.launch.py)
- Parameter files (.yaml)
- URDF/xacro files

Without symlink-install, you must rebuild after every config change.

## Git Workflow

This is a multi-repository workspace managed by vcstool. The main repo contains:
- Submodule references in `dependencies.repos`
- Central bringup package: `pb2025_sentry_bringup`

Each functional module (vision, navigation, behavior, etc.) is a separate git repository.

Update all submodules:
```bash
vcs pull ./src
```

Import dependencies after cloning:
```bash
vcs import --recursive . < dependencies.repos
```

---

## 📚 Detailed Documentation

**Complete Developer Manual**: [`README.md`](./README.md) - Main entry point

Individual chapters in `docs/`:
1. [`01_系统架构.md`](./docs/01_系统架构.md) - System architecture, data flow, tech stack
2. [`02_硬件接口层.md`](./docs/02_硬件接口层.md) - Serial communication, camera driver
3. [`03_感知层.md`](./docs/03_感知层.md) - Armor detection, tracking, ballistics (18KB, most detailed)
4. [`04_导航层.md`](./docs/04_导航层.md) - Point-LIO, localization, Nav2
5. [`05_决策层.md`](./docs/05_决策层.md) - Behavior tree framework, nodes
6. [`06_ROS话题详解.md`](./docs/06_ROS话题详解.md) - All topics and message types
7. [`07_参数配置.md`](./docs/07_参数配置.md) - Parameter tuning guide
8. [`08_运行与调试.md`](./docs/08_运行与调试.md) - Launch, debug, troubleshooting (13KB)

**When to use**: For implementation details, algorithms, parameter tuning, or troubleshooting.

---

## 🗂️ Key File Paths

### Configuration
- **Central config**: `src/pb2025_sentry_bringup/params/node_params.yaml` (ALL node parameters)
- **Main launch**: `src/pb2025_sentry_bringup/launch/bringup.launch.py`
- **Maps**: `src/pb2025_sentry_bringup/map/<world>.yaml` and `<world>.pgm`
- **PCD files**: `src/pb2025_sentry_bringup/pcd/<world>.pcd`
- **RViz config**: `src/pb2025_sentry_bringup/rviz/sentry_default_view.rviz`

### Source Code by Layer
- **Hardware**: `src/standard_robot_pp_ros2/`, `src/dependencies/hik_camera_ros2_driver/`
- **Vision**: `src/pb2025_rm_vision/armor_detector_opencv/`, `armor_detector_openvino/`, `armor_tracker/`, `projectile_motion/`
- **Navigation**: `src/pb2025_sentry_nav/point_lio/`, `small_gicp_relocalization/`, `terrain_analysis/`, `pb_nav2_plugins/`
- **Decision**: `src/pb2025_sentry_behavior/`
- **Interfaces**: `src/interfaces/pb_rm_interfaces/`, `src/interfaces/auto_aim_interfaces/`

### Launch Files
- Vision: `src/pb2025_rm_vision/pb2025_vision_bringup/launch/rm_vision_reality_launch.py`
- Navigation: `src/pb2025_sentry_nav/pb2025_nav_bringup/launch/rm_navigation_reality_launch.py`
- Behavior: `src/pb2025_sentry_behavior/launch/pb2025_sentry_behavior_launch.py`

---

## 📡 Core ROS Topics (16 Most Important)

### Hardware Layer
| Topic | Type | Hz | Publisher | Subscribers | Purpose |
|-------|------|----|-----------| ------------|---------|
| `/front_industrial_camera/image` | sensor_msgs/Image | 165 | camera | detector | Camera images |
| `/serial/gimbal_joint_state` | sensor_msgs/JointState | 100 | serial | tracker | Gimbal state |
| `/referee/robot_status` | pb_rm_interfaces/RobotStatus | 10 | serial | BT | Robot HP, ammo, heat |
| `/referee/game_status` | pb_rm_interfaces/GameStatus | 10 | serial | BT | Game phase, time |

### Perception Layer
| Topic | Type | Hz | Publisher | Subscribers | Purpose |
|-------|------|----|-----------| ------------|---------|
| `/detector/armors` | auto_aim_interfaces/Armors | ~100 | detector | tracker | Detected armors |
| `/tracker/target` | auto_aim_interfaces/Target | ~100 | tracker | projectile, BT | Tracking target (pos + vel) |
| `/cmd_gimbal` | pb_rm_interfaces/GimbalCmd | ~100 | projectile | serial | Gimbal commands |
| `/cmd_shoot` | example_interfaces/UInt8 | ~100 | projectile | serial | Shoot command |

### Navigation Layer
| Topic | Type | Hz | Publisher | Subscribers | Purpose |
|-------|------|----|-----------| ------------|---------|
| `/livox/lidar` | sensor_msgs/PointCloud2 | 20 | lidar | point_lio | Point cloud |
| `/livox/imu` | sensor_msgs/Imu | 200 | lidar | point_lio | IMU data |
| `/Odometry` | nav_msgs/Odometry | 20 | point_lio | nav2 | Robot odometry |
| `/terrain_map` | sensor_msgs/PointCloud2 | 5 | terrain | nav2 | Traversability map |
| `/cmd_vel` | geometry_msgs/Twist | 20 | nav2, BT | serial | Chassis velocity |

### Decision Layer
| Topic | Type | Hz | Publisher | Subscribers | Purpose |
|-------|------|----|-----------| ------------|---------|
| `/goal_pose` | geometry_msgs/PoseStamped | event | BT | nav2 | Navigation goal |
| `/global_costmap/costmap` | nav_msgs/OccupancyGrid | 1 | nav2 | BT | Global costmap |

**Message definitions**: See `src/interfaces/*/msg/` for custom types.

---

## 🤖 Core Nodes (16 Essential)

### Hardware Layer
1. **standard_robot_pp_ros2** (`StandardRobotPpRos2Node`)
   - Package: `standard_robot_pp_ros2`
   - Publishes: `/referee/*` (10 topics), `/serial/*` (4 topics)
   - Subscribes: `/cmd_vel`, `/cmd_gimbal_joint`, `/cmd_shoot`
   - Function: Serial communication with STM32, referee system data

2. **hik_camera_ros2_driver** (`hik_camera_ros2_driver`)
   - Package: `hik_camera_ros2_driver`
   - Publishes: `/{camera_name}/image`, `/camera_info`
   - Function: HikRobot industrial camera driver (165Hz)

### Perception Layer
3. **armor_detector_opencv** (`armor_detector_opencv`)
   - Package: `armor_detector_opencv`
   - Publishes: `/detector/armors`, `/detector/marker`
   - Subscribes: `/{camera_name}/image`, `/camera_info`
   - Function: Traditional CV armor detection

4. **armor_detector_openvino** (`armor_detector_openvino`)
   - Package: `armor_detector_openvino`
   - Same interface as opencv version
   - Function: Deep learning armor detection (OpenVINO)

5. **armor_tracker** (`armor_tracker`)
   - Package: `armor_tracker`
   - Publishes: `/tracker/target`, `/tracker/info`
   - Subscribes: `/detector/armors` (with tf2 filter)
   - Function: EKF-based target state estimation

6. **projectile_motion** (`projectile_motion_node`)
   - Package: `projectile_motion`
   - Publishes: `/cmd_gimbal`, `/cmd_shoot`
   - Subscribes: `/tracker/target` (with tf2 filter)
   - Function: Ballistic calculation and gimbal control

### Navigation Layer
7. **livox_ros_driver2** (`livox_lidar_publisher`)
   - Package: `livox_ros_driver2`
   - Publishes: `/livox/lidar`, `/livox/imu`
   - Function: Livox LiDAR driver (MID360)

8. **point_lio** (`pointlio_mapping`)
   - Package: `point_lio`
   - Publishes: `/Odometry`, `/cloud_registered`
   - Subscribes: `/livox/lidar`, `/livox/imu`
   - Function: LiDAR-inertial odometry

9. **small_gicp_relocalization** (`small_gicp_relocalization_node`)
   - Package: `small_gicp_relocalization`
   - Subscribes: `/registered_scan`, `/initialpose`
   - Publishes TF: `map` → `odom`
   - Function: Global localization via GICP

10. **terrain_analysis** (`terrainAnalysis`)
    - Package: `terrain_analysis`
    - Publishes: `/terrain_map`
    - Subscribes: `/registered_scan`
    - Function: Traversability analysis

11. **Nav2 Stack** (controller_server, planner_server, bt_navigator)
    - Packages: `nav2_*`
    - Custom plugins: `pb_omni_pid_pursuit_controller`, `nav2_theta_star_planner`
    - Function: Path planning and motion control

### Decision Layer
12. **pb2025_sentry_behavior_server** (`pb2025_sentry_behavior_server`)
    - Package: `pb2025_sentry_behavior`
    - Global Blackboard subscribes: `/referee/*`, `/tracker/target`, `/global_costmap/costmap`
    - Publishes: `/goal_pose`, `/cmd_vel` (via action nodes)
    - Function: BehaviorTree.CPP execution engine

**Behavior Tree Nodes** (loaded as plugins):
- Conditions: `IsDetectEnemy`, `IsAttacked`, `IsGameStatus`, `IsRfidDetected`, `IsStatusOK`
- Actions: `CalculateAttackPose`, `PubNav2Goal`, `PubTwist`, `PubGimbalAbsolute`, `PubGimbalVelocity`

---

## ⚡ Quick Commands Reference

### System Status
```bash
# Check all nodes
ros2 node list

# Check all topics
ros2 topic list

# Check topic frequency
ros2 topic hz /front_industrial_camera/image

# Check topic data
ros2 topic echo /tracker/target

# View TF tree
ros2 run tf2_tools view_frames
```

### Single Module Debug
```bash
# Serial (check /dev/ttyACM* first)
ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py params_file:=<path>

# Camera (check lsusb | grep HIK first)
ros2 launch hik_camera_ros2_driver hik_camera_launch.py params_file:=<path>

# Vision only
ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py detector:=opencv use_rviz:=True

# Navigation only
ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py world:=<map_name> use_rviz:=True

# Behavior tree only
ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py
```

### Parameter Operations
```bash
# View all parameters of a node
ros2 param list /armor_detector_opencv

# Get specific parameter
ros2 param get /armor_detector_opencv binary_thres

# Set parameter (runtime)
ros2 param set /armor_detector_opencv binary_thres 100

# Dump all parameters to file
ros2 param dump /armor_detector_opencv > backup.yaml
```

### Rosbag Debug
```bash
# Play rosbag with clock
ros2 bag play <bag_file>.db3 --clock

# Launch with rosbag-compatible settings
ros2 launch pb2025_sentry_bringup bringup.launch.py \
  world:=<map> \
  use_sim_time:=True \
  use_robot_state_pub:=True \
  use_hik_camera:=False
```

---

## 🔧 Critical Parameters Quick Reference

Located in: `src/pb2025_sentry_bringup/params/node_params.yaml`

### Most Frequently Tuned (Top 20)

**Hardware**
- `standard_robot_pp_ros2.device_name`: Serial port (default: `/dev/ttyACM0`)
- `hik_camera_ros2_driver.acquisition_frame_rate`: Camera FPS (default: `165.0`)
- `hik_camera_ros2_driver.exposure_time`: Exposure μs (default: `5000`)
- `hik_camera_ros2_driver.gain`: Camera gain dB (default: `12.0`)

**Vision Detection**
- `armor_detector_opencv.detect_color`: 0=red, 1=blue (default: `0`)
- `armor_detector_opencv.binary_thres`: Binarization threshold (default: `80`, tune: 60-120)
- `armor_detector_opencv.classifier_threshold`: Confidence threshold (default: `0.25`, tune: 0.2-0.6)
- `armor_detector_openvino.detector.confidence_threshold`: Same for OpenVINO (default: `0.25`)

**Vision Tracking**
- `armor_tracker.ekf.sigma2_q_xyz`: Position process noise (default: `0.05`, ↓=trust measurement)
- `armor_tracker.tracker.tracking_thres`: Frames to start tracking (default: `5`)
- `armor_tracker.tracker.lost_time_thres`: Time before losing track (default: `1.0` s)

**Ballistics**
- `projectile_motion.projectile.offset_pitch`: Pitch offset rad (default: `0.0`, calibrate!)
- `projectile_motion.projectile.offset_yaw`: Yaw offset rad (default: `0.0`, calibrate!)
- `projectile_motion.projectile.offset_time`: Prediction time s (default: `0.1`)
- `projectile_motion.projectile.initial_speed`: Projectile speed m/s (default: `21.5`)

**Navigation**
- `pointlio_mapping.mapping.filter_size_surf`: Point cloud filter (default: `0.2`, ↑=faster)
- `controller_server.FollowPath.max_linear_vel`: Max speed m/s (default: `2.0`)
- `controller_server.FollowPath.kp_rho`: Distance gain (default: `1.0`, ↓ if oscillating)

**Decision**
- `pb2025_sentry_behavior_server.tick_frequency`: BT tick Hz (default: `5`)

**Tuning scenarios**:
- Bright environment → increase `binary_thres` to 100-120
- Dark environment → decrease `binary_thres` to 60-80, increase `gain`
- Tracking jitter → decrease `ekf.sigma2_q_xyz` to 0.03
- Missing detections → decrease `classifier_threshold` to 0.2
- Path oscillation → decrease `kp_rho` and `kp_alpha`

---

## 🏗️ Code Organization

### Package Structure
```
src/
├── interfaces/                    # Message definitions
│   ├── pb_rm_interfaces/         # Referee system, gimbal control (9 msg types)
│   └── auto_aim_interfaces/      # Armor detection, tracking (8 msg types)
│
├── standard_robot_pp_ros2/       # Serial communication node
├── pb2025_rm_vision/             # Vision modules
│   ├── armor_detector_opencv/
│   ├── armor_detector_openvino/
│   ├── armor_tracker/
│   └── projectile_motion/
│
├── pb2025_sentry_nav/            # Navigation modules
│   ├── point_lio/
│   ├── small_gicp_relocalization/
│   ├── terrain_analysis/
│   ├── pb_nav2_plugins/          # Custom Nav2 plugins
│   └── pb2025_nav_bringup/
│
├── pb2025_sentry_behavior/       # Behavior tree nodes
│   ├── include/pb2025_sentry_behavior/
│   ├── src/                      # Condition/Action node implementations
│   └── trees/                    # XML behavior trees
│
├── pb2025_sentry_bringup/        # Integration
│   ├── launch/bringup.launch.py
│   ├── params/node_params.yaml   # **CENTRAL CONFIG**
│   ├── map/                      # .yaml + .pgm map files
│   └── pcd/                      # .pcd point cloud maps
│
└── dependencies/                 # Third-party packages
    ├── hik_camera_ros2_driver/
    ├── rmoss_core/
    ├── BehaviorTree.ROS2/
    └── ...
```

### Finding Code Locations
- **Add new message**: `src/interfaces/{package}/msg/NewMessage.msg`
- **Modify detector**: `src/pb2025_rm_vision/armor_detector_opencv/src/detector_node.cpp`
- **Add BT node**: `src/pb2025_sentry_behavior/src/action_nodes/` or `condition_nodes/`
- **Tune Nav2**: `src/pb2025_sentry_bringup/params/node_params.yaml` (search `controller_server` or `planner_server`)
- **Add custom Nav2 plugin**: `src/pb2025_sentry_nav/pb_nav2_plugins/`

---

## ⚠️ Development Best Practices

### Must-Follow Rules
1. **Always use `--symlink-install`** when building
   - Allows modifying .py, .yaml, .xml without rebuilding
   - `colcon build --symlink-install`

2. **BehaviorTree.ROS2 Bug Workaround**
   - DO NOT use `SendNav2Goal` (action-based) - has halt bug
   - USE `PubNav2Goal` (topic-based) instead
   - Trade-off: No real-time action feedback

3. **Testing with Rosbag**
   - Set `use_sim_time:=True` when playing bags
   - Set `use_robot_state_pub:=True` to publish TF from joint_state
   - Set `use_hik_camera:=False` (image is in bag)

4. **Map/PCD Naming Convention**
   - Files MUST share basename with `world` parameter
   - Example: `world:=arena` requires `arena.yaml`, `arena.pgm`, `arena.pcd`

5. **Coordinate Frames**
   - Vision tracking: `gimbal_pitch_odom` frame
   - Navigation: `map` → `odom` → `base_footprint` → `gimbal_yaw`
   - Check TF: `ros2 run tf2_ros tf2_echo map base_footprint`

### Common Pitfalls
- **Forgot to source**: Always `source install/setup.bash` after building
- **Serial port permission**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **OpenVINO not found**: Check `echo $INTEL_OPENVINO_DIR` is set
- **TF lookup failed**: Ensure `robot_state_publisher` is running
- **Detector sees nothing**: Enable `debug: true`, check `/detector/binary_img`

---

## 🔍 Troubleshooting Quick Guide

**Vision not detecting**: Check `binary_thres` (80±20), `classifier_threshold` (0.25±0.15), enable debug mode
**Tracking lost frequently**: Increase `lost_time_thres` to 1.5s, check `max_match_distance`
**Navigation stuck**: Check costmap visualization, increase `inflation_radius`, verify map loaded
**Behavior tree not ticking**: Verify referee system data publishing (`ros2 topic hz /referee/game_status`)
**Serial connection failed**: Check `ls -l /dev/ttyACM*`, verify baud rate 115200
**Camera not streaming**: Check `lsusb | grep HIK`, verify USB 3.0 connection

**For detailed troubleshooting**: See [`docs/08_运行与调试.md`](./docs/08_运行与调试.md) (13KB, comprehensive)

---

## 📖 When Working on Specific Tasks

- **Adding new vision algorithm**: Read `docs/03_感知层.md` (algorithms, data flow)
- **Tuning navigation**: Read `docs/04_导航层.md` + `docs/07_参数配置.md`
- **Creating behavior tree logic**: Read `docs/05_决策层.md` (node types, XML format)
- **Understanding message types**: Read `docs/06_ROS话题详解.md` (all 23 message types)
- **Debugging system issues**: Read `docs/08_运行与调试.md` (step-by-step guides)
- **Modifying launch files**: Check `src/pb2025_sentry_bringup/launch/` + existing launch args

**Quick search tip**: All docs have table of contents at the top for fast navigation.
