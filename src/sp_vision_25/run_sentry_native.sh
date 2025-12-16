#!/bin/bash
# sp_vision_25 原生相机模式启动脚本
# 使用 HikRobot SDK 直接访问相机硬件，避免 ROS2 话题传输延迟

set -e  # 遇到错误立即退出

cd "$(dirname "$0")"
source ~/pb2025_sentry_ws/install/setup.bash

echo "=========================================="
echo "  sentry_ros2 原生相机模式启动检查"
echo "=========================================="
echo ""

# ============================================================================
# 检查 1: 相机驱动冲突检测
# ============================================================================
if pgrep -f "hik_camera_ros2_driver" > /dev/null; then
  echo "⚠️  警告: 检测到 hik_camera_ros2_driver 正在运行！"
  echo ""
  echo "   原生相机模式会与 ROS2 驱动节点产生设备冲突（HikRobot SDK 不支持多进程访问）"
  echo ""
  echo "   请选择："
  echo "   [C] Continue  - 继续启动（可能导致相机设备繁忙错误）"
  echo "   [A] Abort     - 取消启动（推荐，先停止驱动节点）"
  echo ""
  read -r -p "   输入选择: " choice
  case "$choice" in
    [Cc])
      echo "   → 继续启动..."
      echo ""
      ;;
    *)
      echo "   → 已取消启动"
      echo ""
      echo "   提示: 停止相机驱动节点的命令:"
      echo "   ros2 node list | grep hik_camera"
      echo "   ros2 lifecycle set /hik_camera shutdown"
      exit 1
      ;;
  esac
fi

# ============================================================================
# 检查 2: 相机设备连接检测
# ============================================================================
if ! lsusb | grep -q "2bdf:0001"; then
  echo "⚠️  警告: 未检测到 HikRobot 相机 (USB VID:PID 2bdf:0001)"
  echo ""
  echo "   可能原因:"
  echo "   1. 相机未连接"
  echo "   2. USB 线缆故障"
  echo "   3. configs/sentry.yaml 中的 vid_pid 参数不匹配"
  echo ""
  echo "   当前检测到的 USB 设备:"
  lsusb | grep -i "hik\|camera" || echo "   （无相关设备）"
  echo ""
  read -r -p "   按 Enter 继续，或 Ctrl+C 取消..."
fi

# ============================================================================
# 检查 3: 配置文件存在性检测
# ============================================================================
if [ ! -f "configs/sentry.yaml" ]; then
  echo "❌ 错误: 配置文件不存在: configs/sentry.yaml"
  echo ""
  echo "   请确保在 sp_vision_25 根目录运行此脚本"
  exit 1
fi

# ============================================================================
# 检查 4: 可执行文件存在性检测
# ============================================================================
if [ ! -f "build/sentry_ros2" ]; then
  echo "❌ 错误: 可执行文件不存在: build/sentry_ros2"
  echo ""
  echo "   请先编译程序:"
  echo "   cd build && cmake .. && make sentry_ros2 -j\$(nproc)"
  exit 1
fi

# ============================================================================
# 启动 sentry_ros2
# ============================================================================
echo "=========================================="
echo "  启动 sentry_ros2 (原生相机直接访问)"
echo "=========================================="
echo ""
echo "配置文件: configs/sentry.yaml"
echo "相机模式: HikRobot SDK 直接硬件访问"
echo "ROS2 功能: IMU 订阅 + 控制指令发布 + 调试可视化"
echo ""
echo "预期日志:"
echo "  [HikRobot] HikRobot's daemon thread started."
echo "  [HikRobot] HikRobot's capture thread started."
echo "  [Main] Starting main loop..."
echo ""

# 启动程序
./build/sentry_ros2 configs/sentry.yaml

# 捕获退出状态
EXIT_CODE=$?

echo ""
echo "=========================================="
if [ $EXIT_CODE -eq 0 ]; then
  echo "  sentry_ros2 正常退出"
else
  echo "  sentry_ros2 异常退出 (退出码: $EXIT_CODE)"
  echo ""
  echo "  常见问题排查:"
  echo "  1. 设备权限不足 → 运行: sudo chmod 666 /dev/bus/usb/*/*"
  echo "  2. 相机设备冲突 → 停止 hik_camera_ros2_driver 节点"
  echo "  3. 配置文件错误 → 检查 configs/sentry.yaml 参数"
fi
echo "=========================================="

exit $EXIT_CODE
