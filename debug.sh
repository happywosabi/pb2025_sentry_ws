#!/bin/bash
# =============================================================================
# PolarBear Sentry Robot Debug Script
# SMBU PolarBear Robotics Team - RoboMaster 2025
# =============================================================================

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
BOLD='\033[1m'
NC='\033[0m' # No Color

# 工作空间路径
WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PARAMS_FILE="${WS_DIR}/src/pb2025_sentry_bringup/params/node_params.yaml"

# Source ROS2 环境
source_ros2() {
    if [ -f "${WS_DIR}/install/setup.bash" ]; then
        source "${WS_DIR}/install/setup.bash"
    else
        echo -e "${RED}[错误] 未找到 install/setup.bash，请先编译工作空间${NC}"
        echo -e "${YELLOW}  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release${NC}"
        exit 1
    fi
}

# 打印分隔线
print_separator() {
    echo -e "${BLUE}=================================================${NC}"
}

# 打印标题
print_header() {
    clear
    print_separator
    echo -e "${BOLD}${CYAN}  PolarBear Sentry Robot 调试工具${NC}"
    echo -e "${CYAN}  SMBU PolarBear Robotics - RoboMaster 2025${NC}"
    print_separator
    echo ""
}

# 选择地图
select_world() {
    echo -e "${YELLOW}请选择地图:${NC}"
    echo "  1) rmul_2025  (RMUL 2025)"
    echo "  2) rmul_2026  (RMUL 2026)"
    echo "  3) rmuc_2025  (RMUC 2025)"
    echo "  4) rmul_2024  (RMUL 2024)"
    echo "  5) rmuc_2024  (RMUC 2024)"
    echo "  6) 自定义输入"
    echo ""
    read -rp "请选择 [1-6] (默认: 1): " world_choice
    case "${world_choice}" in
        2) WORLD="rmul_2026" ;;
        3) WORLD="rmuc_2025" ;;
        4) WORLD="rmul_2024" ;;
        5) WORLD="rmuc_2024" ;;
        6)
            read -rp "请输入地图名称: " WORLD
            if [ -z "$WORLD" ]; then
                echo -e "${RED}地图名称不能为空${NC}"
                exit 1
            fi
            ;;
        *) WORLD="rmul_2025" ;;
    esac
    echo -e "${GREEN}已选择地图: ${WORLD}${NC}"
    echo ""
}

# =============================================================================
# 1) 全局调试 - 启动 bringup + RViz
# =============================================================================
debug_full_system() {
    echo -e "${GREEN}[全局调试] 启动完整系统 + RViz${NC}"
    select_world
    source_ros2

    echo -e "${CYAN}启动命令:${NC}"
    echo "  ros2 launch pb2025_sentry_bringup bringup.launch.py \\"
    echo "    world:=${WORLD} use_rviz:=True params_file:=${PARAMS_FILE}"
    echo ""
    read -rp "按 Enter 启动，Ctrl+C 取消..."

    ros2 launch pb2025_sentry_bringup bringup.launch.py \
        world:="${WORLD}" \
        use_rviz:=True \
        params_file:="${PARAMS_FILE}"
}

# =============================================================================
# 1b) 全局调试 (sp_vision) - 使用高性能视觉
# =============================================================================
debug_full_system_sp_vision() {
    echo -e "${GREEN}[全局调试 - sp_vision] 启动完整系统 + sp_vision_25 + RViz${NC}"
    select_world

    echo -e "${YELLOW}请选择 sp_vision 配置:${NC}"
    echo "  1) sentry.yaml    (哨兵，默认)"
    echo "  2) standard3.yaml (步兵/英雄)"
    echo "  3) standard4.yaml (备用步兵)"
    echo "  4) uav.yaml       (无人机)"
    echo "  5) 自定义路径"
    read -rp "请选择 [1-5] (默认: 1): " sp_choice
    case "${sp_choice}" in
        2) SP_CONFIG="standard3.yaml" ;;
        3) SP_CONFIG="standard4.yaml" ;;
        4) SP_CONFIG="uav.yaml" ;;
        5)
            read -rp "请输入配置文件绝对路径: " SP_CONFIG
            ;;
        *) SP_CONFIG="sentry.yaml" ;;
    esac

    source_ros2

    echo -e "${CYAN}启动命令:${NC}"
    echo "  ros2 launch pb2025_sentry_bringup bringup.launch.py \\"
    echo "    world:=${WORLD} use_sp_vision:=True sp_vision_config:=${SP_CONFIG} \\"
    echo "    use_rviz:=True params_file:=${PARAMS_FILE}"
    echo ""
    read -rp "按 Enter 启动，Ctrl+C 取消..."

    ros2 launch pb2025_sentry_bringup bringup.launch.py \
        world:="${WORLD}" \
        use_sp_vision:=True \
        sp_vision_config:="${SP_CONFIG}" \
        use_rviz:=True \
        params_file:="${PARAMS_FILE}"
}

# =============================================================================
# 2) 视觉调试 - 串口 + 视觉 + rqt 查看画面
# =============================================================================
debug_vision() {
    echo -e "${GREEN}[视觉调试] 启动串口 + 视觉 + rqt${NC}"
    source_ros2

    echo -e "${YELLOW}请选择检测器:${NC}"
    echo "  1) opencv   (传统CV，默认)"
    echo "  2) openvino (深度学习)"
    read -rp "请选择 [1-2] (默认: 1): " det_choice
    case "${det_choice}" in
        2) DETECTOR="openvino" ;;
        *) DETECTOR="opencv" ;;
    esac

    echo -e "${CYAN}将在3个终端中启动:${NC}"
    echo "  终端1: 串口节点 (standard_robot_pp_ros2)"
    echo "  终端2: 视觉节点 (detector=${DETECTOR})"
    echo "  终端3: rqt_image_view (查看画面)"
    echo ""
    read -rp "按 Enter 启动，Ctrl+C 取消..."

    # 终端1: 串口
    gnome-terminal --title="[串口] standard_robot_pp_ros2" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== 串口节点启动 ==='
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
            params_file:='${PARAMS_FILE}' \
            use_rviz:=False
        exec bash
    " &

    sleep 2

    # 终端2: 视觉
    gnome-terminal --title="[视觉] rm_vision (${DETECTOR})" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== 视觉节点启动 (${DETECTOR}) ==='
        ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py \
            detector:='${DETECTOR}' \
            use_hik_camera:=True \
            params_file:='${PARAMS_FILE}' \
            use_rviz:=False
        exec bash
    " &

    sleep 3

    # 终端3: rqt
    gnome-terminal --title="[RQT] Image View" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== rqt_image_view 启动 ==='
        echo '请在 rqt 中选择话题: /detector/result_img 或 /front_industrial_camera/image'
        ros2 run rqt_image_view rqt_image_view
        exec bash
    " &

    echo -e "${GREEN}所有终端已启动！${NC}"
    echo -e "${YELLOW}提示: 在 rqt 中选择话题查看画面${NC}"
    echo -e "  - /detector/result_img  (检测结果)"
    echo -e "  - /front_industrial_camera/image (原始画面)"
}

# =============================================================================
# 3) 导航调试 - 串口 + 导航 + RViz
# =============================================================================
debug_navigation() {
    echo -e "${GREEN}[导航调试] 启动串口 + 导航 + RViz${NC}"
    select_world
    source_ros2

    echo -e "${YELLOW}是否启用 SLAM 模式?${NC}"
    echo "  1) 否 - 使用已有地图定位 (默认)"
    echo "  2) 是 - SLAM 建图模式"
    read -rp "请选择 [1-2] (默认: 1): " slam_choice
    case "${slam_choice}" in
        2) SLAM="True" ;;
        *) SLAM="False" ;;
    esac

    echo -e "${CYAN}将在2个终端中启动:${NC}"
    echo "  终端1: 串口节点"
    echo "  终端2: 导航 + RViz (world=${WORLD}, slam=${SLAM})"
    echo ""
    read -rp "按 Enter 启动，Ctrl+C 取消..."

    # 终端1: 串口
    gnome-terminal --title="[串口] standard_robot_pp_ros2" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== 串口节点启动 ==='
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
            params_file:='${PARAMS_FILE}' \
            use_rviz:=False
        exec bash
    " &

    sleep 2

    # 终端2: 导航
    gnome-terminal --title="[导航] Navigation (${WORLD})" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== 导航节点启动 (world=${WORLD}, slam=${SLAM}) ==='
        ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
            world:='${WORLD}' \
            slam:='${SLAM}' \
            use_rviz:=True
        exec bash
    " &

    echo -e "${GREEN}所有终端已启动！${NC}"
}

# =============================================================================
# 4) 决策调试 - 串口 + 行为树
# =============================================================================
debug_behavior() {
    echo -e "${GREEN}[决策调试] 启动串口 + 行为树${NC}"
    source_ros2

    echo -e "${CYAN}将在2个终端中启动:${NC}"
    echo "  终端1: 串口节点"
    echo "  终端2: 行为树节点"
    echo ""
    read -rp "按 Enter 启动，Ctrl+C 取消..."

    # 终端1: 串口
    gnome-terminal --title="[串口] standard_robot_pp_ros2" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== 串口节点启动 ==='
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
            params_file:='${PARAMS_FILE}' \
            use_rviz:=False
        exec bash
    " &

    sleep 2

    # 终端2: 行为树
    gnome-terminal --title="[决策] Behavior Tree" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== 行为树节点启动 ==='
        ros2 launch pb2025_sentry_behavior pb2025_sentry_behavior_launch.py \
            params_file:='${PARAMS_FILE}'
        exec bash
    " &

    echo -e "${GREEN}所有终端已启动！${NC}"
}

# =============================================================================
# 5) Rosbag 回放调试 - 回放录像 + 视觉/导航
# =============================================================================
debug_rosbag() {
    echo -e "${GREEN}[Rosbag 回放] 回放调试${NC}"

    # 查找 rosbag 文件
    echo -e "${YELLOW}请输入 rosbag 路径 (支持 .db3 或目录):${NC}"
    echo -e "  常见位置: ${WS_DIR}/rosbags/ 或 ${WS_DIR}/rosbagnew/"
    echo ""

    # 列出可用 rosbag
    if [ -d "${WS_DIR}/rosbags" ] && [ "$(ls -A "${WS_DIR}/rosbags" 2>/dev/null)" ]; then
        echo -e "${CYAN}rosbags/ 目录中的文件:${NC}"
        ls -1 "${WS_DIR}/rosbags/" | head -10
        echo ""
    fi
    if [ -d "${WS_DIR}/rosbagnew" ] && [ "$(ls -A "${WS_DIR}/rosbagnew" 2>/dev/null)" ]; then
        echo -e "${CYAN}rosbagnew/ 目录中的文件:${NC}"
        ls -1 "${WS_DIR}/rosbagnew/" | head -10
        echo ""
    fi

    read -rp "请输入 rosbag 路径: " BAG_PATH
    if [ -z "${BAG_PATH}" ]; then
        echo -e "${RED}路径不能为空${NC}"
        return 1
    fi

    echo -e "${YELLOW}请选择回放模式:${NC}"
    echo "  1) 仅回放 rosbag (默认)"
    echo "  2) 回放 + 视觉节点"
    echo "  3) 回放 + 导航节点"
    echo "  4) 回放 + 完整系统 (bringup)"
    read -rp "请选择 [1-4] (默认: 1): " bag_mode

    source_ros2

    case "${bag_mode}" in
        2)
            echo -e "${CYAN}启动: rosbag 回放 + 视觉${NC}"
            gnome-terminal --title="[Rosbag] 回放" -- bash -c "
                source '${WS_DIR}/install/setup.bash'
                echo '=== Rosbag 回放 ==='
                ros2 bag play '${BAG_PATH}' --clock
                exec bash
            " &
            sleep 2
            gnome-terminal --title="[视觉] 回放模式" -- bash -c "
                source '${WS_DIR}/install/setup.bash'
                ros2 launch pb2025_vision_bringup rm_vision_reality_launch.py \
                    use_sim_time:=True \
                    use_hik_camera:=False \
                    use_robot_state_pub:=True \
                    params_file:='${PARAMS_FILE}' \
                    use_rviz:=True
                exec bash
            " &
            ;;
        3)
            select_world
            gnome-terminal --title="[Rosbag] 回放" -- bash -c "
                source '${WS_DIR}/install/setup.bash'
                ros2 bag play '${BAG_PATH}' --clock
                exec bash
            " &
            sleep 2
            gnome-terminal --title="[导航] 回放模式" -- bash -c "
                source '${WS_DIR}/install/setup.bash'
                ros2 launch pb2025_nav_bringup rm_navigation_reality_launch.py \
                    world:='${WORLD}' \
                    use_sim_time:=True \
                    use_robot_state_pub:=True \
                    use_rviz:=True
                exec bash
            " &
            ;;
        4)
            select_world
            gnome-terminal --title="[Rosbag] 回放" -- bash -c "
                source '${WS_DIR}/install/setup.bash'
                ros2 bag play '${BAG_PATH}' --clock
                exec bash
            " &
            sleep 2
            gnome-terminal --title="[Bringup] 回放模式" -- bash -c "
                source '${WS_DIR}/install/setup.bash'
                ros2 launch pb2025_sentry_bringup bringup.launch.py \
                    world:='${WORLD}' \
                    use_sim_time:=True \
                    use_robot_state_pub:=True \
                    use_hik_camera:=False \
                    use_rviz:=True \
                    params_file:='${PARAMS_FILE}'
                exec bash
            " &
            ;;
        *)
            echo -e "${CYAN}仅回放 rosbag${NC}"
            ros2 bag play "${BAG_PATH}" --clock
            return
            ;;
    esac

    echo -e "${GREEN}所有终端已启动！${NC}"
}

# =============================================================================
# 6) 话题监控 - 查看关键话题状态
# =============================================================================
debug_topic_monitor() {
    echo -e "${GREEN}[话题监控] 查看关键话题频率和数据${NC}"
    source_ros2

    echo -e "${YELLOW}请选择监控模式:${NC}"
    echo "  1) 查看所有活跃话题"
    echo "  2) 监控关键话题频率"
    echo "  3) 查看特定话题数据"
    echo "  4) 查看 TF 树"
    echo "  5) 查看所有节点"
    read -rp "请选择 [1-5]: " topic_choice

    case "${topic_choice}" in
        1)
            echo -e "${CYAN}活跃话题列表:${NC}"
            ros2 topic list
            ;;
        2)
            echo -e "${CYAN}将在多个终端中监控关键话题频率...${NC}"
            TOPICS=(
                "/front_industrial_camera/image"
                "/detector/armors"
                "/tracker/target"
                "/cmd_gimbal"
                "/Odometry"
                "/cmd_vel"
                "/referee/game_status"
            )
            for topic in "${TOPICS[@]}"; do
                gnome-terminal --title="[Hz] ${topic}" --geometry=60x5 -- bash -c "
                    source '${WS_DIR}/install/setup.bash'
                    echo '=== 监控 ${topic} ==='
                    ros2 topic hz '${topic}'
                    exec bash
                " &
            done
            echo -e "${GREEN}已启动 ${#TOPICS[@]} 个频率监控窗口${NC}"
            ;;
        3)
            echo -e "${YELLOW}常用话题:${NC}"
            echo "  1) /tracker/target      (追踪目标)"
            echo "  2) /detector/armors     (检测装甲板)"
            echo "  3) /cmd_gimbal          (云台指令)"
            echo "  4) /cmd_vel             (底盘速度)"
            echo "  5) /Odometry            (里程计)"
            echo "  6) /referee/robot_status (裁判系统)"
            echo "  7) 自定义话题"
            read -rp "请选择 [1-7]: " echo_choice
            case "${echo_choice}" in
                1) TOPIC="/tracker/target" ;;
                2) TOPIC="/detector/armors" ;;
                3) TOPIC="/cmd_gimbal" ;;
                4) TOPIC="/cmd_vel" ;;
                5) TOPIC="/Odometry" ;;
                6) TOPIC="/referee/robot_status" ;;
                7) read -rp "请输入话题名: " TOPIC ;;
                *) TOPIC="/tracker/target" ;;
            esac
            echo -e "${CYAN}查看话题: ${TOPIC}${NC}"
            ros2 topic echo "${TOPIC}"
            ;;
        4)
            echo -e "${CYAN}生成 TF 树...${NC}"
            ros2 run tf2_tools view_frames
            echo -e "${GREEN}TF 树已保存到 frames.pdf${NC}"
            ;;
        5)
            echo -e "${CYAN}活跃节点列表:${NC}"
            ros2 node list
            ;;
    esac
}

# =============================================================================
# 7) 串口调试 - 仅启动串口 + RViz
# =============================================================================
debug_serial() {
    echo -e "${GREEN}[串口调试] 启动串口 + RViz${NC}"
    source_ros2

    echo -e "${CYAN}检查串口设备...${NC}"
    if ls /dev/ttyACM* 1>/dev/null 2>&1; then
        echo -e "${GREEN}找到设备:${NC}"
        ls -l /dev/ttyACM*
    else
        echo -e "${YELLOW}[警告] 未找到 /dev/ttyACM* 设备${NC}"
        echo -e "  请检查 USB 连接或串口权限"
        echo -e "  sudo usermod -a -G dialout \$USER"
    fi
    echo ""
    read -rp "按 Enter 继续启动，Ctrl+C 取消..."

    ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
        params_file:="${PARAMS_FILE}" \
        use_rviz:=True
}

# =============================================================================
# 8) sp_vision 视觉调试 - 串口 + sp_vision + rqt
# =============================================================================
debug_sp_vision() {
    echo -e "${GREEN}[sp_vision 调试] 启动串口 + sp_vision_25 + rqt${NC}"
    source_ros2

    echo -e "${YELLOW}请选择 sp_vision 配置:${NC}"
    echo "  1) sentry.yaml    (哨兵，默认)"
    echo "  2) standard3.yaml (步兵/英雄)"
    echo "  3) standard4.yaml (备用步兵)"
    echo "  4) uav.yaml       (无人机)"
    read -rp "请选择 [1-4] (默认: 1): " sp_choice
    case "${sp_choice}" in
        2) SP_CONFIG="standard3.yaml" ;;
        3) SP_CONFIG="standard4.yaml" ;;
        4) SP_CONFIG="uav.yaml" ;;
        *) SP_CONFIG="sentry.yaml" ;;
    esac

    echo -e "${CYAN}将在3个终端中启动:${NC}"
    echo "  终端1: 串口节点"
    echo "  终端2: sp_vision_25 (config=${SP_CONFIG})"
    echo "  终端3: rqt_image_view (查看 /sentry_debug/image)"
    echo ""
    read -rp "按 Enter 启动，Ctrl+C 取消..."

    # 终端1: 串口
    gnome-terminal --title="[串口] standard_robot_pp_ros2" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== 串口节点启动 ==='
        ros2 launch standard_robot_pp_ros2 standard_robot_pp_ros2.launch.py \
            params_file:='${PARAMS_FILE}' \
            use_rviz:=False
        exec bash
    " &

    sleep 2

    # 终端2: sp_vision
    gnome-terminal --title="[视觉] sp_vision_25 (${SP_CONFIG})" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== sp_vision_25 启动 (${SP_CONFIG}) ==='
        ros2 launch sp_vision_25 sp_vision_25.launch.py \
            config_path:='${SP_CONFIG}'
        exec bash
    " &

    sleep 3

    # 终端3: rqt
    gnome-terminal --title="[RQT] sp_vision debug image" -- bash -c "
        source '${WS_DIR}/install/setup.bash'
        echo '=== rqt_image_view 启动 ==='
        echo '请在 rqt 中选择话题: /sentry_debug/image'
        ros2 run rqt_image_view rqt_image_view
        exec bash
    " &

    echo -e "${GREEN}所有终端已启动！${NC}"
    echo -e "${YELLOW}提示: sp_vision 调试画面在 /sentry_debug/image 话题${NC}"
    echo -e "${YELLOW}  需要在 sp_vision 配置中开启 debug.enable_visualization: true${NC}"
}

# =============================================================================
# 主菜单
# =============================================================================
show_menu() {
    print_header
    echo -e "${BOLD}请选择调试模式:${NC}"
    echo ""
    echo -e "  ${GREEN}1)${NC} 全局调试        - 启动完整系统 (bringup + RViz)"
    echo -e "  ${GREEN}2)${NC} 视觉调试        - 串口 + 视觉 + rqt 查看画面"
    echo -e "  ${GREEN}3)${NC} 导航调试        - 串口 + 导航 + RViz"
    echo -e "  ${GREEN}4)${NC} 决策调试        - 串口 + 行为树"
    echo -e "  ${GREEN}5)${NC} Rosbag 回放     - 录像回放调试"
    echo -e "  ${GREEN}6)${NC} 话题监控        - 查看话题/节点/TF"
    echo -e "  ${GREEN}7)${NC} 串口调试        - 仅串口 + RViz"
    echo -e "  ${GREEN}8)${NC} sp_vision 调试  - 串口 + sp_vision_25 + rqt"
    echo -e "  ${GREEN}9)${NC} 全局调试(sp)    - 完整系统 + sp_vision_25"
    echo ""
    echo -e "  ${RED}0)${NC} 退出"
    echo ""
}

main() {
    while true; do
        show_menu
        read -rp "请选择 [0-9]: " choice
        echo ""

        case "${choice}" in
            1) debug_full_system ;;
            2) debug_vision ;;
            3) debug_navigation ;;
            4) debug_behavior ;;
            5) debug_rosbag ;;
            6) debug_topic_monitor ;;
            7) debug_serial ;;
            8) debug_sp_vision ;;
            9) debug_full_system_sp_vision ;;
            0)
                echo -e "${CYAN}再见！${NC}"
                exit 0
                ;;
            *)
                echo -e "${RED}无效选择，请重试${NC}"
                ;;
        esac

        echo ""
        read -rp "按 Enter 返回主菜单..."
    done
}

# 支持直接传参快速启动: ./debug.sh <编号>
if [ -n "$1" ]; then
    case "$1" in
        1) debug_full_system ;;
        2) debug_vision ;;
        3) debug_navigation ;;
        4) debug_behavior ;;
        5) debug_rosbag ;;
        6) debug_topic_monitor ;;
        7) debug_serial ;;
        8) debug_sp_vision ;;
        9) debug_full_system_sp_vision ;;
        *)
            echo -e "${RED}无效参数: $1${NC}"
            echo "用法: $0 [1-9]"
            exit 1
            ;;
    esac
else
    main
fi
