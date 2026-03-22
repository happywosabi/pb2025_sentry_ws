#include <fmt/core.h>

#include <chrono>
#include <fstream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <yaml-cpp/yaml.h>

// ROS2 IO模块（ROS2CBoard用于IMU和控制指令）
#include "io/camera.hpp"       // 原生相机接口（直接硬件访问）
#include "io/ros2cboard.hpp"

// ROS2调试可视化
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>

// ROS2包索引（用于查找配置文件）
#include <ament_index_cpp/get_package_share_directory.hpp>

// ROS2 Target消息（用于发布跟踪结果给决策层）
#include <auto_aim_interfaces/msg/target.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>

// sp_vision核心算法模块
#include "io/command.hpp"
// #include "io/ros2/publish2nav.hpp"  // 依赖sp_msgs，暂时注释
// #include "io/ros2/ros2.hpp"          // 依赖sp_msgs，暂时注释
#include "io/usbcamera/usbcamera.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/search_mode.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
// #include "tasks/omniperception/decider.hpp"  // 全向感知，暂时不需要
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

using namespace std::chrono;

/// ============================================================================
/// Target转换函数：sp_vision内部Target → auto_aim_interfaces::msg::Target
/// ============================================================================

std::string armor_name_to_string(auto_aim::ArmorName name)
{
  switch (name) {
    case auto_aim::ArmorName::sentry:
      return "sentry";
    case auto_aim::ArmorName::one:
      return "1";
    case auto_aim::ArmorName::two:
      return "2";
    case auto_aim::ArmorName::three:
      return "3";
    case auto_aim::ArmorName::four:
      return "4";
    case auto_aim::ArmorName::five:
      return "5";
    case auto_aim::ArmorName::outpost:
      return "outpost";
    case auto_aim::ArmorName::base:
      return "base";
    case auto_aim::ArmorName::not_armor:
      return "unknown";
    default:
      return "unknown";
  }
}

auto_aim_interfaces::msg::Target convert_target_to_ros2(
  const auto_aim::Target & sp_target,
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  bool is_tracking)
{
  auto_aim_interfaces::msg::Target target_msg;

  // Header
  target_msg.header.stamp = stamp;
  target_msg.header.frame_id = frame_id;  // "base_footprint" or "gimbal_pitch"

  // Tracking状态
  target_msg.tracking = is_tracking;

  // 目标ID（从ArmorName转换）
  target_msg.id = armor_name_to_string(sp_target.name);

  // EKF状态向量：x vx y vy z vz yaw v_yaw radius r_diff dz
  Eigen::VectorXd ekf_x = sp_target.ekf_x();

  // 位置：机器人中心坐标 (x, y, z)
  target_msg.position.x = ekf_x[0];
  target_msg.position.y = ekf_x[2];
  target_msg.position.z = ekf_x[4];

  // 速度：机器人中心速度 (vx, vy, vz)
  target_msg.velocity.x = ekf_x[1];
  target_msg.velocity.y = ekf_x[3];
  target_msg.velocity.z = ekf_x[5];

  // Yaw角和角速度
  target_msg.yaw = ekf_x[6];
  target_msg.v_yaw = ekf_x[7];

  // 半径参数：radius_1 和 radius_2（从radius和r_diff计算）
  double radius = ekf_x[8];
  double r_diff = ekf_x[9];  // l = r2 - r1
  target_msg.radius_1 = radius;  // 使用当前半径作为radius_1
  target_msg.radius_2 = radius + r_diff;  // radius_2 = radius_1 + r_diff

  // 高度差 dz
  target_msg.dz = ekf_x[10];

  // 装甲板数量（需要从sp_vision内部获取，默认设为4）
  // 注意：auto_aim::Target没有公开armor_num_，这里使用经验值
  // 根据机器人类型推断：哨兵=3，前哨站=3，基地=4，其他=4
  if (sp_target.name == auto_aim::ArmorName::sentry ||
      sp_target.name == auto_aim::ArmorName::outpost) {
    target_msg.armors_num = 3;
  } else {
    target_msg.armors_num = 4;  // 默认4个装甲板（步兵、英雄、基地等）
  }

  return target_msg;
}

/// ============================================================================

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder;

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  // 配置文件路径：优先使用命令行参数，否则使用ROS2包share目录
  std::string config_path;
  bool config_from_cmdline = false;

  // 尝试从命令行参数获取配置文件路径（跳过 ROS2 参数）
  if (argc >= 2) {
    std::string potential_config = cli.get<std::string>(0);
    // 检查是否是有效的配置文件路径（不是 ROS2 参数）
    if (!potential_config.empty() &&
        potential_config[0] != '-' &&  // 不是选项参数
        potential_config.find("__") != 0) {  // 不是 ROS2 内部参数
      // 检查文件是否存在
      std::ifstream file(potential_config);
      if (file.good()) {
        config_path = potential_config;
        config_from_cmdline = true;
        fmt::print("[Main] Loading config from command line: {}\n", config_path);
      }
    }
  }

  // 如果命令行没有提供有效配置文件，使用 ROS2 包默认配置
  if (!config_from_cmdline) {
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("sp_vision_25");
      config_path = pkg_share + "/configs/sentry.yaml";
      fmt::print("[Main] Loading config from package share: {}\n", config_path);
    } catch (const std::exception & e) {
      fmt::print("[Main] ERROR: Failed to find sp_vision_25 package: {}\n", e.what());
      fmt::print("[Main] Please provide config path as command line argument.\n");
      return 1;
    }
  }

  // 读取调试配置
  YAML::Node config = YAML::LoadFile(config_path);
  bool enable_visualization = config["debug"]["enable_visualization"].as<bool>(false);
  bool enable_recorder = config["debug"]["enable_recorder"].as<bool>(false);
  double display_scale = config["debug"]["display_scale"].as<double>(1.0);

  fmt::print("\n");
  fmt::print("========================================\n");
  fmt::print("  sp_vision_25 ROS2 Sentry Node\n");
  fmt::print("========================================\n");
  fmt::print("Config: {}\n", config_path);
  fmt::print("Debug visualization: {}\n", enable_visualization ? "ON" : "OFF");
  fmt::print("Debug recorder: {}\n", enable_recorder ? "ON" : "OFF");
  fmt::print("\n");

  // ============================================================================
  // 初始化IO模块
  // ============================================================================

  io::Camera camera(config_path);  // 原生相机（直接硬件访问，从配置文件初始化）
  io::ROS2CBoard cboard("/serial/gimbal_joint_state", "/cmd_gimbal", "/cmd_shoot");

  // ROS2调试可视化发布器（条件初始化）
  std::unique_ptr<image_transport::ImageTransport> it;
  image_transport::Publisher debug_image_pub;

  if (enable_visualization) {
    it = std::make_unique<image_transport::ImageTransport>(cboard.node());
    debug_image_pub = it->advertise("/sentry_debug/image", 1);
    fmt::print("[Main] Debug image publisher created: /sentry_debug/image\n");
  }

  // ROS2 Target发布器（发布跟踪结果给决策层和导航层）
  auto target_pub = cboard.node()->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target", rclcpp::QoS(10));
  fmt::print("[Main] Target publisher created: /tracker/target\n");

  // 其他IO模块（保留用于全向感知）
  // io::ROS2 ros2;  // 依赖sp_msgs，暂时注释
  // io::USBCamera usbcam1("video0", config_path);
  // io::USBCamera usbcam2("video2", config_path);

  // ============================================================================
  // 初始化视觉算法模块
  // ============================================================================

  auto_aim::YOLO yolo(config_path, false);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::SearchMode search_mode(config_path);

  // omniperception::Decider decider(config_path);  // 全向感知，暂时不需要

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  io::Command last_command;

  fmt::print("[Main] Starting main loop...\n");

  // ============================================================================
  // 主循环
  // ============================================================================

  while (!exiter.exit()) {
    static int loop_count = 0;
    fmt::print("[Main] === Loop iteration: {} ===\n", loop_count++);

    // 读取图像（从ROS2话题）
    camera.read(img, timestamp);
    fmt::print("[Main] ✓ Camera data ready\n");

    // 获取IMU四元数（从ROS2话题）
    Eigen::Quaterniond q = cboard.imu_at(timestamp - 1ms);

    // 录制视频（可选）
    if (enable_recorder)
      recorder.record(img, q, timestamp);

    /// 自瞄核心逻辑
    solver.set_R_gimbal2world(q);

    Eigen::Vector3d gimbal_pos = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto armors = yolo.detect(img);

    // Debug: 检测结果
    fmt::print("[Main] ✓ YOLO detected {} armors\n", armors.size());
    if (!armors.empty()) {
      fmt::print("[Main]   First armor: color={}, type={}\n",
                 static_cast<int>(armors.front().color),
                 static_cast<int>(armors.front().type));
    }

    // decider.get_invincible_armor(ros2.subscribe_enemy_status());  // 依赖sp_msgs，暂时注释

    // decider.armor_filter(armors);  // 暂时不使用全向感知

    // decider.get_auto_aim_target(armors, ros2.subscribe_autoaim_target());

    // decider.set_priority(armors);  // 暂时不使用全向感知

    auto targets = tracker.track(armors, timestamp);

    // Debug: 跟踪结果
    fmt::print("[Main] ✓ Tracker: state=[{}], {} targets\n",
               tracker.state(), targets.size());

    // 发布Target消息给决策层和导航层
    bool is_tracking = (tracker.state() == "tracking" && !targets.empty());
    if (!targets.empty()) {
      // 转换并发布第一个目标（最高优先级）
      auto now = cboard.node()->now();
      auto target_msg = convert_target_to_ros2(
        targets.front(), "base_footprint", now, is_tracking);
      target_pub->publish(target_msg);
      fmt::print("[Main] ✓ Published target: id={}, tracking={}, pos=({:.2f}, {:.2f}, {:.2f})\n",
                 target_msg.id, target_msg.tracking,
                 target_msg.position.x, target_msg.position.y, target_msg.position.z);
    } else {
      // 无目标时也发布空消息（tracking=false）
      auto_aim_interfaces::msg::Target empty_target;
      empty_target.header.stamp = cboard.node()->now();
      empty_target.header.frame_id = "base_footprint";
      empty_target.tracking = false;
      empty_target.id = "none";
      target_pub->publish(empty_target);
      fmt::print("[Main] ✓ Published empty target (no tracking)\n");
    }

    /// 自瞄控制逻辑：无条件调用 aimer.aim()
    // 由 Aimer 内部处理空 targets 的情况（返回默认 command）
    // 这样可以确保即使在 "lost" 或 "detecting" 状态也能正确处理
    auto command = aimer.aim(targets, timestamp, cboard.bullet_speed, cboard.shoot_mode);

    // Debug: 瞄准结果
    fmt::print("[Main] ✓ Command: control={}, yaw={:.3f}, pitch={:.3f}\n",
               command.control, command.yaw, command.pitch);

    /// 发射逻辑
    command.shoot = shooter.shoot(command, aimer, targets, gimbal_pos);

    // Debug: 射击决策
    fmt::print("[Main] ✓ Shoot: {}\n", command.shoot);

    /// 寻敌模式：在发送前检查是否需要进入寻敌
    search_mode.update(command, tracker.state(), timestamp);

    // Debug: 寻敌状态
    if (search_mode.is_active()) {
      fmt::print("[Main] ✓ Search mode ACTIVE\n");
    }

    // 发送控制指令（到ROS2话题）
    cboard.send(command);

    /// 调试可视化（条件执行）
    if (enable_visualization) {
      cv::Mat debug_img = img.clone();

      // 绘制跟踪状态文本
      tools::draw_text(
        debug_img, fmt::format("Tracker: [{}]", tracker.state()), {10, 30}, {255, 255, 255});

      // 绘制检测信息
      tools::draw_text(
        debug_img, fmt::format("Armors: {}", armors.size()), {10, 60}, {255, 255, 0});

      // 绘制装甲板检测框（黄色）
      for (const auto & armor : armors) {
        cv::rectangle(debug_img, armor.box, cv::Scalar(0, 255, 255), 2);
      }

      // 如果有跟踪目标
      if (!targets.empty()) {
        auto target = targets.front();

        // 绘制EKF预测的装甲板位置（绿色）
        std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
        for (const Eigen::Vector4d & xyza : armor_xyza_list) {
          auto image_points =
            solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
          tools::draw_points(debug_img, image_points, {0, 255, 0});
        }

        // 绘制瞄准点（蓝色=有效，红色=无效）
        auto aim_point = aimer.debug_aim_point;
        Eigen::Vector4d aim_xyza = aim_point.xyza;
        auto image_points =
          solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
        if (aim_point.valid)
          tools::draw_points(debug_img, image_points, {255, 0, 0});  // 蓝色（BGR）
        else
          tools::draw_points(debug_img, image_points, {0, 0, 255});  // 红色（BGR）

        // 绘制目标信息
        Eigen::VectorXd x = target.ekf_x();
        tools::draw_text(
          debug_img, fmt::format("Pos: ({:.2f},{:.2f},{:.2f})", x[0], x[2], x[4]), {10, 90},
          {0, 255, 0});
      }

      // 绘制控制指令
      if (search_mode.is_active()) {
        tools::draw_text(debug_img, "Search Mode", {10, 120}, {0, 165, 255});
      } else if (aimer.is_center_track_active() && command.control) {
        tools::draw_text(debug_img, "Center Track (Anti-Spin)", {10, 120}, {0, 255, 128});
        tools::draw_text(
          debug_img,
          fmt::format(
            "Cmd: Y={:.1f} P={:.1f} S={}", command.yaw * 57.3, command.pitch * 57.3,
            command.shoot),
          {10, 150}, {0, 255, 255});
      } else if (command.control) {
        tools::draw_text(
          debug_img,
          fmt::format(
            "Cmd: Y={:.1f} P={:.1f} S={}", command.yaw * 57.3, command.pitch * 57.3,
            command.shoot),
          {10, 120}, {0, 255, 255});
      }

      // 按比例缩放图像
      if (display_scale != 1.0) {
        cv::resize(debug_img, debug_img, {}, display_scale, display_scale);
      }

      // 发布调试图像到ROS2话题
      std_msgs::msg::Header header;
      header.stamp = cboard.node()->now();
      header.frame_id = "camera_optical_frame";

      auto debug_msg = cv_bridge::CvImage(header, "bgr8", debug_img).toImageMsg();
      debug_image_pub.publish(debug_msg);
    }

    /// ROS2通信（发布目标信息到导航系统）
    // Eigen::Vector4d target_info = decider.get_target_info(armors, targets);  // 依赖sp_msgs，暂时注释
    // ros2.publish(target_info);  // 依赖sp_msgs，暂时注释
  }

  fmt::print("[Main] Exiting...\n");
  return 0;
}
