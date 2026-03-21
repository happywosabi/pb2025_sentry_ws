#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "io/camera.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

const std::string keys =
  "{help h usage ?  |                          | 输出命令行参数说明}"
  "{@config-path c  | configs/calibration.yaml | yaml配置文件路径 }"
  "{output-folder o |      assets/img_with_q   | 输出文件夹路径   }"
  "{imu-topic t     | /serial/gimbal_joint_state | IMU话题名称    }";

// 线程安全的四元数存储
struct QuaternionData
{
  Eigen::Quaterniond q{Eigen::Quaterniond::Identity()};
  std::chrono::steady_clock::time_point timestamp;
  std::mutex mutex;
  std::atomic<bool> received{false};
};

static QuaternionData g_quat_data;

// 从关节角度(yaw, pitch)转换为四元数
// 与 ROS2CBoard::joint_angles_to_quaternion() 一致
Eigen::Quaterniond joint_angles_to_quaternion(double yaw, double pitch)
{
  Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd pitch_rot(pitch, Eigen::Vector3d::UnitY());
  return Eigen::Quaterniond(yaw_rot * pitch_rot);
}

// JointState 回调
void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (msg->position.size() < 2) {
    return;
  }
  // position[0] = gimbal_pitch_joint (pitch, rad)
  // position[1] = gimbal_yaw_joint (yaw, rad)
  double pitch = msg->position[0];
  double yaw = msg->position[1];

  Eigen::Quaterniond q = joint_angles_to_quaternion(yaw, pitch);

  {
    std::lock_guard<std::mutex> lock(g_quat_data.mutex);
    g_quat_data.q = q;
    g_quat_data.timestamp = std::chrono::steady_clock::now();
    g_quat_data.received = true;
  }
}

void write_q(const std::string & q_path, double w, double x, double y, double z)
{
  std::ofstream q_file(q_path);
  // 输出顺序为wxyz
  q_file << fmt::format("{} {} {} {}", w, x, y, z);
  q_file.close();
}

void capture_loop(
  const std::string & config_path, const std::string & output_folder)
{
  // 读取YAML配置中的标定板参数
  auto yaml = YAML::LoadFile(config_path);
  int pattern_cols = yaml["pattern_cols"].as<int>();
  int pattern_rows = yaml["pattern_rows"].as<int>();
  cv::Size pattern_size(pattern_cols, pattern_rows);

  io::Camera camera(config_path);
  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;

  // 限制显示帧率到 ~10fps，减少CPU占用
  const auto frame_interval = std::chrono::milliseconds(100);
  auto last_frame_time = std::chrono::steady_clock::now();

  int count = 0;
  while (rclcpp::ok()) {
    camera.read(img, timestamp);

    // 跳帧：只在间隔足够时才处理显示和检测
    auto now = std::chrono::steady_clock::now();
    if (now - last_frame_time < frame_interval) {
      continue;
    }
    last_frame_time = now;

    // 先缩小图像再做所有处理（显示+检测），大幅减少CPU占用
    const double scale = 0.5;
    cv::Mat img_small;
    cv::resize(img, img_small, {}, scale, scale);
    cv::Size pattern_size_small = pattern_size;  // 角点检测在缩小图上做

    auto img_display = img_small.clone();

    // 显示当前四元数/欧拉角信息
    if (g_quat_data.received) {
      Eigen::Quaterniond q;
      {
        std::lock_guard<std::mutex> lock(g_quat_data.mutex);
        q = g_quat_data.q;
      }
      Eigen::Vector3d zyx = tools::eulers(q, 2, 1, 0) * 57.3;  // degree
      tools::draw_text(img_display, fmt::format("Z(yaw)  {:.2f}", zyx[0]), {20, 20}, {0, 0, 255});
      tools::draw_text(img_display, fmt::format("Y(pitch) {:.2f}", zyx[1]), {20, 40}, {0, 0, 255});
      tools::draw_text(img_display, fmt::format("X(roll)  {:.2f}", zyx[2]), {20, 60}, {0, 0, 255});
      tools::draw_text(
        img_display, fmt::format("q: w={:.4f} x={:.4f} y={:.4f} z={:.4f}",
          q.w(), q.x(), q.y(), q.z()),
        {20, 80}, {0, 255, 255});
    } else {
      tools::draw_text(img_display, "Waiting for IMU data...", {20, 20}, {0, 0, 255});
    }

    tools::draw_text(
      img_display, fmt::format("ROS2 mode | Saved: {} images", count), {20, 100}, {0, 255, 0});

    // 在缩小图像上检测棋盘格标定板（仅用于预览）
    std::vector<cv::Point2f> centers_2d;
    auto success = cv::findChessboardCorners(img_small, pattern_size_small, centers_2d,
      cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FAST_CHECK);
    if (success) {
      cv::Mat gray;
      cv::cvtColor(img_small, gray, cv::COLOR_BGR2GRAY);
      cv::cornerSubPix(gray, centers_2d, cv::Size(5, 5), cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 20, 0.01));
    }
    cv::drawChessboardCorners(img_display, pattern_size_small, centers_2d, success);

    cv::imshow("Press s to save, q to quit", img_display);
    auto key = cv::waitKey(1);
    if (key == 'q')
      break;
    else if (key != 's')
      continue;

    // 检查四元数是否可用
    if (!g_quat_data.received) {
      tools::logger()->warn("尚未收到IMU数据，无法保存。请检查串口节点是否已启动。");
      continue;
    }

    // 获取当前四元数
    Eigen::Quaterniond q;
    {
      std::lock_guard<std::mutex> lock(g_quat_data.mutex);
      q = g_quat_data.q;
    }

    double w = q.w(), x = q.x(), y = q.y(), z = q.z();

    // 保存图片和四元数
    count++;
    auto img_path = fmt::format("{}/{}.jpg", output_folder, count);
    auto q_path = fmt::format("{}/{}.txt", output_folder, count);
    cv::imwrite(img_path, img);
    write_q(q_path, w, x, y, z);
    tools::logger()->info("[{}] Saved in {} (q: w={:.4f} x={:.4f} y={:.4f} z={:.4f})",
      count, output_folder, w, x, y, z);
  }
}

int main(int argc, char * argv[])
{
  // 初始化ROS2
  rclcpp::init(argc, argv);

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    rclcpp::shutdown();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto output_folder = cli.get<std::string>("output-folder");
  auto imu_topic = cli.get<std::string>("imu-topic");

  std::filesystem::create_directories(output_folder);

  auto yaml = YAML::LoadFile(config_path);
  int pattern_cols = yaml["pattern_cols"].as<int>();
  int pattern_rows = yaml["pattern_rows"].as<int>();

  // 创建ROS2节点并订阅JointState话题
  auto node = std::make_shared<rclcpp::Node>("capture_ros2_node");
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    imu_topic, 10, joint_state_callback);

  // 在独立线程中运行ROS2 spin
  std::atomic<bool> spinning{true};
  std::thread spin_thread([&node, &spinning]() {
    while (spinning && rclcpp::ok()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  });

  tools::logger()->info("ROS2模式：自动从串口节点获取四元数");
  tools::logger()->info("订阅话题: {}", imu_topic);
  tools::logger()->info("标定板内角点尺寸: {}列{}行", pattern_cols, pattern_rows);
  tools::logger()->info("使用流程:");
  tools::logger()->info("  1. 确保 standard_robot_pp_ros2 节点已在另一终端启动");
  tools::logger()->info("  2. 调整云台到合适位置，使标定板在画面中可见");
  tools::logger()->info("  3. 按 's' 自动保存当前图像和四元数");
  tools::logger()->info("  4. 重复步骤直到采集足够图像（建议15-20张）");
  tools::logger()->info("  5. 按 'q' 退出");

  capture_loop(config_path, output_folder);

  tools::logger()->warn("注意四元数输出顺序为wxyz");

  spinning = false;
  spin_thread.join();
  rclcpp::shutdown();

  return 0;
}
