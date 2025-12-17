#include "io/ros2cboard.hpp"

#include <fmt/core.h>

namespace io
{

ROS2CBoard::ROS2CBoard(
  const std::string & imu_topic,
  const std::string & gimbal_cmd_topic,
  const std::string & shoot_cmd_topic)
: bullet_speed(21.5),
  mode(Mode::auto_aim),
  shoot_mode(ShootMode::both_shoot),
  ft_angle(0.0),
  imu_received_(false),
  running_(true)
{
  // 初始化ROS2（如果尚未初始化）
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  // 创建ROS2节点
  node_ = std::make_shared<rclcpp::Node>("sp_vision_cboard");

  // 创建IMU订阅器（订阅关节状态）
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
    imu_topic,
    10,
    std::bind(&ROS2CBoard::joint_state_callback, this, std::placeholders::_1));

  // 创建云台指令发布器
  gimbal_cmd_pub_ = node_->create_publisher<pb_rm_interfaces::msg::GimbalCmd>(
    gimbal_cmd_topic, 10);

  // 创建开火指令发布器
  shoot_cmd_pub_ = node_->create_publisher<example_interfaces::msg::UInt8>(
    shoot_cmd_topic, 10);

  // 初始化手眼标定矩阵为单位矩阵（可从配置文件加载）
  R_gimbal_to_imu_ = Eigen::Matrix3d::Identity();

  // TODO: 从配置文件加载真实的R_gimbal_to_imu矩阵
  // 当前使用单位矩阵作为占位符

  // 启动spin线程
  spin_thread_ = std::make_unique<std::thread>(&ROS2CBoard::spin_thread_func, this);

  fmt::print("[ROS2CBoard] Initialized\n");
  fmt::print("  - IMU topic: {}\n", imu_topic);
  fmt::print("  - Gimbal cmd topic: {}\n", gimbal_cmd_topic);
  fmt::print("  - Shoot cmd topic: {}\n", shoot_cmd_topic);
  fmt::print("  - bullet_speed: {} m/s\n", bullet_speed);
}

ROS2CBoard::~ROS2CBoard()
{
  running_ = false;

  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }

  fmt::print("[ROS2CBoard] Destructor called\n");
}

Eigen::Quaterniond ROS2CBoard::imu_at(std::chrono::steady_clock::time_point timestamp)
{
  // 添加 3 秒超时
  auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(3);

  // 等待IMU数据可用
  while (!imu_received_ && rclcpp::ok()) {
    if (std::chrono::steady_clock::now() > timeout) {
      fmt::print("[ROS2CBoard] ⚠️ TIMEOUT: No IMU data received in 3 seconds\n");
      fmt::print("[ROS2CBoard] Check if serial node is publishing to: /serial/gimbal_joint_state\n");

      // 返回单位四元数避免崩溃
      return Eigen::Quaterniond::Identity();
    }
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  // 返回最新的IMU四元数
  std::lock_guard<std::mutex> lock(imu_mutex_);
  return latest_imu_.q;

  // 注意：原CBoard实现了时间戳插值，这里简化为返回最新值
  // 对于高频IMU数据(100Hz)，这个简化是可接受的
}

void ROS2CBoard::send(Command command)
{
  // 发布云台指令
  auto gimbal_cmd = pb_rm_interfaces::msg::GimbalCmd();
  gimbal_cmd.header.stamp = node_->now();
  gimbal_cmd.header.frame_id = "gimbal_yaw";

  if (command.control) {
    // 自瞄控制模式：使用绝对角度控制
    gimbal_cmd.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    gimbal_cmd.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    gimbal_cmd.position.yaw = static_cast<float>(command.yaw);
    gimbal_cmd.position.pitch = static_cast<float>(command.pitch);
  } else if (command.yaw != 0.0 || command.pitch != 0.0) {
    // 目标丢失但有最后已知位置：发送绝对角度以保持云台位置
    gimbal_cmd.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    gimbal_cmd.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    gimbal_cmd.position.yaw = static_cast<float>(command.yaw);
    gimbal_cmd.position.pitch = static_cast<float>(command.pitch);
  } else {
    // 完全无控制模式：速度设为0
    gimbal_cmd.yaw_type = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;
    gimbal_cmd.pitch_type = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;
    gimbal_cmd.velocity.yaw = 0.0f;
    gimbal_cmd.velocity.pitch = 0.0f;
  }

  // 设置角度范围
  gimbal_cmd.position.yaw_min_range = -3.14159f;
  gimbal_cmd.position.yaw_max_range = 3.14159f;
  gimbal_cmd.position.pitch_min_range = -0.5f;
  gimbal_cmd.position.pitch_max_range = 0.5f;

  gimbal_cmd_pub_->publish(gimbal_cmd);

  // 发布开火指令
  auto shoot_cmd = example_interfaces::msg::UInt8();
  shoot_cmd.data = command.shoot ? 1 : 0;
  shoot_cmd_pub_->publish(shoot_cmd);
}

void ROS2CBoard::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 提取yaw和pitch关节角度
  if (msg->position.size() < 2) {
    fmt::print("[ROS2CBoard] Warning: JointState has insufficient position data\n");
    return;
  }

  double pitch = msg->position[0];    // yaw关节角度 (rad)
  double yaw = msg->position[1];  // pitch关节角度 (rad)

  // 转换为四元数
  Eigen::Quaterniond q = joint_angles_to_quaternion(yaw, pitch);

  // 转换时间戳（ROS时间 → std::chrono）
  auto timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
  auto timestamp = std::chrono::steady_clock::time_point(
    std::chrono::nanoseconds(timestamp_ns));

  // 更新最新IMU数据
  {
    std::lock_guard<std::mutex> lock(imu_mutex_);
    latest_imu_.q = q;
    latest_imu_.timestamp = timestamp;
    imu_received_ = true;
  }
}

void ROS2CBoard::spin_thread_func()
{
  while (running_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

Eigen::Quaterniond ROS2CBoard::joint_angles_to_quaternion(double yaw, double pitch)
{
  // 从yaw和pitch关节角度构造云台坐标系旋转
  // 云台坐标系：Z轴向上，X轴向前

  // 步骤1：构造yaw旋转（绕Z轴）
  Eigen::AngleAxisd yaw_rot(yaw, Eigen::Vector3d::UnitZ());

  // 步骤2：构造pitch旋转（绕Y轴）
  Eigen::AngleAxisd pitch_rot(pitch, Eigen::Vector3d::UnitY());

  // 步骤3：组合旋转（先yaw后pitch）
  Eigen::Quaterniond q_gimbal = yaw_rot * pitch_rot;

  // 步骤4：应用手眼标定变换到IMU坐标系
  // R_gimbal_to_imu: 从云台坐标系到IMU坐标系的旋转矩阵
  Eigen::Matrix3d R_world = R_gimbal_to_imu_ * q_gimbal.toRotationMatrix();

  // 步骤5：转换为四元数
  return Eigen::Quaterniond(R_world);
}

}  // namespace io
