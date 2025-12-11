#include "sp_vision_ros2_adapter/adapter_node.hpp"

#include <vector>

namespace sp_vision_ros2_adapter
{

AdapterNode::AdapterNode(const rclcpp::NodeOptions & options)
: Node("sp_vision_ros2_adapter", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing sp_vision_ros2_adapter node");

  // 声明和加载参数
  declare_parameters();
  load_calibration_matrix();

  // ============================================================================
  // 创建订阅器 (pb2025 → sp_vision)
  // ============================================================================

  image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/front_industrial_camera/image",
    10,
    std::bind(&AdapterNode::image_callback, this, std::placeholders::_1));

  joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/serial/gimbal_joint_state",
    10,
    std::bind(&AdapterNode::joint_state_callback, this, std::placeholders::_1));

  robot_status_sub_ = this->create_subscription<pb_rm_interfaces::msg::RobotStatus>(
    "/referee/robot_status",
    10,
    std::bind(&AdapterNode::robot_status_callback, this, std::placeholders::_1));

  // ============================================================================
  // 创建发布器 (sp_vision → pb2025)
  // ============================================================================

  gimbal_cmd_pub_ = this->create_publisher<pb_rm_interfaces::msg::GimbalCmd>(
    "/cmd_gimbal",
    10);

  shoot_cmd_pub_ = this->create_publisher<example_interfaces::msg::UInt8>(
    "/cmd_shoot",
    10);

  target_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>(
    "/tracker/target",
    10);

  // ============================================================================
  // 可选功能：订阅sp_vision的目标话题（如果启用）
  // ============================================================================

  if (use_sp_target_topic_) {
    // 订阅 sp_vision_25 通过 ROS2 发布的目标字符串
    // 格式："x,y,z,id"
    sp_target_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/sp_vision/target",
      10,
      std::bind(&AdapterNode::sp_target_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Subscribed to /sp_vision/target (optional feature)");
  }

  RCLCPP_INFO(this->get_logger(), "sp_vision_ros2_adapter node initialized successfully");
  RCLCPP_INFO(this->get_logger(), "  - bullet_speed: %.2f m/s", bullet_speed_);
  RCLCPP_INFO(this->get_logger(), "  - use_sp_target_topic: %s",
              use_sp_target_topic_ ? "true" : "false");
}

void AdapterNode::declare_parameters()
{
  // 声明参数
  this->declare_parameter<double>("bullet_speed", 21.5);
  this->declare_parameter<bool>("use_sp_target_topic", false);

  // 声明手眼标定矩阵参数（9个元素）
  this->declare_parameter<std::vector<double>>(
    "R_gimbal_to_imu",
    std::vector<double>{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0}  // 单位矩阵默认值
  );

  // 获取参数值
  bullet_speed_ = this->get_parameter("bullet_speed").as_double();
  use_sp_target_topic_ = this->get_parameter("use_sp_target_topic").as_bool();
}

void AdapterNode::load_calibration_matrix()
{
  // 从参数服务器获取手眼标定矩阵
  auto R_vec = this->get_parameter("R_gimbal_to_imu").as_double_array();

  if (R_vec.size() != 9) {
    RCLCPP_ERROR(this->get_logger(),
                 "R_gimbal_to_imu parameter must have 9 elements, got %zu",
                 R_vec.size());
    RCLCPP_WARN(this->get_logger(), "Using identity matrix as fallback");
    R_gimbal_to_imu_ = Eigen::Matrix3d::Identity();
    return;
  }

  // 转换为Eigen矩阵（行主序）
  R_gimbal_to_imu_ << R_vec[0], R_vec[1], R_vec[2],
                      R_vec[3], R_vec[4], R_vec[5],
                      R_vec[6], R_vec[7], R_vec[8];

  RCLCPP_INFO(this->get_logger(), "Loaded R_gimbal_to_imu calibration matrix");
}

// ============================================================================
// 回调函数实现
// ============================================================================

void AdapterNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // 转换 ROS Image → OpenCV Mat
  cv::Mat cv_img;
  std::chrono::steady_clock::time_point timestamp;

  convert_ros_image_to_opencv(msg, cv_img, timestamp);

  if (cv_img.empty()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Failed to convert image");
    return;
  }

  // 存储最新图像时间戳
  latest_image_timestamp_ = msg->header.stamp;

  // TODO: Phase 4 - 将转换后的图像传递给 sp_vision_25
  // 当前实现：仅转换和验证，未实际传递
  // 未来可以通过共享内存、管道或修改 sp_vision_25 订阅 ROS2 话题实现
}

void AdapterNode::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
  // 提取 yaw 和 pitch 关节角度
  if (msg->position.size() < 2) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "JointState message has insufficient position data");
    return;
  }

  double yaw = msg->position[0];    // yaw 关节角度 (rad)
  double pitch = msg->position[1];  // pitch 关节角度 (rad)

  // 转换为 IMU 四元数
  latest_imu_quaternion_ = joint_angles_to_quaternion(yaw, pitch, R_gimbal_to_imu_);

  // TODO: Phase 4 - 将四元数传递给 sp_vision_25
  // 当前实现：仅转换和存储，未实际传递
}

void AdapterNode::robot_status_callback(
  const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg)
{
  // 提取子弹速度（当前RobotStatus不包含此字段，使用参数默认值）
  double current_bullet_speed = extract_bullet_speed(msg, bullet_speed_);

  // 子弹速度已在初始化时从参数服务器加载
  // 如果未来RobotStatus包含bullet_speed字段，可以在这里更新

  (void)current_bullet_speed;  // 避免未使用变量警告
}

void AdapterNode::sp_target_callback(const std_msgs::msg::String::SharedPtr msg)
{
  // 解析 sp_vision_25 发布的目标字符串 "x,y,z,id"
  auto target = parse_sp_target_string(msg->data, this->now());

  if (!target.tracking) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Failed to parse sp_vision target string: %s", msg->data.c_str());
    return;
  }

  // 发布转换后的 Target 消息到 pb2025 系统
  target_pub_->publish(target);

  RCLCPP_DEBUG(this->get_logger(),
               "Published target: pos=(%.2f, %.2f, %.2f), id=%s",
               target.position.x, target.position.y, target.position.z,
               target.id.c_str());
}

}  // namespace sp_vision_ros2_adapter
