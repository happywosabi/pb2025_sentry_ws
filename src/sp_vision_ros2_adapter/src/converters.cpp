#include "sp_vision_ros2_adapter/converters.hpp"

#include <sstream>
#include <vector>
#include <Eigen/Geometry>

namespace sp_vision_ros2_adapter
{

// ============================================================================
// 输出转换函数（sp_vision → pb2025）
// ============================================================================

void convert_sp_command_to_gimbal(
  const SpCommand & sp_cmd,
  pb_rm_interfaces::msg::GimbalCmd & ros2_cmd,
  const rclcpp::Time & timestamp)
{
  // 设置消息头
  ros2_cmd.header.stamp = timestamp;
  ros2_cmd.header.frame_id = "gimbal_yaw";

  // 设置控制模式
  // sp_cmd.control == true 表示接管云台控制，使用绝对角度控制
  // sp_cmd.control == false 表示不控制云台
  if (sp_cmd.control) {
    ros2_cmd.yaw_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
    ros2_cmd.pitch_type = pb_rm_interfaces::msg::GimbalCmd::ABSOLUTE_ANGLE;
  } else {
    // 不控制时设置为速度控制模式，速度为0
    ros2_cmd.yaw_type = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;
    ros2_cmd.pitch_type = pb_rm_interfaces::msg::GimbalCmd::VELOCITY;
  }

  // 设置角度值（rad → rad，直接映射）
  ros2_cmd.position.yaw = static_cast<float>(sp_cmd.yaw);
  ros2_cmd.position.pitch = static_cast<float>(sp_cmd.pitch);

  // 设置范围（使用默认值）
  ros2_cmd.position.yaw_min_range = -3.14159f;  // -π
  ros2_cmd.position.yaw_max_range = 3.14159f;   // +π
  ros2_cmd.position.pitch_min_range = -0.5f;    // 云台pitch下限
  ros2_cmd.position.pitch_max_range = 0.5f;     // 云台pitch上限

  // 速度清零（位置控制模式不使用速度）
  ros2_cmd.velocity.yaw = 0.0f;
  ros2_cmd.velocity.pitch = 0.0f;
  ros2_cmd.velocity.yaw_min_range = -3.14159f;
  ros2_cmd.velocity.yaw_max_range = 3.14159f;
  ros2_cmd.velocity.pitch_min_range = -3.14159f;
  ros2_cmd.velocity.pitch_max_range = 3.14159f;
}

void convert_sp_shoot_to_uint8(
  bool shoot,
  example_interfaces::msg::UInt8 & shoot_msg)
{
  // 简单映射：true → 1 (开火), false → 0 (不开火)
  shoot_msg.data = shoot ? static_cast<uint8_t>(1) : static_cast<uint8_t>(0);
}

auto_aim_interfaces::msg::Target parse_sp_target_string(
  const std::string & target_str,
  const rclcpp::Time & timestamp)
{
  auto_aim_interfaces::msg::Target target;

  // 设置消息头
  target.header.stamp = timestamp;
  target.header.frame_id = "gimbal_pitch_odom";

  // 解析字符串 "x,y,z,id"
  // sp_vision_25格式：x,y,z,id (逗号分隔)
  std::istringstream iss(target_str);
  std::string token;
  std::vector<std::string> tokens;

  while (std::getline(iss, token, ',')) {
    tokens.push_back(token);
  }

  // 验证格式
  if (tokens.size() >= 4) {
    try {
      // 提取位置信息 (m)
      target.position.x = std::stod(tokens[0]);
      target.position.y = std::stod(tokens[1]);
      target.position.z = std::stod(tokens[2]);

      // 提取ID (sp_vision使用1-9，pb2025使用0-8或字符串)
      int armor_id = static_cast<int>(std::stod(tokens[3]));
      target.id = std::to_string(armor_id - 1);  // 转换为0-based索引

      // 设置跟踪状态
      target.tracking = true;

      // 初始化其他字段（sp_vision的字符串格式不包含这些信息）
      target.armors_num = 4;  // 默认4块装甲板（平衡步兵）
      target.velocity.x = 0.0;
      target.velocity.y = 0.0;
      target.velocity.z = 0.0;
      target.yaw = 0.0;
      target.v_yaw = 0.0;
      target.radius_1 = 0.26;  // 默认半径（标准装甲板）
      target.radius_2 = 0.26;
      target.dz = 0.0;

    } catch (const std::exception & e) {
      // 解析失败，标记为未跟踪
      target.tracking = false;
    }
  } else {
    // 格式错误，标记为未跟踪
    target.tracking = false;
  }

  return target;
}

// ============================================================================
// 输入转换函数（pb2025 → sp_vision）
// ============================================================================

void convert_ros_image_to_opencv(
  const sensor_msgs::msg::Image::SharedPtr & ros_img,
  cv::Mat & cv_img,
  std::chrono::steady_clock::time_point & timestamp)
{
  try {
    // 使用cv_bridge转换 ROS Image → OpenCV Mat
    // 指定目标编码为BGR8（sp_vision_25使用BGR格式）
    auto cv_ptr = cv_bridge::toCvCopy(ros_img, sensor_msgs::image_encodings::BGR8);

    // 获取转换后的cv::Mat
    cv_img = cv_ptr->image;

    // 转换时间戳（ROS时间 → std::chrono）
    timestamp = ros_time_to_chrono(rclcpp::Time(ros_img->header.stamp));

  } catch (const cv_bridge::Exception & e) {
    // cv_bridge转换异常处理
    // 输出空图像，时间戳为当前时间
    cv_img = cv::Mat();
    timestamp = std::chrono::steady_clock::now();

    // 可以在这里记录日志（但需要访问logger）
    // RCLCPP_ERROR(logger, "cv_bridge exception: %s", e.what());
  }
}

Eigen::Quaterniond joint_angles_to_quaternion(
  double yaw,
  double pitch,
  const Eigen::Matrix3d & R_gimbal_to_imu)
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
  Eigen::Matrix3d R_world = R_gimbal_to_imu * q_gimbal.toRotationMatrix();

  // 步骤5：转换为四元数
  return Eigen::Quaterniond(R_world);
}

double extract_bullet_speed(
  const pb_rm_interfaces::msg::RobotStatus::SharedPtr & status_msg,
  double default_speed)
{
  // 当前pb2025的RobotStatus消息不包含bullet_speed字段
  // 因此直接返回默认值（从参数服务器获取）
  //
  // 未来如果RobotStatus添加了bullet_speed字段，可以在这里提取：
  // if (status_msg != nullptr) {
  //   return status_msg->bullet_speed;
  // }

  (void)status_msg;  // 避免未使用参数警告
  return default_speed;
}

// ============================================================================
// 辅助函数
// ============================================================================

std::chrono::steady_clock::time_point ros_time_to_chrono(
  const rclcpp::Time & ros_time)
{
  // ROS Time → std::chrono::steady_clock::time_point
  //
  // 注意：ROS时间是系统时间（wall time），而std::chrono::steady_clock
  // 是单调时钟（monotonic clock），两者起点不同
  //
  // 这里的转换用于sp_vision_25内部的时间戳管理
  // 只要时间间隔正确即可，绝对时间值不重要

  auto ns = ros_time.nanoseconds();
  return std::chrono::steady_clock::time_point(std::chrono::nanoseconds(ns));
}

rclcpp::Time chrono_to_ros_time(
  const std::chrono::steady_clock::time_point & chrono_time)
{
  // std::chrono::steady_clock::time_point → ROS Time
  //
  // 将std::chrono时间点转换为ROS时间
  // 用于设置输出消息的时间戳

  auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
    chrono_time.time_since_epoch()).count();
  return rclcpp::Time(ns);
}

}  // namespace sp_vision_ros2_adapter
