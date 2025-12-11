#ifndef SP_VISION_ROS2_ADAPTER__ADAPTER_NODE_HPP_
#define SP_VISION_ROS2_ADAPTER__ADAPTER_NODE_HPP_

#include <memory>
#include <string>
#include <Eigen/Geometry>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <pb_rm_interfaces/msg/gimbal_cmd.hpp>
#include <pb_rm_interfaces/msg/robot_status.hpp>
#include <auto_aim_interfaces/msg/target.hpp>
#include <example_interfaces/msg/u_int8.hpp>

#include "sp_vision_ros2_adapter/converters.hpp"

namespace sp_vision_ros2_adapter
{

/**
 * @brief ROS2适配器节点类
 *
 * 功能：
 * - 订阅pb2025系统话题（图像、IMU、裁判系统）
 * - 转换数据格式供sp_vision_25使用
 * - 接收sp_vision_25输出并转换为ROS2消息
 * - 发布到pb2025系统（云台指令、开火指令、目标信息）
 */
class AdapterNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数
   */
  explicit AdapterNode(const rclcpp::NodeOptions & options);

  /**
   * @brief 析构函数
   */
  ~AdapterNode() override = default;

private:
  // ============================================================================
  // 订阅器 (pb2025 → sp_vision)
  // ============================================================================

  /**
   * @brief 相机图像订阅器
   */
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  /**
   * @brief 云台关节状态订阅器（用于IMU四元数转换）
   */
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  /**
   * @brief 裁判系统机器人状态订阅器（用于提取子弹速度）
   */
  rclcpp::Subscription<pb_rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;

  /**
   * @brief sp_vision目标话题订阅器（可选功能）
   * 订阅 sp_vision_25 通过 ROS2 发布的目标字符串
   */
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sp_target_sub_;

  // ============================================================================
  // 发布器 (sp_vision → pb2025)
  // ============================================================================

  /**
   * @brief 云台控制指令发布器
   */
  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;

  /**
   * @brief 开火指令发布器
   */
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_cmd_pub_;

  /**
   * @brief 目标信息发布器
   */
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr target_pub_;

  // ============================================================================
  // 回调函数
  // ============================================================================

  /**
   * @brief 图像回调函数
   * @param msg 图像消息
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief 关节状态回调函数
   * @param msg 关节状态消息
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief 机器人状态回调函数
   * @param msg 机器人状态消息
   */
  void robot_status_callback(const pb_rm_interfaces::msg::RobotStatus::SharedPtr msg);

  /**
   * @brief sp_vision目标话题回调函数（可选功能）
   * @param msg 目标字符串消息 "x,y,z,id"
   */
  void sp_target_callback(const std_msgs::msg::String::SharedPtr msg);

  // ============================================================================
  // 参数
  // ============================================================================

  /**
   * @brief 子弹速度参数 (m/s)
   */
  double bullet_speed_;

  /**
   * @brief 是否使用sp_vision的目标话题（可选功能）
   */
  bool use_sp_target_topic_;

  // ============================================================================
  // 内部状态
  // ============================================================================

  /**
   * @brief 手眼标定矩阵：云台到IMU的旋转矩阵
   * 从sentry.yaml加载：R_gimbal2imubody
   */
  Eigen::Matrix3d R_gimbal_to_imu_;

  /**
   * @brief 最近接收的IMU四元数（从关节角度转换得到）
   */
  Eigen::Quaterniond latest_imu_quaternion_;

  /**
   * @brief 最近接收的图像时间戳
   */
  rclcpp::Time latest_image_timestamp_;

  /**
   * @brief 参数声明和初始化
   */
  void declare_parameters();

  /**
   * @brief 加载手眼标定矩阵
   */
  void load_calibration_matrix();
};

}  // namespace sp_vision_ros2_adapter

#endif  // SP_VISION_ROS2_ADAPTER__ADAPTER_NODE_HPP_
