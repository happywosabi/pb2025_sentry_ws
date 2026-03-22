#ifndef IO__ROS2CBOARD_HPP
#define IO__ROS2CBOARD_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pb_rm_interfaces/msg/gimbal_cmd.hpp>
#include <example_interfaces/msg/u_int8.hpp>

#include <Eigen/Geometry>
#include <chrono>
#include <memory>
#include <mutex>
#include <thread>
#include <atomic>

#include "io/command.hpp"
#include "io/cboard.hpp"  // 引用原始枚举定义

namespace io
{

/**
 * @brief ROS2 CBoard实现类
 *
 * 订阅ROS2 IMU话题，发布云台控制指令，提供与CBoard相同的接口
 */
class ROS2CBoard
{
public:
  double bullet_speed;
  Mode mode;
  ShootMode shoot_mode;
  double ft_angle;  // 无人机专有

  /**
   * @brief 构造函数
   * @param imu_topic IMU话题名称（JointState）
   * @param gimbal_cmd_topic 云台指令话题名称
   * @param shoot_cmd_topic 开火指令话题名称
   */
  explicit ROS2CBoard(
    const std::string & imu_topic = "/serial/gimbal_joint_state",
    const std::string & gimbal_cmd_topic = "/cmd_gimbal",
    const std::string & shoot_cmd_topic = "/cmd_shoot");

  /**
   * @brief 析构函数
   */
  ~ROS2CBoard();

  /**
   * @brief 获取指定时刻的IMU四元数（与CBoard::imu_at接口相同）
   * @param timestamp 时间戳
   * @return IMU四元数
   */
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

  /**
   * @brief 发送云台控制指令（与CBoard::send接口相同）
   * @param command 控制指令
   */
  void send(Command command);

  /**
   * @brief 获取ROS2节点指针（用于创建其他发布器/订阅器）
   * @return ROS2节点智能指针
   */
  std::shared_ptr<rclcpp::Node> node() { return node_; }

private:
  /**
   * @brief 关节状态回调函数
   * @param msg 关节状态消息
   */
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

  /**
   * @brief ROS2 spin线程函数
   */
  void spin_thread_func();

  /**
   * @brief 从关节角度转换为四元数
   * @param yaw yaw角度（rad）
   * @param pitch pitch角度（rad）
   * @return 四元数
   */
  Eigen::Quaterniond joint_angles_to_quaternion(double yaw, double pitch);

  // ROS2节点
  std::shared_ptr<rclcpp::Node> node_;

  // 订阅器
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  // 发布器
  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_pub_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_cmd_pub_;

  // IMU数据缓存
  struct IMUData {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };
  IMUData latest_imu_;
  std::mutex imu_mutex_;
  bool imu_received_;

  // 手眼标定矩阵（从gimbal坐标系到IMU坐标系）
  Eigen::Matrix3d R_gimbal_to_imu_;

  // Spin线程
  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> running_;

  // 速率限制（100Hz）
  std::chrono::steady_clock::time_point last_send_time_;
  std::mutex send_mutex_;
  static constexpr double send_rate_hz_ = 100.0;  // 发送频率限制为100Hz，匹配GimbalManagerNode
};

}  // namespace io

#endif  // IO__ROS2CBOARD_HPP
