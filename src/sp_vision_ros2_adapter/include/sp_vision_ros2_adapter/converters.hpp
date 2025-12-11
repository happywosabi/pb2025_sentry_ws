#ifndef SP_VISION_ROS2_ADAPTER__CONVERTERS_HPP_
#define SP_VISION_ROS2_ADAPTER__CONVERTERS_HPP_

#include <string>
#include <chrono>
#include <Eigen/Geometry>

// ROS2 messages
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <example_interfaces/msg/u_int8.hpp>

// Custom interfaces
#include <pb_rm_interfaces/msg/gimbal_cmd.hpp>
#include <pb_rm_interfaces/msg/robot_status.hpp>
#include <auto_aim_interfaces/msg/target.hpp>

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace sp_vision_ros2_adapter
{

/**
 * @brief sp_vision Command 结构体（简化版，仅用于类型定义）
 */
struct SpCommand {
    bool control;
    bool shoot;
    double yaw;    // rad
    double pitch;  // rad
};

// ============================================================================
// 输出转换函数（sp_vision → pb2025）
// ============================================================================

/**
 * @brief 函数1: 转换云台控制指令
 *
 * @param sp_cmd sp_vision的Command结构
 * @param ros2_cmd pb2025的GimbalCmd消息（输出）
 * @param timestamp ROS2时间戳
 */
void convert_sp_command_to_gimbal(
    const SpCommand& sp_cmd,
    pb_rm_interfaces::msg::GimbalCmd& ros2_cmd,
    const rclcpp::Time& timestamp
);

/**
 * @brief 函数2: 转换开火指令
 *
 * @param shoot sp_vision的开火标志
 * @param shoot_msg pb2025的UInt8消息（输出）
 */
void convert_sp_shoot_to_uint8(
    bool shoot,
    example_interfaces::msg::UInt8& shoot_msg
);

/**
 * @brief 函数3: 解析目标字符串为Target消息
 *
 * @param target_str sp_vision发布的字符串 "x,y,z,id"
 * @param timestamp ROS2时间戳
 * @return auto_aim_interfaces::msg::Target pb2025的Target消息
 */
auto_aim_interfaces::msg::Target parse_sp_target_string(
    const std::string& target_str,
    const rclcpp::Time& timestamp
);

// ============================================================================
// 输入转换函数（pb2025 → sp_vision）
// ============================================================================

/**
 * @brief 函数4: 转换ROS图像为OpenCV Mat
 *
 * @param ros_img pb2025的Image消息
 * @param cv_img OpenCV Mat对象（输出）
 * @param timestamp 图像时间戳（输出，std::chrono格式）
 */
void convert_ros_image_to_opencv(
    const sensor_msgs::msg::Image::SharedPtr& ros_img,
    cv::Mat& cv_img,
    std::chrono::steady_clock::time_point& timestamp
);

/**
 * @brief 函数5: 关节角度转四元数（关键函数）
 *
 * @param yaw yaw关节角度 (rad)
 * @param pitch pitch关节角度 (rad)
 * @param R_gimbal_to_imu 云台到IMU的旋转矩阵（3×3）
 * @return Eigen::Quaterniond IMU四元数
 */
Eigen::Quaterniond joint_angles_to_quaternion(
    double yaw,
    double pitch,
    const Eigen::Matrix3d& R_gimbal_to_imu
);

/**
 * @brief 函数6: 提取子弹速度（从参数或RobotStatus）
 *
 * @param status_msg pb2025的RobotStatus消息（可选）
 * @param default_speed 默认子弹速度 (m/s)
 * @return double 子弹速度 (m/s)
 */
double extract_bullet_speed(
    const pb_rm_interfaces::msg::RobotStatus::SharedPtr& status_msg,
    double default_speed = 21.5
);

// ============================================================================
// 辅助函数
// ============================================================================

/**
 * @brief ROS Time 转 std::chrono时间戳
 *
 * @param ros_time ROS2时间
 * @return std::chrono::steady_clock::time_point chrono时间点
 */
std::chrono::steady_clock::time_point ros_time_to_chrono(
    const rclcpp::Time& ros_time
);

/**
 * @brief std::chrono时间戳转 ROS Time
 *
 * @param chrono_time chrono时间点
 * @return rclcpp::Time ROS2时间
 */
rclcpp::Time chrono_to_ros_time(
    const std::chrono::steady_clock::time_point& chrono_time
);

}  // namespace sp_vision_ros2_adapter

#endif  // SP_VISION_ROS2_ADAPTER__CONVERTERS_HPP_
