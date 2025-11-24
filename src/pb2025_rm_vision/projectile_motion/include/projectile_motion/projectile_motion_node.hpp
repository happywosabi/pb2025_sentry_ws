// Copyright 2023 Yunlong Feng
// Copyright 2025 Lihan Chen
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_
#define PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>

#include "auto_aim_interfaces/msg/target.hpp"
#include "example_interfaces/msg/u_int8.hpp"
#include "message_filters/subscriber.h"
#include "pb_rm_interfaces/msg/gimbal_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rmoss_projectile_motion/gimbal_transform_tool.hpp"
#include "rmoss_projectile_motion/projectile_solver_interface.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker.hpp"

namespace projectile_motion
{

class ProjectileMotionNode : public rclcpp::Node
{
public:
  using tf2_filter = tf2_ros::MessageFilter<auto_aim_interfaces::msg::Target>;

public:
  explicit ProjectileMotionNode(const rclcpp::NodeOptions & options);

private:
  void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  void publishGimbalCommand(double hit_pitch, double hit_yaw, uint8_t shoot);
  void calculateTargetPosition(
    const auto_aim_interfaces::msg::Target::SharedPtr & msg,
    const Eigen::Vector3d & center_position, const Eigen::Vector3d & center_velocity,
    double cur_yaw, double & hit_yaw, double & hit_pitch);
  bool getCurrentGimbalAngle(
    const std::string & frame_id, const rclcpp::Time & stamp, double & cur_roll, double & cur_pitch,
    double & cur_yaw);
  void publishHitYawMarker(double hit_yaw, double hit_pitch);

private:
  double offset_x_;
  double offset_y_;
  double offset_z_;
  double offset_pitch_;
  double offset_yaw_;
  double offset_time_;
  double shoot_speed_;
  double friction_{0.001};

  double cur_pitch_;
  double cur_yaw_;
  double cur_roll_;

  std::string target_topic_;
  std::string gimbal_cmd_topic_;
  std::string shoot_cmd_topic_;
  std::string solver_type_;
  std::string shooter_frame_;

  std::shared_ptr<rmoss_projectile_motion::ProjectileSolverInterface> solver_;
  rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_subscriber_;
  rclcpp::Publisher<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_publisher_;
  rclcpp::Publisher<example_interfaces::msg::UInt8>::SharedPtr shoot_cmd_publisher_;

  message_filters::Subscriber<auto_aim_interfaces::msg::Target> target_sub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_filter> tf_filter_;

  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr aiming_marker_publisher_;
};

}  // namespace projectile_motion

#endif  // PROJECTILE_MOTION__PROJECTILE_MOTION_NODE_HPP_
