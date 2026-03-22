#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  spin_center_track_speed_ = yaml["spin_center_track_speed"].as<double>(6.0);
  spin_fire_angle_ = yaml["spin_fire_angle"].as<double>(15.0) / 57.3;  // degree to rad
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    // 中心瞄准模式：额外检查是否有装甲板朝向正面
    if (aimer.is_center_track_active()) {
      auto ekf_x = targets.front().ekf_x();
      auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);
      auto armor_xyza_list = targets.front().armor_xyza_list();
      bool has_facing_armor = false;
      for (const auto & xyza : armor_xyza_list) {
        if (std::abs(tools::limit_rad(xyza[3] - center_yaw)) < spin_fire_angle_) {
          has_facing_armor = true;
          break;
        }
      }
      if (!has_facing_armor) {
        last_command_ = command;
        return false;  // 没有装甲板朝向正面，等待
      }
    }
    last_command_ = command;
    return true;
  }

  last_command_ = command;
  return false;
}

}  // namespace auto_aim