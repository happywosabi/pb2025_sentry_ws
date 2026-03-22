#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  /// 当前帧是否处于反陀螺中心瞄准模式
  bool is_center_track_active() const { return center_track_active_; }

  /// 反陀螺中心瞄准模式的射击朝向角度窗口 (rad)
  double spin_fire_angle() const { return spin_fire_angle_; }

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;
  io::Command last_valid_command_;  // Store last known good command for lost target handling

  // 反陀螺中心瞄准模式参数
  double spin_center_track_speed_;  // 激活中心瞄准的角速度阈值 (rad/s)
  double spin_fire_angle_;          // 装甲板朝向角度射击窗口 (rad)
  bool center_track_active_ = false;

  AimPoint choose_aim_point(const Target & target);
  AimPoint choose_aim_point_center_track(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP