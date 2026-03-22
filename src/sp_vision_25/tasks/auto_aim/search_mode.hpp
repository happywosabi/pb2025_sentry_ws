#ifndef AUTO_AIM__SEARCH_MODE_HPP
#define AUTO_AIM__SEARCH_MODE_HPP

#include <chrono>
#include <string>

#include "io/command.hpp"

namespace auto_aim
{

class SearchMode
{
public:
  explicit SearchMode(const std::string & config_path);

  /// 根据 tracker 状态更新寻敌模式，可能修改 command
  void update(
    io::Command & command,
    const std::string & tracker_state,
    std::chrono::steady_clock::time_point now);

  bool is_active() const { return active_; }

private:
  bool enabled_;
  double lost_timeout_;       // 丢失后进入寻敌的延时(秒)
  double yaw_speed_;          // yaw 旋转速度 (rad/s)
  double pitch_speed_;        // pitch 点头速度 (rad/s)
  double pitch_center_;       // pitch 中心角度 (rad)
  double pitch_amplitude_;    // pitch 上下幅度 (rad)

  bool active_ = false;
  bool timing_ = false;
  std::chrono::steady_clock::time_point lost_start_time_;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SEARCH_MODE_HPP
