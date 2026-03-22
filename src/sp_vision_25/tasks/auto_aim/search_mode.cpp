#include "search_mode.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>

#include "tools/logger.hpp"

namespace auto_aim
{

SearchMode::SearchMode(const std::string & config_path)
{
  auto yaml = YAML::LoadFile(config_path);

  if (yaml["search_mode"].IsDefined()) {
    auto node = yaml["search_mode"];
    enabled_ = node["enabled"].as<bool>(false);
    lost_timeout_ = node["lost_timeout"].as<double>(3.0);
    yaw_speed_ = node["yaw_speed"].as<double>(1.5);
    pitch_speed_ = node["pitch_speed"].as<double>(0.3);
    pitch_center_ = node["pitch_center"].as<double>(0.0);
    pitch_amplitude_ = node["pitch_amplitude"].as<double>(0.15);
    pitch_frequency_ = node["pitch_frequency"].as<double>(0.5);
  } else {
    enabled_ = false;
    lost_timeout_ = 3.0;
    yaw_speed_ = 1.5;
    pitch_speed_ = 0.3;
    pitch_center_ = 0.0;
    pitch_amplitude_ = 0.15;
    pitch_frequency_ = 0.5;
  }

  tools::logger()->info(
    "[SearchMode] enabled={}, lost_timeout={:.1f}s, yaw_speed={:.2f} rad/s, "
    "pitch_frequency={:.2f} Hz, pitch_center={:.2f}, pitch_amplitude={:.2f}",
    enabled_, lost_timeout_, yaw_speed_, pitch_frequency_, pitch_center_, pitch_amplitude_);
}

void SearchMode::update(
  io::Command & command,
  const std::string & tracker_state,
  std::chrono::steady_clock::time_point now)
{
  if (!enabled_) {
    active_ = false;
    return;
  }

  // 有目标（tracking/detecting/switching）时重置计时器
  // temp_lost 和 lost 状态视为目标丢失，应该开始计时
  if (tracker_state == "tracking" || tracker_state == "detecting" || tracker_state == "switching") {
    active_ = false;
    timing_ = false;
    return;
  }

  // 进入 lost 状态，开始计时
  if (!timing_) {
    timing_ = true;
    lost_start_time_ = now;
  }

  double elapsed =
    std::chrono::duration<double>(now - lost_start_time_).count();

  if (elapsed < lost_timeout_) {
    active_ = false;
    return;
  }

  // 超时，激活寻敌模式
  if (!active_) {
    // 首次激活，记录开始时间
    active_ = true;
    search_start_time_ = now;
  }

  // 计算寻敌模式运行时间
  double search_elapsed =
    std::chrono::duration<double>(now - search_start_time_).count();

  // 计算正弦波 pitch 速度: v = A * 2π * f * cos(2π * f * t)
  double omega = 2.0 * M_PI * pitch_frequency_;
  double pitch_velocity = pitch_amplitude_ * omega * std::cos(omega * search_elapsed);

  command.control = false;
  command.shoot = false;
  command.search = true;
  command.search_yaw_speed = yaw_speed_;
  command.search_pitch_speed = pitch_velocity;
  command.search_pitch_min = pitch_center_ - pitch_amplitude_;
  command.search_pitch_max = pitch_center_ + pitch_amplitude_;
}

}  // namespace auto_aim
