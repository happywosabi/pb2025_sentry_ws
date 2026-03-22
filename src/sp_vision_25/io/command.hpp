#ifndef IO__COMMAND_HPP
#define IO__COMMAND_HPP

namespace io
{
struct Command
{
  bool control;
  bool shoot;
  double yaw;
  double pitch;
  double horizon_distance = 0;  //无人机专有

  // 寻敌模式字段
  bool search = false;
  double search_yaw_speed = 0;
  double search_pitch_speed = 0;
  double search_pitch_min = 0;
  double search_pitch_max = 0;
};

}  // namespace io

#endif  // IO__COMMAND_HPP