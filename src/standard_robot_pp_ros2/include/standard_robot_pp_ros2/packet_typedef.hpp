// Copyright 2025 SMBU-PolarBear-Robotics-Team
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

#ifndef STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
#define STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace standard_robot_pp_ros2
{
const uint8_t SOF_RECEIVE = 0x5A;
const uint8_t SOF_SEND = 0x5A;

// Receive
const uint8_t ID_DEBUG = 0x01;
const uint8_t ID_IMU = 0x02;
const uint8_t ID_ROBOT_STATE_INFO = 0x03;

const uint8_t ID_GAME_STATUS = 0x04;
const uint8_t ID_GAME_RESULT = 0x05;
const uint8_t ID_GAME_ALL_ROBOT_HP = 0x06;
const uint8_t ID_EVENT_DATA = 0x07;
const uint8_t ID_REFEREE_WARNING = 0x08;
const uint8_t ID_DART_INFO = 0x09;
const uint8_t ID_ROBOT_STATUS = 0x0A;
const uint8_t ID_POWER_HEAT_DATA = 0x0B;
const uint8_t ID_ROBOT_POS = 0x0C;
const uint8_t ID_BUFF = 0x0D;
const uint8_t ID_HURT = 0x0E;
const uint8_t ID_SHOOT = 0x0F;
const uint8_t ID_PROJECTILE_ALLOWANCE = 0x10;
const uint8_t ID_RFID_STATUS = 0x11;
const uint8_t ID_DART_CLIENT_CMD = 0x12;
const uint8_t ID_GROUND_ROBOT_POSITION = 0x13;
const uint8_t ID_RADAR_MARK_DATA = 0x14;
const uint8_t ID_SENTRY_INFO = 0x15;
const uint8_t ID_RADAR_INFO = 0x16;
const uint8_t ID_ROBOT_INTERACTION_DATA= 0x17;
const uint8_t ID_CUSTOM_CONTROLLER = 0x18;
const uint8_t ID_MAP_COMMAND = 0x19;
const uint8_t ID_ROBOT_CUSTOM_DATA = 0x1A;
const uint8_t ID_ROBOT_CUSTOM_DATA_3 = 0x1B;
const uint8_t ID_PID_DEBUG_DATA = 0x1C;
const uint8_t ID_JOINT_STATE = 0x1D;
const uint8_t ID_ROBOT_MOTION_DATA = 0x1E;


// Send
const uint8_t ID_ROBOT_CMD = 0x01;

const uint8_t DEBUG_PACKAGE_NUM = 10;
const uint8_t DEBUG_PACKAGE_NAME_LEN = 10;

struct HeaderFrame
{
  uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
  uint8_t len;  // 数据段长度
  uint8_t id;   // 数据段id
  uint8_t crc;  // 数据帧头的 CRC8 校验
} __attribute__((packed));

/********************************************************/
/* Receive data                                         */
/********************************************************/

// 串口调试数据包
struct ReceiveDebugData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;
  struct
  {
    uint8_t name[DEBUG_PACKAGE_NAME_LEN];
    uint8_t type;
    float data;
  } __attribute__((packed)) packages[DEBUG_PACKAGE_NUM];

  uint16_t checksum;
} __attribute__((packed));

// IMU 数据包
struct ReceiveImuData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    float yaw;    // rad
    float pitch;  // rad
    float roll;   // rad

    float yaw_vel;    // rad/s
    float pitch_vel;  // rad/s
    float roll_vel;   // rad/s

    // float x_accel;  // m/s^2
    // float y_accel;  // m/s^2
    // float z_accel;  // m/s^2
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed)); 

// 机器人信息数据包
struct ReceiveRobotInfoData
{
  HeaderFrame frame_header;
  uint32_t time_stamp;

  struct
  {
    /// @brief 机器人部位类型 2 bytes
    struct
    {
      uint16_t chassis : 3;
      uint16_t gimbal : 3;
      uint16_t shoot : 3;
      uint16_t arm : 3;
      uint16_t custom_controller : 3;
      uint16_t reserve : 1;
    } __attribute__((packed)) type;

    /// @brief 机器人部位状态 1 byte
    /// @note 0: 错误，1: 正常
    struct
    {
      uint8_t chassis : 1;
      uint8_t gimbal : 1;
      uint8_t shoot : 1;
      uint8_t arm : 1;
      uint8_t custom_controller : 1;
      uint8_t reserve : 3;
    } __attribute__((packed)) state;
  } __attribute__((packed)) data;

  uint16_t crc;
} __attribute__((packed));

// 比赛信息数据包
struct ReceiveGameStatusData
{
  HeaderFrame frame_header;  // 数据段id = 0x04
  uint32_t time_stamp;
  struct
  {
    uint8_t game_type : 4;
    uint8_t game_progress : 4;
    uint16_t stage_remain_time;
    uint64_t SyncTimeStamp;
  } __attribute__((packed)) data;
  uint16_t crc;
} __attribute__((packed)) ;


struct ReceiveGameResultData
{
    HeaderFrame frame_header;  // 数据段id = 0x05
    uint32_t time_stamp;
    struct
    {
        uint8_t winner;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

// 全场机器人hp信息数据包
struct ReceiveAllRobotHpData
{
    HeaderFrame frame_header;  // 数据段id = 0x06
    uint32_t time_stamp;
    struct
    {
        uint16_t ally_1_robot_HP;  
        uint16_t ally_2_robot_HP;  
        uint16_t ally_3_robot_HP; 
        uint16_t ally_4_robot_HP;  
        uint16_t reserved;  
        uint16_t ally_7_robot_HP;  
        uint16_t ally_outpost_HP;  
        uint16_t ally_base_HP;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;


// 事件数据包
struct ReceiveEventData
{
    HeaderFrame frame_header;  // 数据段id = 0x07
    uint32_t time_stamp;

    struct
    {
        uint32_t event_data;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

// 裁判警告数据包
struct ReceiveRefereeWarningData
{
    HeaderFrame frame_header;  // 数据段id = 0x08
    uint32_t time_stamp;

    struct
    {
        uint8_t level;
        uint8_t offending_robot_id;
        uint8_t count;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

struct ReceiveDartInfoData
{
    HeaderFrame frame_header;  // 数据段id = 0x09
    uint32_t time_stamp;

    struct
    {
        uint8_t dart_remaining_time;
        uint16_t dart_info;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

// 机器人状态数据包
struct ReceiveRobotStatusData
{
    HeaderFrame frame_header;  // 数据段id = 0x0A
    uint32_t time_stamp;

    struct
    {
        uint8_t robot_id; 
        uint8_t robot_level; 
        uint16_t current_HP;  
        uint16_t maximum_HP; 
        uint16_t shooter_barrel_cooling_value; 
        uint16_t shooter_barrel_heat_limit; 
        uint16_t chassis_power_limit;  
        float x;      //本机器人位置 x 坐标，单位：m
        float y;      //本机器人位置 y 坐标，单位：m
        float angle; 
        uint8_t armor_id : 4;
        uint8_t HP_deduction_reason : 4;
        uint16_t projectile_allowance_17mm;     // 17mm弹丸允许发弹量
        uint16_t projectile_allowance_42mm;     // 42mm弹丸允许发弹量
        uint16_t remaining_gold_coin;           // 剩余金币
        uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm弹丸
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;


// 电源热量数据包
struct ReceivePowerHeatData
{
    HeaderFrame frame_header;  // 数据段id = 0x0B
    uint32_t time_stamp;

    struct
    {
        uint16_t reserved1; 
        uint16_t reserved2; 
        float reserved; 
        uint16_t buffer_energy; 
        uint16_t shooter_17mm_barrel_heat; 
        uint16_t shooter_42mm_barrel_heat;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

// 地面机器人位置数据包
struct ReceiveRobotPosData
{
    HeaderFrame frame_header;  // 数据段id = 0x0C
    uint32_t time_stamp;

    struct
    {
        float x;      //本机器人位置 x 坐标，单位：m
        float y;      //本机器人位置 y 坐标，单位：m
        float angle;  //本机器人测速模块的朝向，单位：度。正北为 0 度
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

// 机器人增益和底盘能量数据包
struct ReceiveBuffData
{
    HeaderFrame frame_header; // 数据段id = 0x0D
    uint32_t time_stamp;

    struct
    {
        uint8_t recovery_buff;
        uint16_t cooling_buff;
        uint8_t defence_buff;
        uint8_t vulnerability_buff;
        uint16_t attack_buff;
        uint8_t remaining_energy;
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 机器人受伤数据包
struct ReceiveHurtData
{
    HeaderFrame frame_header; // 数据段id = 0x0E
    uint32_t time_stamp;

    struct
    {
        uint8_t armor_id : 4;
        uint8_t HP_deduction_reason : 4;
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 机器人射击数据包
struct ReceiveShootData
{
    HeaderFrame frame_header; // 数据段id = 0x0F
    uint32_t time_stamp;

    struct
    {
        uint8_t bullet_type;          //弹丸类型
        uint8_t shooter_number;       //发射机构 ID：
        uint8_t launching_frequency;  //弹丸射速（单位：Hz）
        float initial_speed;          //弹丸初速度（单位：m/s）
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 允许发弹量数据包
struct ReceiveProjectileAllowanceData
{
    HeaderFrame frame_header; // 数据段id = 0x10
    uint32_t time_stamp;

    struct
    {
        uint16_t projectile_allowance_17mm;     // 17mm弹丸允许发弹量
        uint16_t projectile_allowance_42mm;     // 42mm弹丸允许发弹量
        uint16_t remaining_gold_coin;           // 剩余金币
        uint16_t projectile_allowance_fortress; // 堡垒增益点提供的储备17mm弹丸
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// RFID状态数据包
struct ReceiveRfidStatusData
{
    HeaderFrame frame_header; // 数据段id = 0x11
    uint32_t time_stamp;

    struct
    {
        uint32_t rfid_status;                    // 32位RFID状态
        uint8_t rfid_status_2;                    // 扩展RFID状态
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 飞镖客户端命令数据包
struct ReceiveDartClientCmdData
{
    HeaderFrame frame_header; // 数据段id = 0x12
    uint32_t time_stamp;

    struct
    {
        uint8_t dart_launch_opening_status;
        uint8_t reserved;
        uint16_t target_change_time;
        uint16_t latest_launch_cmd_time;
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 地面机器人位置数据包
struct ReceiveGroundRobotPositionData
{
    HeaderFrame frame_header; // 数据段id = 0x13
    uint32_t time_stamp;

    struct
    {
        float hero_x;
        float hero_y;
        float engineer_x;
        float engineer_y;
        float standard_3_x;
        float standard_3_y;
        float standard_4_x;
        float standard_4_y;
        float reserved1;
        float reserved2;
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 雷达标志数据包
struct ReceiveRadarMarkDataData
{
    HeaderFrame frame_header; // 数据段id = 0x14
    uint32_t time_stamp;

    struct
    {
        uint16_t mark_progress;                    // 位域定义参见协议
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

//哨兵信息数据包
struct ReceiveSentryInfoData
{
    HeaderFrame frame_header; // 数据段id = 0x15
    uint32_t time_stamp;

    struct
    {
        uint32_t sentry_info;                       // 32位哨兵信息
        uint16_t sentry_info_2;                      // 扩展哨兵信息
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 雷达信息数据包
struct ReceiveRadarInfoData
{
    HeaderFrame frame_header; // 数据段id = 0x16
    uint32_t time_stamp;

    struct
    {
        uint8_t radar_info;
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 机器人交互数据包
struct ReceiveRobotInteractionData
{
    HeaderFrame frame_header; // 数据段id = 0x17
    uint32_t time_stamp;

    struct
    {
        uint16_t data_cmd_id;
        uint16_t sender_id;
        uint16_t receiver_id;
        uint8_t user_data[112];                      // 最大112字节
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 自定义控制器数据包
struct ReceiveCustomControllerData
{
    HeaderFrame frame_header; // 数据段id = 0x18
    uint32_t time_stamp;

    struct
    {
        uint8_t data[30];
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 地图命令数据包
struct ReceiveMapCommandData
{
    HeaderFrame frame_header; // 数据段id = 0x19
    uint32_t time_stamp;

    struct
    {
        float target_position_x; 
        float target_position_y; 
        uint8_t cmd_keyboard; 
        uint8_t target_robot_id; 
        uint16_t cmd_source; 
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 机器人自定义数据包
struct ReceiveRobotCustomDataData
{
    HeaderFrame frame_header; // 数据段id = 0x1A
    uint32_t time_stamp;

    struct
    {
        uint8_t data[30]; 
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// 机器人自定义数据包3
struct ReceiveRobotCustomData3Data
{
    HeaderFrame frame_header; // 数据段id = 0x1B
    uint32_t time_stamp;

    struct
    {
        uint8_t data[30]; 
    } __attribute__((packed)) data;

    uint16_t crc;
} __attribute__((packed)) ;

// PID调参数据包
struct ReceivePidDebugData
{
    HeaderFrame frame_header;  // 数据段id = 0x1C
    uint32_t time_stamp;
    struct
    {
        float fdb;
        float ref;
        float pid_out;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

// 云台状态数据包
struct ReceiveJointStateData
{
    HeaderFrame frame_header;  // 数据段id = 0x1D
    uint32_t time_stamp;
    struct
    {
        float pitch;
        float yaw;

    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;

// 机器人运动数据包
struct ReceiveRobotMotionData
{
    HeaderFrame frame_header;  // 数据段id = 0x09
    uint32_t time_stamp;
    struct
    {
        struct
        {
            float vx;
            float vy;
            float wz;
        } __attribute__((packed)) speed_vector;
    } __attribute__((packed)) data;
    uint16_t crc;
} __attribute__((packed)) ;



/********************************************************/
/* Send data                                            */
/********************************************************/

struct SendRobotCmdData
{
  HeaderFrame frame_header;

  uint32_t time_stamp;

  struct
  {
    struct
    {
      float vx;
      float vy;
      float wz;
    } __attribute__((packed)) speed_vector;

    struct
    {
      float roll;
      float pitch;
      float yaw;
      float leg_lenth;
    } __attribute__((packed)) chassis;

    struct
    {
      float pitch;
      float yaw;
    } __attribute__((packed)) gimbal;

    struct
    {
      uint8_t fire;
      uint8_t fric_on;
    } __attribute__((packed)) shoot;

    struct
    {
      bool tracking;
    } __attribute__((packed)) tracking;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __attribute__((packed));

/********************************************************/
/* template                                             */
/********************************************************/

template <typename T>
inline T fromVector(const std::vector<uint8_t> & data)
{
  T packet;
  std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
  return packet;
}

template <typename T>
inline std::vector<uint8_t> toVector(const T & data)
{
  std::vector<uint8_t> packet(sizeof(T));
  std::copy(
    reinterpret_cast<const uint8_t *>(&data), reinterpret_cast<const uint8_t *>(&data) + sizeof(T),
    packet.begin());
  return packet;
}

}  // namespace standard_robot_pp_ros2

#endif  // STANDARD_ROBOT_PP_ROS2__PACKET_TYPEDEF_HPP_
