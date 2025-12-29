#ifndef IO__SERIAL_IMU_HPP
#define IO__SERIAL_IMU_HPP

#include <serial/serial.h>

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <thread>

#include "tools/thread_safe_queue.hpp"

namespace io
{

// 协议常量
const uint8_t SOF_RECEIVE = 0x5A;
const uint8_t ID_IMU = 0x02;

// 包头结构（4字节）
struct __attribute__((packed)) HeaderFrame
{
  uint8_t sof;  // 0x5A
  uint8_t len;  // 数据长度
  uint8_t id;   // 数据包ID
  uint8_t crc;  // CRC8校验
};

// IMU数据包（34字节）
struct __attribute__((packed)) ReceiveImuData
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
  } __attribute__((packed)) data;
  uint16_t crc;  // CRC16
};

class SerialIMU
{
public:
  SerialIMU(const std::string & config_path);
  ~SerialIMU();

  // 获取指定时刻的四元数（SLERP插值）
  Eigen::Quaterniond imu_at(std::chrono::steady_clock::time_point timestamp);

private:
  struct IMUData
  {
    Eigen::Quaterniond q;
    std::chrono::steady_clock::time_point timestamp;
  };

  void init_serial();
  void receive_thread_func();

  // CRC 函数
  uint8_t get_crc8(const uint8_t * data, uint16_t len);
  bool verify_crc8(const uint8_t * data, uint16_t len);
  uint16_t get_crc16(const uint8_t * data, uint32_t len);
  bool verify_crc16(const uint8_t * data, uint32_t len);

  serial::Serial serial_;
  std::thread receive_thread_;
  std::atomic<bool> stop_thread_{false};

  tools::ThreadSafeQueue<IMUData> queue_;
  IMUData data_ahead_;
  IMUData data_behind_;

  std::string serial_port_;
  uint32_t baudrate_;
};

}  // namespace io

#endif  // IO__SERIAL_IMU_HPP
