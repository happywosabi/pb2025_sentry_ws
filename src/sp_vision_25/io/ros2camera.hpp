#ifndef IO__ROS2CAMERA_HPP
#define IO__ROS2CAMERA_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <chrono>
#include <memory>
#include <queue>
#include <mutex>
#include <atomic>
#include <thread>

namespace io
{

/**
 * @brief ROS2相机实现类
 *
 * 订阅ROS2图像话题，提供与Camera相同的接口
 */
class ROS2Camera
{
public:
  /**
   * @brief 构造函数
   * @param topic_name ROS2图像话题名称
   */
  explicit ROS2Camera(const std::string & topic_name = "/front_industrial_camera/image");

  /**
   * @brief 析构函数
   */
  ~ROS2Camera();

  /**
   * @brief 读取图像（与io::Camera::read接口相同）
   * @param img 输出图像
   * @param timestamp 输出时间戳
   */
  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp);

private:
  /**
   * @brief 图像回调函数
   * @param msg 图像消息
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief ROS2 spin线程函数
   */
  void spin_thread_func();

  // ROS2节点
  std::shared_ptr<rclcpp::Node> node_;

  // 图像订阅器
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  // 图像缓存
  struct ImageData {
    cv::Mat image;
    std::chrono::steady_clock::time_point timestamp;
  };
  ImageData latest_image_;
  std::mutex image_mutex_;
  bool image_received_;

  // Spin线程
  std::unique_ptr<std::thread> spin_thread_;
  std::atomic<bool> running_;
};

}  // namespace io

#endif  // IO__ROS2CAMERA_HPP
