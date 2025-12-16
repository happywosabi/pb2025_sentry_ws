#include "io/ros2camera.hpp"

#include <fmt/core.h>

namespace io
{

ROS2Camera::ROS2Camera(const std::string & topic_name)
: image_received_(false), running_(true)
{
  // 初始化ROS2（如果尚未初始化）
  if (!rclcpp::ok()) {
    int argc = 0;
    char ** argv = nullptr;
    rclcpp::init(argc, argv);
  }

  // 创建ROS2节点
  node_ = std::make_shared<rclcpp::Node>("sp_vision_camera_subscriber");

  // 创建图像订阅器（使用 sensor_data QoS 确保与相机驱动兼容）
  auto qos = rclcpp::SensorDataQoS();  // BEST_EFFORT + VOLATILE
  image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
    topic_name,
    qos,
    std::bind(&ROS2Camera::image_callback, this, std::placeholders::_1));

  // 启动spin线程
  spin_thread_ = std::make_unique<std::thread>(&ROS2Camera::spin_thread_func, this);

  fmt::print("[ROS2Camera] Subscribed to topic: {}\n", topic_name);
}

ROS2Camera::~ROS2Camera()
{
  running_ = false;

  if (spin_thread_ && spin_thread_->joinable()) {
    spin_thread_->join();
  }

  fmt::print("[ROS2Camera] Destructor called\n");
}

void ROS2Camera::read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  fmt::print("[ROS2Camera] Waiting for image (image_received_={})\n", image_received_);

  // 添加 3 秒超时
  auto timeout = std::chrono::steady_clock::now() + std::chrono::seconds(3);

  // 等待图像可用
  while (!image_received_ && rclcpp::ok()) {
    if (std::chrono::steady_clock::now() > timeout) {
      fmt::print("[ROS2Camera] ⚠️ TIMEOUT: No image received in 3 seconds\n");
      fmt::print("[ROS2Camera] Check if camera driver is publishing to: /front_industrial_camera/image\n");

      // 返回空图像避免崩溃
      img = cv::Mat::zeros(720, 1280, CV_8UC3);
      timestamp = std::chrono::steady_clock::now();
      return;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  fmt::print("[ROS2Camera] ✓ Image received, copying data\n");

  // 复制最新图像
  std::lock_guard<std::mutex> lock(image_mutex_);

  if (!latest_image_.image.empty()) {
    img = latest_image_.image.clone();
    timestamp = latest_image_.timestamp;
  } else {
    // 如果没有图像，返回空图像
    img = cv::Mat();
    timestamp = std::chrono::steady_clock::now();
  }
}

void ROS2Camera::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  fmt::print("[ROS2Camera] ✓ Callback triggered: {}x{}\n", msg->width, msg->height);
  try {
    // 将ROS图像消息转换为OpenCV Mat
    auto cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // 转换时间戳（ROS时间 → std::chrono）
    auto timestamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    auto timestamp = std::chrono::steady_clock::time_point(
      std::chrono::nanoseconds(timestamp_ns));

    // 更新最新图像
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      latest_image_.image = cv_ptr->image;
      latest_image_.timestamp = timestamp;
      image_received_ = true;
    }

  } catch (const cv_bridge::Exception & e) {
    fmt::print("[ROS2Camera] cv_bridge exception: {}\n", e.what());
  }
}

void ROS2Camera::spin_thread_func()
{
  while (running_ && rclcpp::ok()) {
    rclcpp::spin_some(node_);
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }
}

}  // namespace io
