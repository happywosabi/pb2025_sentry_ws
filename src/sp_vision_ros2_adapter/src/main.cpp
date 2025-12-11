#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sp_vision_ros2_adapter/adapter_node.hpp"

int main(int argc, char ** argv)
{
  // 初始化ROS2
  rclcpp::init(argc, argv);

  // 创建节点选项
  rclcpp::NodeOptions options;

  // 创建适配器节点
  auto node = std::make_shared<sp_vision_ros2_adapter::AdapterNode>(options);

  // 日志输出
  RCLCPP_INFO(node->get_logger(), "sp_vision_ros2_adapter node started");

  // 进入spin循环
  rclcpp::spin(node);

  // 关闭ROS2
  rclcpp::shutdown();

  return 0;
}
