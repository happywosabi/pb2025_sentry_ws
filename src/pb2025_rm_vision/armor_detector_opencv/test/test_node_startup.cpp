// Copyright 2022 Chen Jun
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include <memory>

#include "armor_detector_opencv/detector_node.hpp"
#include "gtest/gtest.h"
#include "rclcpp/executors.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/utilities.hpp"

TEST(ArmorDetectorNodeTest, NodeStartupTest)
{
  rclcpp::NodeOptions options;
  auto node = std::make_shared<rm_auto_aim::ArmorDetectorOpencvNode>(options);
  node.reset();
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
