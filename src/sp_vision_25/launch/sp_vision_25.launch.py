#!/usr/bin/env python3
# Copyright 2025 SMBU PolarBear Robotics Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0

"""
sp_vision_25 launch file for pb2025_sentry_ws

This launch file starts the sentry_ros2 node from sp_vision_25 package.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包的share目录
    sp_vision_pkg = get_package_share_directory('sp_vision_25')

    # 默认配置文件路径
    default_config = os.path.join(sp_vision_pkg, 'configs', 'sentry.yaml')

    # Launch参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    config_path = LaunchConfiguration('config_path', default=default_config)

    # 声明launch参数
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_config_path_cmd = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='Path to sp_vision configuration YAML file')

    # sentry_ros2节点
    sentry_ros2_node = Node(
        package='sp_vision_25',
        executable='sentry_ros2',
        name='sp_vision_25',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        # 注意：不传递config_path作为参数，因为sentry_ros2从package share自动加载
        # 如果需要自定义配置，可以通过命令行参数传递给可执行文件
    )

    # 创建launch description
    ld = LaunchDescription()

    # 添加launch参数声明
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_config_path_cmd)

    # 添加节点
    ld.add_action(sentry_ros2_node)

    return ld
