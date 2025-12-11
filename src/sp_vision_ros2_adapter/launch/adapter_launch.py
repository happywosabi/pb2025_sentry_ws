#!/usr/bin/env python3

"""
sp_vision_ros2_adapter launch file

功能：
  - 启动sp_vision_ros2_adapter节点
  - 加载参数配置文件

使用方法：
  ros2 launch sp_vision_ros2_adapter adapter_launch.py
  ros2 launch sp_vision_ros2_adapter adapter_launch.py params_file:=<绝对路径>
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # 获取包路径
    pkg_share = get_package_share_directory('sp_vision_ros2_adapter')

    # 默认参数文件路径
    default_params_file = os.path.join(pkg_share, 'config', 'adapter_params.yaml')

    # 声明launch参数
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file to use for the adapter node'
    )

    # 创建适配器节点
    adapter_node = Node(
        package='sp_vision_ros2_adapter',
        executable='adapter_node',
        name='sp_vision_ros2_adapter',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('params_file')],
        # 重新映射话题名称（如果需要）
        # remappings=[
        #     ('/cmd_gimbal', '/my_cmd_gimbal'),
        # ],
    )

    return LaunchDescription([
        params_file_arg,
        adapter_node,
    ])
