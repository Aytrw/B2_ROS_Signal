#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
B2 Native RC Bridge 启动文件

启动遥控器信号桥接节点，监听 B2 遥控器信号并发布到 ROS2。

Author: Your Name
Date: 2026-01-09
License: BSD-3-Clause
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """生成 Launch 描述"""
    
    # 获取包路径
    pkg_share = get_package_share_directory('b2_native_rc_interface')
    
    # 默认配置文件路径
    default_config_file = os.path.join(pkg_share, 'config', 'rc_bridge_params.yaml')
    
    # ========== Launch 参数 ==========
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Path to the configuration YAML file'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic',
        default_value='/b2_native_rc_signal',
        description='Output topic name for NativeRC messages'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error)'
    )
    
    # ========== 节点定义 ==========
    rc_bridge_node = Node(
        package='b2_native_rc_interface',
        executable='rc_bridge_node',
        name='b2_native_rc_bridge',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'output_topic': LaunchConfiguration('output_topic'),
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        emulate_tty=True,
    )
    
    # ========== 返回 Launch 描述 ==========
    return LaunchDescription([
        # 参数声明
        config_file_arg,
        output_topic_arg,
        log_level_arg,
        
        # 节点
        rc_bridge_node,
    ])
